package memory

import chisel3._
import chisel3.util._
import fetch.iFetchParams
import memory.{AXI4_AR, AXI4_AW, RData}

/*
MemController is the bridge between your chip's internal bus and physical DRAM.
Every module that needs to read or write memory — ICache, PageWalker, later DCache — sends requests to the MemController.
It serializes them, handles DRAM timing, and returns responses.

The most important insight in the reference is that it does not use a simple single-request state machine.
Instead it maintains a transfer table —an array of in-flight transaction slots called transfers[AXI_NUM_TRANS].
Each slot tracks one independent transaction from enqueue to completion independently.

This means the MemController can have multiple transactions in flight simultaneously —
one might be waiting for an AXI read response while another is sending write data.
This is far more efficient than a sequential state machine that blocks on each transaction.

*/

class MemController(
                     NUM_TFS     : Int = 8,
                     NUM_TFS_IN  : Int = 3,
                     axiParams   : AXIParams,
                     fetchParams : iFetchParams
                   ) extends Module {

  val AXI_WIDTH = axiParams.AXI_WIDTH

  val io = IO(new Bundle {
    val IN_ctrl = Input(Vec(NUM_TFS_IN, new MemC_Req(fetchParams, AXI_WIDTH)))
    val OUT_stat = Output(new MemC_Res(fetchParams, NUM_TFS, NUM_TFS_IN, AXI_WIDTH))
    val axi = new AXI4Bundle(axiParams)

    // Cache SRAM interfaces wired to CacheWriteInterface / CacheReadInterface
    // These are the output ports that connect to the actual cache SRAMs
    val icacheW = Output(new CacheSRAMWriteIF(fetchParams))
    val dcacheW = Output(new CacheSRAMWriteIF(fetchParams)) // stub
    val dcacheR = Output(new CacheSRAMReadIF(fetchParams)) // stub
    val dcacheRData = Input(UInt((32 * 4).W)) // stub
    val clear = Input(Bool())
  })

  // Transfer table
  val transfers = RegInit(VecInit(
    Seq.fill(NUM_TFS)(0.U.asTypeOf(new Transfer(fetchParams, AXI_WIDTH)))
  ))

  //isMMIO — combinational per slot
  val isMMIO = VecInit(transfers.map { t =>
    t.valid && MuxLookup(t.cmd.asUInt, false.B)(Seq(
      MemC_Cmd.MEMC_READ_BYTE.asUInt -> true.B,
      MemC_Cmd.MEMC_READ_HALF.asUInt -> true.B,
      MemC_Cmd.MEMC_READ_WORD.asUInt -> true.B,
      MemC_Cmd.MEMC_WRITE_BYTE.asUInt -> true.B,
      MemC_Cmd.MEMC_WRITE_HALF.asUInt -> true.B,
      MemC_Cmd.MEMC_WRITE_WORD.asUInt -> true.B
    ))
  })


  //enqIdx — first free slot
  val enqIdxValid = transfers.map(!_.valid).reduce(_ || _)
  val enqIdx = PriorityEncoder(VecInit(transfers.map(!_.valid)).asUInt)

  def isCacheOp(cmd: MemC_Cmd.Type): Bool =
    cmd === MemC_Cmd.MEMC_REPLACE ||
      cmd === MemC_Cmd.MEMC_CP_CACHE_TO_EXT ||
      cmd === MemC_Cmd.MEMC_CP_EXT_TO_CACHE

  val selReq = WireDefault(0.U.asTypeOf(new MemC_Req(fetchParams, AXI_WIDTH)))
  val selPort = WireDefault(NUM_TFS_IN.U) // which port won, NUM_TFS_IN = none
  val stallVec = WireDefault(((1 << NUM_TFS_IN) - 1).U(NUM_TFS_IN.W))

  when(enqIdxValid) {
    for (i <- 0 until NUM_TFS_IN) {
      when(selPort === NUM_TFS_IN.U &&
        io.IN_ctrl(i).cmd =/= MemC_Cmd.MEMC_NONE) {

        // Collision check — no in-flight cache-line op on same line
        val coll = transfers.zipWithIndex.map { case (t, _) =>
          t.valid &&
            isCacheOp(t.cmd) &&
            isCacheOp(io.IN_ctrl(i).cmd) &&
            t.cacheID === io.IN_ctrl(i).cacheID &&
            t.cacheAddr(fetchParams.CACHE_SIZE_E-3, fetchParams.CLSIZE_E-2) ===
              io.IN_ctrl(i).cacheAddr(fetchParams.CACHE_SIZE_E-3, fetchParams.CLSIZE_E-2)
        }.reduce(_ || _)

        when(!coll) { 
          selReq  := io.IN_ctrl(i)
          selPort := i.U
          stallVec := ~(1.U(NUM_TFS_IN.W) << i.U)
        }
      }
    }
  }

  io.OUT_stat.stall := stallVec
  io.OUT_stat.busy  := true.B

  // Fires when combinational selection found a valid request this cycle.
  // Writes selReq into the free slot at enqIdx.

  val isCacheLineCmd = selReq.cmd === MemC_Cmd.MEMC_REPLACE ||
    selReq.cmd === MemC_Cmd.MEMC_CP_CACHE_TO_EXT ||
    selReq.cmd === MemC_Cmd.MEMC_CP_EXT_TO_CACHE

  when(selReq.cmd =/= MemC_Cmd.MEMC_NONE) {

    transfers(enqIdx).valid          := true.B
    transfers(enqIdx).cmd            := selReq.cmd
    transfers(enqIdx).cacheID        := selReq.cacheID
    transfers(enqIdx).progress       := 0.U
    transfers(enqIdx).addrCounter    := 0.U
    transfers(enqIdx).fwdAddrCounter := 0.U
    // evictProgress starts at full line word count — overridden below
    // for commands that actually need eviction
    transfers(enqIdx).evictProgress  := (1 << (fetchParams.CLSIZE_E - 2)).U
    // Start optimistically — per-command block clears what is not done
    transfers(enqIdx).readDone       := true.B
    transfers(enqIdx).writeDone      := true.B
    // Pending flags default to clear — per-command block sets what is needed
    transfers(enqIdx).needReadRq     := false.B
    transfers(enqIdx).needWriteRq    := 0.U

    when(isCacheLineCmd) {
      // Cache-line ops must be aligned to AXI bus width
      transfers(enqIdx).writeAddr :=
        selReq.writeAddr & ~((AXI_WIDTH / 8 - 1).U(32.W))
      transfers(enqIdx).readAddr  :=
        selReq.readAddr  & ~((AXI_WIDTH / 8 - 1).U(32.W))
      transfers(enqIdx).cacheAddr :=
        selReq.cacheAddr & ~(((AXI_WIDTH / 8 - 1) >> 2).U(
          (fetchParams.CACHE_SIZE_E - 2).W))
    }.otherwise {
      // MMIO ops — use addresses as-is
      transfers(enqIdx).writeAddr := selReq.writeAddr
      transfers(enqIdx).readAddr  := selReq.readAddr
      transfers(enqIdx).cacheAddr := selReq.cacheAddr
    }

    when(isCacheLineCmd) {
      transfers(enqIdx).storeData := selReq.data
      transfers(enqIdx).storeMask := selReq.mask
    }.otherwise {
      // MMIO — zero-extend data, mask unused but not DontCare
      transfers(enqIdx).storeData := selReq.data
      transfers(enqIdx).storeMask := ~(0.U((AXI_WIDTH / 8).W))
    }

    switch(selReq.cmd) {

      is(MemC_Cmd.MEMC_REPLACE) {
        // Dirty eviction + fresh fill — both read and write needed
        transfers(enqIdx).needReadRq     := true.B
        transfers(enqIdx).needWriteRq    := 3.U   // bit0=dcache read, bit1=AXI AW
        transfers(enqIdx).evictProgress  := 0.U   // eviction has not started
        transfers(enqIdx).readDone       := false.B
        transfers(enqIdx).writeDone      := false.B
      }

      is(MemC_Cmd.MEMC_CP_EXT_TO_CACHE) {
        // Fill from DRAM only — no eviction
        transfers(enqIdx).needReadRq  := true.B
        transfers(enqIdx).needWriteRq := 0.U
        transfers(enqIdx).readDone    := false.B
      }

      is(MemC_Cmd.MEMC_CP_CACHE_TO_EXT) {
        // Evict dirty line only — no fill
        transfers(enqIdx).needReadRq    := false.B
        transfers(enqIdx).needWriteRq   := 3.U
        transfers(enqIdx).evictProgress := 0.U
        transfers(enqIdx).writeDone     := false.B
      }

      is(MemC_Cmd.MEMC_READ_BYTE, MemC_Cmd.MEMC_READ_HALF, MemC_Cmd.MEMC_READ_WORD) {
        // MMIO single-word read
        transfers(enqIdx).needReadRq  := true.B
        transfers(enqIdx).needWriteRq := 0.U
        transfers(enqIdx).readDone    := false.B
      }

      is(MemC_Cmd.MEMC_WRITE_BYTE, MemC_Cmd.MEMC_WRITE_HALF, MemC_Cmd.MEMC_WRITE_WORD) {
        // MMIO single-word write
        transfers(enqIdx).needReadRq  := false.B
        transfers(enqIdx).needWriteRq := 3.U
        transfers(enqIdx).writeDone   := false.B
      }
    }
  }

  val arFIFO = Module(new Queue(new AXI4_AR(axiParams), 4))

  val arIdxValid = transfers.map(t => t.valid && t.needReadRq).reduce(_ || _)
  val arIdx = PriorityEncoder(VecInit(transfers.map(t => t.valid && t.needReadRq)).asUInt)

    arFIFO.io.enq.valid := arIdxValid
    arFIFO.io.enq.bits.arid := arIdx
    arFIFO.io.enq.bits.araddr := transfers(arIdx).readAddr
    arFIFO.io.enq.bits.arlen := Mux(isMMIO(arIdx), 0.U,
      ((1 << (fetchParams.CLSIZE_E -
        log2Ceil(AXI_WIDTH / 8))) - 1).U)
    arFIFO.io.enq.bits.arsize := Mux(isMMIO(arIdx),
      MuxLookup(transfers(arIdx).cmd.asUInt, 2.U)(Seq(
        MemC_Cmd.MEMC_READ_BYTE.asUInt -> 0.U,
        MemC_Cmd.MEMC_READ_HALF.asUInt -> 1.U,
        MemC_Cmd.MEMC_READ_WORD.asUInt -> 2.U
      )),
      log2Ceil(AXI_WIDTH / 8).U)
    arFIFO.io.enq.bits.arburst := Mux(isMMIO(arIdx), 0.U, 2.U) // FIXED=0, WRAP=2
    arFIFO.io.enq.bits.arlock := 0.U
    arFIFO.io.enq.bits.arcache := 2.U

  when(arIdxValid === false.B) {
    arFIFO.io.enq.valid    := arIdxValid
    arFIFO.io.deq.ready    := io.axi.arready
    io.axi.ar.arvalid      := arFIFO.io.deq.valid
    io.axi.ar.arid         := arFIFO.io.deq.bits.arid
    io.axi.ar.araddr       := arFIFO.io.deq.bits.araddr
    io.axi.ar.arlen        := arFIFO.io.deq.bits.arlen
    io.axi.ar.arsize       := arFIFO.io.deq.bits.arsize
    io.axi.ar.arburst      := arFIFO.io.deq.bits.arburst
    io.axi.ar.arlock       := arFIFO.io.deq.bits.arlock
    io.axi.ar.arcache      := arFIFO.io.deq.bits.arcache
  }

  when(arFIFO.io.enq.fire) {
    transfers(arIdx).needReadRq := false.B
  }


}


