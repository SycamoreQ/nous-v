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


BASIC MENTAL MODEL : DCache SRAM → dcacheReadIF → AXI W channel → DRAM (slave)

dcacheReadIF is the one doing the reading from DCache SRAM. It produces beat-by-beat data on its output port.
forwarding that output directly onto the AXI W channel which carries it to DRAM.

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
    val icacheW = Output(new CacheSRAMWriteIF(fetchParams)) //stub
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
  val arIdx      = PriorityEncoder(VecInit(transfers.map(t => t.valid && t.needReadRq)).asUInt)

  // Enqueue side — drive from transfer table when a slot needs a read request
  arFIFO.io.enq.valid            := arIdxValid
  arFIFO.io.enq.bits.arid        := arIdx
  arFIFO.io.enq.bits.araddr      := transfers(arIdx).readAddr
  arFIFO.io.enq.bits.arlen       := Mux(isMMIO(arIdx), 0.U,
    ((1 << (fetchParams.CLSIZE_E -
      log2Ceil(AXI_WIDTH / 8))) - 1).U)
  arFIFO.io.enq.bits.arsize      := Mux(isMMIO(arIdx),
    MuxLookup(transfers(arIdx).cmd.asUInt, 2.U)(Seq(
      MemC_Cmd.MEMC_READ_BYTE.asUInt -> 0.U,
      MemC_Cmd.MEMC_READ_HALF.asUInt -> 1.U,
      MemC_Cmd.MEMC_READ_WORD.asUInt -> 2.U
    )),
    log2Ceil(AXI_WIDTH / 8).U)
  arFIFO.io.enq.bits.arburst     := Mux(isMMIO(arIdx), 0.U, 2.U) // FIXED=0, WRAP=2
  arFIFO.io.enq.bits.arlock      := 0.U
  arFIFO.io.enq.bits.arcache     := 2.U

  // Dequeue side — always-active wires to AXI AR channel
  arFIFO.io.deq.ready            := io.axi.arready
  io.axi.ar.arvalid              := arFIFO.io.deq.valid
  io.axi.ar.arid                 := arFIFO.io.deq.bits.arid
  io.axi.ar.araddr               := arFIFO.io.deq.bits.araddr
  io.axi.ar.arlen                := arFIFO.io.deq.bits.arlen
  io.axi.ar.arsize               := arFIFO.io.deq.bits.arsize
  io.axi.ar.arburst              := arFIFO.io.deq.bits.arburst
  io.axi.ar.arlock               := arFIFO.io.deq.bits.arlock
  io.axi.ar.arcache              := arFIFO.io.deq.bits.arcache

  // Sequential: clear needReadRq when enqueue is accepted by the FIFO
  when(arFIFO.io.enq.fire) {
    transfers(arIdx).needReadRq := false.B
  }


  val rFIFO = Module(new Queue(new RData(axiParams), 32))

  // AXI R channel (slave - master) feeds into rFIFO
  // Slave drives rvalid and data fields, master drives rready
  rFIFO.io.enq.valid          := io.axi.r.rvalid
  rFIFO.io.enq.bits.rid       := io.axi.r.rid
  rFIFO.io.enq.bits.rdata     := io.axi.r.rdata
  rFIFO.io.enq.bits.rlast     := io.axi.r.rlast
  io.axi.rready               := rFIFO.io.enq.ready

  // Registered MMIO load result, defaulted to zero every cycle,
  // set when an MMIO read response is consumed from rFIFO
  val sglLdRes_r = RegInit(0.U.asTypeOf(new MemController_SglLdRes(fetchParams)))
  sglLdRes_r.valid := false.B   // default every cycle — overridden below when fired

  // Consumer side — route dequeued beats to the correct destination
  // buf_ prefix matches the reference naming for the rFIFO output
  val buf_rid   = rFIFO.io.deq.bits.rid
  val buf_rdata = rFIFO.io.deq.bits.rdata
  val buf_rlast = rFIFO.io.deq.bits.rlast

  // Default deq.ready to false — overridden in each routing case below
  rFIFO.io.deq.ready := false.B

  // Instantiate cache write interfaces
  // icacheWriteIF: IWIDTH = AXI_WIDTH, CWIDTH = 8 << FSIZE_E (ICache line width)
  val icacheWriteIF = Module(new CacheWriteInterface(
    ADDR_BITS = fetchParams.CACHE_SIZE_E - 2,
    IWIDTH    = AXI_WIDTH,
    CWIDTH    = 8 << fetchParams.FSIZE_E,
    ID_LEN    = log2Ceil(NUM_TFS)
  ))

  // dcacheWriteIF: stubbed — CWIDTH matches DCache line width placeholder
  val dcacheWriteIF = Module(new CacheWriteInterface(
    ADDR_BITS = fetchParams.CACHE_SIZE_E - 2,
    IWIDTH    = AXI_WIDTH,
    CWIDTH    = AXI_WIDTH,   // placeholder until DCache is defined
    ID_LEN    = log2Ceil(NUM_TFS)
  ))

  // Default write interface inputs to zero — overridden when routing below
  icacheWriteIF.io.IN_valid := false.B
  icacheWriteIF.io.IN_addr  := 0.U
  icacheWriteIF.io.IN_data  := 0.U
  icacheWriteIF.io.IN_id    := 0.U
  icacheWriteIF.io.IN_CACHE_ready := true.B

  dcacheWriteIF.io.IN_valid := false.B
  dcacheWriteIF.io.IN_addr  := 0.U
  dcacheWriteIF.io.IN_data  := 0.U
  dcacheWriteIF.io.IN_id    := 0.U
  dcacheWriteIF.io.IN_CACHE_ready := true.B

  // Wire SRAM-facing outputs to MemController IO cache ports
  io.icacheW.ce   := icacheWriteIF.io.OUT_CACHE_ce
  io.icacheW.we   := icacheWriteIF.io.OUT_CACHE_we
  io.icacheW.wm   := icacheWriteIF.io.OUT_CACHE_wm
  io.icacheW.addr := icacheWriteIF.io.OUT_CACHE_addr
  io.icacheW.data := icacheWriteIF.io.OUT_CACHE_data

  io.dcacheW.ce   := dcacheWriteIF.io.OUT_CACHE_ce
  io.dcacheW.we   := dcacheWriteIF.io.OUT_CACHE_we
  io.dcacheW.wm   := dcacheWriteIF.io.OUT_CACHE_wm
  io.dcacheW.addr := dcacheWriteIF.io.OUT_CACHE_addr
  io.dcacheW.data := dcacheWriteIF.io.OUT_CACHE_data

  // Routing logic — runs when rFIFO has a beat waiting
  when(rFIFO.io.deq.valid) {
    when(isMMIO(buf_rid)) {
      // MMIO response — consume immediately, bypass write interfaces
      rFIFO.io.deq.ready := true.B
      sglLdRes_r.valid   := true.B
      sglLdRes_r.id      := transfers(buf_rid).cacheAddr
      sglLdRes_r.data    := buf_rdata(31, 0)

      // Invalidate the slot. MMIO transfers are single-beat, always done
      transfers(buf_rid).valid := false.B

    }.otherwise {
      // Cache-line response — route to correct write interface
      when(transfers(buf_rid).cacheID === 1.U) {
        // ICache
        icacheWriteIF.io.valid := true.B
        icacheWriteIF.io.addr  := transfers(buf_rid).cacheAddr +
          transfers(buf_rid).addrCounter
        icacheWriteIF.io.data  := buf_rdata
        icacheWriteIF.io.id    := buf_rid
        // Only dequeue when write interface is ready to accept
        rFIFO.io.deq.ready        := icacheWriteIF.io.OUT_ready

      }.otherwise {
        // DCache
        dcacheWriteIF.io.valid := true.B
        dcacheWriteIF.io.addr  := transfers(buf_rid).cacheAddr +
          transfers(buf_rid).addrCounter
        dcacheWriteIF.io.data  := buf_rdata
        dcacheWriteIF.io.id    := buf_rid
        rFIFO.io.deq.ready        := dcacheWriteIF.io.OUT_ready
      }

      // Sequential: increment addrCounter per beat consumed
      when(rFIFO.io.deq.fire) {
        transfers(buf_rid).addrCounter :=
          transfers(buf_rid).addrCounter + (AXI_WIDTH / 32).U
      }
    }
  }

  // Sequential: handle write interface acks — increment progress, set readDone
  val WIDTH_W = AXI_WIDTH / 32
  when(icacheWriteIF.io.OUT_ackValid) {
    val id = icacheWriteIF.io.OUT_ackId
    transfers(id).progress := transfers(id).progress + WIDTH_W.U
    when(transfers(id).progress >= ((1 << (fetchParams.CLSIZE_E - 2)) - 1).U) {
      transfers(id).readDone := true.B
    }
  }
  when(dcacheWriteIF.io.OUT_ackValid) {
    val id = dcacheWriteIF.io.OUT_ackId
    transfers(id).progress := transfers(id).progress + WIDTH_W.U
    when(transfers(id).progress >= ((1 << (fetchParams.CLSIZE_E - 2)) - 1).U) {
      transfers(id).readDone := true.B
    }
  }

  // Drive OUT_stat MMIO load result from register
  io.OUT_stat.sglLdRes := sglLdRes_r

  //The host chip (master) is writing dirty cache data out to DRAM (slave).
  // awFIFO sends the write address to DRAM telling it where to write
  val awFIFO = Module(new Queue(new AXI4_AW(axiParams) , 8))
  val awIdxValid  = WireDefault(false.B)
  val awIdx       = WireDefault(0.U(log2Ceil(NUM_TFS).W))
  val isExclusive = WireDefault(false.B)

  for (i <- 0 until NUM_TFS){
    val partialIssue = transfers(i).needWriteRq =/= 0.U &&
      transfers(i).needWriteRq =/= 3.U

    val fullPending = transfers(i).needWriteRq =/= 3.U

    when(transfers(i).valid){
      when(partialIssue && !isExclusive) {
        awIdx := i.U
        awIdxValid := true.B
        isExclusive := true.B
      }.elsewhen(fullPending && !isExclusive && !awIdxValid){
        // full-pending slot only wins if no partial-issue slot found yet
        awIdx      := i.U
        awIdxValid := true.B
      }
    }

    when(awIdxValid) {
      when (transfers(awIdx).needWriteRq(1)){
        awFIFO.io.enq.valid            := awIdxValid
        awFIFO.io.enq.bits.awid        := awIdx
        awFIFO.io.enq.bits.awaddr      := transfers(awIdx).writeAddr
        awFIFO.io.enq.bits.awlen       := Mux(isMMIO(awIdx), 0.U,
          ((1 << (fetchParams.CLSIZE_E -
            log2Ceil(AXI_WIDTH / 8))) - 1).U)
        awFIFO.io.enq.bits.awsize      := Mux(isMMIO(awIdx),
          MuxLookup(transfers(awIdx).cmd.asUInt, 2.U)(Seq(
            MemC_Cmd.MEMC_WRITE_BYTE.asUInt -> 0.U,
            MemC_Cmd.MEMC_WRITE_HALF.asUInt -> 1.U,
            MemC_Cmd.MEMC_WRITE_WORD.asUInt -> 2.U
          )),
          log2Ceil(AXI_WIDTH / 8).U)
        awFIFO.io.enq.bits.awburst     := Mux(isMMIO(awIdx), 0.U, 2.U) // FIXED=0, WRAP=2
        awFIFO.io.enq.bits.awlock      := 0.U
        awFIFO.io.enq.bits.awcache     := 2.U
      }

      when(transfers(awIdx).needWriteRq(0)){
        dcacheReadIF.io.IN_valid    := true.B
        dcacheReadIF.io.IN_id       := awIdx
        dcacheReadIF.io.IN_addr     := transfers(awIdx).cacheAddr
        dcacheReadIF.io.IN_len      := Mux(isMMIO(awIdx), 0.U,
          ((1 << (fetchParams.CLSIZE_E - 2)) - 1).U)
        dcacheReadIF.io.IN_mmio     := isMMIO(awIdx)
        dcacheReadIF.io.IN_mmioData := transfers(awIdx).storeData(31, 0)
      }

      awFIFO.io.deq.ready        := io.axi.awready
      io.axi.aw.awvalid          := awFIFO.io.deq.valid
      io.axi.aw.awid             := awFIFO.io.deq.bits.awid
      io.axi.aw.awaddr           := awFIFO.io.deq.bits.awaddr
      io.axi.aw.awlen            := awFIFO.io.deq.bits.awlen
      io.axi.aw.awsize           := awFIFO.io.deq.bits.awsize
      io.axi.aw.awburst          := awFIFO.io.deq.bits.awburst
      io.axi.aw.awlock           := awFIFO.io.deq.bits.awlock
      io.axi.aw.awcache          := awFIFO.io.deq.bits.awcache

      when(awFIFO.io.enq.fire) {
        transfers(awIdx).needWriteRq := transfers(awIdx).needWriteRq & ~(2.U(2.W))
      }
      when(dcacheReadIF.io.IN_valid && dcacheReadIF.io.OUT_ready) {
        transfers(awIdx).needWriteRq := transfers(awIdx).needWriteRq & ~(1.U(2.W))
      }
    }
  }

  io.axi.w.wvalid          := dcacheReadIF.io.outValid
  io.axi.w.wdata           := dcacheReadIF.io.outData
  io.axi.w.wlast           := dcacheReadIF.io.outLast
  io.axi.w.wstrb           := ~(0.U((AXI_WIDTH / 8).W))  // all bytes valid
  dcacheReadIF.io.outReady := io.axi.wready

  io.axi.bready := true.B
  val sglStRes_r = RegInit(0.U.asTypeOf(new MemController_SglStRes(fetchParams)))
  sglStRes_r.valid := false.B

  when(io.axi.b.bvalid) {
    transfers(io.axi.b.bid).writeDone := true.B

    when(isMMIO(io.axi.b.bid)) {
      sglStRes_r.valid := true.B
      sglStRes_r.id    := transfers(io.axi.b.bid).cacheAddr
    }

    // Invalidate slot if read side is also complete
    when(transfers(io.axi.b.bid).readDone) {
      transfers(io.axi.b.bid).valid := false.B
    }
  }

  io.OUT_stat.sglStRes := sglStRes_r

  // GC — sequential sweep every cycle
  // Frees any slot where both read and write sides have completed.
  // Runs in addition to the per-event invalidations in Steps 5 and 7
  // to catch cases where both flags were set in the same cycle.
  for (i <- 0 until NUM_TFS) {
    when(transfers(i).valid && transfers(i).readDone && transfers(i).writeDone) {
      transfers(i).valid := false.B
    }
  }

  // OUT_stat.transfers — combinational, one entry per slot
  for (i <- 0 until NUM_TFS) {
    // MMIO slots are internal bookkeeping — clients do not track them
    when(transfers(i).valid && !isMMIO(i)) {
      io.OUT_stat.transfers(i).valid     := true.B
      io.OUT_stat.transfers(i).cacheID   := transfers(i).cacheID
      io.OUT_stat.transfers(i).progress  := transfers(i).progress
      io.OUT_stat.transfers(i).cacheAddr := transfers(i).cacheAddr
      io.OUT_stat.transfers(i).readAddr  := transfers(i).readAddr
      io.OUT_stat.transfers(i).writeAddr := transfers(i).writeAddr
      // active = both requests have been issued and eviction is in progress
      io.OUT_stat.transfers(i).active    :=
        !transfers(i).needReadRq        &&
          transfers(i).needWriteRq === 0.U &&
          transfers(i).evictProgress =/= 0.U
    }.otherwise {
      io.OUT_stat.transfers(i) := 0.U.asTypeOf(new TransferStatus(fetchParams))
    }
  }

  // Stubbed — DCache load forward path not yet implemented
  io.OUT_stat.ldDataFwd := 0.U.asTypeOf(new MemController_LdDataFwd(AXI_WIDTH))

}


