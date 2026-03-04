package cache

import chisel3._
import chisel3.util._
import fetch.FetchID

/*
First Improvement : Make it 4 way set assosciative using the pseudo-lru method - DONE
Second Improvement: fetchid tagging mispr flush , so basically am improved IcacheController of sorts. - DONE

Desc:

Think of it as gates , we need the fetchid from the iFetchPipeline (which is empty) and the io.mispr ,io.fetchid and
other metadata to streamed out from that module. This is the first gate.

FetchID Assignment
Once the pipeline IO exists, you need to decide where fetchID gets stamped onto a request.
It needs to happen at the point where a new fetch is initiated — when ifetchEn is high and pc is valid. That's inside IFetchPipeline, not in the ICache itself.
The ICache should receive an already-stamped fetchID alongside the address, and simply register it at the start of sALLOCATE.

Concrete steps are:

Define IFetchPipeline IO skeleton matching what iFetch.scala already wires up
Decide and implement fetchID stamping inside IFetchPipeline
Pass fetchID down to the ICache alongside the request address
Verify the flush signal path from BranchSelector through iFetch into IFetchPipeline is coherent
Only then write the sALLOCATE comparison — at that point it really is just a register, a comparison, and a conditional reset of cntWords
*/

class IDirectCache(
  val BSIZE    : Int = 4,
  val LINES    : Int = 1024,
  val MEM_SIZE : Int = 65536,
  val NUM_WAYS : Int = 4

) extends Module {

  val OFFSET_WIDTH : Int = log2Ceil(BSIZE)
  val LINE_WIDTH   : Int = log2Ceil(LINES)
  val TAG_WIDTH    : Int = log2Ceil(MEM_SIZE) - LINE_WIDTH - OFFSET_WIDTH

  val io = IO(new Bundle {
    val addr        = Input(UInt(32.W))
    val din         = Input(UInt(32.W))
    val write       = Input(Bool())
    val setValid    = Input(Bool())
    val setInvalid  = Input(Bool())
    val snoopAddr   = Input(UInt(32.W))
    val hit         = Output(Bool())
    val snoopHit    = Output(Bool())
    val dout        = Output(UInt(32.W))
    val victimWay = Output(UInt(log2Ceil(NUM_WAYS).W))
  })

  val offset    = io.addr(OFFSET_WIDTH - 1, 0)
  val lineIdx   = io.addr(LINE_WIDTH + OFFSET_WIDTH - 1, OFFSET_WIDTH)
  val tag       = io.addr(log2Ceil(MEM_SIZE) - 1, LINE_WIDTH + OFFSET_WIDTH)

  val snoopTag  = io.snoopAddr(log2Ceil(MEM_SIZE) - 1, LINE_WIDTH + OFFSET_WIDTH)
  val snoopLine = io.snoopAddr(LINE_WIDTH + OFFSET_WIDTH - 1, OFFSET_WIDTH)

  val setWidth = NUM_WAYS * LINES

  val tagMem  = Vec(NUM_WAYS, SyncReadMem(LINES, UInt(TAG_WIDTH.W)))
  val dataMem = Vec(NUM_WAYS, SyncReadMem(LINES, Vec(BSIZE, UInt(8.W))))
  val validMem = Vec(NUM_WAYS, RegInit(VecInit(Seq.fill(LINES)(false.B))))

  val tagRead  = VecInit(tagMem.map(_.read(lineIdx)))
  val dataRead = VecInit(dataMem.map(_.read(lineIdx)))
  val hitVec   = VecInit((0 until NUM_WAYS).map(w => tagRead(w) === tag && validMem(w)(lineIdx)))
  val snoopTagRead = VecInit(tagMem.map(_.read(snoopLine)))
  val snoopHitVec  = VecInit((0 until NUM_WAYS).map(w =>
    snoopTagRead(w) === snoopTag && validMem(w)(snoopLine)))

  val plruBits = RegInit(VecInit(Seq.fill(LINES)(0.U(3.W))))
  val plru = plruBits(lineIdx)

  val victimWay = Cat(plru(0), Mux(plru(0), plru(2), plru(1)))
  io.victimWay := victimWay

  io.hit      := hitVec.asUInt.orR
  io.snoopHit := snoopHitVec.asUInt.orR
  val hitWay = OHToUInt(hitVec)

  when(hitVec.asUInt.orR) {
    plruBits(lineIdx)(0) := ~hitWay(1)
    when(hitWay(1) === 0.U) {
      plruBits(lineIdx)(1) := ~hitWay(0)
    }.otherwise {
      plruBits(lineIdx)(2) := ~hitWay(0)
    }
  }

  val hitData    = dataRead(hitWay)
  val wordOffset = offset(OFFSET_WIDTH - 1, 2)
  io.dout := Cat(
    hitData(wordOffset * 4.U + 3.U),
    hitData(wordOffset * 4.U + 2.U),
    hitData(wordOffset * 4.U + 1.U),
    hitData(wordOffset * 4.U + 0.U)
  )

  when(io.write) {
    tagMem(victimWay).write(lineIdx, tag)
    val writeVec = Wire(Vec(BSIZE, UInt(8.W)))
    for (i <- 0 until BSIZE) writeVec(i) := dataRead(i)
    for (i <- 0 until 4) {
      writeVec(offset + i.U) := io.din(8 * i + 7, 8 * i)
    }
    dataMem(victimWay).write(lineIdx, writeVec)
  }

  when(io.setValid)   { validMem(lineIdx)   := true.B  }
  when(io.setInvalid) {
    (0 until NUM_WAYS).foreach(w => validMem(w)(snoopLine) := false.B)
  }
}

class ICacheController(
  val BSIZE    : Int = 4,
  val LINES    : Int = 1024,
  val MEM_SIZE : Int = 65536,
  val NUM_WAYS : Int = 4
) extends Module {

  val OFFSET_WIDTH  : Int = log2Ceil(BSIZE)
  val CNT_MAX_WORDS : Int = BSIZE / 4
  val CNT_WIDTH     : Int = log2Ceil(CNT_MAX_WORDS + 1)

  val io = IO(new Bundle {
    val core_addr  = Input(UInt(32.W))
    val core_fatal = Output(Bool())
    val core_valid = Output(Bool())
    val core_dout  = Output(UInt(32.W))

    val cache_addr       = Output(UInt(32.W))
    val cache_din        = Output(UInt(32.W))
    val cache_write      = Output(Bool())

    val cache_setValid   = Output(Bool())
    val cache_setInvalid = Output(Bool())
    val cache_snoopAddr  = Output(UInt(32.W))
    val cache_hit        = Input(Bool())
    val cache_snoopHit   = Input(Bool())
    val cache_dout       = Input(UInt(32.W))

    val core_fetchID      = Input(new FetchID)
    val flush             = Input(Bool())
    val flush_fetchID     = Input(new FetchID)

    val ram_addr      = Output(UInt(32.W))
    val ram_dout      = Output(UInt(32.W))
    val ram_valid     = Output(Bool())
    val ram_we        = Output(Bool())
    val ram_memWidth  = Output(UInt(2.W))
    val ram_burst     = Output(Bool())
    val ram_ready     = Input(Bool())
    val ram_din       = Input(UInt(32.W))

    val ram_snoop_addr       = Input(UInt(32.W))
    val ram_snoop_invalidate = Input(Bool())

    val cache_victimWay = Input(UInt(log2Ceil(NUM_WAYS).W))
    val cache_fillWay   = Output(UInt(log2Ceil(NUM_WAYS).W))
  })

  val sIDLE     = 0.U(2.W)
  val sREAD     = 1.U(2.W)
  val sALLOCATE = 2.U(2.W)


  val state    = RegInit(sIDLE)
  val cntWords = RegInit(0.U(CNT_WIDTH.W))
  val fillFetchID = RegInit(0.U.asTypeOf(new FetchID))

  val lineBase     = Cat(
    io.core_addr(log2Ceil(MEM_SIZE) - 1, OFFSET_WIDTH),
    0.U(OFFSET_WIDTH.W)
  )
  val allocateAddr = lineBase + Cat(cntWords, 0.U(2.W))

  io.cache_addr       := io.core_addr
  io.cache_din        := io.ram_din
  io.cache_write      := false.B
  io.cache_setValid   := false.B
  io.cache_snoopAddr  := io.ram_snoop_addr
  io.cache_setInvalid := io.cache_snoopHit && io.ram_snoop_invalidate

  io.ram_addr     := Cat(io.core_addr(log2Ceil(MEM_SIZE) - 1, 2), 0.U(2.W))
  io.ram_dout     := 0.U
  io.ram_valid    := false.B
  io.ram_we       := false.B
  io.ram_memWidth := 2.U
  io.ram_burst    := true.B

  io.core_fatal := io.core_addr(1, 0) =/= 0.U
  io.core_valid := false.B
  io.core_dout  := io.cache_dout

  switch(state) {

    is(sIDLE) {
      state := sREAD
    }

    val fillWay = RegInit(0.U(log2Ceil(NUM_WAYS).W))

    is(sREAD) {
      io.cache_addr := io.core_addr
      when(io.cache_hit) {
        io.core_valid := true.B
      }.otherwise {
        fillWay := io.cache_victimWay // lock in victim before transitioning
        fillFetchID := io.core_fetchID // lock in the fetchID before transitioning
        state := sALLOCATE //The controller just reacts to the hit signal, it doesn't need to know which way hit.
      }
    }

    is(sALLOCATE) {
      val fillIsStale = io.flush && (
        (fillFetchID.value - io.flush_fetchID.value).asSInt >= 0.S
        )

      when(fillIsStale) {
        cntWords          := 0.U
        io.cache_setValid := false.B
        state             := sREAD
      }.otherwise {
        io.ram_valid  := true.B
        io.ram_addr   := allocateAddr
        io.cache_addr := allocateAddr

        when(io.ram_ready) {
          io.cache_write := true.B
          io.cache_din   := io.ram_din

          val isLastWord = (cntWords === (CNT_MAX_WORDS - 1).U)
          io.cache_setValid := isLastWord

          when(isLastWord) {
            cntWords := 0.U
            state    := sREAD
          }.otherwise {
            cntWords := cntWords + 1.U
          }
        }
      }
    }
  }
}


class ICache(
  val BSIZE    : Int = 4,
  val LINES    : Int = 1024,
  val MEM_SIZE : Int = 65536
) extends Module {

  val io = IO(new Bundle {
    val core_addr  = Input(UInt(32.W))
    val core_fatal = Output(Bool())
    val core_valid = Output(Bool())
    val core_dout  = Output(UInt(32.W))

    val ram_addr     = Output(UInt(32.W))
    val ram_dout     = Output(UInt(32.W))
    val ram_valid    = Output(Bool())
    val ram_we       = Output(Bool())
    val ram_memWidth = Output(UInt(2.W))
    val ram_burst    = Output(Bool())
    val ram_ready    = Input(Bool())
    val ram_din      = Input(UInt(32.W))
    val icache_fetchID      = Input(new FetchID)
    val icache_flush        = Input(Bool())
    val icache_flushFetchID = Input(new FetchID)

    val ram_snoop_addr       = Input(UInt(32.W))
    val ram_snoop_invalidate = Input(Bool())
  })

  val controller = Module(new ICacheController(BSIZE, LINES, MEM_SIZE))
  val storage    = Module(new IDirectCache(BSIZE, LINES, MEM_SIZE))

  controller.io.core_addr  := io.core_addr
  io.core_fatal            := controller.io.core_fatal
  io.core_valid            := controller.io.core_valid
  io.core_dout             := controller.io.core_dout

  storage.io.addr       := controller.io.cache_addr
  storage.io.din        := controller.io.cache_din
  storage.io.write      := controller.io.cache_write
  storage.io.setValid   := controller.io.cache_setValid
  storage.io.setInvalid := controller.io.cache_setInvalid
  storage.io.snoopAddr  := controller.io.cache_snoopAddr

  controller.io.cache_hit      := storage.io.hit
  controller.io.cache_snoopHit := storage.io.snoopHit
  controller.io.cache_dout     := storage.io.dout

  controller.io.core_fetchID    := io.icache_fetchID
  controller.io.flush           := io.icache_flush
  controller.io.flush_fetchID   := io.icache_flushFetchID

  io.ram_addr     := controller.io.ram_addr
  io.ram_dout     := controller.io.ram_dout
  io.ram_valid    := controller.io.ram_valid
  io.ram_we       := controller.io.ram_we
  io.ram_memWidth := controller.io.ram_memWidth
  io.ram_burst    := controller.io.ram_burst

  controller.io.ram_ready := io.ram_ready
  controller.io.ram_din   := io.ram_din

  controller.io.ram_snoop_addr       := io.ram_snoop_addr
  controller.io.ram_snoop_invalidate := io.ram_snoop_invalidate
}
