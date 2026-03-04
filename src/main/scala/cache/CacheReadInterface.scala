package cache

import chisel3._
import chisel3.util._

class CacheReadInterface(
                          ADDR_BITS: Int = 10,
                          LEN_BITS: Int = 8,
                          IWIDTH: Int = 128,
                          CWIDTH: Int = 32,
                          BUF_LEN: Int = 32,
                          ID_LEN: Int = 2,
                          CLSIZE_E: Int = 6
                        ) extends Module {
  val io = IO(new Bundle {
    val ready = Output(Bool())
    val valid = Input(Bool())
    val id = Input(UInt(ID_LEN.W))
    val len = Input(UInt(LEN_BITS.W))
    val addr = Input(UInt(ADDR_BITS.W))
    val mmio = Input(Bool())
    val mmioData = Input(UInt(32.W))

    val outReady = Input(Bool())
    val outValid = Output(Bool())
    val outId = Output(UInt(ID_LEN.W))
    val outData = Output(UInt(IWIDTH.W))
    val outLast = Output(Bool())

    val cache = new Bundle {
      val ready = Input(Bool())
      val ce = Output(Bool())
      val we = Output(Bool())
      val addr = Output(UInt(ADDR_BITS.W))
      val data = Input(UInt(CWIDTH.W))
    }

    val cacheReadValid = Output(Bool())
    val cacheReadId = Output(UInt(ID_LEN.W))
  })

  val WNUM = IWIDTH / CWIDTH
  val CWIDTH_W_ = CWIDTH / 32
  val CWIDTH_W = CWIDTH_W_.U(LEN_BITS.W)

  class Transfer extends Bundle {
    val mmioData = UInt(32.W)
    val mmio = Bool()
    val progress = UInt(LEN_BITS.W)
    val len = UInt(LEN_BITS.W)
    val addr = UInt(ADDR_BITS.W)
    val id = UInt(ID_LEN.W)
    val valid = Bool()
  }

  class ReadMeta extends Bundle {
    val mmioData = UInt(32.W)
    val id = UInt(ID_LEN.W)
    val mmio = Bool()
    val last = Bool()
    val valid = Bool()
  }

  val FIFO_free = Wire(UInt((log2Ceil(BUF_LEN) + 1).W))
  val FIFO_valid = Wire(Bool())
  val FIFO_data = Wire(UInt(IWIDTH.W))
  val FIFO_id = Wire(UInt(ID_LEN.W))
  val FIFO_last = Wire(Bool())
  val FIFO_ready = Wire(Bool())

  val fifo = Module(new FIFO(IWIDTH + ID_LEN + 1, BUF_LEN))
  fifo.io.free := FIFO_free
  fifo.io.inValid := FIFO_valid
  fifo.io.inData := Cat(FIFO_last, FIFO_id, FIFO_data)
  FIFO_ready := fifo.io.outReady
  io.outValid := fifo.io.outValid
  fifo.io.inReady := io.outReady
  io.outLast := fifo.io.outData(IWIDTH + ID_LEN)
  io.outId := fifo.io.outData(IWIDTH + ID_LEN - 1, IWIDTH)
  io.outData := fifo.io.outData(IWIDTH - 1, 0)

  val cur = RegInit(0.U.asTypeOf(new Transfer))
  val next = RegInit(0.U.asTypeOf(new Transfer))
  cur.valid := false.B

  val acc = Reg(UInt(IWIDTH.W))
  val accIdx_r = RegInit(0.U((log2Ceil(WNUM) + 1).W))
  val accIdx_c = Wire(UInt((log2Ceil(WNUM) + 1).W))
  val doAcc = Wire(Bool())

  val readMetaSR = RegInit(VecInit(Seq.fill(2)(0.U.asTypeOf(new ReadMeta))))

  accIdx_c := accIdx_r
  FIFO_valid := false.B
  FIFO_data := DontCare
  FIFO_id := DontCare
  FIFO_last := DontCare
  doAcc := false.B

  when(readMetaSR(1).valid) {
    accIdx_c := accIdx_c + 1.U

    when(readMetaSR(1).mmio) {
      FIFO_valid := true.B
      FIFO_data := 0.U
      FIFO_data := readMetaSR(1).mmioData
      FIFO_id := readMetaSR(1).id
      FIFO_last := readMetaSR(1).last
      accIdx_c := 0.U
    }.elsewhen(accIdx_c(log2Ceil(WNUM))) {
      FIFO_valid := true.B
      FIFO_data := acc
      FIFO_data := Cat(io.cache.data, acc((WNUM - 1) * CWIDTH - 1, 0))
      FIFO_id := readMetaSR(1).id
      FIFO_last := readMetaSR(1).last
      accIdx_c := 0.U
    }.otherwise {
      doAcc := true.B
    }
  }

  accIdx_r := accIdx_c
  when(doAcc) {
    acc := Cat(acc(IWIDTH - CWIDTH - 1, 0), io.cache.data)
  }

  val inFlight = readMetaSR(1).valid.asUInt + readMetaSR(0).valid.asUInt
  val allowNewRead = (FIFO_free * WNUM.U) > inFlight

  val readMeta = Wire(new ReadMeta)
  io.cache.ce := true.B
  io.cache.we := true.B
  io.cache.addr := DontCare
  readMeta := 0.U.asTypeOf(new ReadMeta)

  when(cur.valid && allowNewRead) {
    io.cache.ce := false.B
    io.cache.we := true.B
    io.cache.addr := Cat(
      cur.addr(ADDR_BITS - 1, CLSIZE_E - 2),
      cur.addr(CLSIZE_E - 3, 0) + cur.progress(CLSIZE_E - 3, 0)
    )

    readMeta.valid := true.B
    readMeta.id := cur.id
    readMeta.last := (cur.progress(LEN_BITS - 1, log2Ceil(CWIDTH_W_)) ===
      cur.len(LEN_BITS - 1, log2Ceil(CWIDTH_W_))) || cur.mmio
    readMeta.mmio := cur.mmio
    readMeta.mmioData := cur.mmioData
  }

  val readSucc = readMeta.valid && (io.cache.ready || readMeta.mmio)
  io.ready := !next.valid || (readSucc && readMeta.last)

  io.cacheReadValid := false.B
  io.cacheReadId := DontCare
  when(readSucc) {
    io.cacheReadValid := true.B
    io.cacheReadId := readMeta.id
  }

  val incoming = Wire(new Transfer)
  incoming := 0.U.asTypeOf(new Transfer)

  when(io.valid && io.ready) {
    incoming.valid := true.B
    incoming.id := io.id
    incoming.addr := io.addr
    incoming.progress := 0.U
    incoming.len := io.len
    incoming.mmio := io.mmio
    incoming.mmioData := io.mmioData
  }

  readMetaSR := VecInit(readMetaSR(0), Mux(readSucc, readMeta, 0.U.asTypeOf(new ReadMeta)))

  when(readSucc) {
    when(readMeta.last) {
      when(next.valid) {
        cur := next
        next.valid := false.B
      }.otherwise {
        cur := incoming
        incoming.valid := false.B
      }
    }.otherwise {
      cur.progress := cur.progress + CWIDTH_W
    }
  }

  when(incoming.valid) {
    when(!cur.valid) {
      cur := incoming
    }.otherwise {
      next := incoming
    }
  }
}

class FIFO(DATA_WIDTH: Int, DEPTH: Int) extends Module {
  val io = IO(new Bundle {
    val free = Output(UInt((log2Ceil(DEPTH) + 1).W))
    val inValid = Input(Bool())
    val inData = Input(UInt(DATA_WIDTH.W))
    val outReady = Output(Bool())
    val outValid = Output(Bool())
    val inReady = Input(Bool())
    val outData = Output(UInt(DATA_WIDTH.W))
  })

  val mem = Reg(Vec(DEPTH, UInt(DATA_WIDTH.W)))
  val wrPtr = RegInit(0.U((log2Ceil(DEPTH) + 1).W))
  val rdPtr = RegInit(0.U((log2Ceil(DEPTH) + 1).W))

  val empty = wrPtr === rdPtr
  val full = (wrPtr(log2Ceil(DEPTH) - 1, 0) === rdPtr(log2Ceil(DEPTH) - 1, 0)) &&
    (wrPtr(log2Ceil(DEPTH)) =/= rdPtr(log2Ceil(DEPTH)))

  io.free := Mux(full, 0.U, DEPTH.U - (wrPtr - rdPtr))
  io.outValid := !empty
  io.outReady := !full
  io.outData := mem(rdPtr(log2Ceil(DEPTH) - 1, 0))

  when(io.inValid && !full) {
    mem(wrPtr(log2Ceil(DEPTH) - 1, 0)) := io.inData
    wrPtr := wrPtr + 1.U
  }

  when(io.inReady && !empty) {
    rdPtr := rdPtr + 1.U
  }
} 