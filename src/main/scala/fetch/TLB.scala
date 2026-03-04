package fetch

import chisel3._
import chisel3.util._

class TLB_Req extends Bundle {
  val valid = Bool()
  val vpn   = UInt(20.W)
}

class TLB_Res extends Bundle {
  val hit         = Bool()
  val ppn         = UInt(20.W)
  val rwx         = UInt(3.W)
  val user        = Bool()
  val isSuper     = Bool()
  val pageFault   = Bool()
  val accessFault = Bool()
}

class PageWalk_Res_Full extends Bundle {
  val valid       = Bool()
  val busy        = Bool()
  val rqID        = UInt(2.W)
  val vpn         = UInt(20.W)
  val ppn         = UInt(22.W)
  val rwx         = UInt(3.W)
  val user        = Bool()
  val globl       = Bool()
  val isSuperPage = Bool()
  val pageFault   = Bool()
}

class TLBEntry(VIRT_LEN: Int) extends Bundle {
  val vpn         = UInt(VIRT_LEN.W)
  val ppn         = UInt(20.W)
  val isSuper     = Bool()
  val user        = Bool()
  val globl       = Bool()
  val rwx         = UInt(3.W)
  val accessFault = Bool()
  val pageFault   = Bool()
  val valid       = Bool()
}

class TLB(
  NUM_RQ    : Int     = 1,
  SIZE      : Int     = 8,
  ASSOC     : Int     = 4,
  IS_IFETCH : Boolean = false
) extends Module {

  val LEN      = SIZE / ASSOC
  val IDX_W    = log2Ceil(LEN)
  val ASSOC_W  = log2Ceil(ASSOC)
  val VIRT_LEN = 20 - IDX_W

  val io = IO(new Bundle {
    val clear = Input(Bool())
    val pw    = Input(new PageWalk_Res_Full)
    val rqs   = Input(Vec(NUM_RQ, new TLB_Req))
    val res   = Output(Vec(NUM_RQ, new TLB_Res))
  })

  val tlb      = RegInit(VecInit(Seq.fill(LEN)(
                   VecInit(Seq.fill(ASSOC)(0.U.asTypeOf(new TLBEntry(VIRT_LEN)))))))
  val counters = RegInit(VecInit(Seq.fill(LEN)(0.U(ASSOC_W.W))))
  val ignoreCur = RegInit(false.B)

  // ---- Combinational lookup ----------------------------------
  // inc tracks which set had a counter-way hit, so we can
  // advance the round-robin counter for that set.
  val inc = Wire(Vec(LEN, Bool()))
  for (i <- 0 until LEN) inc(i) := false.B

  for (i <- 0 until NUM_RQ) {
    val idx     = io.rqs(i).vpn(IDX_W - 1, 0)
    val resWire = Wire(new TLB_Res)
    resWire := 0.U.asTypeOf(new TLB_Res)

    when(io.rqs(i).valid) {
      for (j <- 0 until ASSOC) {
        val entry = tlb(idx)(j.U)

        val superMatch = entry.isSuper &&
          entry.vpn(VIRT_LEN - 1, 10 - IDX_W) === io.rqs(i).vpn(19, 10)
        val pageMatch  = !entry.isSuper &&
          entry.vpn === io.rqs(i).vpn(19, IDX_W)

        when(entry.valid && (superMatch || pageMatch)) {
          resWire.accessFault := entry.accessFault
          resWire.pageFault   := entry.pageFault
          resWire.rwx         := entry.rwx
          resWire.user        := entry.user
          resWire.isSuper     := entry.isSuper
          resWire.hit         := true.B
          resWire.ppn         := Mux(
            entry.isSuper,
            Cat(entry.ppn(19, 10), io.rqs(i).vpn(9, 0)),
            entry.ppn
          )
          when(counters(idx) === j.U) {
            inc(idx) := true.B
          }
        }
      }
    }
    io.res(i) := resWire
  }

  // ---- Sequential: reset / clear / page-walk fill / counter -
  when(reset.asBool) {
    for (i <- 0 until LEN) {
      counters(i) := 0.U
      for (j <- 0 until ASSOC) {
        tlb(i)(j) := 0.U.asTypeOf(new TLBEntry(VIRT_LEN))
      }
    }
    ignoreCur := false.B

  }.elsewhen(io.clear) {
    for (i <- 0 until LEN)
      for (j <- 0 until ASSOC)
        tlb(i)(j).valid := false.B
    ignoreCur := true.B

  }.otherwise {
    // Once the page walker finishes its current walk, stop ignoring
    when(ignoreCur && !io.pw.busy) {
      ignoreCur := false.B
    }

    // Install a new TLB entry when the page walker returns a result.
    // IS_IFETCH gates on rqID: ifetch uses rqID == 0, LSU uses rqID != 0.
    val pwTargetsUs = if (IS_IFETCH) io.pw.rqID === 0.U
                      else           io.pw.rqID =/= 0.U

    when(io.pw.valid && !ignoreCur && pwTargetsUs) {
      val idx      = io.pw.vpn(IDX_W - 1, 0)
      val assocIdx = counters(idx)

      tlb(idx)(assocIdx).rwx         := io.pw.rwx
      tlb(idx)(assocIdx).isSuper     := io.pw.isSuperPage
      tlb(idx)(assocIdx).ppn         := io.pw.ppn(19, 0)
      tlb(idx)(assocIdx).vpn         := io.pw.vpn(19, IDX_W)
      tlb(idx)(assocIdx).globl       := io.pw.globl
      tlb(idx)(assocIdx).user        := io.pw.user
      tlb(idx)(assocIdx).pageFault   := io.pw.pageFault
      // accessFault: upper two bits of ppn being non-zero signals a fault
      tlb(idx)(assocIdx).accessFault := io.pw.ppn(21, 20) =/= 0.U
      tlb(idx)(assocIdx).valid       := true.B
    }

    // Advance round-robin counter for any set that had a counter-way hit
    for (i <- 0 until LEN) {
      when(inc(i)) {
        counters(i) := counters(i) + 1.U
      }
    }
  }
}
