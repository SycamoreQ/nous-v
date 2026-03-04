package fetch

import chisel3._
import chisel3.util._
import branch_pred.SqN

//  AGU_UOp — the micro-op that the TLB miss queue holds.
//  Fields mirror the SV AGU_UOp struct; extend as needed when
//  you wire up your execute stage.

class AGU_UOp extends Bundle {
  val valid  = Bool()
  val addr   = UInt(32.W)
  val sqN    = new SqN
}

//  TLBMissQueue
//
//  A small fixed-size queue that holds load/store micro-ops
//  that missed the TLB and are waiting for the page walker.
//
//  Each entry has a ready bit that is set when the page walker
//  returns a result covering that entry's address.  The output
//  port always presents the oldest ready entry; if no entry is
//  ready and the page walker is idle, it presents an unready
//  entry so a new walk can be started.
//
//  Parameter
//    SIZE : number of queue slots (must be power of 2, max 4
//           for the current free-count adder tree)

class TLBMissQueue(SIZE: Int = 4) extends Module {

  val ID_LEN = log2Ceil(SIZE)

  val io = IO(new Bundle {
    val free      = Output(UInt((ID_LEN + 1).W))
    val outReady  = Output(Bool())

    // Flush on branch misprediction
    val branch    = Input(new BranchProv)

    val vmem      = Input(new VirtMemState)
    val pw        = Input(new PageWalk_Res_Full)
    val pwActive  = Input(Bool())

    // Enqueue side
    val enqueue   = Input(Bool())
    val uopReady  = Input(Bool())       // TLB already hit at enqueue time
    val uopIn     = Input(new AGU_UOp)

    // Dequeue side
    val dequeue   = Input(Bool())
    val uopOut    = Output(new AGU_UOp)
  })

  val queue = RegInit(VecInit(Seq.fill(SIZE)(0.U.asTypeOf(new AGU_UOp))))
  val ready = RegInit(VecInit(Seq.fill(SIZE)(false.B)))

  // ---- Find a free slot (priority encoder over invalid entries)
  val freeSlotValid = Wire(Bool())
  val freeSlot      = Wire(UInt(ID_LEN.W))
  freeSlotValid := false.B
  freeSlot      := 0.U
  for (i <- 0 until SIZE) {
    when(!queue(i).valid) {
      freeSlot      := i.U
      freeSlotValid := true.B
    }
  }
  io.outReady := freeSlotValid

  // ---- Count free slots (adder tree, works for SIZE <= 4) ----
  val stage0_0 = !queue(0).valid +& !queue(1).valid
  val stage0_1 = !queue(2).valid +& !queue(3).valid
  val freeCount = stage0_0 +& stage0_1
  // Subtract one if the current output is valid (it occupies a slot)
  io.free := Mux(io.uopOut.valid, freeCount - 1.U, freeCount)

  // ---- Find the oldest entry to dequeue ----------------------
  // When the page walker is idle (!pwActive) we also allow
  // non-ready entries so they can start a new walk.
  val deqSlotValid = Wire(Bool())
  val deqSlot      = Wire(UInt(ID_LEN.W))
  deqSlotValid := false.B
  deqSlot      := 0.U
  for (i <- 0 until SIZE) {
    when(queue(i).valid && (ready(i) || !io.pwActive)) {
      deqSlot      := i.U
      deqSlotValid := true.B
    }
  }

  // ---- Sequential logic --------------------------------------
  val uopOut_r = RegInit(0.U.asTypeOf(new AGU_UOp))
  io.uopOut := uopOut_r

  when(reset.asBool) {
    for (i <- 0 until SIZE) {
      queue(i).valid := false.B
      ready(i)       := false.B
    }
    uopOut_r.valid := false.B

  }.otherwise {

    // 1. Mark entries ready when the page walker covers their address
    when(io.pw.valid) {
      for (i <- 0 until SIZE) {
        when(queue(i).valid && !ready(i)) {
          val superMatch = io.pw.isSuperPage &&
            io.pw.vpn(19, 10) === queue(i).addr(31, 22)
          val pageMatch  = !io.pw.isSuperPage &&
            io.pw.vpn === queue(i).addr(31, 12)
          when(superMatch || pageMatch) {
            ready(i) := true.B
          }
        }
      }
    }

    // 2. Flush entries that are younger than a mispredicted branch
    when(io.branch.taken) {
      for (i <- 0 until SIZE) {
        when(queue(i).valid &&
          (queue(i).sqN.value.asSInt - io.branch.sqN.value.asSInt) > 0.S) {
          queue(i).valid := false.B
          ready(i)       := false.B
        }
      }
    }

    // 3. Enqueue — only accept if the uop is not being flushed
    when(io.enqueue && io.uopIn.valid &&
      (!io.branch.taken ||
        (io.uopIn.sqN.value.asSInt - io.branch.sqN.value.asSInt) <= 0.S)) {
      queue(freeSlot) := io.uopIn
      // Entry is immediately ready if VM is disabled or TLB already hit
      ready(freeSlot) := !io.vmem.enabled || io.uopReady
    }

    // 4. Dequeue output register
    //    Clear if dequeue requested, or if branch flushes current output
    when(io.dequeue ||
      (io.branch.taken &&
        (uopOut_r.sqN.value.asSInt - io.branch.sqN.value.asSInt) > 0.S)) {
      uopOut_r.valid := false.B
    }

    // 5. Advance output register from queue when slot is free
    when((!uopOut_r.valid || io.dequeue) && deqSlotValid) {
      val candidate = queue(deqSlot)
      when(candidate.valid &&
        (!io.branch.taken ||
          (candidate.sqN.value.asSInt - io.branch.sqN.value.asSInt) <= 0.S)) {
        uopOut_r          := candidate
        ready(deqSlot)    := false.B
        queue(deqSlot).valid := false.B
      }
    }
  }
}
