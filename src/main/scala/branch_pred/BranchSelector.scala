package branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.iFetchParams;
import fetch._;


class BranchSelector(NUM_BRANCHES:Int = 4) {
  val io = IO(new Bundle {
    val isUOps = Input(Vec(2, new IS_UOp))
    val branches = Input(Vec(NUM_BRANCHES, new BranchProv))
    val branch = Output(new BranchProv)
    val branchMispr = Output(Reg(0.U))
    val ROB_curSQN = Input(new SqN)
    val ROB_nextSQN = Input(new SqN)
    val misprFlush = Input(Bool())
  })

  val branch_mispc = Wire(0.U)
  val branch_c = Wire(new BranchProv)

  io.branch := branch_c
  io.branchMispr := branch_mispc

  // Priority: Special port > Oldest branch (lowest SqN)
  val priorityPort = RegNext((io.isUOps(1).sqN.value.asSInt - io.isUOps(0).sqN.value.asSInt) > 0.S)
  val intPortBranch = Wire(new BranchProv)

  when(io.branches(0).taken &&
    (!io.branches(1).taken || priorityPort)) {
    intPortBranch := io.branches(0)
    }.otherwise {
      intPortBranch := io.branches(1)
    }

  val compBranches = VecInit(io.branches(2), intPortBranch)
  val PERFC_branchMispr_c = Wire(Bool())
  branch_c := DontCare
  branch_c.flush := false.B
  branch_c.taken := false.B
  PERFC_branchMispr_c := false.B
  // Select oldest branch between compBranches[0] and compBranches[1]
  when(compBranches(0).taken &&
     (!compBranches(1).taken ||
      $signed(compBranches(0).sqN.value.asSInt -
        compBranches(1).sqN.value.asSInt) < 0.S)) {

    branch_c := compBranches(0)
    when(!io.misprFlush) {
      PERFC_branchMispr_c := true.B
    }
    }.otherwise {
    branch_c := compBranches(1)
    when(compBranches(1).taken && !io.misprFlush) {
      PERFC_branchMispr_c := true.B
      }
  }
}

