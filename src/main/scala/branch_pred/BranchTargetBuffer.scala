package  branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{BTUpdate, FetchOff, PredBranch}


class BranchTargetBuffer {
  val io = IO(new Bundle {
    val pcValid = Input(Bool())
    val pc = Input(VecInit(0.U(31.W)))
    val branch = Output(new PredBranch)
    val btUpdate = Output(new BTUpdate)
  })

  val LENGTH = `BTB_ENTRIES
  val 
}

