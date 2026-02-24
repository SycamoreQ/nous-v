package  branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.FetchOff


// Sequence Number (SqN)
// Used to track program order in out-of-order execution
class SqN extends Bundle {
  val value = UInt(8.W)  // Adjust width based on ROB size
  // Program order comparison
  def >(that: SqN): Bool = this.value > that.value
  def <(that: SqN): Bool = this.value < that.value
  def >=(that: SqN): Bool = this.value >= that.value
  def <=(that: SqN): Bool = this.value <= that.value
}


// IS_UOp: Instruction Scheduler Micro-Operation
// This is a subset of the full UOp, containing only fields needed for
// branch arbitration
class IS_UOp extends Bundle {
  val sqN = new SqN // Sequence number (program order)
  val valid = Bool()
}

class BTBEntry {
  val compr = Bool()
  val valid = Bool()
  val dst = UInt(31.W)
  val src = UInt(`BTB_TAG_SIZE-1.W)
  val offs = new FetchOff
}

object BranchType extends ChiselEnum {
  val BT_CALL , BT_JUMP = Value;
}

