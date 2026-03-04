package branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{FetchID, FetchOff, RetStackIdx}


/// sequence number: tracks OOE with sequence numbers for each instr
class SqN extends Bundle {
  val value = UInt(8.W)
  def >(that: SqN): Bool = this.value > that.value
  def <(that: SqN): Bool = this.value < that.value
  def >=(that: SqN): Bool = this.value >= that.value
  def <=(that: SqN): Bool = this.value <= that.value
}

/// micro-instruction with limited fields
class IS_UOp extends Bundle {
  val sqN = new SqN
  val valid = Bool()
}

class BTBEntry(tagSize: Int) extends Bundle {
  val btype = BranchType()
  val compr = Bool()
  val valid = Bool()
  val dst = UInt(31.W)
  val src = UInt(tagSize.W)
  val offs = new FetchOff
}

class FetchedBundle(tagSize: Int) extends Bundle {
  val entry = new BTBEntry(tagSize)
  val multiple = Bool()
  val pc = UInt(31.W)
}

class SetMultiple(idxWidth: Int) extends Bundle {
  val idx = UInt(idxWidth.W)
  val valid = Bool()
}

object BranchType extends ChiselEnum {
  val BT_NONE, BT_CALL, BT_JUMP, BT_RETURN, BT_BRANCH = Value
}

class RetRecEntry extends Bundle {
  val addr = UInt(31.W)
  val idx = new RetStackIdx
  val offs = new FetchOff
  val fetchID = new FetchID
}

class RetRecQEntry extends Bundle {
  val addr = UInt(31.W)
  val idx = new RetStackIdx
  val offs = new FetchOff
  val fetchID = new FetchID
}

class PostRecSave extends Bundle {
  val addr = UInt(31.W)
  val rIdx = new RetStackIdx
  val fetchID = new FetchID
  val offs = new FetchOff
  val valid = Bool()
}

class PredBranch extends Bundle {
  val valid = Bool()
  val taken = Bool()
  val dst = UInt(31.W)
  val offs = new FetchOff
  val btype = BranchType()
  val compr = Bool()
  val multiple = Bool()
  val dirOnly = Bool()
}

class FetchBranchProv extends Bundle {
  val taken = Bool()
  val fetchID = new FetchID
  val fetchOffs = new FetchOff
  val isFetchBranch = Bool()
}

class ReturnDecUpdate extends Bundle {
  val valid = Bool()
  val idx = new RetStackIdx
  val addr = UInt(31.W)
}

object BH_BranchType extends ChiselEnum {
  val JUMP, IJUMP, CALL, ICALL, BRANCH, PAD0, PAD1, RETURN = Value
}

class LoopPredictorIndices extends Bundle {
  val bank = Vec(4, UInt(8.W))  // 4 banks for set-associative lookup
}

class LoopPredictorEntry extends Bundle {
  val totalIterations = UInt(10.W)
  val tag = UInt(10.W)
  val confidence = UInt(4.W)
  val age = UInt(4.W)
  val dir = Bool()
  val speculativeCurrentIter = UInt(10.W)
  val currentIter = UInt(10.W)
}

class LoopPredictionInfo extends Bundle {
  val hitBank = SInt(3.W)  // -1 for no hit, 0-3 for bank index
  val valid = Bool()
  val prediction = Bool()
  val indices = new LoopPredictorIndices
  val tag = UInt(10.W)
  val currentIterCheckpoint = UInt(10.W)
}

