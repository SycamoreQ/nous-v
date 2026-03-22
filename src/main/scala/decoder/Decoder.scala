package decoder

/*
Takes four PD_Instr inputs from InstrAligner.
Runs each through its own InstrDecoder instance.
The decBranch signal from slot i feeds into slot i+1 as inBranch — once any slot produces a taken branch,
all subsequent slots see it and invalidate themselves.
Registers the four D_UOp outputs and the final decBranch into flip-flops.
Handles flush when IN_branch.taken comes from the execute stage.
 */

import chisel3._
import chisel3.util._
import fetch.{BranchProv, FetchID, FetchOff, PD_Instr , iFetchParams}
import branch_pred.SqN
import decoder.OpcodeConst
import decoder.InstrDecoder


class Decoder (NUM_UOPS: Int = 4)(params: iFetchParams) extends Module{
  val io = IO(new Bundle{
    val inInstrs = Input(Vec(NUM_UOPS , new PD_Instr))
    val inDec = Input(new DecodeState)
    val inBranch = Input(new BranchProv)
    val outUop = Output(Vec(NUM_UOPS , new D_UOp))
    val outBranch = Output(new DecodeBranch)
  })

  val decoders = Seq.fill(NUM_UOPS)(Module(new InstrDecoder(params)))
  val zeroBranch = WireDefault(0.U.asTypeOf(new DecodeBranch))
  decoders(0).io.inBranch := zeroBranch

  for (i <- 1 until NUM_UOPS) {
    decoders(i).io.inBranch := decoders(i-1).io.outBranch
  }

  // Wire inputs and collect outputs
  for (i <- 0 until NUM_UOPS) {
    decoders(i).io.in       := io.inInstrs(i)
    decoders(i).io.decState := io.inDec
    io.outUop(i)            := decoders(i).io.out
  }

  io.outBranch := decoders(NUM_UOPS-1).io.outBranch


  val uopRegs = RegInit(VecInit(Seq.fill(NUM_UOPS)(0.U.asTypeOf(new D_UOp))))
  val decBranchReg = RegInit(0.U.asTypeOf(new DecodeBranch))

  for (i <- 0 until NUM_UOPS) {
    uopRegs(i) := decoders(i).io.out
  }
  decBranchReg := decoders(NUM_UOPS-1).io.outBranch

  io.outUop    := uopRegs
  io.outBranch := decBranchReg

  when(reset.asBool) {
    for (i <- 0 until NUM_UOPS) {
      uopRegs(i)       := 0.U.asTypeOf(new D_UOp)
      uopRegs(i).valid := false.B
    }
    decBranchReg := 0.U.asTypeOf(new DecodeBranch)
  }.elsewhen(io.inBranch.flush) {
    for (i <- 0 until NUM_UOPS) {
      uopRegs(i)       := 0.U.asTypeOf(new D_UOp)
      uopRegs(i).valid := false.B
    }
    decBranchReg := 0.U.asTypeOf(new DecodeBranch)
  }.otherwise {
    for (i <- 0 until NUM_UOPS) {
      uopRegs(i) := decoders(i).io.out
    }
    decBranchReg := decoders(NUM_UOPS-1).io.outBranch
  }



}