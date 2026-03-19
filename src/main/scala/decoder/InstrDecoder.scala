package  decoder

import branch_pred.IS_UOp
import chisel3._
import chisel3.util._
import decoder.{D_UOp, DecodeBranch, DecodeState, OpcodeConst}
import fetch.iFetchParams
import fetch.PD_Instr


class InstrDecoder(params: iFetchParams) extends Module {
  val io = IO(new Bundle {
    val in = Input(new PD_Instr) // from fetch.ifetchutil
    val decState = Input(new DecodeState)
    val inBranch = Input(new DecodeBranch) // from previous slot
    val out = Output(new D_UOp)
    val outBranch = Output(new DecodeBranch)
  })

  val decBranch = WireDefault(0.U.asTypeOf(new DecodeBranch))
  val invalidEnc = WireDefault(true.B)
  val instr32 = RegInit(0.U(32.W))
  val instr16 = RegInit(0.U(32.W))
  val uop = WireDefault(0.U.asTypeOf(new D_UOp))

  val instr = io.in.bits // full 32-bit instruction word
  val opcode = instr(6, 0) // 7-bit opcode field
  val funct3 = instr(14, 12)
  val funct7 = instr(31, 25)
  val rs1 = instr(19, 15)
  val rs2 = instr(24, 20)
  val rd = instr(11, 7)

  uop.valid := io.in.valid && !io.inBranch.taken
  uop.fetchID := io.in.fetchID
  uop.fetchOffs := io.in.fetchID

  val imm_i = Cat(Fill(20, instr(31)), instr(31, 20))
  val imm_s = Cat(Fill(20, instr(31)), instr(31, 25), instr(11, 7))
  val imm_b = Cat(Fill(19, instr(31)), instr(31), instr(7),
    instr(30, 25), instr(11, 8), 0.U(1.W))
  val imm_u = Cat(instr(31, 12), 0.U(12.W))
  val imm_j = Cat(Fill(11, instr(31)), instr(31), instr(19, 12),
    instr(20), instr(30, 21), 0.U(1.W))

  uop.imm := MuxLookup(opcode , 0.U)(Seq(
    OpcodeConst.OPC_LUI     -> imm_u,
    OpcodeConst.OPC_AUIPC   -> imm_u,
    OpcodeConst.OPC_JAL     -> imm_j,
    OpcodeConst.OPC_JALR    -> imm_i,
    OpcodeConst.OPC_LOAD    -> imm_i,
    OpcodeConst.OPC_REG_IMM -> imm_i,
    OpcodeConst.OPC_ENV     -> imm_i,
    OpcodeConst.OPC_BRANCH  -> imm_b,
    OpcodeConst.OPC_STORE   -> imm_s
  ))

  uop.imm12 := instr(31, 20)

  opcode match {
    case OpcodeConst.OPC_LUI => {
      uop.fu := FU_t.FU_INT
      uop.rs1 := 0.U
      uop.rs2 := 0.U
      uop.immB := 1.U
      uop.rd := rd
      uop.opcode := INT_Op.INT_LUI
      invalidEnc := false.B
    }

    case OpcodeConst.OPC_AUIPC => {
      uop.fu := FU_t.FU_BRANCH
      uop.rs1 := 0.U
      uop.rs2 := 0.U
      uop.rd := rd
      uop.opcode := BR_Op.BR_AUIPC
      invalidEnc := false.B
    }

    case OpcodeConst.OPC_JAL => {
      uop.fu := FU_t.FU_BRANCH
      uop.rs1 := 0.U
      uop.rs2 := 0.U
      uop.immB := 1.U
      uop.rd := rd
      uop.opcode := BR_Op.BR_JAL
      invalidEnc := false.B
      // No need to execute jumps that don't write to a register
      if (uop.rd == 0) uop.fu := FU_t.FU_RN;
    }

    case OpcodeConst.OPC_JALR => {
      uop.fu := FU_t.FU_BRANCH
      uop.rs1 := rs1
      uop.immB := 1.U
      uop.imm12 := instr(31 , 20)
      uop.rd := rd
    }
  }
}
