package decoder

import chisel3._
import chisel3.util._
import fetch.{FetchID, FetchOff}
import branch_pred.SqN

// RISC-V 7-bit opcode constants
// Used to match against instr(6,0) in the decode switch

object OpcodeConst {
  val OPC_LUI     = "b0110111".U(7.W)
  val OPC_AUIPC   = "b0010111".U(7.W)
  val OPC_JAL     = "b1101111".U(7.W)
  val OPC_JALR    = "b1100111".U(7.W)
  val OPC_LOAD    = "b0000011".U(7.W)
  val OPC_STORE   = "b0100011".U(7.W)
  val OPC_BRANCH  = "b1100011".U(7.W)
  val OPC_REG_IMM = "b0010011".U(7.W)
  val OPC_REG_REG = "b0110011".U(7.W)
  val OPC_ENV     = "b1110011".U(7.W)
  val OPC_FENCE   = "b0001111".U(7.W)
  val OPC_ATOMIC  = "b0101111".U(7.W)
}

// Functional Unit enum
// Identifies which execution unit handles a given micro-op
object FU_t extends ChiselEnum {
  val FU_RN,      // rename only — NOP, eliminated before issue
      FU_INT,     // integer ALU
      FU_MUL,     // multiplier
      FU_DIV,     // divider
      FU_BRANCH,  // branch / jump unit
      FU_AGU,     // address generation unit (loads + stores)
      FU_CSR,     // CSR read/write unit
      FU_TRAP,    // trap / exception handling
      FU_ATOMIC   // atomic memory operations
      = Value
}

// Per-FU opcode enums
// Each FU has its own opcode space. D_UOp stores opcode as UInt(7.W)
// and the issue queue casts to the appropriate enum based on fu field.

object INT_Op extends ChiselEnum {
  val INT_ADD,    // add / addi
      INT_SUB,    // sub
      INT_SLL,    // shift left logical
      INT_SRL,    // shift right logical
      INT_SRA,    // shift right arithmetic
      INT_AND,    // and / andi
      INT_OR,     // or  / ori
      INT_XOR,    // xor / xori
      INT_SLT,    // set less than (signed)
      INT_SLTU,   // set less than (unsigned)
      INT_LUI,    // load upper immediate
      INT_SYS     // system / fence (serializing)
      = Value
}

object BR_Op extends ChiselEnum {
  val BR_JAL,     // jump and link (unconditional)
      BR_V_JR,    // jump register, no link (rd=0)
      BR_V_RET,   // return — jr ra (rs1=ra, rd=0)
      BR_V_JALR,  // jump and link register
      BR_BEQ,     // branch equal
      BR_BNE,     // branch not equal
      BR_BLT,     // branch less than (signed)
      BR_BGE,     // branch greater or equal (signed)
      BR_BLTU,    // branch less than (unsigned)
      BR_BGEU,    // branch greater or equal (unsigned)
      BR_AUIPC    // add upper immediate to PC
      = Value
}

object LSU_Op extends ChiselEnum {
  val LSU_LB,     // load byte (signed)
      LSU_LH,     // load halfword (signed)
      LSU_LW,     // load word
      LSU_LBU,    // load byte (unsigned)
      LSU_LHU,    // load halfword (unsigned)
      LSU_SB,     // store byte
      LSU_SH,     // store halfword
      LSU_SW,     // store word
      LSU_LR_W,   // load-reserved word (A extension)
      LSU_SC_W    // store-conditional word (A extension)
      = Value
}

object MUL_Op extends ChiselEnum {
  val MUL_MUL,    // multiply low 32 bits
      MUL_MULH,   // multiply high 32 bits (signed x signed)
      MUL_MULSU,  // multiply high 32 bits (signed x unsigned)
      MUL_MULU    // multiply high 32 bits (unsigned x unsigned)
      = Value
}

object DIV_Op extends ChiselEnum {
  val DIV_DIV,    // divide (signed)
      DIV_DIVU,   // divide (unsigned)
      DIV_REM,    // remainder (signed)
      DIV_REMU    // remainder (unsigned)
      = Value
}

object CSR_Op extends ChiselEnum {
  val CSR_RW,     // csrrw  — read/write
      CSR_RS,     // csrrs  — read/set
      CSR_RC,     // csrrc  — read/clear
      CSR_RW_I,   // csrrwi — read/write immediate
      CSR_RS_I,   // csrrsi — read/set immediate
      CSR_RC_I,   // csrrci — read/clear immediate
      CSR_R,      // csrrs/csrrc with rs1=0 — read only
      CSR_MRET,   // machine return
      CSR_SRET    // supervisor return
      = Value
}

object ATOMIC_Op extends ChiselEnum {
  val ATOMIC_AMOSWAP_W,   // atomic swap
      ATOMIC_AMOADD_W,    // atomic add
      ATOMIC_AMOXOR_W,    // atomic xor
      ATOMIC_AMOAND_W,    // atomic and
      ATOMIC_AMOOR_W,     // atomic or
      ATOMIC_AMOMIN_W,    // atomic min (signed)
      ATOMIC_AMOMAX_W,    // atomic max (signed)
      ATOMIC_AMOMINU_W,   // atomic min (unsigned)
      ATOMIC_AMOMAXU_W    // atomic max (unsigned)
      = Value
}

object TRAP_Op extends ChiselEnum {
  val TRAP_ILLEGAL_INSTR, // undefined / illegal encoding
      TRAP_ECALL_M,       // environment call from M-mode
      TRAP_BREAK,         // ebreak / c.ebreak
      TRAP_I_PAGE_FAULT,  // instruction page fault (from fetch)
      TRAP_I_ACC_FAULT,   // instruction access fault (from fetch)
      TRAP_V_INTERRUPT,   // interrupt (from fetch fault)
      TRAP_V_SFENCE_VMA   // sfence.vma (serializing)
      = Value
}

// D_UOp — decoded micro-op
//
// Produced by the decoder, consumed by the issue queue.
// opcode is stored as UInt(7.W) — wide enough for all per-FU enums.
// The issue queue casts to the correct enum type based on the fu field.

class D_UOp extends Bundle {
  val valid      = Bool()
  val fu         = FU_t()
  val opcode     = UInt(7.W)      // cast to per-FU enum at issue queue
  val sqN        = new SqN        // sequence number for OoO tracking
  val fetchID    = new FetchID    // which fetch group this came from
  val fetchOffs  = new FetchOff   // offset within fetch group
  val rs1        = UInt(5.W)      // source register 1 (0 = x0)
  val rs2        = UInt(5.W)      // source register 2 (0 = x0)
  val rd         = UInt(5.W)      // destination register (0 = discard)
  val immB       = Bool()         // true when imm field is valid
  val imm        = UInt(32.W)     // sign-extended immediate
  val imm12      = UInt(12.W)     // raw 12-bit immediate (JALR offset)
  val compressed = Bool()         // was originally a 16-bit C instruction
}

// DecodeBranch
//
// Signals a serializing event within the current decode group.
// Once taken, all subsequent instructions in the group are invalidated.
// wfi is set for any instruction that must drain the pipeline before
// fetch can continue (branches, traps, WFI, MRET, SRET, FENCE.I etc.)
class DecodeBranch extends Bundle {
  val taken     = Bool()
  val fetchID   = new FetchID
  val fetchOffs = new FetchOff
  val wfi       = Bool()
  val pc  = UInt(32.W)
}

// DecodeState
//
// Privilege-level control signals passed into the decoder from the CSR module.
// Gate certain instructions based on current privilege level.

class DecodeState extends Bundle {
  val allowWFI      = Bool()   // WFI permitted at current privilege level
  val allowSFENCE   = Bool()   // SFENCE.VMA permitted
  val allowCustom   = Bool()   // custom opcode extensions enabled
}
