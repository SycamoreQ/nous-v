package ROB

import chisel3._
import chisel3.util._
import fetch.{FetchID, FetchOff}
import branch_pred.SqN

// Physical register tag
// MSB = 1 means special tag (zero register or immediate — no physical reg needed)
// Lower 6 bits = physical register file index
// TAG_SIZE = 7 bits → 64 physical registers + special values
object TagConst {
  val TAG_SIZE = 7
  val TAG_ZERO = "b1000000".U(7.W)   // x0 — always reads as zero, no writeback
}

// Physical register file index — 6 bits, indexes into the actual register file
// Separate from Tag so the MSB special flag is stripped before RF access
class RFTag extends Bundle {
  val value = UInt(6.W)
}

// Flags — execution status written back to the ROB flags array by execute units
//
// Ordering matters — the reference uses numeric comparisons on flag values
// to determine categories (e.g. flags >= FLAGS_ILLEGAL_INSTR are exceptions).
// Keep this order.
object Flags extends ChiselEnum {
  val FLAGS_NX,            // not yet executed — still in flight
      FLAGS_NONE,          // completed normally, no side effects
      FLAGS_ORDERING,      // completed, requires ordering (FENCE)
      FLAGS_FENCE,         // FENCE.I — requires ICache flush + pipeline drain
      FLAGS_BRANCH,        // unconditional branch/jump completed
      FLAGS_PRED_TAKEN,    // conditional branch resolved as taken
      FLAGS_PRED_NTAKEN,   // conditional branch resolved as not taken
      FLAGS_XRET,          // MRET or SRET completed
      FLAGS_TRAP,          // decode-time trap (ECALL, EBREAK, SFENCE.VMA etc.)
      FLAGS_ILLEGAL_INSTR, // illegal instruction encoding
      FLAGS_LD_PF,         // load page fault
      FLAGS_ST_PF,         // store page fault
      FLAGS_LD_AF,         // load access fault
      FLAGS_ST_AF,         // store access fault
      // FP exception flags — must remain contiguous and in this order
      // as the reference indexes them arithmetically
      FLAGS_FP_NX,         // inexact
      FLAGS_FP_UF,         // underflow (implies inexact)
      FLAGS_FP_OF,         // overflow  (implies inexact)
      FLAGS_FP_DZ,         // divide by zero
      FLAGS_FP_NV          // invalid operation
      = Value
}

// ROBEntry — static per-instruction data written at dispatch, read at commit
// Stored in the banked entry array, one bank per decode slot
class ROBEntry extends Bundle {
  val tag        = UInt(TagConst.TAG_SIZE.W)  // physical destination tag
  val rd         = UInt(5.W)                  // architectural destination register
  val fetchOffs  = new FetchOff               // offset within fetch group
  val fetchID    = new FetchID                // which fetch group
  val compressed = Bool()                     // was a 16-bit compressed instruction
  val isLd       = Bool()                     // is a load (AGU load or atomic)
  val isSt       = Bool()                     // is a store (AGU store or atomic)
}

// CommitUOp — sent by ROB to register file and rename table on commit
// One per commit slot per cycle, valid when an instruction retires
class CommitUOp extends Bundle {
  val valid      = Bool()
  val rd         = UInt(5.W)                  // architectural destination
  val tagDst     = UInt(TagConst.TAG_SIZE.W)  // physical tag being committed
  val sqN        = new SqN                    // sequence number of committed instruction
  val isBranch   = Bool()                     // was a branch (for BP update)
  val compressed = Bool()
}

// FlagsUOp — sent by execution units to the ROB flags array when complete
// Written out-of-order — the ROB uses sqN to index the correct flags entry
class FlagsUOp extends Bundle {
  val valid       = Bool()
  val sqN         = new SqN
  val tagDst      = UInt(TagConst.TAG_SIZE.W) // physical tag of the result
  val flags       = Flags()                   // execution result status
  val doNotCommit = Bool()                    // suppress commit (used for cancelled ops)
}


// Trap_UOp — sent by ROB to the trap handler when an exception commits
// Contains everything needed to redirect PC and set CSR trap state
class Trap_UOp extends Bundle {
  val valid      = Bool()
  val timeout    = Bool()                     // true if this is a hang-detection timeout
  val flags      = Flags()                    // which exception/trap type
  val tag        = UInt(TagConst.TAG_SIZE.W)
  val sqN        = new SqN
  val loadSqN    = new SqN                    // load sequence number at point of trap
  val storeSqN   = new SqN                    // store sequence number at point of trap
  val rd         = UInt(5.W)
  val fetchOffs  = new FetchOff
  val fetchID    = new FetchID
  val compressed = Bool()
}

// R_UOp — renamed micro-op produced by Rename, consumed by issue queues + ROB
// Extends the decoded D_UOp with physical tags for source operands
class R_UOp(NUM_PORTS_TOTAL: Int = 4) extends Bundle {
  // Validity
  val valid    = Bool()
  val validIQ  = UInt(NUM_PORTS_TOTAL.W)     // per-IQ valid bits (held while IQ stalls)

  // Sequence numbers
  val sqN      = new SqN
  val loadSqN  = new SqN                     // load sequence number assigned at rename
  val storeSqN = new SqN                     // store sequence number assigned at rename

  // Physical tags
  val tagDst   = UInt(TagConst.TAG_SIZE.W)   // destination physical tag
  val tagA     = UInt(TagConst.TAG_SIZE.W)   // source 1 physical tag
  val tagB     = UInt(TagConst.TAG_SIZE.W)   // source 2 physical tag
  val tagC     = UInt(TagConst.TAG_SIZE.W)   // source 3 (atomics only — mem operand)

  // Tag availability — set when the physical register has a valid result
  val availA   = Bool()
  val availB   = Bool()
  val availC   = Bool()

  // Architectural fields — passed through from D_UOp
  val rd       = UInt(5.W)
  val fu       = UInt(4.W)                   // FU_t as UInt to avoid cross-package enum issues
  val opcode   = UInt(7.W)
  val imm      = UInt(32.W)
  val imm12    = UInt(12.W)
  val immB     = Bool()
  val fetchID  = new FetchID
  val fetchOffs = new FetchOff
  val compressed = Bool()
}

// StallCause — why the ROB could not commit this cycle (for perf counters)
object StallCause extends ChiselEnum {
  val STALL_NONE,      // no stall — committed successfully
      STALL_FRONTEND,  // frontend not producing instructions (ICache miss, TLB miss, etc.)
      STALL_BACKEND,   // execution unit not finishing (long-latency op)
      STALL_LOAD,      // load buffer blocking commit
      STALL_STORE,     // store queue blocking commit
      STALL_ROB        // ROB internal flag conflict (branch followed by exception etc.)
      = Value
}

// ROB_PERFC_Info — performance counter output from ROB to monitoring logic
// Driven combinationally every cycle
class ROB_PERFC_Info(WIDTH: Int = 4) extends Bundle {
  val validRetire  = UInt(WIDTH.W)    // bitmask of slots that retired valid instructions
  val branchRetire = UInt(WIDTH.W)    // bitmask of slots that retired branches
  val stallCause   = StallCause()     // why we stalled (if we did)
  val stallWeight  = UInt(2.W)        // how many slots were stalled (0-3)
}

// MisprReplay — tracks the multi-cycle misprediction replay state
// When a branch mispredicts the ROB replays all in-flight instructions
// from baseIndex to the mispredicting SqN through the commit path
// to roll back the rename map to committed state
class MisprReplay extends Bundle {
  val valid    = Bool()
  val endSqN   = new SqN    // SqN of the mispredicting instruction
  val iterSqN  = new SqN    // current replay position (advances by WIDTH each cycle)
}

// ComLimit — load/store commit limit signals from load buffer and store queue
// The ROB must not commit a load/store past these limits
class ComLimit extends Bundle {
  val valid = Bool()
  val sqN   = new SqN
}

// HANG_COUNTER_LEN — number of cycles without a commit before hang is detected
// 2^20 cycles approx 1M cycles at 100MHz approx ~10ms
object HangConst {
  val HANG_COUNTER_LEN = 20
}
