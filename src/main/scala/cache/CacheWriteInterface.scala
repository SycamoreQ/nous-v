package cache

import chisel3._
import chisel3.util._


// CacheWriteInterface
//
// Sits between incoming AXI burst data and the cache SRAM.
// Handles three cases based on IWIDTH vs CWIDTH:
//
//   IWIDTH == CWIDTH : one beat = one SRAM word, simple passthrough
//   IWIDTH >  CWIDTH : one beat must be split across multiple SRAM writes
//                      idx counter walks through sub-words
//   IWIDTH <  CWIDTH : multiple beats must be accumulated before one SRAM
//                      word can be written, wm tracks which chunks arrived
//

class CacheWriteInterface(
  ADDR_BITS : Int = 12,  //cache SRAM address width (CACHE_SIZE_E - 2)
  IWIDTH    : Int = 64,  //incoming data width in bits (AXI beat width)
  CWIDTH    : Int = 32,  //cache SRAM word width in bits
  ID_LEN    : Int = 2    //transaction ID width, log2(AXI_NUM_TRANS)
) extends Module {

  val WNUM      = if (IWIDTH > CWIDTH) IWIDTH / CWIDTH else 1
  val CNUM      = if (CWIDTH > IWIDTH) CWIDTH / IWIDTH else 1
  val WIDTH     = scala.math.max(IWIDTH, CWIDTH)
  val WM_LEN    = scala.math.max(1, CNUM)
  // CHUNK_LEN is the number of bits needed to index chunks.
  // Only meaningful when IWIDTH < CWIDTH (accumulation case).
  val CHUNK_LEN = if (CWIDTH > IWIDTH) log2Ceil(CWIDTH / IWIDTH) else 1
  val IDX_BITS  = log2Ceil(WNUM) + 1   // idx counter width

  // In the split case (IWIDTH > CWIDTH) we have WNUM sub-words of CWIDTH.
  // In the accumulate case (IWIDTH < CWIDTH) we have CNUM sub-words of IWIDTH.
  // In the equal case both reduce to 1 sub-word.
  class Transfer extends Bundle {
    val data  = Vec(scala.math.max(WNUM, CNUM), UInt(scala.math.min(IWIDTH, CWIDTH).W))
    val addr  = UInt(ADDR_BITS.W)
    val idx   = UInt(IDX_BITS.W)     // sub-word index for split case
    val id    = UInt(ID_LEN.W)
    val wm    = UInt(WM_LEN.W)       // write mask for accumulation case
    val valid = Bool()
  }

  val io = IO(new Bundle {
    val IN_valid = Input(Bool())
    val IN_addr  = Input(UInt(ADDR_BITS.W))
    val IN_data  = Input(UInt(IWIDTH.W))
    val IN_id    = Input(UInt(ID_LEN.W))

    val OUT_ready    = Output(Bool())
    val OUT_ackValid = Output(Bool())
    val OUT_ackId    = Output(UInt(ID_LEN.W))

    val OUT_CACHE_ce   = Output(Bool())
    val OUT_CACHE_we   = Output(Bool())
    val OUT_CACHE_wm   = Output(UInt(WM_LEN.W))
    val OUT_CACHE_addr = Output(UInt(ADDR_BITS.W))
    val OUT_CACHE_data = Output(UInt(CWIDTH.W))

    // From SRAM
    val IN_CACHE_ready = Input(Bool())
  })

  val cur_r = RegInit(0.U.asTypeOf(new Transfer()))
  val cur_c = WireDefault(cur_r)

  // writeLast fires (combinationally) when the last sub-word of a transfer
  // is committed to SRAM. The registered version becomes OUT_ackValid.
  val writeLast   = WireDefault(false.B)
  val writeLastId = WireDefault(0.U(ID_LEN.W))



  val addrConflict   = WireDefault(false.B)
  val wm_new         = WireDefault(1.U(WM_LEN.W))
  val chunkInsertIdx = WireDefault(0.U(CHUNK_LEN.W))

  if (IWIDTH >= CWIDTH) {
    // Wide or equal: one beat fills one or more SRAM words.
    // No address conflict possible — each beat maps to a distinct address.
    wm_new         := 1.U
    addrConflict   := false.B
    chunkInsertIdx := 0.U
    io.OUT_ready   := !cur_r.valid
  } else {
    // Accumulation: multiple beats needed to fill one SRAM word.
    // addrConflict when a new beat targets a different SRAM word than
    // the one currently being accumulated.
    addrConflict   := cur_r.valid && io.IN_valid &&
                      (cur_r.addr(ADDR_BITS-1, CHUNK_LEN) =/=
                       io.IN_addr(ADDR_BITS-1, CHUNK_LEN))
    wm_new         := 1.U << io.IN_addr(log2Ceil(WM_LEN)-1, 0)
    chunkInsertIdx := io.IN_addr(log2Ceil(WM_LEN)-1, 0)
    // Ready even if slot is occupied as long as we are still mid-accumulation
    // (not all chunk address bits are set yet)
    io.OUT_ready   := !cur_r.valid ||
                      !cur_r.addr(CHUNK_LEN-1, 0).andR
  }

  // ce is active-low in the reference — default high means inactive
  io.OUT_CACHE_ce   := true.B
  io.OUT_CACHE_we   := DontCare
  io.OUT_CACHE_wm   := DontCare
  io.OUT_CACHE_addr := DontCare
  io.OUT_CACHE_data := DontCare

  // Case 1: Slot is empty and new data is arriving.
  // Start a fresh transfer.
  when(!cur_c.valid && io.OUT_ready && io.IN_valid) {
    cur_c.valid              := true.B
    cur_c.addr               := io.IN_addr
    cur_c.data(chunkInsertIdx) := io.IN_data
    cur_c.id                 := io.IN_id
    cur_c.idx                := 0.U
    cur_c.wm                 := wm_new

    // In the accumulation case the ack fires on each beat accepted,
    // allowing the MemController to track progress incrementally.
    if (IWIDTH < CWIDTH) {
      writeLast   := true.B
      writeLastId := io.IN_id
    }

  // Case 2: Slot is occupied, more data arriving, no address conflict.
  // Continue accumulating into the existing slot.
  }.elsewhen(cur_c.valid && io.OUT_ready && io.IN_valid && !addrConflict) {
    cur_c.data(chunkInsertIdx) := io.IN_data
    cur_c.addr                 := io.IN_addr
    cur_c.wm                   := cur_c.wm | wm_new

    if (IWIDTH < CWIDTH) {
      writeLast   := true.B
      writeLastId := cur_c.id
    }
  }

  // Case 3: Slot is ready to flush to SRAM.
  // Fires when:
  //   - IWIDTH >= CWIDTH (each beat is immediately complete), OR
  //   - all chunk address bits are set (accumulation complete), OR
  //   - address conflict forces a flush of the current slot
  // Requires SRAM to be ready (IN_CACHE_ready).
  when((
    (IWIDTH.B >= CWIDTH.B && cur_c.valid) ||
    (IWIDTH.B < CWIDTH.B && cur_c.valid && cur_c.addr(CHUNK_LEN-1, 0).andR) ||
    addrConflict
  ) && cur_c.valid && io.IN_CACHE_ready) {

    io.OUT_CACHE_ce   := false.B   // active-low enable
    io.OUT_CACHE_we   := false.B   // active-low write
    io.OUT_CACHE_addr := cur_c.addr + cur_c.idx
    io.OUT_CACHE_data := cur_c.data(cur_c.idx)
    io.OUT_CACHE_wm   := cur_c.wm

    // Advance sub-word index for split case (IWIDTH > CWIDTH)
    cur_c.idx := cur_c.idx + 1.U

    // When all sub-words have been written, free the slot
    when(cur_c.idx(log2Ceil(WNUM))) {
      if (IWIDTH >= CWIDTH) {
        writeLast   := true.B
        writeLastId := cur_c.id
      }
      cur_c.valid := false.B
      cur_c.wm    := 0.U
    }
  }

  // Case 4: Address conflict with incoming beat.
  // The current slot has been (or is being) flushed above.
  // Now start a new transfer for the conflicting incoming beat.
  when(addrConflict && io.IN_valid) {
    cur_c.valid              := true.B
    cur_c.addr               := io.IN_addr
    cur_c.data(chunkInsertIdx) := io.IN_data
    cur_c.id                 := io.IN_id
    cur_c.idx                := 0.U
    cur_c.wm                 := wm_new

    if (IWIDTH < CWIDTH) {
      writeLast   := true.B
      writeLastId := io.IN_id
    }
  }

  // Register the combinational next-state and the ack signals.
  cur_r            := cur_c
  io.OUT_ackValid  := RegNext(writeLast,   false.B)
  io.OUT_ackId     := RegNext(writeLastId, 0.U)
}
