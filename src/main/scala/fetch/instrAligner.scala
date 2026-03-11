package fetch

import chisel3._
import chisel3.util._

// ---------------------------------------------------------------------------
// IF_Packet — fetch packet produced by IFetchPipeline, consumed by
// InstrAligner.  Contains raw 16-bit half-words from the ICache line
// plus metadata needed to reconstruct PD_Instr outputs.
// ---------------------------------------------------------------------------
class IF_Packet(NUM_HALFWORDS: Int) extends Bundle {
  val valid      = Bool()
  val pc         = UInt(32.W)
  val fetchID    = new FetchID
  val firstValid = new FetchOff
  val lastValid  = new FetchOff
  val predPos    = new FetchOff      // half-word index of the predicted branch
  val predTarget = UInt(32.W)
  val predTaken  = Bool()
  val fault      = IFetchFault()
  val halfWords  = Vec(NUM_HALFWORDS, UInt(16.W))
}

// ---------------------------------------------------------------------------
// FetchCycle — pairs a full IF_Packet with two NUM_PACKETS-wide bitmaps.
//   start(i)   — half-word slot i is an instruction start
//   start32(i) — that start is the lower half of a 32-bit instruction
// ---------------------------------------------------------------------------
class FetchCycle(NUM_PACKETS: Int) extends Bundle {
  val op      = new IF_Packet(NUM_PACKETS)
  val start   = UInt(NUM_PACKETS.W)
  val start32 = UInt(NUM_PACKETS.W)
}

// ---------------------------------------------------------------------------
// InstrAligner
//
// Converts a stream of IF_Packets (raw RISC-V half-words) into up to
// DEC_WIDTH decoded PD_Instr entries per cycle.
//
// Advancements implemented:
//   1 — BUF_SIZE prev_r buffer + isInstrStart bitmap computation
//   2 — Window assembly (WINDOW_SIZE flat arrays)
//   3 — Multi-output priority encoder across windowStart_c
//   4 — unhandled_c tracking + canShift gating
//   5 — FF_OUTPUT pipeline register on outputs
//   6 — Full PD_Instr field population (predTarget, predTaken,
//         fetchStartOffs, fetchPredOffs) from per-cycle metadata
// ---------------------------------------------------------------------------
class InstrAligner(
  NUM_PACKETS : Int     = 8,
  DEC_WIDTH   : Int     = 4,
  BUF_SIZE    : Int     = 2,
  FF_OUTPUT   : Boolean = true
) extends Module {

  val WINDOW_SIZE = (BUF_SIZE + 1) * NUM_PACKETS
  val IDX_W       = log2Ceil(WINDOW_SIZE)

  val io = IO(new Bundle {
    val clear    = Input(Bool())
    val accept   = Input(Bool())
    val ready    = Output(Bool())
    val inPkt    = Input(new IF_Packet(NUM_PACKETS))
    val outReady  = Input(Bool())
    val outInstrs = Output(Vec(DEC_WIDTH, new PD_Instr))
  })

  // =========================================================================
  // Advancement 1 — prev_r buffer + isInstrStart bitmap computation
  // =========================================================================

  val prev_r = RegInit(VecInit(Seq.fill(BUF_SIZE)(0.U.asTypeOf(new FetchCycle(NUM_PACKETS)))))

  val startBits   = Wire(Vec(NUM_PACKETS, Bool()))
  val start32Bits = Wire(Vec(NUM_PACKETS, Bool()))
  for (i <- 0 until NUM_PACKETS) {
    startBits(i)   := false.B
    start32Bits(i) := false.B
  }

  val prevSplit32 = prev_r(BUF_SIZE - 1).start32(NUM_PACKETS - 1)

  // Scala elaboration-time carry — not a hardware register.
  var startCarry: chisel3.Bool = !prevSplit32

  for (i <- 0 until NUM_PACKETS) {
    val inWindow  = io.inPkt.valid &&
                    (i.U >= io.inPkt.firstValid.value) &&
                    (i.U <= io.inPkt.lastValid.value)
    val canStart  = startCarry && inWindow
    val hw        = io.inPkt.halfWords(i)
    val is32bit_w = hw(1, 0) === 3.U

    startBits(i)   := canStart
    start32Bits(i) := canStart && is32bit_w

    startCarry = Mux(canStart && is32bit_w, false.B, true.B)
  }

  val isInstrStart_w   = startBits.asUInt
  val isInstrStart32_w = start32Bits.asUInt

  val cur_c = Wire(new FetchCycle(NUM_PACKETS))
  cur_c.op      := io.inPkt
  cur_c.start   := isInstrStart_w
  cur_c.start32 := isInstrStart32_w

  // =========================================================================
  // Advancement 2 — Window assembly
  // =========================================================================

  // cycles_c: oldest at index 0, cur_c at index BUF_SIZE.
  val cycles_c = Wire(Vec(BUF_SIZE + 1, new FetchCycle(NUM_PACKETS)))
  for (i <- 0 until BUF_SIZE) {
    cycles_c(i) := prev_r(BUF_SIZE - 1 - i)
  }
  cycles_c(BUF_SIZE) := cur_c

  val window_c        = Wire(Vec(WINDOW_SIZE, UInt(16.W)))
  val windowStart_c   = Wire(Vec(WINDOW_SIZE, Bool()))
  val windowStart32_c = Wire(Vec(WINDOW_SIZE, Bool()))

  for (i <- 0 to BUF_SIZE) {
    for (j <- 0 until NUM_PACKETS) {
      window_c       (i * NUM_PACKETS + j) := cycles_c(i).op.halfWords(j)
      windowStart_c  (i * NUM_PACKETS + j) := cycles_c(i).start  (j)
      windowStart32_c(i * NUM_PACKETS + j) := cycles_c(i).start32(j)
    }
  }

  // Split-32 boundary corrections.
  val middleIsSplit32 = prev_r(BUF_SIZE - 1).start32(NUM_PACKETS - 1) && !io.inPkt.valid
  val lastIsSplit32   = isInstrStart32_w(NUM_PACKETS - 1)

  windowStart_c(BUF_SIZE * NUM_PACKETS - 1) :=
    cycles_c(BUF_SIZE - 1).start(NUM_PACKETS - 1) && !middleIsSplit32

  windowStart_c(WINDOW_SIZE - 1) :=
    cycles_c(BUF_SIZE).start(NUM_PACKETS - 1) && !lastIsSplit32

  // =========================================================================
  // Advancement 3 — Multi-output priority encoder
  //
  // Find up to DEC_WIDTH instruction starts in windowStart_c simultaneously.
  // Carry-chain: each step finds the lowest remaining set bit, records it,
  // then clears it for the next step using an inverted one-hot mask.
  // =========================================================================

  val pencIdx      = Wire(Vec(DEC_WIDTH, UInt(IDX_W.W)))
  val pencIdxValid = Wire(Vec(DEC_WIDTH, Bool()))

  var remaining: chisel3.UInt = windowStart_c.asUInt

  for (i <- 0 until DEC_WIDTH) {
    pencIdxValid(i) := remaining.orR
    pencIdx(i)      := PriorityEncoder(remaining)
    val mask         = UIntToOH(pencIdx(i), WINDOW_SIZE)
    remaining        = remaining & ~mask
  }

  // =========================================================================
  // Advancement 4 — unhandled_c + canShift
  //
  // unhandled_c(k): slot k has a start that has not been dispatched yet.
  // True when output is not ready, or k is beyond the last dispatched start.
  //
  // canShift: safe to accept a new packet when all starts in the oldest
  // buffered cycle's slots are handled.
  // =========================================================================

  // outputReady declared here, driven below in Advancement 5.
  val outputReady = Wire(Bool())

  val lastValid_penc = pencIdxValid(DEC_WIDTH - 1)
  val lastPencIdx    = pencIdx(DEC_WIDTH - 1)

  val unhandledBits = Wire(Vec(WINDOW_SIZE, Bool()))
  for (k <- 0 until WINDOW_SIZE) {
    unhandledBits(k) := !outputReady || (
      lastValid_penc && (
        Mux(windowStart32_c(lastPencIdx),
          k.U > lastPencIdx + 1.U,   // 32-bit: occupies idx and idx+1
          k.U > lastPencIdx)
      )
    )
  }

  val unhandled_c: chisel3.UInt =
    unhandledBits.asUInt |
    (middleIsSplit32.asUInt << (BUF_SIZE * NUM_PACKETS - 1)) |
    (lastIsSplit32.asUInt   << (WINDOW_SIZE - 1))

  // canShift: oldest cycle window slots all handled.
  val oldestUnhandled = unhandled_c(NUM_PACKETS - 1, 0) &
                        windowStart_c.asUInt(NUM_PACKETS - 1, 0)
  val canShift   = !oldestUnhandled.orR
  val validCycle = io.accept && io.inPkt.valid && canShift

  io.ready := validCycle

  // Shift prev_r on a valid cycle; mask out handled starts before storing.
  when(io.clear) {
    for (i <- 0 until BUF_SIZE) {
      prev_r(i) := 0.U.asTypeOf(new FetchCycle(NUM_PACKETS))
    }
  }.elsewhen(validCycle) {
    for (i <- BUF_SIZE - 1 to 1 by -1) {
      prev_r(i) := prev_r(i - 1)
    }
    val maskedStart = cur_c.start &
      unhandled_c(WINDOW_SIZE - 1, BUF_SIZE * NUM_PACKETS)
    prev_r(0).op      := cur_c.op
    prev_r(0).start   := maskedStart
    prev_r(0).start32 := cur_c.start32
  }.otherwise {
    // No new packet: update start bitmaps to mask out handled starts.
    for (i <- 0 until BUF_SIZE) {
      prev_r(i).start := prev_r(i).start &
        unhandled_c((i + 1) * NUM_PACKETS - 1, i * NUM_PACKETS)
    }
  }

  // =========================================================================
  // Advancement 6 — Full PD_Instr population
  //
  // For each priority-encoder result, build a PD_Instr from window_c and
  // the per-cycle metadata (predTarget, predTaken, predPos, fault) in
  // cycles_c.  The FetchCycle that contains the instruction's last word is
  // used for all fetch-level metadata fields.
  // =========================================================================

  val instr_c = Wire(Vec(DEC_WIDTH, new PD_Instr))
  for (i <- 0 until DEC_WIDTH) {
    instr_c(i) := 0.U.asTypeOf(new PD_Instr)

    when(pencIdxValid(i)) {
      val idx      = pencIdx(i)
      val is32     = windowStart32_c(idx)
      val idxLast  = idx + is32.asUInt   // last half-word of this instruction

      // Which FetchCycle holds the last word?
      val cycleIdxLast  = idxLast >> log2Ceil(NUM_PACKETS).U
      // Which FetchCycle holds the first word (for PC base)?
      val cycleIdxFirst = idx    >> log2Ceil(NUM_PACKETS).U

      val ifetchOp      = cycles_c(cycleIdxLast).op
      val ifetchOpFirst = cycles_c(cycleIdxFirst).op

      val hw0       = window_c(idx)
      val hw1       = Mux(is32, window_c(idxLast), 0.U(16.W))
      val instrBits = Mux(is32, Cat(hw1, hw0), Cat(0.U(16.W), hw0))

      // PC: base from first-word packet, slot offset from idx.
      val slotInPkt = idx(log2Ceil(NUM_PACKETS) - 1, 0)
      val instrPC   = Cat(
        ifetchOpFirst.pc(31, log2Ceil(NUM_PACKETS) + 1),
        slotInPkt,
        0.U(1.W)
      )

      // predTaken only applies to this instruction if the packet's predicted
      // branch position equals the last word's slot within its packet.
      val slotInPktLast = idxLast(log2Ceil(NUM_PACKETS) - 1, 0)

      instr_c(i).valid          := true.B
      instr_c(i).instr          := instrBits
      instr_c(i).pc             := instrPC
      instr_c(i).fetchID        := ifetchOp.fetchID
      instr_c(i).fetchstart := ifetchOp.pc(log2Ceil(NUM_PACKETS), 1)
      instr_c(i).fetchPredOffs  := ifetchOp.predPos.value
      instr_c(i).predTarget     := ifetchOp.predTarget
      instr_c(i).predTaken      := ifetchOp.predTaken &&
                                   (ifetchOp.predPos.value === slotInPktLast)
      instr_c(i).fetchFault     := ifetchOp.fault
      instr_c(i).is16bit        := !is32
    }
  }

  // =========================================================================
  // Advancement 5 — FF_OUTPUT pipeline register
  //
  // FF_OUTPUT=true  (default): output is registered; outputReady is high
  //   when the register is empty or the decode stage is ready.
  // FF_OUTPUT=false: output is combinational; outputReady == io.outReady.
  // =========================================================================

  if (FF_OUTPUT) {
    val outReg = RegInit(VecInit(Seq.fill(DEC_WIDTH)(0.U.asTypeOf(new PD_Instr))))

    outputReady := !outReg(0).valid || io.outReady

    when(io.clear) {
      for (i <- 0 until DEC_WIDTH) outReg(i) := 0.U.asTypeOf(new PD_Instr)
    }.elsewhen(outputReady) {
      for (i <- 0 until DEC_WIDTH) outReg(i) := instr_c(i)
    }

    io.outInstrs := outReg

  } else {
    outputReady  := io.outReady
    io.outInstrs := instr_c
  }
}
