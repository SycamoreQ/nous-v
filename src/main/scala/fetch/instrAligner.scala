package fetch

import chisel3._
import chisel3.util._

class IF_Packet(NUM_HALFWORDS: Int) extends Bundle {
  val valid      = Bool()
  val pc         = UInt(32.W)
  val fetchID    = new FetchID
  val firstValid = new FetchOff      // index of first live half-word
  val lastValid  = new FetchOff      // index of last live half-word (inclusive)
  val fault      = IFetchFault()
  val halfWords  = Vec(NUM_HALFWORDS, UInt(16.W))
}

// FetchCycle pairs a full IF_Packet with two NUM_PACKETS-wide bitmaps.
// start(i)   — half-word slot i is an instruction start
// start32(i) — that start is the lower half of a 32-bit instruction
class FetchCycle(NUM_PACKETS: Int) extends Bundle {
  val op      = new IF_Packet(NUM_PACKETS)
  val start   = UInt(NUM_PACKETS.W)
  val start32 = UInt(NUM_PACKETS.W)
}

class InstrAligner(
  NUM_PACKETS : Int = 8,
  DEC_WIDTH   : Int = 4,
  BUF_SIZE    : Int = 2
) extends Module {

  val io = IO(new Bundle {
    val clear    = Input(Bool())
    val accept   = Input(Bool())
    val ready    = Output(Bool())

    val inPkt    = Input(new IF_Packet(NUM_PACKETS))

    val outReady  = Input(Bool())
    val outInstrs = Output(Vec(DEC_WIDTH, new PD_Instr))
  })

  // ---- prev_r: ring of BUF_SIZE previous FetchCycles ---------
  // prev_r(0) is the most recent previous cycle.
  // prev_r(BUF_SIZE-1) is the oldest.
  // On reset every entry has start = 0 so the first incoming
  // packet is never treated as a continuation of a split 32-bit
  // instruction.
  val prev_r = RegInit(VecInit(Seq.fill(BUF_SIZE)(0.U.asTypeOf(new FetchCycle(NUM_PACKETS)))))

  // ---- isInstrStart bitmap computation -----------------------
  // Walk the incoming packet's half-words from 0 to NUM_PACKETS-1.
  // Carry a Scala var (elaboration-time boolean) that tracks
  // whether the current half-word slot is a valid instruction start.
  //
  // The initial value of validInstrStart depends on whether the
  // last slot of the oldest buffered cycle was the lower half of
  // a split 32-bit instruction. If it was, the first slot of the
  // incoming packet is consumed by that upper half, not a new start.
  val isInstrStart_w   = Wire(UInt(NUM_PACKETS.W))
  val isInstrStart32_w = Wire(UInt(NUM_PACKETS.W))

  // Scala mutable arrays to collect per-slot Bool wires before
  // packing them into UInt bitmaps.
  val startBits   = Wire(Vec(NUM_PACKETS, Bool()))
  val start32Bits = Wire(Vec(NUM_PACKETS, Bool()))
  for (i <- 0 until NUM_PACKETS) {
    startBits(i)   := false.B
    start32Bits(i) := false.B
  }

  // The oldest buffered entry's last slot tells us whether we are
  // in the middle of a split 32-bit instruction entering this cycle.
  val prevSplit32 = prev_r(BUF_SIZE - 1).start32(NUM_PACKETS - 1)

  // validInstrStart is a Scala var — it is not a Chisel register.
  // It exists only at elaboration time to thread the carry
  // condition through the unrolled for loop.
  // It starts as true unless the previous packet ended mid-32-bit.
  var validInstrStart: chisel3.Bool = !prevSplit32

  for (i <- 0 until NUM_PACKETS) {
    // Outside firstValid..lastValid window: suppress starts.
    // These comparisons are hardware — i is a Scala constant so
    // they elaborate to constant-select muxes.
    val inWindow = io.inPkt.valid &&
                   (i.U >= io.inPkt.firstValid.value) &&
                   (i.U <= io.inPkt.lastValid.value)

    // Gate the carried validInstrStart by the window check.
    val canStart = validInstrStart && inWindow

    val hw        = io.inPkt.halfWords(i)
    val is32bit_w = hw(1, 0) === 3.U    // RISC-V: bits[1:0]==11 means 32-bit

    startBits(i)   := canStart
    start32Bits(i) := canStart && is32bit_w

    // Advance the carry:
    //   - If we just marked a 32-bit start, the next slot is the
    //     upper half of that instruction so it is NOT a new start.
    //     The slot after that IS a new start again (handled when
    //     validInstrStart becomes true on the following iteration).
    //   - If we just marked a compressed start, the next slot IS
    //     a new start.
    //   - If we were outside the window, the next slot's
    //     validInstrStart resets to true (the window suppression
    //     is re-applied at the top of the next iteration via inWindow).
    validInstrStart = Mux(canStart && is32bit_w,
                        false.B,           // upper half consumes next slot
                        Mux(canStart,
                          true.B,          // compressed: next slot is free
                          true.B))         // outside window: default true,
                                           // inWindow gates it next iteration
  }

  isInstrStart_w   := startBits.asUInt
  isInstrStart32_w := start32Bits.asUInt

  // ---- cur_c: current FetchCycle built from incoming packet --
  val cur_c = Wire(new FetchCycle(NUM_PACKETS))
  cur_c.op      := io.inPkt
  cur_c.start   := isInstrStart_w
  cur_c.start32 := isInstrStart32_w

  // ---- Advancement 2: Window Assembly -----------------------
  // Combine prev_r and cur_c into a single flat window of
  // WINDOW_SIZE half-word slots.
  //
  // cycles_c(0)           = prev_r(BUF_SIZE-1)  oldest buffered cycle
  // cycles_c(BUF_SIZE-1)  = prev_r(0)           most recent previous cycle
  // cycles_c(BUF_SIZE)    = cur_c               current incoming cycle
  //
  // Window layout (slot 0 = oldest half-word):
  //   slots [0            .. NUM_PACKETS-1]   from cycles_c(0)
  //   slots [NUM_PACKETS  .. 2*NUM_PACKETS-1] from cycles_c(1)
  //   ...
  //   slots [BUF_SIZE*NUM_PACKETS .. WINDOW_SIZE-1] from cur_c

  val WINDOW_SIZE = (BUF_SIZE + 1) * NUM_PACKETS

  // cycles_c: combinational Vec assembling prev_r oldest-first then cur_c
  val cycles_c = Wire(Vec(BUF_SIZE + 1, new FetchCycle(NUM_PACKETS)))
  for (i <- 0 until BUF_SIZE) {
    cycles_c(i) := prev_r(BUF_SIZE - 1 - i)   // oldest at index 0
  }
  cycles_c(BUF_SIZE) := cur_c

  // Three flat arrays across the full window
  val window_c        = Wire(Vec(WINDOW_SIZE, UInt(16.W)))
  val windowStart_c   = Wire(Vec(WINDOW_SIZE, Bool()))
  val windowStart32_c = Wire(Vec(WINDOW_SIZE, Bool()))

  // Base assembly — straight copy from cycles_c
  for (i <- 0 to BUF_SIZE) {
    for (j <- 0 until NUM_PACKETS) {
      window_c       (i * NUM_PACKETS + j) := cycles_c(i).op.halfWords(j)
      windowStart_c  (i * NUM_PACKETS + j) := cycles_c(i).start  (j)
      windowStart32_c(i * NUM_PACKETS + j) := cycles_c(i).start32(j)
    }
  }

  // Split-32 corrections applied after base assembly.
  //
  // middleIsSplit32: the oldest buffered cycle's last slot was the
  // lower half of a 32-bit instruction AND no new packet arrived
  // to supply the upper half.  Suppress that start bit.
  //
  // lastIsSplit32: the current incoming cycle's last slot is the
  // lower half of a 32-bit instruction whose upper half has not
  // arrived yet.  Suppress that start bit.
  val middleIsSplit32 = prev_r(BUF_SIZE - 1).start32(NUM_PACKETS - 1) && !io.inPkt.valid
  val lastIsSplit32   = isInstrStart32_w(NUM_PACKETS - 1)

  // Override the two affected slots.  These override the values
  // written in the loop above for those specific indices.
  windowStart_c(BUF_SIZE * NUM_PACKETS - 1) :=
    cycles_c(BUF_SIZE - 1).start(NUM_PACKETS - 1) && !middleIsSplit32

  windowStart_c(WINDOW_SIZE - 1) :=
    cycles_c(BUF_SIZE).start(NUM_PACKETS - 1) && !lastIsSplit32

  // ---- canShift / validCycle ---------------------------------
  // canShift: no unhandled instruction starts remain in the
  // oldest buffered slot's position — safe to shift in a new cycle.
  // For Advancement 1 this is always true; Advancement 4 will
  // replace this with the unhandled_c check.
  val canShift   = true.B
  val validCycle = io.accept && io.inPkt.valid && canShift

  io.ready := validCycle

  // ---- Shift prev_r on a valid cycle -------------------------
  // Shift from oldest to newest so prev_r(0) always holds the
  // most recent previous cycle.
  when(io.clear) {
    for (i <- 0 until BUF_SIZE) {
      prev_r(i) := 0.U.asTypeOf(new FetchCycle(NUM_PACKETS))
    }
  }.elsewhen(validCycle) {
    for (i <- BUF_SIZE - 1 to 1 by -1) {
      prev_r(i) := prev_r(i - 1)
    }
    prev_r(0) := cur_c
  }

  // ---- Carry register ----------------------------------------
  // When a 32-bit instruction straddles two fetch packets, the
  // lower 16 bits arrive in packet N and the upper 16 bits
  // arrive in packet N+1.  We save the lower half here.
  val carryValid = RegInit(false.B)
  val carryBits  = RegInit(0.U(16.W))
  val carryPC    = RegInit(0.U(32.W))
  val carryID    = RegInit(0.U.asTypeOf(new FetchID))

  // ---- Internal packet buffer --------------------------------
  // We accept one IF_Packet per cycle (when ready) and hold it
  // while draining DEC_WIDTH instructions per cycle.
  val bufValid  = RegInit(false.B)
  val buf       = RegInit(0.U.asTypeOf(new IF_Packet(NUM_PACKETS)))
  val drainPtr  = RegInit(0.U(log2Ceil(NUM_PACKETS + 1).W))

  when(io.clear) {
    bufValid   := false.B
    carryValid := false.B
    drainPtr   := 0.U
  }.elsewhen(io.inPkt.valid && io.ready && io.accept) {
    bufValid := true.B
    buf      := io.inPkt
    drainPtr := io.inPkt.firstValid.value
  }.elsewhen(io.outReady && drainPtr > buf.lastValid.value) {
    bufValid := false.B
  }

  // ---- Combinational decode of up to DEC_WIDTH instrs --------
  // We walk the half-word array starting at drainPtr and emit
  // up to DEC_WIDTH instructions into io.outInstrs.
  val outWires = Wire(Vec(DEC_WIDTH, new PD_Instr))
  for (i <- 0 until DEC_WIDTH) {
    outWires(i) := 0.U.asTypeOf(new PD_Instr)
  }

  // Slot pointer: tracks which half-word we are examining for
  // each output slot.  We use a carry-chain approach in Chisel
  // by computing slot pointers as a running sum.
  //
  // slotPtr(i) = half-word index for output slot i
  // slotAdv(i) = 1 if slot i is a 16-bit instr, 2 if 32-bit
  val slotPtr = Wire(Vec(DEC_WIDTH + 1, UInt(log2Ceil(NUM_PACKETS + 2).W)))
  val slotAdv = Wire(Vec(DEC_WIDTH, UInt(2.W)))

  slotPtr(0) := drainPtr

  for (i <- 0 until DEC_WIDTH) {
    val ptr   = slotPtr(i)
    val inBuf = bufValid && ptr <= buf.lastValid.value
    val fault = buf.fault =/= IFetchFault.IF_FAULT_NONE

    // Half-words at ptr and ptr+1 (guard against out-of-range)
    val hw0 = Mux(ptr < NUM_PACKETS.U,
                buf.halfWords(ptr(log2Ceil(NUM_PACKETS) - 1, 0)), 0.U)
    val hw1 = Mux(ptr + 1.U < NUM_PACKETS.U,
                buf.halfWords((ptr + 1.U)(log2Ceil(NUM_PACKETS) - 1, 0)), 0.U)

    // Is this a compressed instruction?
    // RISC-V compressed: bits[1:0] != 2'b11
    val isCarry      = carryValid && (i.U === 0.U)
    val rawBits      = Mux(isCarry, Cat(hw0, carryBits), Cat(hw1, hw0))
    val isCompressed = Mux(isCarry,
      false.B,                       // carry is always upper half of 32-bit
      hw0(1, 0) =/= 3.U
    )

    slotAdv(i)     := Mux(isCarry || !isCompressed, 2.U, 1.U)
    slotPtr(i + 1) := slotPtr(i) + slotAdv(i)

    // PC for this instruction
    // Each half-word slot is 2 bytes so pc = basePC + ptr*2
    val instrPC = Mux(isCarry,
      carryPC,
      buf.pc(31, log2Ceil(NUM_PACKETS) + 1) ##
        (ptr(log2Ceil(NUM_PACKETS) - 1, 0) ## 0.U(1.W))
    )

    outWires(i).valid      := inBuf || (isCarry && i.U === 0.U)
    outWires(i).bits       := Mux(fault, 0.U,
                                Mux(isCompressed,
                                  Cat(0.U(16.W), hw0),
                                  rawBits))
    outWires(i).pc         := instrPC
    outWires(i).fetchID    := Mux(isCarry, carryID, buf.fetchID)
    outWires(i).compressed := isCompressed
    outWires(i).fault      := Mux(inBuf || isCarry, buf.fault,
                                IFetchFault.IF_FAULT_NONE)
  }

  io.outInstrs := outWires

  // ---- Advance drain pointer ---------------------------------
  when(!io.clear && bufValid && io.outReady) {
    // Advance by the number of half-words consumed this cycle.
    // Count only the valid output slots.
    val consumed = slotAdv.zipWithIndex.map { case (adv, i) =>
      Mux(outWires(i).valid, adv, 0.U)
    }.reduce(_ +& _)

    drainPtr := drainPtr + consumed

    // Update carry: if the last valid slot ends at lastValid and
    // the last half-word of a 32-bit instruction is out of range,
    // save the lower half for the next packet.
    val lastSlotPtr = slotPtr(DEC_WIDTH)
    val lastHW      = lastSlotPtr - 1.U

    when(lastHW === buf.lastValid.value &&
         !outWires(DEC_WIDTH - 1).compressed &&
         buf.halfWords(buf.lastValid.value)(1, 0) === 3.U) {
      carryValid := true.B
      carryBits  := buf.halfWords(buf.lastValid.value)
      carryPC    := buf.pc(31, log2Ceil(NUM_PACKETS) + 1) ##
                    (buf.lastValid.value ## 0.U(1.W))
      carryID    := buf.fetchID
    }.otherwise {
      carryValid := false.B
    }
  }

  when(io.clear) {
    drainPtr   := 0.U
    carryValid := false.B
    bufValid   := false.B
  }
}
