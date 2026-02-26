package  branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{BTUpdate, FetchOff, PredBranch}
import branch_pred._

class BranchTargetBuffer(btbEntries: Int, btbTagSize: Int) extends Module {
  val io = IO(new Bundle {
    val pcValid = Input(Bool())
    val pc = Input(UInt(31.W))
    val branch = Output(new PredBranch)
    val btUpdate = Input(new BTUpdate)
  })

  val LENGTH = btbEntries
  val idxWidth = log2Ceil(LENGTH)
  val offsetWidth = (new FetchOff).getWidth

  val entries = SyncReadMem(LENGTH, new BTBEntry(btbTagSize))
  val multiple = SyncReadMem(LENGTH, Bool())

  val fetched = RegInit(0.U.asTypeOf(new FetchedBundle(btbTagSize)))

  when(io.pcValid) {
    fetched.entry := entries.read(io.pc(idxWidth - 1, 0))
    fetched.multiple := multiple.read(io.pc(idxWidth - 1, 0))
    fetched.pc := io.pc
  }

  val tagMatch = fetched.entry.src === fetched.pc(idxWidth + btbTagSize - 1, idxWidth)
  val offsValid = fetched.entry.offs.value >= fetched.pc(offsetWidth - 1, 0)

  io.branch := 0.U.asTypeOf(new PredBranch)
  when(fetched.entry.valid && tagMatch && offsValid) {
    io.branch.valid := true.B
    io.branch.multiple := fetched.multiple
    io.branch.target := fetched.entry.dst
    io.branch.btype := fetched.entry.btype
    io.branch. := fetched.entry.compr
    io.branch.offs := fetched.entry.offs
    io.branch.taken := (fetched.entry.btype === BranchType.BT_CALL) ||
      (fetched.entry.btype === BranchType.BT_JUMP)
    io.branch.dirOnly := false.B
  }

  val setMult = RegInit(0.U.asTypeOf(new SetMultiple(idxWidth)))
  val resetIdx = RegInit(0.U((idxWidth + 1).W))
  val resetDone = resetIdx(idxWidth)

  when(!resetDone) {
    multiple.write(resetIdx(idxWidth - 1, 0), false.B)
    entries.write(resetIdx(idxWidth - 1, 0), 0.U.asTypeOf(new BTBEntry(btbTagSize)))
    resetIdx := resetIdx + 1.U
  }.otherwise {
    when(io.btUpdate.valid) {
      val baseIdx = Cat(
        io.btUpdate.(idxWidth + offsetWidth, offsetWidth + 1),
        io.btUpdate.fetchStartOffs.value
      )
      var idx = WireDefault(baseIdx)

      when(io.btUpdate.clean) {
        val cleanEntry = Wire(new BTBEntry(btbTagSize))
        cleanEntry := DontCare
        cleanEntry.valid := false.B
        entries.write(idx, cleanEntry)
      }.otherwise {
        when(io.btUpdate.multiple) {
          setMult.valid := true.B
          setMult.idx := idx
          idx = Cat(idx(idxWidth - 1, offsetWidth), io.btUpdate.multipleOffs.value)
        }

        val newEntry = Wire(new BTBEntry(btbTagSize))
        newEntry.valid := true.B
        newEntry.compr := io.btUpdate.compr
        newEntry.btype := io.btUpdate.btype
        newEntry.dst := io.btUpdate.target(31, 1)
        newEntry.src := io.btUpdate.source(idxWidth + btbTagSize, idxWidth + 1)
        newEntry.offs.value := io.btUpdate.source(offsetWidth, 1)

        entries.write(idx, newEntry)
        multiple.write(idx, false.B)
      }
    }.otherwise {
      when(setMult.valid) {
        multiple.write(setMult.idx, true.B)
        setMult.valid := false.B
        setMult.idx := DontCare
      }
    }
  }
}

