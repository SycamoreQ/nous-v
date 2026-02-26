package branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{FetchID, FetchOff, RetStackIdx}

/*
Return Stack implementation . It predicts where the RET instr goes to or where the stack pointer points
to during branch predictions for function calls. Implements a stack where the return addresses are present and
the algorithm either pops or pushes addresses based on if a call is issued or not.
- Stack pointer: rindex
- Stack - rstack
- Recovery Queue - rrqueue - basically if there is a speculative decision and if we want to undo that speculative decision
 */
class ReturnStack(SIZE: Int, RQSIZE: Int) extends Module {
  val io = IO(new Bundle {
    val stall = Output(Bool())
    val valid = Input(Bool())
    val fetchID = Input(new FetchID)
    val comFetchID = Input(new FetchID)
    val lastPC = Input(UInt(31.W))
    val branch = Input(new PredBranch)
    val curRetAddr = Output(UInt(31.W))
    val lateRetAddr = Output(UInt(31.W))
    val recoveryIdx = Input(new RetStackIdx)
    val mispr = Input(new FetchBranchProv)
    val curIdx = Output(new RetStackIdx)
    val predBr = Output(new PredBranch)
    val returnUpd = Input(new ReturnDecUpdate)
  })

  val rstack = Reg(Vec(SIZE, UInt(31.W)))
  val rstack_dbg = Wire(Vec(SIZE, UInt(32.W)))
  for (i <- 0 until SIZE) {
    rstack_dbg(i) := Cat(rstack(i), 0.U(1.W))
  }


  /// The recovery queue is a circular buffer with an extra bit to distinguish full vs empty
  val qindex_r = RegInit(0.U((log2Ceil(RQSIZE) + 1).W))
  val qindexEnd_r = RegInit(0.U((log2Ceil(RQSIZE) + 1).W))
  val qindex = qindex_r(log2Ceil(RQSIZE) - 1, 0)
  val qindexEnd = qindexEnd_r(log2Ceil(RQSIZE) - 1, 0)
  val rrqueue = Reg(Vec(RQSIZE, new RetRecQEntry))

  val forwardRindex = RegInit(false.B)

  val offsetWidth = log2Ceil((new FetchOff).getWidth)
  val addrToPush = Cat(
    io.lastPC(30, offsetWidth),
    io.branch.offs.value
  ) + 1.U

  val rindexReg = RegInit(0.U.asTypeOf(new RetStackIdx))
  val rindex = Wire(new RetStackIdx)

  rindex := rindexReg
  when(forwardRindex) {
    rindex := io.recoveryIdx
  }.elsewhen(io.branch.valid && io.branch.btype === BranchType.BT_CALL && lastValid) {
    rindex.value := rindexReg.value + 1.U
  }.elsewhen(io.branch.valid && io.branch.btype === BranchType.BT_RETURN && lastValid) {
    rindex.value := rindexReg.value - 1.U
  }

  val curRetAddr = RegInit(0.U(31.W))
  val curIdx = RegInit(0.U.asTypeOf(new RetStackIdx))
  val predBr = RegInit(0.U.asTypeOf(new PredBranch))

  when(io.valid) {
    curIdx := rindex
    // if the call is done ( denoted by BT_CALL) and so we add the current return address (currRetAddr)
    // into the stack using addrToPush
    when(io.mispr.taken) {
      curRetAddr := DontCare
      predBr.dst := DontCare
    }.elsewhen(io.branch.valid && io.branch.btype === BranchType.BT_CALL) {
      curRetAddr := addrToPush
      predBr.dst := addrToPush
    }.otherwise {
      curRetAddr := rstack(rindex.value)
      predBr.dst := rstack(rindex.value)
    }

    predBr.valid := true.B
    predBr.taken := true.B
    predBr.dirOnly := false.B
    predBr.btype := BranchType.BT_RETURN
    predBr.offs := DontCare
    predBr.compr := DontCare
    predBr.multiple := DontCare
  }

  io.curRetAddr := curRetAddr
  io.curIdx := curIdx
  io.predBr := predBr
  io.lateRetAddr := curRetAddr

  val recoveryInProgress = RegInit(false.B)
  val recoveryID = RegInit(0.U.asTypeOf(new FetchID))
  val recoveryBase = RegInit(0.U.asTypeOf(new FetchID))
  val recoveryOffs = RegInit(0.U.asTypeOf(new FetchOff))
  val lastInvalComFetchID = RegInit(0.U.asTypeOf(new FetchID))
  val recoveryOverwOwn = RegInit(false.B)
  val lastValid = RegInit(false.B)
  val postRecSave = RegInit(0.U.asTypeOf(new PostRecSave))

  val queueEmpty = qindex_r === qindexEnd_r
  val queueFull = qindex_r === (qindexEnd_r + RQSIZE.U)

  val queueRel = rrqueue(qindex - 1.U).fetchID.value - recoveryBase.value
  val recRel = recoveryID.value - recoveryBase.value
  val recoveryContinue_c = !queueEmpty &&
    (queueRel > recRel ||
      (queueRel === recRel &&
        Mux(recoveryOverwOwn,
          rrqueue(qindex - 1.U).offs.value >= recoveryOffs.value,
          rrqueue(qindex - 1.U).offs.value > recoveryOffs.value
        )
        )
      )

  io.stall := recoveryInProgress && (recoveryContinue_c || postRecSave.valid)

  forwardRindex := false.B
  lastValid := io.valid

  when(io.mispr.taken) {
    val doPostRecSave = io.mispr.isFetchBranch && io.returnUpd.valid
    val startRecovery = !queueEmpty || doPostRecSave

    forwardRindex := true.B
    recoveryInProgress := startRecovery
    recoveryID := io.mispr.fetchID
    recoveryBase := io.comFetchID
    recoveryOffs := io.mispr.fetchOffs
    recoveryOverwOwn := io.mispr.isFetchBranch && !io.returnUpd.valid
    lastValid := false.B

    postRecSave.valid := false.B
    when(doPostRecSave) {
      postRecSave.valid := true.B
      postRecSave.fetchID := io.mispr.fetchID
      postRecSave.offs := io.mispr.fetchOffs
      postRecSave.rIdx.value := io.returnUpd.idx.value + 1.U
      postRecSave.addr := io.returnUpd.addr + 1.U
    }
  }.otherwise {
    rindexReg := rindex

    when(recoveryInProgress) {
      when(recoveryContinue_c) {
        rstack(rrqueue(qindex - 1.U).idx.value) := rrqueue(qindex - 1.U).addr
        qindex_r := qindex_r - 1.U
      }.otherwise {
        recoveryInProgress := false.B
        when(postRecSave.valid) {
          postRecSave.valid := false.B
          when(!queueFull) {
            rrqueue(qindex).fetchID := postRecSave.fetchID
            rrqueue(qindex).offs := postRecSave.offs
            rrqueue(qindex).idx := postRecSave.rIdx
            rrqueue(qindex).addr := rstack(postRecSave.rIdx.value)
            rstack(postRecSave.rIdx.value) := postRecSave.addr
            qindex_r := qindex_r + 1.U
          }
        }
      }
    }.elsewhen(lastValid) {
      when(io.branch.valid && io.branch.btype === BranchType.BT_CALL) {
        rstack(rindex.value) := addrToPush
        when(!queueFull) {
          rrqueue(qindex).fetchID := io.fetchID
          rrqueue(qindex).offs := io.branch.offs
          rrqueue(qindex).idx := rindex
          rrqueue(qindex).addr := rstack(rindex.value)
          qindex_r := qindex_r + 1.U
        }
      }
    }
  }

  when(lastInvalComFetchID.value =/= io.comFetchID.value) {
    when(!queueEmpty &&
      (rrqueue(qindexEnd).fetchID.value - lastInvalComFetchID.value) <
        (io.comFetchID.value - lastInvalComFetchID.value)) {
      lastInvalComFetchID := rrqueue(qindexEnd).fetchID
      qindexEnd_r := qindexEnd_r + 1.U
    }.otherwise {
      lastInvalComFetchID := io.comFetchID
    }
  }
}