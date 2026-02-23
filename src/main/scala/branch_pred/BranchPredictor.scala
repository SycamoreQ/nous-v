package branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.BranchTgtSpec.BR_TGT_CUR32
import fetch.{BPBackup, BPUpdate, BTUpdate, BranchTgtSpec, FetchBranchProv, FetchID, FetchLimit, FetchOff, PCFileEntry, PCFileReadReq, PredBranch, RecoveryInfo, RetAct, RetStackIdx, ReturnDecUpdate}
import ujson.IndexedValue.True

class BranchPredictor(NUM_IN: Int = 2) {
  val io  = IO(new Bundle{
    val en1 = Input(Bool())
    val stall = Output(Bool())
    val mispr = Input(new FetchBranchProv)
    val pcValid = Input(Bool())
    val recovery = Input(new RecoveryInfo)
    val fetchLimit = Output(new FetchLimit)
    val fetchid = Input(new FetchID)
    val comfetchid = Input(new FetchID)
    val pc = Output(RegInit(0.U(31.W)))
    val lastOffs = Output(new FetchOff)
    val currRetAddr = Output(Wire(UInt(31.W)))
    val lateRetAddr = Output(Wire(UInt(31.W)))
    val rIdx = Output(new RetStackIdx)
    val predBr = Output(new PredBranch)
    val recDetUpdate = Output(new ReturnDecUpdate)
    val pcFileRead = Output(new PCFileReadReq)
    val pcFileRData = Output(new PCFileEntry)
    val pcFileEntry = Output(new PCFileEntry)
    val BT_Updates = Input(Vec(NUM_IN-1, new BTUpdate))
    val BP_Updates = Input(new BPUpdate)
  })

  val ret_stall = io.stall
  val bp_backup = Wire(new BPBackup)

  bp_backup.history := history
  bp_backup.rIdx := io.rIdx
  bp_backup.isRegularBranch := io.predBr.taken
  bp_backup.predTaken := io.predBr.taken
  bp_backup.predOffs := io.predBr.offs
  bp_backup.pred := io.predBr.valid
  bp_backup.tageID := TAGE_tageID
  bp_backup.altPred := TAGE_altPred

  val BP_FileRData = Wire(new BPBackup)
  val BP_FileRAddr = Wire(new FetchID)
  val BP_FileRE = Wire(())

  val fetchIDWidth = UInt(8.W);
  val bpFileDepth = 1 << fetchIDWidth

  val bpFile = SyncReadMem(bpFileDepth, new BPBackup)

  when(io.en1) {
    bpFile.write(io.fetchid.value, bp_backup)
  }

  /// try to find valid branch target update
  val bt_update = WireInit(0.U.asTypeOf(new BTUpdate))
  bt_update.valid := false.B

  for (i <- 0 until NUM_IN) {
    when(io.BT_Updates(i).valid) {
      bt_update := io.BT_Updates(i)
    }
  }

  val pcRegNoInc = RegInit(31.W)
  val recoveredPC = Wire(UInt(31.W))
  val pcFromFile = io.pcFileRData.pc
  // Extract base PC (upper bits) and combine with recovery offset
  recoveredPC := Cat(
    pcFromFile(30, log2Ceil((new FetchOff).getWidth)),
    io.recovery.fetchOffs.value
  )

  switch(io.recovery.tgtSpec) {
    is(BranchTgtSpec.BR_TGT_CUR32) {
      recoveredPC := recoveredPC - 1.U
    }

    is(BranchTgtSpec.BR_TGT_CUR16) {
    }

    is(BranchTgtSpec.BR_TGT_NEXT) {
      recoveredPC := recoveredPC + 1.U
    }

    is(BranchTgtSpec.BR_TGT_MANUAL) {

     }
  }

  val branchAddr = RegInit(0.U(31.W))
  branchAddr := io.pc

  io.predBr := false.B
  io.predBr.target := io.currRetAddr
  io.lastOffs := Fill(io.lastOffs.getWidth, 1.U)

  val ignoredPred = Wire(Bool())
  when (ignoredPred) {
    when(io.recovery.valid && io.recovery.tgtSpec =/= BranchTgtSpec.BR_TGT_MANUAL) {
      io.pc := recoveredPC
    }.elsewhen(BTB_br.valid && BTB_br.btype =/= BT_RETURN) {
      io.predBr := BTB_br
      io.predBr.taken =/= TAGE_taken
      io.predBr.multiple := ~io.predBr.taken && BTB_br.multiple

      when(io.predBr.taken) {
        io.pc := io.predBr.target
        io.lastOffs := io.predBr.offs
      }

      when(io.predBr.multiple && io.predBr.offs =/= io.lastOffs) {
        io.lastOffs := io.predBr.offs
        io.pc := Cat(
          pcRegNoInc(30, log2Ceil((new FetchOff).getWidth)),
          Fill(io.lastOffs.getWidth, 1.U))
      }
    }.elsewhen(BTB_br.valid && BTB_br.btype == BT_RETURN && RET_BR.valid) {
      io.predBr := BTB_br
      io.predBr.taken := true.B
      io.predBr.multiple := false.B
      io.predBr.target := io.currRetAddr
      io.pc := io.predBr.target
      io.lastOffs := io.predBr.offs
    }.otherwise{ // No target found, but we still output the direction prediction.
      io.predBr.valid := true.B
      io.predBr.btype := BT_BRANCH
      io.predBr.dirOnly := true.B
      io.predBr.taken := TAGE_taken
    }
  }

  val btb_br = Wire(new PredBranch)

  val btb = Wire(new BranchTargetBuffer)
  btb.pc_valid := io.pcValid
  btb.pc := io.pc
  btb.branch := btb_br
  btb.bt_update := bt_update

  val tage_taken = Wire(())
  val TAGE_tagid = Wire(new Tageid)
  val TAGE_altPred = Wire(())
  val tage_predictor = Wire(new TagePredictor)
  tage_predictor.predvalid = io.pcValid
  tage_predictor.predAddr = branchAddr
  tage_predictor.predHistory = lookupHistory
  tage_predictor.predTageID = TAGE_tagid
  tage_predictor.altPred = TAGE_altPred
  tage_predictor.predTaken = tage_taken
  tage_predictor.writeValid = bpUpdateActive.valid
  tage_predictor.writeAddr = io.pcFileRData.pc
  tage_predictor.writeHistory = updHistory
  tage_predictor.writeTageId = BP_FileRData.Tageid
  tage_predictor.writeTaken = BP_FileRData.branchTaken
  tage_predictor.writeAltPred = BP_FileRData.altPred
  tage_predictor.writePred = BP_FileRData.predTaken

  val retStack = Module(new ReturnStack)
  retStack.io.clk := clock
  retStack.io.rst := reset

  retStack.io.IN_valid     := io.pcValid
  retStack.io.IN_fetchID   := io.fetchid
  retStack.io.IN_comFetchID:= io.comfetchid
  retStack.io.IN_lastPC    := pcRegNoInc
  retStack.io.IN_branch    := io.predBr
  retStack.io.IN_mispr     := io.mispr
  retStack.io.IN_recoveryIdx := recRIdx
  retStack.io.IN_returnUpd := io.retdecupdate

  val RET_br     = retStack.io.predBr // Placeholder for logic
  val RET_stall  = retStack.io.stall
  val RET_idx    = retStack.io.curIdx
  io.OUT_rIdx    := RET_idx

  // --- 2. Mispredict Recovery Logic (recHistory) ---
  val recHistory = Wire(UInt(BHist_t_LEN.W))
  val tempHistory = Wire(UInt(BHist_t_LEN.W))

  // Start with base history
  tempHistory := BP_FileRData.history

  // Conditional updates (mimicking SV procedural flow)
  val recHistStep1 = Mux(BP_FileRData.pred && io.recovery.fetchOffs > BP_FileRData.predOffs,
    Cat(tempHistory(BHist_t_LEN-2, 0), BP_FileRData.predTaken), tempHistory)

  val recHistStep2 = Mux(io.recovery.histAct === HIST_WRITE_0 || io.recovery.histAct === HIST_WRITE_1,
    Cat(recHistStep1(BHist_t_LEN-2, 0), io.recovery.histAct === HIST_WRITE_1), recHistStep1)

  recHistory := Mux(io.recovery.histAct === HIST_APPEND_1 || io.recovery.histAct === HIST_APPEND_0,
    Cat(recHistStep2(BHist_t_LEN-2, 0), io.recovery.histAct === HIST_APPEND_1), recHistStep2)

  // --- 3. Return Stack Index Recovery (recRIdx) ---
  val recRIdx = Wire(new RetStackIdx)
  recRIdx := BP_FileRData.rIdx
  switch(io.recovery.retAct) {
    is(RetAct.RET_POP)  { recRIdx := BP_FileRData.rIdx - 1.U }
    is(RetAct.RET_PUSH) { recRIdx := BP_FileRData.rIdx + 1.U }
  }

  // --- 4. Global History Updates ---
  val lookupHistory = Wire(UInt(BHist_t_LEN.W))
  lookupHistory := history // Default

  when(io.recovery.valid) {
    lookupHistory := recHistory
  } .elsewhen(io.predBr.valid && io.predBr.btype === BT_BRANCH && !io.predBr.dirOnly) {
    lookupHistory := Cat(history(BHist_t_LEN-2, 0), io.predBr.taken)
  }

  val updHistory = Wire(UInt(BHist_t_LEN.W)) // Placeholder for logic
  updHistory := BP_FileRData.history
  when(BP_FileRAddr.pred && BP_FileRData.isRegularBranch && bpUpdateActive.fetchoffs > BP_FileRData.predOffs) {
    updHistory := Cat(BP_FileRData.history(BHist_t_LEN-2, 0), BP_FileRData.predTaken)
  }

  // --- 5. Branch Update FIFO (Using Chisel Queue) ---
  // FIFO#(bits, depth, pipe, flow)
  val updFIFO = Module(new Queue(new BPUpdate, 4))
  updFIFO.io.enq.valid := io.BP_Updates.valid
  updFIFO.io.enq.bits  := io.BP_Updates

  val bpUpdate = updFIFO.io.deq.bits
  val updFIFO_deq = WireInit(false.B)
  updFIFO.io.deq.ready := updFIFO_deq

  // --- 6. Pipeline Registers & Fetch Limits ---
  val bpUpdateActive = RegInit(0.U.asTypeOf(new BPUpdate))
  when(updFIFO_deq) {
    bpUpdateActive := bpUpdate
  } .otherwise {
    bpUpdateActive.valid := false.B
  }

  io.fetchLimit := DontCare
  io.fetchLimit.valid := false.B
  when(bpUpdate.valid) {
    io.fetchLimit.valid := true.B
    io.fetchLimit.fetchid := bpUpdate.fetchID
  } .elsewhen(io.BP_Updates.valid) {
    io.fetchLimit.valid := true.B
    io.fetchLimit.fetchid := io.BP_Updates.fetchID
  }

  // --- 7. Memory/File Read Control ---
  io.pcFileRead.addr := DontCare
  io.pcFileRead.valid := false.B
  BP_FileRAddr := DontCare
  BP_FileRE := false.B
  updFIFO_deq := false.B

  when(io.mispr.taken) {
    BP_FileRAddr := io.mispr.fetchid
    BP_FileRE := true.B
    when(io.mispr.tgtspec =/= BranchTgtSpec.BR_TGT_MANUAL) {
      io.pcFileRead.addr := io.mispr.fetchid
      io.pcFileRead.valid := true.B
    }
  } .elsewhen(bpUpdate.valid) {
    updFIFO_deq := true.B
    BP_FileRAddr := bpUpdate.fetchID
    BP_FileRE := true.B
    io.pcFileRead.addr := bpUpdate.fetchID
    io.pcFileRead.valid := true.B
  }

  // --- 8. Main State Registers (The Final FF Block) ---
  val pcReg      = RegInit((ENTRY_POINT >> 1).U(31.W))
  val pcRegNoInc = Reg(UInt(31.W))
  val history    = RegInit(0.U(BHist_t_LEN.W))
  val ignorePred = RegInit(true.B)
  val recovery   = RegInit(0.U.asTypeOf(new RecoveryInfo))

  recovery.valid := false.B // Pulse-like behavior

  when(io.pcValid) {
    // Equivalent to {$bits(FetchOff_t)'(1'b0)}
    val fetchOffsBits = (new FetchOff).getWidth
    pcReg := Cat(io.pc(30, fetchOffsBits) + 1.U, 0.U(fetchOffsBits.W))
    pcRegNoInc := io.pc
    ignorePred := false.B
    history := lookupHistory
  } .elsewhen(recovery.valid) {
    history := recHistory
    when(recovery.tgtSpec =/= BranchTgtSpec.BR_TGT_MANUAL) {
      pcReg := recoveredPC
    }
  }

  when(io.mispr.taken) {
    recovery.valid     := true.B
    recovery.tgtSpec   := io.mispr.tgtspec
    recovery.fetchid  := io.mispr.fetchid
    recovery.retAct    := io.mispr.retAct
    recovery.histAct   := io.mispr.histAct
    recovery.fetchOffs := io.mispr.fetchoffs

    pcReg := Mux(io.mispr.tgtspec === BranchTgtSpec.BR_TGT_MANUAL, io.mispr.dst, DontCare)
    ignorePred := true.B
  }
}




