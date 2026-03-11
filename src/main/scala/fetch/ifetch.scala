package fetch

import chisel3._
import chisel3.util._
import branch_pred.{BranchPredictor, SqN, branchParams}

// iFetch is the top-level fetch unit. It contains the branch predictor,
// the fetch pipeline (ICache + TLB + instruction alignment), a PC file
// that snapshots the PC at every fetchID for misprediction recovery,
// and the wait-for-interrupt / reset-delay logic that gates the entire
// frontend. The branch predictor drives the PC; the fetch pipeline
// consumes it and produces decoded instruction packets for the decode stage.
class iFetch(params: iFetchParams) extends Module {

  val io = IO(new Bundle {
    val en               = Input(Bool())
    val interruptPending = Input(Bool())
    val memBusy          = Input(Bool())
    val rob_currFetchID  = Input(new FetchID)
    val branch           = Input(new BranchProv)
    val decBranch        = Input(new DecoderBranch)
    val clearICache      = Input(Bool())
    val flushTLB         = Input(Bool())
    val btUpdates        = Input(Vec(params.numBPUpd, new BTUpdate))
    val bpUpdate         = Input(new BPUpdate)
    val pcRead           = Input(Vec(params.numBranchPorts, new PCFileReadReq))
    val pcReadData       = Output(Vec(params.numBranchPorts, new PCFileEntry))
    val pcReadTH         = Input(new PCFileReadReqTH)
    val pcReadDataTH     = Output(new PCFileEntry)
    val ready            = Input(Bool())
    val instrs           = Output(Vec(params.decWidth, new PD_Instr))
    val vmem             = Input(new VirtMemState)
    val memc             = Output(new MemController_Req)
    val memcRes          = Input(new MemController_Res)
    val satp             = Input(new SATP)
    val clearPTE         = Input(Bool())
  })

  // ---- Reset delay and WFI gate ----------------------------------
  // The frontend is held idle for RESET_DELAY cycles after reset to
  // allow downstream structures to initialise. It is also gated by
  // WFI (wait-for-interrupt) instructions and by interrupt injection.
  val RESET_DELAY = 16
  val WFI_DELAY   = 4

  val waitForInterrupt = RegInit(true.B)
  val issuedInterrupt  = RegInit(false.B)
  val resetWait        = RegInit(true.B)
  val wfiCount         = RegInit((RESET_DELAY - 1).U(log2Ceil(RESET_DELAY + 1).W))

  // ---- Stall signals ---------------------------------------------
  val BP_stall    = Wire(Bool())
  val icacheStall = Wire(Bool())

  val baseEn   = io.en && !waitForInterrupt && !issuedInterrupt && !BP_stall
  val ifetchEn = baseEn && !icacheStall

  // ---- BP_mispr: misprediction source priority -------------------
  // A ROB-sourced branch misprediction takes priority over a
  // decode-stage branch correction. Both are funnelled into a single
  // FetchBranchProv that the branch predictor uses for recovery.
  val BH_fetchBranch = Wire(new FetchBranchProv)
  val BP_mispr       = WireDefault(BH_fetchBranch)

  when(io.branch.taken) {
    BP_mispr.taken     := io.branch.taken
    BP_mispr.fetchid   := io.branch.fetchid
    BP_mispr.target      := io.branch.dstPC(31, 1)
    BP_mispr.histAct   := io.branch.histAct
    BP_mispr.retAct    := io.branch.retAct
    BP_mispr.fetchoffs := io.branch.fetchoffs
    BP_mispr.tgtspec   := io.branch.tgtspec
    BP_mispr.wfi       := false.B
  }.elsewhen(io.decBranch.taken) {
    BP_mispr.taken     := io.decBranch.taken
    BP_mispr.fetchid   := io.decBranch.fetchid
    BP_mispr.target      := DontCare
    BP_mispr.histAct   := HistAct.HIST_NONE
    BP_mispr.retAct    := RetAct.RET_NONE
    BP_mispr.fetchoffs := io.decBranch.fetchoff
    BP_mispr.tgtspec   := BranchTgtSpec.BR_TGT_NEXT
    BP_mispr.wfi       := false.B
  }

  // ---- Branch predictor ------------------------------------------
  val pc           = Wire(UInt(31.W))
  val BP_lastOffs  = Wire(new FetchOff)
  val BP_curRetAddr  = Wire(UInt(31.W))
  val BP_lateRetAddr = Wire(UInt(31.W))
  val BP_rIdx      = Wire(new RetStackIdx)
  val predBr       = Wire(new PredBranch)
  val BP_fetchLimit  = Wire(new FetchLimit)
  val BP_pcFileRead  = Wire(new PCFileReadReq)
  val BP_pcFileRData = Wire(new PCFileEntry)
  val BPF_we         = Wire(Bool())
  val BPF_writeAddr  = Wire(new FetchID)
  val BH_btUpdate    = Wire(new BTUpdate)
  val BH_retDecUpd   = Wire(new ReturnDecUpdate)

  val bp = Module(new BranchPredictor(NUM_IN = params.numBPUpd + 1 , new branchParams))

  bp.io.en        := BPF_we
  bp.io.mispr      := BP_mispr
  bp.io.pcValid    := ifetchEn
  bp.io.fetchid    := BPF_writeAddr
  bp.io.comfetchid := io.rob_currFetchID
  bp.io.retDecUpdate := BH_retDecUpd
  bp.io.pcFileRead := BP_pcFileRead
  bp.io.pcFileRData  := BP_pcFileRData
  bp.io.BT_Updates := VecInit(BH_btUpdate +: io.btUpdates.toSeq)
  bp.io.BP_Updates := io.bpUpdate

  BP_stall       := bp.io.stall
  pc             := bp.io.pc
  BP_lastOffs    := bp.io.lastOffs
  BP_curRetAddr  := bp.io.currRetAddr
  BP_lateRetAddr := bp.io.lateRetAddr
  BP_rIdx        := bp.io.rIdx
  predBr         := bp.io.predBr
  BP_fetchLimit  := bp.io.fetchLimit
  BP_pcFileRead  := bp.io.pcFileRead

  // ---- IFetchOp construction -------------------------------------
  // The IFetchOp carries the current PC and any pending fault into
  // the fetch pipeline. If a flush is in progress or ifetch is
  // disabled, the op is invalid and the pipeline drains.
  val ifetchOp = Wire(new IFetchOp)
  ifetchOp      := 0.U.asTypeOf(new IFetchOp)
  ifetchOp.valid := false.B

  when(!io.branch.taken && !BH_fetchBranch.taken && ifetchEn) {
    ifetchOp.valid      := true.B
    ifetchOp.pc         := Cat(pc, 0.U(1.W))
    ifetchOp.fetchFault := Mux(io.interruptPending,
                             IFetchFault.IF_INTERRUPT,
                             IFetchFault.IF_FAULT_NONE)
  }

  // ---- Fetch pipeline (ICache + TLB + aligner) -------------------
  val PCF_writeAddr  = Wire(new FetchID)
  val PCF_writeData  = Wire(new PCFileEntry)
  val pcFileWriteEn  = Wire(Bool())

  val ifp = Module(new IFetchPipeline(params))

  ifp.io.MEM_busy        := io.memBusy
  ifp.io.mispr           := io.branch.taken || io.decBranch.taken
  ifp.io.misprFetchID    := Mux(io.branch.taken,
                              io.branch.fetchid,
                              io.decBranch.fetchid)
  ifp.io.ROB_curFetchID  := io.rob_currFetchID
  ifp.io.BP_fetchLimit   := BP_fetchLimit
  ifp.io.ifetchOp        := ifetchOp
  ifp.io.predBranch      := predBr
  ifp.io.rIdx            := BP_rIdx
  ifp.io.lastValid       := BP_lastOffs
  ifp.io.lateRetAddr     := BP_lateRetAddr
  ifp.io.ready           := io.ready
  ifp.io.clearICache     := io.clearICache
  ifp.io.flushTLB        := io.flushTLB
  ifp.io.vmem            := io.vmem
  ifp.io.pwRes           := io.pwRes
  ifp.io.memcRes         := io.memcRes

  icacheStall    := ifp.io.stall
  BPF_we         := ifp.io.bpFileWE
  BPF_writeAddr  := ifp.io.bpFileAddr
  pcFileWriteEn  := ifp.io.pcFileWE
  PCF_writeAddr  := ifp.io.pcFileAddr
  PCF_writeData  := ifp.io.pcFileEntry
  BH_fetchBranch := ifp.io.fetchBranch
  BH_btUpdate    := ifp.io.btUpdate
  BH_retDecUpd   := ifp.io.retUpdate
  io.instrs      := ifp.io.instrs
  io.pw          := ifp.io.pw
  io.memc        := ifp.io.memc

  val page_walker = Module(new PageWalker(NUM_RQ = 1))

  page_walker.io.in_pwReq(0) := ifp.io.pw

  // Walker receives satp from CSR input
  page_walker.io.satp        := io.satp

  // Walker receives clear signal
  page_walker.io.clear       := io.clearPTE

  // IFetchPipeline receives walk result from walker
  ifp.io.pwRes               := page_walker.io.out_pwRes(0)

  // Walker receives memory responses from outside
  page_walker.io.memc_res    := io.memcRes

  // Walker's memory requests go out — but needs arbitration with ICache
  // For now wire directly, add arbiter when MemController is implemented
  io.memc                    := page_walker.io.memc_req

  // ---- PC file ---------------------------------------------------
  // A synchronous register file that maps fetchID → PCFileEntry.
  // The branch predictor reads it during recovery to find the base
  // PC of the mispredicted fetch block. External branch ports give
  // the ROB and decode stage access to the same table.
  // The TH (thread) read port and port 0 share one physical read
  // port, with TH taking priority when both are valid and TH.prio
  // is set, mirroring the SV sharedReq arbitration.
  val pcFileDepth = 1 << (new FetchID).getWidth
  val pcFile      = SyncReadMem(pcFileDepth, new PCFileEntry)

  when(pcFileWriteEn) {
    pcFile.write(PCF_writeAddr.value, PCF_writeData)
  }

  BP_pcFileRData := pcFile.read(BP_pcFileRead.addr.value, BP_pcFileRead.valid)

  val sharedReq = Mux(
    io.pcReadTH.valid && (io.pcReadTH.prio || !io.pcRead(0).valid),
    io.pcReadTH.asTypeOf(new PCFileReadReq),
    io.pcRead(0)
  )
  val sharedData = pcFile.read(sharedReq.addr.value, sharedReq.valid)
  io.pcReadData(0) := sharedData
  io.pcReadDataTH  := sharedData

  for (i <- 1 until params.numBranchPorts) {
    io.pcReadData(i) := pcFile.read(io.pcRead(i).addr.value, io.pcRead(i).valid)
  }

  // ---- WFI / reset-delay / interrupt sequential logic ------------
  when(waitForInterrupt) {
    val wfiDone = wfiCount === 0.U
    wfiCount := wfiCount - 1.U
    when((io.interruptPending && !resetWait) || wfiDone) {
      waitForInterrupt := false.B
    }
  }

  when(io.branch.taken || BH_fetchBranch.taken || io.decBranch.taken) {
    when(io.branch.taken) {
      waitForInterrupt := false.B
    }.otherwise {
      val wfiFlush = (BH_fetchBranch.taken && BH_fetchBranch.wfi) ||
                     (io.decBranch.taken    && io.decBranch.wfi)
      when(wfiFlush) {
        waitForInterrupt := true.B
        wfiCount         := (WFI_DELAY - 1).U
      }
    }
    issuedInterrupt := false.B
    resetWait       := false.B
  }.elsewhen(ifetchEn && io.interruptPending) {
    issuedInterrupt := true.B
  }
}
