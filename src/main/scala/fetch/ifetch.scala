package fetch;

import chisel3._
import chisel3.util._
import common._
import _root_.circt.stage.ChiselStage
import iFetchParams._;

class iFetch(params: iFetchParams) extends Module {
  val io = IO( new Bundle {
    val en = Input(Bool())
    val interruptPending = Input(Bool())
    val memBusy = Input(Bool())
    val rob_currfetchid = Input(new FetchID)
    val branch = Input(new BranchProv)
    val decBranch = Input(new DecoderBranch) 
    val clearCache = Input(Bool())
    val clearTlb = Input(Bool())
    val btUpdates = Input(Vec(params.numBPUpd, new BTUpdate))
    val bpUpdate =Input(new BPUpdate) 
    val pcRead = Input(Vec(params.numBranchPorts , new PCFileReadReq))
    val pcReadData = Output(Vec(params.numBranchPorts, new PCFileEntry))
    val pcReadTH = Input(new PCFileReadReqTH)
    val pcReadDataTH = Output(new PCFileEntry)
    val ready = Input(Bool())
    val instrs = Output(Vec(params.decWidth, new PD_Instr))
    val vmem = Input(new VirtMemState)
    val pw = new PageWalk_Req
    val pwRes = Flipped(new PageWalk_Res)
    val memc = new MemController_Req
    val memcRes = Flipped(new MemController_Res)
  } )
  
  val pc = Wire(UInt(31.W))
  val pcFull = Cat(pc, 0.U(1.W))

  val Bpf_we = Wire(UInt(31.W))
  val fetchoff_t = new FetchOff
  val predBranch = new PredBranch
  val BP_stall = Wire(Bool())
  val currRetAddr = Wire(31.W)
  val BP_rIdx = new RetStackIdx

  val BP_PcFileRead = new PCFileReadReq
  val BP_PcFileRData = new PCFileEntry

  val BH_fetchBranch = Wire(new FetchBranchProv)
  val BH_btUpdate = Wire(new BTUpdate)
  val BH_retDecUpd = Wire(new ReturnDecUpdate)

  val branch_misprov = new FetchBranchProv
  val BP_mispr = WireDefault(BH_fetchBranch)

   when(io.branch.taken) {
     BP_mispr.taken := io.branch.taken
     BP_mispr.fetchid := io.branch.fetchid
     BP_mispr.dst := io.branch.dstPC(31, 1)
     BP_mispr.histAct := io.branch.histAct
     BP_mispr.retAct := io.branch.retAct
     BP_mispr.fetchoffs := io.branch.fetchoffs
     BP_mispr.tgtspec := io.branch.tgtspec
     BP_mispr.wfi := false.B
   }.elsewhen(io.decBranch.taken) {
     BP_mispr.taken := io.decBranch.taken
     BP_mispr.fetchid := io.decBranch.fetchid
     BP_mispr.dst := DontCare
     BP_mispr.histAct := HistAct.HIST_NONE
     BP_mispr.retAct := RetAct.RET_NONE
     BP_mispr.fetchoffs := io.decBranch.fetchoff
     BP_mispr.tgtspec := BranchTgtSpec.BR_TGT_NEXT
     BP_mispr.wfi := false.B
   }

  val BP_lateRetAddr = Wire(UInt(31.W))
  val BP_fetchLimit = Wire(new FetchLimit)

  val bp = Module(new BranchPredictor(params.numBPUpd + 1))

  bp.io.en := BPF_we
  bp.io.mispr := BP_mispr
  bp.io.pcValid := ifetchEn
  bp.io.fetchID := BPF_writeAddr
  bp.io.comFetchID := io.ROB_curFetchID
  bp.io.retDecUpd := BH_retDecUpd
  bp.io.pcFileRead := BP_pcFileRead
  bp.io.pcFileRData := BP_pcFileRData
  bp.io.btUpdates := VecInit(BH_btUpdate +: io.btUpdates.toSeq)
  bp.io.bpUpdate := io.bpUpdate
  BP_stall := bp.io.stall
  pc := bp.io.pc
  BP_lastOffs := bp.io.lastOffs
  BP_curRetAddr := bp.io.curRetAddr
  BP_lateRetAddr := bp.io.lateRetAddr
  BP_rIdx := bp.io.rIdx
  predBr := bp.io.predBr
  BP_fetchLimit := bp.io.fetchLimit

  val waitForInterrupt = RegInit(true.B)
  val issuedInterrupt = RegInit(false.B)
  val baseEn = io.en && !waitForInterrupt && !issuedInterrupt && !BP_stall
  val icacheStall = Wire(Bool())
  val ifetchEn = baseEn && !icacheStall

  val PCF_writeAddr = Wire(new FetchID)
  val PCF_writeData = Wire(new PCFileEntry)
  val pcFileWriteEn = Wire(Bool())
  val ifp = Module(new IFetchPipeline(params))
  ifp.io.MEM_busy := io.MEM_busy
  ifp.io.mispr := io.branch.taken || io.decBranch.taken
  ifp.io.misprFetchID := Mux(io.branch.taken, io.branch.fetchID, io.decBranch.fetchID)
  ifp.io.ROB_curFetchID := io.ROB_curFetchID
  ifp.io.BP_fetchLimit := BP_fetchLimit
  ifp.io.ifetchOp := ifetchOp
  ifp.io.predBranch := predBr
  ifp.io.rIdx := BP_rIdx
  ifp.io.lastValid := BP_lastOffs
  ifp.io.lateRetAddr := BP_lateRetAddr
  ifp.io.ready := io.ready
  ifp.io.clearICache := io.clearICache
  ifp.io.flushTLB := io.flushTLB
  ifp.io.vmem := io.vmem
  ifp.io.pwRes <> io.pwRes
  ifp.io.memcRes <> io.memcRes
  icacheStall := ifp.io.stall
  BPF_we := ifp.io.bpFileWE
  BPF_writeAddr := ifp.io.bpFileAddr
  pcFileWriteEn := ifp.io.pcFileWE
  PCF_writeAddr := ifp.io.pcFileAddr
  PCF_writeData := ifp.io.pcFileEntry
  BH_fetchBranch := ifp.io.fetchBranch
  BH_btUpdate := ifp.io.btUpdate
  BH_retDecUpd := ifp.io.retUpdate
  io.instrs := ifp.io.instrs
  io.pw <> ifp.io.pw
  io.memc <> ifp.io.memc
  ifp.io.icache <> io.icache
  ifp.io.ict <> io.ict

