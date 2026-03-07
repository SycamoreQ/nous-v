package fetch

/*
On advancement 3 - VIPT Addressing + I-TLB

*/

import chisel3._
import chisel3.util._
import branch_pred._
import branch_pred.BranchType._
import cache.ICache


class IFetchPipeline(params: iFetchParams) extends Module {

  val NUM_INST      = 1 << (params.FSIZE_E - 1)
  val FIFO_SIZE     = 4
  val VIRT_IDX_LEN  = params.VIRT_IDX_LEN
  val CLSIZE_E     = params.CLSIZE_E
  val CACHE_SIZE_E = params.CACHE_SIZE_E
  val CASSOC       = params.CASSOC
  val ITLB_SIZE    = params.ITLB_SIZE
  val ITLB_ASSOC   = params.ITLB_ASSOC
  val LINES        = 1 << (params.CACHE_SIZE_E - log2Ceil(params.CASSOC) - 2)

  val io = IO(new Bundle {
    val MEM_busy          = Input(Bool())

    val mispr             = Input(Bool())
    val misprFetchID      = Input(new FetchID)

    val ROB_curFetchID    = Input(new FetchID)
    val BP_fetchLimit     = Input(new FetchLimit)

    val ifetchOp          = Input(new IFetchOp)
    val stall             = Output(Bool())

    val predBranch        = Input(new PredBranch)
    val rIdx              = Input(new RetStackIdx)
    val lastValid         = Input(new FetchOff)

    val bpFileWE          = Output(Bool())
    val bpFileAddr        = Output(new FetchID)

    val pcFileWE          = Output(Bool())
    val pcFileAddr        = Output(new FetchID)
    val pcFileEntry       = Output(new PCFileEntry)

    val fetchBranch       = Output(new FetchBranchProv)
    val btUpdate          = Output(new BTUpdate)
    val retUpdate         = Output(new ReturnDecUpdate)

    val lateRetAddr       = Input(UInt(31.W))

    val ictRe             = Output(Bool())
    val ictRaddr          = Output(UInt(VIRT_IDX_LEN.W))
    val ictRdata          = Input(Vec(CASSOC, new ICTEntry))

    val ictWe             = Output(Bool())
    val ictWaddr          = Output(UInt(VIRT_IDX_LEN.W))
    val ictWassoc         = Output(UInt(log2Ceil(CASSOC).W))
    val ictWdata          = Output(new ICTEntry)

    val ready             = Input(Bool())
    val instrs            = Output(Vec(params.decWidth, new PD_Instr))

    val clearICache       = Input(Bool())
    val flushTLB          = Input(Bool())
    val vmem              = Input(new VirtMemState)

    val pw                = Output(new PageWalk_Req)
    val pwRes             = Input(new PageWalk_Res)

    val memc              = Output(new MemController_Req)
    val memcRes           = Input(new MemController_Res)
  })

  val icache = Module(new ICache(
    BSIZE    = 4,
    LINES    = LINES,
    MEM_SIZE = 1 << params.CACHE_SIZE_E
  ))

  //  Pipeline registers
  //  fetch0: after ICache read issued (1st cycle)
  //  fetch1: tag check + TLB (2nd cycle)
  val fetchID   = RegInit(0.U.asTypeOf(new FetchID))
  val assocCnt  = RegInit(0.U(log2Ceil(CASSOC).W))

  val fetch0    = RegInit(0.U.asTypeOf(new IFetchOp))
  val fetch1    = RegInit(0.U.asTypeOf(new IFetchOp))

  //  BranchHandler
  val bh = Module(new BranchHandler(NUM_INST))
  bh.io.clear  := io.mispr
  bh.io.accept := false.B       // driven below once packet is known
  bh.io.op     := fetch1
  bh.io.instrs := DontCare      // driven from assocHitInstrs below

  val BH_endOffsValid = bh.io.endOffsValid
  val BH_endOffs      = bh.io.endOffs
  val BH_newPredTaken = bh.io.newPredTaken
  val BH_newPredPos   = bh.io.newPredPos
  val BH_decBranch    = bh.io.decBranch

  //  Flush state machine  (mirrors SV FlushState enum)
  val sFlushIdle     = 0.U(2.W)
  val sFlushQueued   = 1.U(2.W)
  val sFlushActive   = 2.U(2.W)
  val sFlushFinalize = 3.U(2.W)

  val flushState     = RegInit(sFlushIdle)
  val flushAssocIter = RegInit(0.U(log2Ceil(CASSOC).W))
  val flushAddrIter  = RegInit(0.U((CACHE_SIZE_E - CLSIZE_E - log2Ceil(CASSOC)).W))

  //  Address translation (TLB)
  //  Structural placeholder — wire up your TLB module here.
  //  The combinational outputs tlbMiss / pageFault / phyPC are
  //  what the rest of the pipeline consumes.
  val tlb = Module(new TLB(
    NUM_RQ    = 1,
    SIZE      = params.ITLB_SIZE,
    ASSOC     = params.ITLB_ASSOC,
    IS_IFETCH = true
  ))

  tlb.io.clear        := io.clearICache || io.flushTLB
  tlb.io.pw           := io.pwRes.asTypeOf(new PageWalk_Res_Full)
  tlb.io.rqs(0).valid := fetch0.valid
  tlb.io.rqs(0).vpn   := fetch0.pc(31, 12)

  val tlbMiss   = Wire(Bool())
  val pageFault = Wire(Bool())
  val phyPC     = Wire(UInt(32.W))

  tlbMiss   := false.B
  pageFault := false.B
  phyPC     := fetch1.pc

  def inRange(addr: UInt, base: Long, size: Long): Bool =
    addr >= base.U && addr < (base + size).U

  val isLegalAddr = inRange(phyPC, params.dramBase, params.dramSize)
  val isMmioAddr  = inRange(phyPC, params.mmioBase, params.mmioSize)

  val tlbRes = tlb.io.res(0)

  // tlb.io.rqs is driven from fetch0 (one cycle early so the result is ready), but tlbRes is consumed against fetch1 state.
  // And phyPC is only valid when tlbRes.hit is true and tlbMiss is false
  when(fetch1.valid && io.vmem.enabled) {
    tlbMiss := !tlbRes.hit

    when(tlbRes.hit) {
      phyPC := Cat(tlbRes.ppn, fetch1.pc(11, 0))

      when(
        tlbRes.pageFault             ||
          tlbRes.accessFault           ||
          !tlbRes.rwx(0)               ||   // not executable
          (io.vmem.mode === 0.U && tlbRes.user)   // supervisor accessing user page
      ) {
        pageFault := true.B
      }
    }
  }

  //  ICache wiring
  icache.io.core_addr            := phyPC
  icache.io.icache_fetchID       := fetch1.fetchid
  icache.io.flushDone                := io.clearICache
  icache.io.flushDone         := io.mispr
  icache.io.flushFetchID  := io.misprFetchID
  icache.io.va_lineIdx           := fetch0.pc(CLSIZE_E + log2Ceil(LINES) - 1, CLSIZE_E)
  icache.io.ram_ready            := io.memcRes.ready
  icache.io.ram_din              := io.memcRes.data
  icache.io.ram_snoop_addr       := 0.U
  icache.io.ram_snoop_invalidate := false.B

  //  Tag check across all ways
  val assocHitUnary = VecInit((0 until CASSOC).map { i =>
    io.ictRdata(i).valid &&
    io.ictRdata(i).addr === phyPC(31, VIRT_IDX_LEN)
  })

  val hitWayValid    = assocHitUnary.asUInt.orR
  val hitWay         = PriorityEncoder(assocHitUnary)
  val assocHitInstrs = icache.io.core_dout

  bh.io.instrs := DontCare      // driven from assocHitInstrs by InstrAligner

  //  Stall logic
  val fetchLimit = Mux(
    io.BP_fetchLimit.valid,
    io.BP_fetchLimit.offs,
    io.ROB_curFetchID.value.asTypeOf(new FetchOff)
  )

  // FIFO free counter — placeholder, connect to outFIFO.io.count below TODO
  val FIFO_free = Wire(UInt((log2Ceil(FIFO_SIZE) + 1).W))
  FIFO_free := FIFO_SIZE.U   // placeholder until FIFO wired

  io.stall := false.B
  when(io.pwRes.valid)                                        { io.stall := true.B }
  when(FIFO_free < 2.U)                                       { io.stall := true.B }
  when(flushState =/= sFlushIdle)                             { io.stall := true.B }
  when(!icache.io.core_valid && !icache.io.fillHit && fetch1.valid && !tlbMiss)     { io.stall := true.B }

  //  ICache / ICT read (cycle 0)
  io.ictRe       := false.B
  io.ictRaddr    := DontCare

  when(io.ifetchOp.valid && !io.stall) {
    io.ictRe    := true.B
    io.ictRaddr := io.ifetchOp.pc(VIRT_IDX_LEN - 1, 0)
  }

  //  Cache hit / miss classification (cycle 1, combinational) and also send the fetchid along with the request
  val cacheHit    = Wire(Bool())
  val cacheMiss   = Wire(Bool())
  val doCacheLoad = Wire(Bool())
  val cacheData = Wire(UInt(32.W))

  cacheHit    := false.B
  cacheMiss   := false.B
  doCacheLoad := true.B

  when(fetch1.valid && !tlbMiss && fetch1.fetchFault === IFetchFault.IF_FAULT_NONE) {
    cacheHit    := icache.io.core_valid || icache.io.fillHit
    cacheData := Mux(icache.io.fillHit, icache.io.fillData, icache.io.core_dout)
    doCacheLoad := !icache.io.core_valid
    cacheMiss   := !icache.io.core_valid
  }

  // flushing cache ports . We need to expose them in order to wire it to the Icache itself
  io.memc       := 0.U.asTypeOf(new MemController_Req)
  io.memc.valid := false.B
  io.memc.addr  := icache.io.ram_addr
  io.memc.valid := icache.io.ram_valid
  io.memc.we    := icache.io.ram_we
  io.memc.burst := icache.io.ram_burst
  io.memc.wdata := DontCare
  io.memc.size  := 2.U

  //  Build fetch packet (IF_Instr equivalent)
  //  We use PD_Instr directly here; adapt if you have a
  //  separate IF_Instr bundle before InstrAligner.
  val packetValid      = Wire(Bool())
  val packetFetchFault = Wire(IFetchFault())
  val packetFetchID    = Wire(new FetchID)
  val packetFirstValid = Wire(new FetchOff)
  val packetLastValid  = Wire(new FetchOff)
  val packetPredTaken  = Wire(Bool())
  val packetPredTarget = Wire(UInt(31.W))
  val packetPredPos    = Wire(new FetchOff)

  packetValid      := false.B
  packetFetchFault := IFetchFault.IF_FAULT_NONE
  packetFetchID    := fetch1.fetchid
  packetFirstValid := fetch1.pc(params.FSIZE_E - 1, 1).asTypeOf(new FetchOff)
  packetLastValid  := fetch1.lastValid
  packetPredTaken  := false.B
  packetPredTarget := DontCare
  packetPredPos    := fetch1.predBr.offs

  when(fetch1.valid) {
    //The priority is intentional. An interrupt takes precedence because even if the fetch address is bad, the interrupt handler needs to run
      //first and it will re-execute the faulting instruction.
    //The ICache fault comes before TLB faults because if the physical address is misaligned there is no point checking TLB permissions — the access was never valid.
      //PMA comes last because it needs phyPC which is only valid after TLB translation.

    // ICache access fault (misalignment, bad physical address)
    when(icache.io.core_fault =/= IFetchFault.IF_FAULT_NONE) {
      packetFetchFault := icache.io.core_fault
    }

    // TLB page fault or access fault
    when(fetch1.valid && io.vmem.enabled && tlbRes.hit) {
      when(tlbRes.pageFault || tlbRes.accessFault || !tlbRes.rwx(0)) {
        packetFetchFault := IFetchFault.IF_PAGE_FAULT
      }
    }

    when(!tlbMiss && packetFetchFault === IFetchFault.IF_FAULT_NONE) {
      when(!isLegalAddr || isMmioAddr) {
        packetFetchFault := IFetchFault.IF_ACCESS_FAULT
      }
    }

    when(io.ifetchOp.fetchFault =/= IFetchFault.IF_FAULT_NONE) {
      packetFetchFault := io.ifetchOp.fetchFault
    }

    when(packetFetchFault =/= IFetchFault.IF_FAULT_NONE) {
      packetValid := true.B
    }.elsewhen(!tlbMiss && cacheHit) {
      packetValid      := true.B
      packetPredTaken  := fetch1.predBr.valid && fetch1.predBr.taken && !fetch1.predBr.dirOnly
      packetPredTarget := fetch1.predBr.target
    }
  }

  //  Apply BranchHandler corrections (packetRePred)
  val rePredValid      = Wire(Bool())
  val rePredLastValid  = Wire(new FetchOff)
  val rePredPredTaken  = Wire(Bool())
  val rePredPredTarget = Wire(UInt(31.W))
  val rePredPredPos    = Wire(new FetchOff)

  rePredValid      := packetValid
  rePredLastValid  := packetLastValid
  rePredPredTaken  := packetPredTaken
  rePredPredTarget := packetPredTarget
  rePredPredPos    := packetPredPos

  when(packetFetchFault === IFetchFault.IF_FAULT_NONE) {
    when(BH_endOffsValid) {
      when(BH_endOffs.value === 0.U) {
        rePredValid := false.B
      }.otherwise {
        rePredLastValid.value := BH_endOffs.value - 1.U
      }
      when(packetFirstValid.value > rePredLastValid.value) {
        rePredValid := false.B
      }
    }
    when(BH_decBranch.taken) {
      rePredPredTarget := BH_decBranch.target
      rePredPredPos    := BH_newPredPos
      rePredPredTaken  := BH_newPredTaken
    }
  }

  bh.io.accept := packetValid

  //  fetchBranch output
  //  On cache/TLB miss: force a re-fetch misprovision so the
  //  pipeline flushes and retries the same fetchID.
  io.fetchBranch := BH_decBranch
  when(cacheMiss || tlbMiss) {
    io.fetchBranch.taken         := true.B
    io.fetchBranch.fetchid       := fetch1.fetchid
    io.fetchBranch.fetchoffs     := fetch1.pc(params.FSIZE_E - 1, 1).asTypeOf(new FetchOff)
    io.fetchBranch.isFetchBranch := true.B
  }

  io.btUpdate  := bh.io.btUpdate
  io.retUpdate := bh.io.retUpdate

  //  PC file write (registered, cycle after fetch1)
  io.pcFileWE    := false.B
  io.pcFileAddr  := DontCare
  io.pcFileEntry := 0.U.asTypeOf(new PCFileEntry)

  when(fetch1.valid) {
    io.pcFileWE              := true.B
    io.pcFileAddr            := fetch1.fetchid
    io.pcFileEntry.pc        := fetch1.pc(31, 1)
    io.pcFileEntry.history   := DontCare   // filled by BP
    io.pcFileEntry.retAddr   := fetch1.predRetAddr
  }

  //  BP file write (combinational, from cycle 0)
  io.bpFileWE   := fetch0.valid
  io.bpFileAddr := fetchID

  //  ICT / flush write port
  io.ictWe     := false.B
  io.ictWaddr  := DontCare
  io.ictWassoc := DontCare
  io.ictWdata  := 0.U.asTypeOf(new ICTEntry)

  when(flushState === sFlushActive) {
    io.ictWe             := true.B
    io.ictWdata.valid    := false.B
    io.ictWdata.addr     := 0.U
    io.ictWassoc         := flushAssocIter
    io.ictWaddr          := Cat(flushAddrIter, 0.U(CLSIZE_E.W))
  }.elsewhen(doCacheLoad && cacheMiss) {
    io.ictWe             := true.B
    io.ictWdata.valid    := true.B
    io.ictWdata.addr     := phyPC(31, VIRT_IDX_LEN)
    io.ictWassoc         := assocCnt
    io.ictWaddr          := phyPC(VIRT_IDX_LEN - 1, 0)
  }

  //  Memory controller — cache miss fill request
  val handlingMiss = Wire(Bool())
  handlingMiss := false.B

  when(io.memcRes.fault) {
    io.memc.valid := RegNext(io.memc.valid)
    io.memc.addr  := RegNext(io.memc.addr)
  }.elsewhen(cacheMiss && doCacheLoad) {
    io.memc.valid := true.B
    io.memc.addr  := Cat(phyPC(31, CLSIZE_E), 0.U(CLSIZE_E.W))
    io.memc.we    := false.B
    io.memc.wdata := DontCare
    io.memc.size  := 2.U
    handlingMiss  := true.B
  }

  //  Page walk request
  val pw_r = RegInit(0.U.asTypeOf(new PageWalk_Req))
  io.pw := pw_r

  when(tlbMiss && !pw_r.valid) {
    pw_r.valid := true.B
    pw_r.addr  := fetch1.pc
    pw_r.write := false.B
  }.elsewhen(io.pwRes.valid) {
    pw_r.valid := false.B
  }

  //  Output FIFO + InstrAligner
  //  Structural placeholder — wire rePred packet into a FIFO TODO
  //  then into InstrAligner feeding io.instrs.
  //  Outputs are DontCare until those modules are connected.
  for (i <- 0 until params.decWidth) {
    io.instrs(i)          := 0.U.asTypeOf(new PD_Instr)
    io.instrs(i).valid    := rePredValid && (i.U >= packetFirstValid.value) &&
                             (i.U <= rePredLastValid.value)
    io.instrs(i).fetchID  := packetFetchID
    io.instrs(i).fault    := packetFetchFault
    io.instrs(i).pc       := fetch1.pc
    io.instrs(i).bits     := DontCare   // filled from assocHitInstrs by InstrAligner
  }

  //  Pipeline register updates
  when(reset.asBool) {
    fetchID.value  := 0.U
    assocCnt       := 0.U
    fetch0         := 0.U.asTypeOf(new IFetchOp)
    fetch1         := 0.U.asTypeOf(new IFetchOp)
  }.elsewhen(io.mispr) {
    fetchID.value  := io.misprFetchID.value + 1.U
    fetch0         := 0.U.asTypeOf(new IFetchOp)
    fetch1         := 0.U.asTypeOf(new IFetchOp)
  }.elsewhen(BH_decBranch.taken) {
    fetchID.value  := BH_decBranch.fetchid.value + 1.U
  }.otherwise {
    when(cacheMiss || tlbMiss) {
      fetchID      := fetch1.fetchid
      fetch0       := 0.U.asTypeOf(new IFetchOp)
      fetch1       := 0.U.asTypeOf(new IFetchOp)
    }.otherwise {
      when(io.ifetchOp.valid && !io.stall) {
        fetch0 := io.ifetchOp
      }
      when(fetch0.valid) {
        fetch1             := fetch0
        fetch1.fetchid     := fetchID
        fetch1.lastValid   := io.lastValid
        fetch1.predBr      := io.predBranch
        fetch1.predRetAddr := io.lateRetAddr
        fetch1.rIdx        := io.rIdx
        fetchID.value      := fetchID.value + 1.U
      }
    }
    when(handlingMiss) {
      assocCnt := assocCnt + 1.U
    }
  }

  //  Flush state machine
  switch(flushState) {
    is(sFlushIdle) {
      flushState     := sFlushIdle
      flushAssocIter := 0.U
      flushAddrIter  := 0.U
      when(io.clearICache) { flushState := sFlushQueued }
    }
    is(sFlushQueued) {
      flushState := sFlushActive
      when(fetch0.valid || fetch1.valid) { flushState := sFlushQueued }
      flushAssocIter := 0.U
      flushAddrIter  := 0.U
    }
    is(sFlushActive) {
      val addrWidth  = flushAddrIter.getWidth
      val assocWidth = flushAssocIter.getWidth
      val combined   = Cat(flushAssocIter, flushAddrIter) + 1.U
      val flushDone  = combined(addrWidth + assocWidth)

      flushAssocIter := combined(addrWidth + assocWidth - 1, addrWidth)
      flushAddrIter  := combined(addrWidth - 1, 0)

      when(flushDone) {
        flushState := Mux(io.MEM_busy, sFlushFinalize, sFlushIdle)
      }
    }
    is(sFlushFinalize) {
      when(!io.MEM_busy && icache.io.flushDone) { flushState := sFlushIdle }
    }
  }
}


//  ICTEntry — tag entry for the ICT (ICache Tag) array.
//  One entry per way per set.
class ICTEntry extends Bundle {
  val valid = Bool()
  val addr  = UInt(32.W)    // physical tag (upper bits above VIRT_IDX_LEN)
}
