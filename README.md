Quasar is a RISC-V based Neural Processing Unit in Chisel. 

Currently working on the Branch Prediction for the host CPU.
TODO: 
- Finish the Branch Predictor then the TAGE based stuff and finally wire it to the 
  iFetch module 

- Build the ICache and DCache which are part of the L1 cache layer. 

- A more farther goal is to integrate AXI system bus inspired and used in the Coral NPU

Advancement 1 — Set Associativity (2-way or 4-way) with tree-PLRU
Touch IDirectCache. Duplicate tagMem, validMem, dataMem into a Vec(NUM_WAYS, ...). Hit becomes an OR across all ways. 
Add a plruBits RegInit array — 1 bit per set for 2-way, 3 bits per set for 4-way. On hit, update the PLRU tree toward the hit way. On miss, evict the way pointed to by the PLRU root. In ICacheController,
read PLRU bits during sREAD, register the victim way, and use it throughout sALLOCATE.

Advancement 2 — fetchID Tagging and Misprediction Flush
Touch ICacheController. Add inputs fetchID: FetchID, flushFetchID: FetchID, and flush: Bool. Register the fetchID of the request currently in sALLOCATE. On flush, if the in-flight fetchID is older than flushFetchID, abort the fill — return to sREAD, 
reset cntWords, and suppress cache_setValid. Wires directly to ifp.io.mispr and ifp.io.misprFetchID already present in ifetch.scala.

Advancement 3 — VIPT Addressing + I-TLB
Add a new ITlb module and a new FSM state sTRANSLATE entered from sREAD before the tag check. The TLB is a fully-associative CAM with entries of { vpn, ppn, asid, valid }. On TLB hit, translate VA→PA in one cycle and proceed. On TLB miss, 
enter sTLB_MISS, issue PageWalk_Req (already in your IO), wait for PageWalk_Res, 
install the entry, and return to sTRANSLATE. Use VA bits for the cache index (no translation needed) and PA bits for tag only. Safe constraint: LINE_WIDTH + OFFSET_WIDTH <= 12 for 4KB pages — with LINES=1024 you'd need to reduce sets or switch to PIPT.

Advancement 4 — IFetchFault Output
Touch ICacheController IO. Replace core_fatal: Bool with core_fault: IFetchFault(). Map misalignment to IF_ACCESS_FAULT, TLB page fault to IF_PAGE_FAULT (requires Advancement 3), 
and interrupt injection to IF_INTERRUPT via a new input from iFetch. This output feeds directly into PD_Instr.fault defined in ifetchutil.scala.

Advancement 5 — FLUSH State for FENCE.I / clearCache
Touch ICacheController. Add input flush: Bool wired to clearCache in iFetch. Add state sFLUSH with a flush counter of log2Ceil(LINES) bits. Each cycle in sFLUSH, assert cache_setInvalid for one line index and increment the counter, 
exiting when it wraps. From sIDLE or sREAD, when flush is asserted, transition immediately to sFLUSH and reset cntWords. Cost is LINES cycles — acceptable since FENCE.I is rare.

Advancement 6 — Fill Buffer for Atomic Line Installation
Touch ICacheController. Add val fillBuf = Reg(Vec(CNT_MAX_WORDS, UInt(32.W))). During sALLOCATE, write each incoming RAM word into fillBuf(cntWords) instead of directly into the cache. On isLastWord, write all words to dataMem in one operation and assert setValid. 
This makes partial-line visibility structurally impossible rather than just gated, and sets up Advancement 7 naturally.

Advancement 7 — Critical-Word-First / Early Restart
Requires Advancement 6. Register the requested word offset at the start of sALLOCATE using RegEnable. Reorder the RAM burst to start at core_addr and wrap modulo CNT_MAX_WORDS. As soon as the first word arrives (the requested word), assert core_valid early 
so the pipeline can proceed. Add a flag or sub-state sFILL_DRAIN to continue writing the remaining words into fillBuf in the background after core_valid has fired.

Advancement 8 — Next-Line Prefetcher
Add a new INextLinePrefetcher module alongside ICacheController. Add inputs predTaken: Bool and predTarget: UInt(32.W) wired from PredBranch.taken and PredBranch.target in ifetchutil.scala. On a predicted taken branch, check if the target line is already present (tag check without stalling), and if not, push the target address into a 4-entry prefetch queue. Add a small 8-entry direct-mapped prefetch filter to 
avoid re-requesting lines already present or already queued. The prefetch queue feeds into the same sALLOCATE state via a priority MUX: demand fill wins over prefetch fill.