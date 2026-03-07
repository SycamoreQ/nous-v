Quasar is a RISC-V based Neural Processing Unit in Chisel. 

Currently working on the Branch Prediction for the host CPU.
TODO: 
- Finish the Branch Predictor then the TAGE based stuff and finally wire it to the 
  iFetch module 

- Build the ICache and DCache which are part of the L1 cache layer. 

- A more farther goal is to integrate AXI system bus inspired and used in the Coral NPU

CURRENT: Writing testbenches in Verilator for the branch prediction and fetching mechanism. 
This is to validate their working before I work on the advancements of the ICache.

Advancement 1 — Set Associativity (2-way or 4-way) with tree-PLRU  --- DONE (possible issues exist)
Touch IDirectCache. Duplicate tagMem, validMem, dataMem into a Vec(NUM_WAYS, ...). Hit becomes an OR across all ways. 
Add a plruBits RegInit array — 1 bit per set for 2-way, 3 bits per set for 4-way. On hit, update the PLRU tree toward the hit way. On miss, evict the way pointed to by the PLRU root. In ICacheController,
read PLRU bits during sREAD, register the victim way, and use it throughout sALLOCATE.

Advancement 2 — fetchID Tagging and Misprediction Flush   ---- DONE (NO issue)
Touch ICacheController. Add inputs fetchID: FetchID, flushFetchID: FetchID, and flush: Bool. Register the fetchID of the request currently in sALLOCATE. On flush, if the in-flight fetchID is older than flushFetchID, abort the fill — return to sREAD, 
reset cntWords, and suppress cache_setValid. Wires directly to ifp.io.mispr and ifp.io.misprFetchID already present in ifetch.scala.

Advancement 3 — VIPT Addressing + I-TLB  ---- DONE 
Add a new ITlb module and a new FSM state sTRANSLATE entered from sREAD before the tag check. The TLB is a fully-associative CAM with entries of { vpn, ppn, asid, valid }. On TLB hit, translate VA→PA in one cycle and proceed. On TLB miss, 
enter sTLB_MISS, issue PageWalk_Req (already in your IO), wait for PageWalk_Res, 
install the entry, and return to sTRANSLATE. Use VA bits for the cache index (no translation needed) and PA bits for tag only. Safe constraint: LINE_WIDTH + OFFSET_WIDTH <= 12 for 4KB pages — with LINES=1024 you'd need to reduce sets or switch to PIPT.

- First create an ITLB, probably smaller than the actual TLB or the TLB itself. 
- The split TLB design is particularly efficient. A TLB miss on an instruction fetch is rather disastrous. However, a separate iTLB exhibits a much lower miss rate compared to a dTLB or unified TLB. Having a low miss rate for the iTLB is so important that L1 iTLBs 
are made larger than L1 dTLBs and even with higher associativity (especially when there are multiple hardware threads in which case the iTLB can be statically or dynamically partitioned among the threads). Memory management at the OS or runtime level can have also a significant impact on the efficiency of the split TLB design.
- Then the FSM states are easy in the ICacheController. 

Advancement 4 — IFetchFault Output   --- DONE 
Touch ICacheController IO. Replace core_fatal: Bool with core_fault: IFetchFault(). Map misalignment to IF_ACCESS_FAULT, TLB page fault to IF_PAGE_FAULT (requires Advancement 3), 
and interrupt injection to IF_INTERRUPT via a new input from iFetch. This output feeds directly into PD_Instr.fault defined in ifetchutil.scala.

Advancement 5 — FLUSH State for FENCE.I / clearCache  --- DONE 
Touch ICacheController. Add input flush: Bool wired to clearCache in iFetch. Add state sFLUSH with a flush counter of log2Ceil(LINES) bits. Each cycle in sFLUSH, assert cache_setInvalid for one line index and increment the counter, 
exiting when it wraps. From sIDLE or sREAD, when flush is asserted, transition immediately to sFLUSH and reset cntWords. Cost is LINES cycles — acceptable since FENCE.I is rare.


Advancement 6 — Fill Buffer for Atomic Line Installation    ---- DONE 
Touch ICacheController. Add val fillBuf = Reg(Vec(CNT_MAX_WORDS, UInt(32.W))). During sALLOCATE, write each incoming RAM word into fillBuf(cntWords) instead of directly into the cache. On isLastWord, write all words to dataMem in one operation and assert setValid. 
This makes partial-line visibility structurally impossible rather than just gated, and sets up Advancement 7 naturally.

Advancement 7 — Critical-Word-First / Early Restart
Requires Advancement 6. Register the requested word offset at the start of sALLOCATE using RegEnable. Reorder the RAM burst to start at core_addr and wrap modulo CNT_MAX_WORDS. As soon as the first word arrives (the requested word), assert core_valid early 
so the pipeline can proceed. Add a flag or sub-state sFILL_DRAIN to continue writing the remaining words into fillBuf in the background after core_valid has fired.

- Memory controller currently always starts the burst from the beginning of the cache line — word 0 — regardless of which word the fetch actually needs. If the fetch is targeting word 5 of a 16-word line, you still have to wait for words 0 through 4 to arrive before word 5 appears in the fill buffer. 
Critical word first fixes this by starting the burst at the requested word, wrapping around to fill the rest of the line afterward.

- The cache line is a fixed ring of BURST_LEN words. You can enter the ring at any point and walk forward, wrapping at the end back to the start. You always visit every word exactly once before returning to your entry point. That is the definition of a circular traversal.

Advancement 8 — Next-Line Prefetcher   --- DONE 
Add a new INextLinePrefetcher module alongside ICacheController. Add inputs predTaken: Bool and predTarget: UInt(32.W) wired from PredBranch.taken and PredBranch.target in ifetchutil.scala. On a predicted taken branch, check if the target line is already present (tag check without stalling), 
and if not, push the target address into a 4-entry prefetch queue. Add a small 8-entry direct-mapped prefetch filter to 
avoid re-requesting lines already present or already queued. The prefetch queue feeds into the same sALLOCATE state via a priority MUX: demand fill wins over prefetch fill.



Instr Aligner advancements 

Difference 1 — Window-based vs drain-pointer approach
The SV uses a sliding window of BUF_SIZE + 1 fetch cycles with a PriorityEncoder selecting the next NUM_INSTRS instruction starts simultaneously. Your Chisel version uses a single-packet buffer with a linear drainPtr walking forward one slot at a time. 
The SV approach is superscalar — it finds all NUM_INSTRS starts in one combinational pass across multiple buffered packets. Your version finds them sequentially. This means your version will produce fewer instructions per cycle on average.

Difference 2 — BUF_SIZE buffering is missing
The SV holds BUF_SIZE previous fetch cycles in prev_r alongside the current cycle. This is what enables the window to span packet boundaries for the priority encoder. 
Your version only holds one packet at a time. The carry register handles the cross-packet 32-bit instruction case but nothing else.

Difference 3 — unhandled_c tracking is missing
The SV tracks which instruction starts have already been dispatched via unhandled_c. This is what prevents the same instruction being output twice across consecutive cycles. 
Your version advances drainPtr instead, which works for single-packet but breaks down when the window spans multiple buffered packets.