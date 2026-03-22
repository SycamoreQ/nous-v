Quasar is a RISC-V based Neural Processing Unit in Chisel. 

TODO: 
- Write the ROB and Rename mechanism for proper OoO 
- work deeper on the mem model i.e SRAM , DCache

Whats Done- 

- Full fetch frontend — ICache, TLB, BranchPredictor, IFetchPipeline, InstrAligner, iFetch
- PageWalker, MemArbiter, MemController 
- CacheReadInterface - 
- AXI4 bundles, Transfer, MemCBundles - light mem model work 
- DecodeTypes, InstrDecoder, Decoder basically the entire decoding step 

Look into this paper for FlashAttn : https://www.together.ai/blog/flashattention-4
