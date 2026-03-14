package memory

import chisel3._
import chisel3.util._
import fetch.iFetchParams
import memory.MemC_Cmd


// MemC_Req
//
// Full request bundle sent by clients (ICache, PageWalker, DCache) to the
// MemController. cmd == MEMC_NONE means no request — no separate valid field.

class MemC_Req(
  fetchParams : iFetchParams,
  AXI_WIDTH   : Int = 64
) extends Bundle {
  val cmd       = MemC_Cmd()
  val cacheID   = UInt(1.W)                              // 0=DCache, 1=ICache
  val cacheAddr = UInt((fetchParams.CACHE_SIZE_E - 2).W) // index into cache SRAM
  val readAddr  = UInt(32.W)                             // DRAM source address
  val writeAddr = UInt(32.W)                             // DRAM eviction address
  val data      = UInt(AXI_WIDTH.W)                      // store fuse data
  val mask      = UInt((AXI_WIDTH / 8).W)                // store fuse byte mask
}

// TransferStatus
// Per-slot progress information broadcast to clients via OUT_stat.
class TransferStatus(fetchParams: iFetchParams) extends Bundle {
  val valid     = Bool()
  val cacheID   = UInt(1.W)
  val progress  = UInt((fetchParams.CLSIZE_E - 1).W)
  val cacheAddr = UInt((fetchParams.CACHE_SIZE_E - 2).W)
  val readAddr  = UInt(32.W)
  val writeAddr = UInt(32.W)
  // active = both AXI requests have been sent, transfer is streaming
  val active    = Bool()
}

// MemController_SglLdRes
// Result of a single-word MMIO load (MEMC_READ_BYTE/HALF/WORD).
// id echoes the cacheAddr field used as the MMIO transaction identifier.
class MemController_SglLdRes(fetchParams: iFetchParams) extends Bundle {
  val valid = Bool()
  val id    = UInt((fetchParams.CACHE_SIZE_E - 2).W)
  val data  = UInt(32.W)
}

// MemController_SglStRes
// Completion signal for a single-word MMIO store (MEMC_WRITE_BYTE/HALF/WORD).
class MemController_SglStRes(fetchParams: iFetchParams) extends Bundle {
  val valid = Bool()
  val id    = UInt((fetchParams.CACHE_SIZE_E - 2).W)
}

// MemController_LdDataFwd
// Load data forward path — forwards AXI read data directly to the LSU before
// the cache write interface acknowledges. Stubbed until DCache exists.
class MemController_LdDataFwd(AXI_WIDTH: Int = 64) extends Bundle {
  val valid = Bool()
  val addr  = UInt(32.W)
  val data  = UInt(AXI_WIDTH.W)
}

// MemC_Res

// Combined status output from MemController back to all clients.
class MemC_Res(
  fetchParams : iFetchParams,
  NUM_TFS     : Int = 8,
  NUM_TFS_IN  : Int = 3,
  AXI_WIDTH   : Int = 64
) extends Bundle {
  // busy is always true — legacy signal that makes old clients stall.
  // Actual per-port stall control is via the stall vector.
  val busy      = Bool()

  // One stall bit per input port. Bit i low means port i's request was
  // accepted this cycle. All bits high when no request was accepted.
  val stall     = UInt(NUM_TFS_IN.W)

  // Per-slot transfer progress, visible to clients for dependency checking
  val transfers = Vec(NUM_TFS, new TransferStatus(fetchParams))

  // MMIO results
  val sglLdRes  = new MemController_SglLdRes(fetchParams)
  val sglStRes  = new MemController_SglStRes(fetchParams)

  // Load data forward — stubbed, always driven to zero in MemController
  val ldDataFwd = new MemController_LdDataFwd(AXI_WIDTH)
}

// RData
// Internal bundle for the rFIFO — captures one AXI R channel beat.
// rid is used to look up the owning Transfer slot and route data correctly.
class RData(axiParams: AXIParams) extends Bundle {
  val rid   = UInt(axiParams.xAXI_ID_LEN.W)
  val rdata = UInt(axiParams.AXI_WIDTH.W)
  val rlast = Bool()
}
