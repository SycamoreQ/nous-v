package memory

import chisel3._
import chisel3.util._

/*
MemController is the bridge between your chip's internal bus and physical DRAM.
Every module that needs to read or write memory — ICache, PageWalker, later DCache — sends requests to the MemController.
It serializes them, handles DRAM timing, and returns responses.

The most important insight in the reference is that it does not use a simple single-request state machine.
Instead it maintains a transfer table —an array of in-flight transaction slots called transfers[AXI_NUM_TRANS].
Each slot tracks one independent transaction from enqueue to completion independently.

This means the MemController can have multiple transactions in flight simultaneously —
one might be waiting for an AXI read response while another is sending write data.
This is far more efficient than a sequential state machine that blocks on each transaction.

*/

class MemController_Req extends Bundle {
  val valid = Bool()
  val addr = UInt(32.W)
  val we = Bool()
  val wdata = UInt(32.W)
  val size = UInt(2.W)
  val burst = Bool()
}

class MemController_Res extends Bundle {
  val valid = Bool()
  val rdata = UInt(32.W)
  val fault = Bool()
  val ready = Bool()
  val done = Bool()
}

