package cache

/*
A small buffer that sits between the memory controller response path and IDirectCache.
It holds one in-flight cache line — essentially BURST_LEN words wide.
As each word arrives from the memory controller it is written into the fill buffer immediately.
The buffer tracks which words have arrived via a valid bitmap. On a hit against the fill buffer the controller can return data
to the core without waiting for the SRAM write to complete.
*/

import chisel3._
import chisel3.util._
import branch_pred._
import branch_pred.BranchType._
import cache.ICache
import fetch.{FetchID, IFetchFault , iFetchParams }



class ICacheFillBuffer(val BSIZE: Int = 4 , val BURST_LEN: Int = 4) extends Module{
  val io = IO(new Bundle{
    val fillValid = Input(Bool())
    val fillAddr = Input(UInt(32.W))
    val fillWordIdx = Input(UInt(32.W))
    val fillData = Input(UInt(32.W))
    val flush = Input(Bool())

    val queryAddr = Input(UInt(32.W))
    val hit = Output(Bool())
    val data = Output(UInt(32.W))

    val lineComplete = Output(Bool())

  })

  val baseAddr = RegInit(0.U(32.W))
  val wordValid = RegInit(VecInit(Seq.fill(BURST_LEN)(false.B)))
  val dataReg = RegInit(VecInit(Seq.fill(BURST_LEN)(0.U(32.W))))

  when(io.flush) {
    wordValid.foreach(_ := false.B)
  }

  when(io.fillValid) {
    baseAddr := io.fillAddr
    dataReg(io.fillWordIdx) := io.fillData
    wordValid(io.fillWordIdx) := true.B
  }

  val wordIdx = io.queryAddr(log2Ceil(BURST_LEN * BSIZE) - 1, log2Ceil(BSIZE))
  val addrMatch  = io.queryAddr(31, log2Ceil(BURST_LEN * BSIZE)) ===
    baseAddr(31, log2Ceil(BURST_LEN * BSIZE))
  io.hit         := addrMatch && wordValid(wordIdx)
  io.data        := dataReg(wordIdx)
  io.lineComplete := wordValid.asUInt.andR


}