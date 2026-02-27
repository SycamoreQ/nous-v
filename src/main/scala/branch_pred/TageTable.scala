package  branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{BTUpdate, FetchOff, PredBranch}
import branch_pred._


class TageTable(SIZE: Int, TAG_SIZE: Int, USF_SIZE: Int = 2, CNT_SIZE: Int = 2, INTERVAL: Int = 1024) extends Module {
  val io = IO(new Bundle {
    val readValid = Input(Bool())
    val readAddr = Input(UInt(log2Ceil(SIZE).W))
    val readTag = Input(UInt(TAG_SIZE.W))
    val readValid_out = Output(Bool())
    val readTaken = Output(Bool())

    val writeValid = Input(Bool())
    val writeAddr = Input(UInt(log2Ceil(SIZE).W))
    val writeTag = Input(UInt(TAG_SIZE.W))
    val writeTaken = Input(Bool())
    val writeUpdate = Input(Bool())
    val writeUseful = Input(Bool())
    val writeCorrect = Input(Bool())

    val allocAvail = Output(Bool())
    val doAlloc = Input(Bool())
    val allocFailed = Input(Bool())
  })

  val tag = Reg(Vec(SIZE, UInt(TAG_SIZE.W)))
  val useful = Reg(Vec(SIZE, UInt(USF_SIZE.W)))

  val counters = Module(new BranchPredictionTable(log2Ceil(SIZE)))
  counters.io.readValid := io.readValid
  counters.io.readAddr := io.readAddr
  io.readTaken := counters.io.taken
  counters.io.write_en := io.writeValid && (io.writeUpdate || (io.doAlloc && useful(io.writeAddr) === 0.U))
  counters.io.writeAddr := io.writeAddr
  counters.io.writeInit := !io.writeUpdate
  counters.io.writeTaken := io.writeTaken

  val tagRegA = RegInit(0.U(TAG_SIZE.W))
  val tagRegB = RegInit(0.U(TAG_SIZE.W))

  when(io.readValid) {
    tagRegA := tag(io.readAddr)
    tagRegB := io.readTag
  }

  io.readValid_out := tagRegA === tagRegB
  io.allocAvail := useful(io.writeAddr) === 0.U

  val decrCnt = RegInit(0.U(INTERVAL.W))
  val decrBit = RegInit(false.B)
  val resetIdx = RegInit(0.U((log2Ceil(SIZE) + 1).W))

  when(!resetIdx(log2Ceil(SIZE))) {
    tag(resetIdx(log2Ceil(SIZE) - 1, 0)) := 0.U
    useful(resetIdx(log2Ceil(SIZE) - 1, 0)) := 0.U
    resetIdx := resetIdx + 1.U
  }.elsewhen(io.writeValid) {
    when(io.writeUpdate) {
      when(io.writeUseful) {
        when(io.writeCorrect && useful(io.writeAddr) =/= Fill(USF_SIZE, 1.U)) {
          useful(io.writeAddr) := useful(io.writeAddr) + 1.U
        }.elsewhen(!io.writeCorrect && useful(io.writeAddr) =/= 0.U) {
          useful(io.writeAddr) := useful(io.writeAddr) - 1.U
        }
      }
    }.elsewhen(io.doAlloc) {
      when(useful(io.writeAddr) === 0.U) {
        tag(io.writeAddr) := io.writeTag
      }
    }.elsewhen(io.allocFailed) {
      useful(io.writeAddr) := useful(io.writeAddr) - 1.U
    }
  }

  when(decrCnt === 0.U) {
    for (i <- 0 until SIZE) {
      useful(i) := useful(i) & ~(1.U << decrBit).asUInt
    }
    decrBit := !decrBit
  }
  decrCnt := decrCnt - 1.U
}