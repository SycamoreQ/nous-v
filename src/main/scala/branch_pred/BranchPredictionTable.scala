package branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.iFetchParams;

class BranchPredictionTable(params: iFetchParams) {
  val io = IO(new Bundle {
    val readValid = Input(Bool())
    val readAddr = Input(WireInit(0.U(31.W)))
    val taken = Input(Bool())
    val write_en = Input(Bool())
    val writeAddr = Input(WireInit(0.U(31.W)))
    val writeInit = Input(Bool())
    val writeTaken = Input(Bool())
  })

  val numCounters = 1 << params.IDX_LEN
  val pred = RegInit(VecInit(Seq.fill(params.IDX_LEN)(false.B)))
  val hist = RegInit(VecInit(Seq.fill(params.IDX_LEN)(false.B)))

  when (io.readValid) {
      io.taken := pred(io.readAddr)
  }

  class Write extends Bundle {
    val taken = Bool()
    val init = Bool()
    val addr = RegInit(0.U((params.IDX_LEN-1).W))
    val valid = Bool()
  }
  val writeTempReg = Reg(UInt(2.W))
  val write_c = Wire(new Write)
  val write_r = RegInit(0.U.asTypeOf(new Write))

  write_c.valid := io.write_en
  write_c.init  := io.writeInit
  write_c.addr  := io.writeAddr
  write_c.taken := io.writeTaken

  val resetIdx = RegInit(0.U((params.IDX_LEN + 1).W))

  when(!resetIdx(params.IDX_LEN)) {
    pred(resetIdx(params.IDX_LEN - 1, 0)) := false.B
    hist(resetIdx(params.IDX_LEN - 1, 0)) := false.B
    resetIdx := resetIdx + 1.U
  } .otherwise {
    write_r := write_c

    when(write_c.valid) {
      writeTempReg := Cat(pred(write_c.addr), hist(write_c.addr))
    }

    when(write_r.valid) {
      val nextVal = Wire(UInt(2.W))
      nextVal := writeTempReg

      when(write_r.init) {
        nextVal := Cat(write_r.taken, !write_r.taken)
      } .elsewhen(writeTempReg =/= 3.U && write_r.taken) {
        nextVal := writeTempReg + 1.U
      } .elsewhen(writeTempReg =/= 0.U && !write_r.taken) {
        nextVal := writeTempReg - 1.U
      }

      pred(write_r.addr) := nextVal(1)
      hist(write_r.addr) := nextVal(0)
    }
  }
}