package branch_pred

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{BTUpdate, FetchOff, PredBranch}
import branch_pred._

class TagePredictor(NUM_STAGES:Int = TAGE_STAGES,
                    FACTOR: Int = 2 , BASE:Int = BASE ,
                    TABLE_SIZE:Int = TABLE_SIZE ,
                    TAG_SIZE: Int = 9 ) {
  val io = IO(new Bundle {
    val predValid = Input(Bool())
    val predHistory = Input(new BHist)
    val predAddr = Input(WireInit(0.U(31.W)))

    val predTageID = Output(new TageID)
    val altPred = Output(RegInit(0.U))
    val predTaken = Output(RegInit(Bool()))

    val writeAddr = Input(WireInit(0.U(31.W)))
    val writeValid = Input(Bool())
    val writeHistory = Input(new BHist)
    val writeTageID = Input(new TageID)
    val writeTaken = Input(Bool())
    val writeAltPred = Input(Bool())
    val writePred = Input(Bool())
  })

  var HASH_SIZE = Wire(log2Ceil(TABLE_SIZE))
  val valid = Wire(Vec(NUM_STAGES, Bool()))
  val predictions = Wire(Vec(NUM_STAGES, Bool()))

  val basePredictor = Module(new BranchPredictionTable)
  basePredictor.io.readValid := io.predValid
  basePredictor.io.readAddr := io.predAddr(BP_BASEP_ID_LEN-1, 0)
  predictions(0) := basePredictor.io.taken
  basePredictor.io.write_en := io.writeValid
  basePredictor.io.writeAddr := io.writeAddr(BP_BASEP_ID_LEN-1, 0)
  basePredictor.io.writeInit := false.B
  basePredictor.io.writeTaken := io.writeTaken


  valid(0) := 1.U

  val predHashes = Reg(Vec(NUM_STAGES - 1, UInt(HASH_SIZE.W)))
  val writeHashes = Reg(Vec(NUM_STAGES - 1, UInt(HASH_SIZE.W)))
  val predTags = Reg(Vec(NUM_STAGES - 1, UInt(TAG_SIZE.W)))
  val writeTags = Reg(Vec(NUM_STAGES - 1, UInt(TAG_SIZE.W)))


  for (i <- 0 until NUM_STAGES - 1) {
    val hist_bits = BASE * math.pow(FACTOR, i).toInt

    predTags(i) := io.predAddr(TAG_SIZE - 1, 0)
    writeTags(i) := io.writeAddr(TAG_SIZE - 1, 0)
    predHashes(i) := 0.U
    writeHashes(i) := 0.U

    for (j <- 0 until (io.predAddr.getWidth / HASH_SIZE)) {
      predHashes(i) := predHashes(i) ^ io.predAddr(j * HASH_SIZE + HASH_SIZE - 1, j * HASH_SIZE)
      writeHashes(i) := writeHashes(i) ^ io.writeAddr(j * HASH_SIZE + HASH_SIZE - 1, j * HASH_SIZE)
    }

    for (j <- 0 until (BASE * math.pow(FACTOR, i).toInt)) {
      predHashes(i) := predHashes(i) ^ (io.predHistory(j) << (j % HASH_SIZE)).U
      writeHashes(i) := writeHashes(i) ^ (io.writeHistory(j) << (j % HASH_SIZE)).U
      predTags(i) := predTags(i) ^ ((io.predHistory(j) ^ io.predHistory((j + 1) % hist_bits)) << (j % TAG_SIZE)).U
      writeTags(i) := writeTags(i) ^ ((io.writeHistory(j) ^ io.writeHistory((j + 1) % hist_bits)) << (j % TAG_SIZE)).U
    }
  }

  val random = Wire(RegInit(0.U(8.W)))
  when(_) {
    random := 1.U
  }.otherwise{
    random := Cat(random(6,0) , random(7) ^ random(5) ^ random(4) ^ random(3))
  }

  val avail = Wire(Vec(NUM_STAGES , Bool()))
  avail(0) := 1.U

  val alloc = Wire(Reg(NUM_STAGES.U))
  val followingAllocAvail = Wire(Vec(NUM_STAGES, Bool()))
  val temp = Wire(Bool())
  val doAlloc = Wire(Vec(NUM_STAGES, Bool()))
  val allocFailed = Wire(Bool())

  for (i <- 0 until NUM_STAGES) {
    followingAllocAvail(i) := false.B
    for (j <- i + 1 until NUM_STAGES) {
      followingAllocAvail(i) := followingAllocAvail(i) | avail(j)
    }
  }

  temp := false.B
  doAlloc := VecInit(Seq.fill(NUM_STAGES)(false.B))
  allocFailed := false.B

  when(io.writeTaken =/= io.writePred) {
    var tempVar = false.B
    for (i <- 0 until NUM_STAGES) {
      when(i.U > io.writeTageID && avail(i) && !tempVar &&
        (!followingAllocAvail(i) || random((i % 4) * 2 + 1, (i % 4) * 2) =/= 0.U)) {
        tempVar = true.B
        doAlloc(i) := true.B
      }
    }
    temp := tempVar
    allocFailed := !temp
  }

  val tageTable = for (i <- 1 until NUM_STAGES) yield {
    val tage = Module(new TageTable(TABLE_SIZE, TAG_SIZE))
    tage.io.readValid := io.predValid
    tage.io.readAddr := predHashes(i - 1)
    tage.io.readTag := predTags(i - 1)
    valid(i) := tage.io.readValid
    predictions(i) := tage.io.readTaken
    tage.io.writeValid := io.writeValid
    tage.io.writeAddr := writeHashes(i - 1)
    tage.io.writeTag := writeTags(i - 1)
    tage.io.writeTaken := io.writeTaken
    tage.io.writeUpdate := i.U === io.writeTageID
    tage.io.writeUseful := io.writePred =/= io.writeAltPred
    tage.io.writeCorrect := io.writePred === io.writeTaken
    avail(i) := tage.io.allocAvail
    tage.io.doAlloc := doAlloc(i)
    tage.io.allocFailed := allocFailed && i.U > io.writeTageID
    tage
  }

  io.altPred := predictions(0)
  io.predTaken := predictions(0)
  io.predTageID := 0.U

  for (i <- 0 until NUM_STAGES) {
    when(valid(i)) {
      io.predTageID := i.U
      io.altPred := io.predTaken
      io.predTaken := predictions(i)
    }
  }

}
