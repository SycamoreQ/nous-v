package branch_pred

import chisel3._
import chisel3.util._

class LoopConfig {
  val LOG_NUM_ENTRIES = 8  // 256 entries
  val TAG_BITS = 10
  val CONFIDENCE_THRESHOLD = 3
  val ITERATION_COUNTER_WIDTH = 10
}

class LoopPredictorIndices extends Bundle {
  val bank = Vec(4, UInt(8.W))  // 4 banks for set-associative lookup
}

class LoopPredictorEntry extends Bundle {
  val totalIterations = UInt(10.W)
  val tag = UInt(10.W)
  val confidence = UInt(4.W)
  val age = UInt(4.W)
  val dir = Bool()
  val speculativeCurrentIter = UInt(10.W)
  val currentIter = UInt(10.W)
}

class LoopPredictionInfo extends Bundle {
  val hitBank = SInt(3.W)  // -1 for no hit, 0-3 for bank index
  val valid = Bool()
  val prediction = Bool()
  val indices = new LoopPredictorIndices
  val tag = UInt(10.W)
  val currentIterCheckpoint = UInt(10.W)
}

class LoopPredictor(config: LoopConfig) extends Module {
  val io = IO(new Bundle {
    // Prediction
    val predPC = Input(UInt(32.W))
    val predValid = Input(Bool())
    val predictionInfo = Output(new LoopPredictionInfo)

    // Speculative update
    val updateSpec = Input(Bool())
    val updateSpecInfo = Input(new LoopPredictionInfo)

    // Commit/Update
    val commitValid = Input(Bool())
    val commitPC = Input(UInt(32.W))
    val commitDir = Input(Bool())
    val commitInfo = Input(new LoopPredictionInfo)
    val commitMispred = Input(Bool())
    val tagePrediction = Input(Bool())

    // Recovery
    val recover = Input(Bool())
    val recoverInfo = Input(new LoopPredictionInfo)
  })

  val NUM_ENTRIES = 1 << config.LOG_NUM_ENTRIES
  val table = Reg(Vec(NUM_ENTRIES, new LoopPredictorEntry))
  val random = LFSR(8)

  // Initialize table
  when(reset.asBool) {
    for (i <- 0 until NUM_ENTRIES) {
      table(i).totalIterations := 0.U
      table(i).tag := 0.U
      table(i).confidence := 0.U
      table(i).age := 0.U
      table(i).dir := false.B
      table(i).speculativeCurrentIter := 0.U
      table(i).currentIter := 0.U
    }
  }

  // =========================================================================
  // Helper Functions
  // =========================================================================

  def getIndices(pc: UInt): LoopPredictorIndices = {
    val indices = Wire(new LoopPredictorIndices)
    val component1 = ((pc ^ (pc >> 2).B) & ((1 << (config.LOG_NUM_ENTRIES - 2)) - 1).U) << 2
    val component2 = (pc >> (config.LOG_NUM_ENTRIES - 2)) & ((1 << (config.LOG_NUM_ENTRIES - 2)) - 1).U

    for (i <- 0 until 4) {
      indices.bank(i) := (component1 ^ ((component2 >> i) << 2)) + i.U
    }
    indices
  }

  def getTag(pc: UInt): UInt = {
    val tagRaw = (pc >> (config.LOG_NUM_ENTRIES - 2)) & ((1 << (2 * config.TAG_BITS)) - 1).U
    val tagFolded = tagRaw ^ (tagRaw >> config.TAG_BITS)
    tagFolded & ((1 << config.TAG_BITS) - 1).U
  }

  // =========================================================================
  // Prediction
  // =========================================================================

  val predInfo = Wire(new LoopPredictionInfo)
  predInfo.valid := false.B
  predInfo.prediction := false.B
  predInfo.hitBank := -1.S
  predInfo.indices := getIndices(io.predPC)
  predInfo.tag := getTag(io.predPC)
  predInfo.currentIterCheckpoint := 0.U

  when(io.predValid) {
    for (i <- 0 until 4) {
      val index = predInfo.indices.bank(i)
      val entry = table(index)

      when(entry.tag === predInfo.tag) {
        predInfo.hitBank := i.S
        predInfo.valid := (entry.confidence === config.CONFIDENCE_THRESHOLD.U) ||
          ((entry.confidence * entry.totalIterations) > 128.U)
        predInfo.currentIterCheckpoint := entry.speculativeCurrentIter

        when((entry.speculativeCurrentIter + 1.U) === entry.totalIterations) {
          predInfo.prediction := !entry.dir
        }.otherwise {
          predInfo.prediction := entry.dir
        }
      }
    }
  }

  io.predictionInfo := predInfo

  // =========================================================================
  // Speculative Update
  // =========================================================================

  when(io.updateSpec && io.updateSpecInfo.hitBank >= 0.S) {
    val hitBank = io.updateSpecInfo.hitBank.asUInt
    val index = io.updateSpecInfo.indices.bank(hitBank)
    val entry = table(index)

    when(entry.totalIterations =/= 0.U) {
      val newIter = entry.speculativeCurrentIter + 1.U
      when(newIter >= entry.totalIterations) {
        table(index).speculativeCurrentIter := 0.U
      }.otherwise {
        table(index).speculativeCurrentIter := newIter
      }
    }
  }

  // =========================================================================
  // Commit State (Branch Resolution)
  // =========================================================================

  when(io.commitValid) {
    when(io.commitInfo.hitBank >= 0.S) {
      val hitBank = io.commitInfo.hitBank.asUInt
      val index = io.commitInfo.indices.bank(hitBank)
      val entry = table(index)

      when(entry.tag === io.commitInfo.tag) {
        when(io.commitInfo.valid) {
          when(io.commitDir =/= io.commitInfo.prediction) {
            // Free the entry on misprediction
            table(index).totalIterations := 0.U
            table(index).confidence := 0.U
            table(index).age := 0.U
            table(index).currentIter := 0.U
            table(index).speculativeCurrentIter := 0.U
          }.elsewhen((io.commitInfo.prediction =/= io.tagePrediction) ||
            ((random & 7.U) === 0.U)) {
            when(entry.age < config.CONFIDENCE_THRESHOLD.U) {
              table(index).age := entry.age + 1.U
            }
          }
        }

        val newCurrentIter = entry.currentIter + 1.U
        table(index).currentIter := newCurrentIter

        when(newCurrentIter > entry.totalIterations) {
          // Treat like first encounter of loop
          table(index).totalIterations := 0.U
          table(index).confidence := 0.U
        }

        when(io.commitDir =/= entry.dir) {
          when(entry.currentIter === entry.totalIterations) {
            when(entry.confidence < config.CONFIDENCE_THRESHOLD.U) {
              table(index).confidence := entry.confidence + 1.U
            }

            when(entry.totalIterations < 3.U) {
              // Don't predict loops with iteration count 1 or 2
              table(index).dir := io.commitDir
              table(index).totalIterations := 0.U
              table(index).age := 0.U
              table(index).currentIter := 0.U
              table(index).speculativeCurrentIter := 0.U
            }
          }.otherwise {
            when(entry.totalIterations === 0.U) {
              // First complete iteration
              table(index).confidence := 0.U
              table(index).totalIterations := entry.currentIter
              table(index).speculativeCurrentIter := 0.U
            }.otherwise {
              // Not same number of iterations, free entry
              table(index).totalIterations := 0.U
              table(index).confidence := 0.U
            }
          }
          table(index).currentIter := 0.U
        }

        when(io.commitMispred) {
          table(index).speculativeCurrentIter := entry.currentIter
        }
      }
    }.elsewhen(io.commitMispred) {
      // Allocate new entry on misprediction
      val randomBank = random(1, 0)

      when((random(3, 2) === 0.U)) {
        val tag = getTag(io.commitPC)
        val indices = getIndices(io.commitPC)
        val index = indices.bank(randomBank)

        when(table(index).age === 0.U) {
          // Most mispredictions are on last iterations
          table(index).dir := !io.commitDir
          table(index).tag := tag
          table(index).totalIterations := 0.U
          table(index).age := 7.U
          table(index).confidence := 0.U
          table(index).currentIter := 0.U
          table(index).speculativeCurrentIter := 0.U
        }.otherwise {
          table(index).age := table(index).age - 1.U
        }
      }
    }
  }

  // =========================================================================
  // Recovery (Restore Speculative State)
  // =========================================================================

  when(io.recover && io.recoverInfo.hitBank >= 0.S) {
    val hitBank = io.recoverInfo.hitBank.asUInt
    val index = io.recoverInfo.indices.bank(hitBank)

    when(table(index).tag === io.recoverInfo.tag) {
      table(index).speculativeCurrentIter := io.recoverInfo.currentIterCheckpoint
    }
  }
}

class SaturatingCounter(width: Int, initVal: Int = 0) extends Module {
  val io = IO(new Bundle {
    val increment = Input(Bool())
    val decrement = Input(Bool())
    val set = Input(Bool())
    val setValue = Input(UInt(width.W))
    val value = Output(UInt(width.W))
  })

  val counter = RegInit(initVal.U(width.W))
  val maxVal = (1 << width) - 1

  when(io.set) {
    counter := io.setValue
  }.elsewhen(io.increment && counter =/= maxVal.U) {
    counter := counter + 1.U
  }.elsewhen(io.decrement && counter =/= 0.U) {
    counter := counter - 1.U
  }

  io.value := counter
}


// LFSR for Random Number Generation
object LFSR {
  def apply(width: Int): UInt = {
    val lfsr = RegInit(1.U(width.W))
    val feedback = lfsr(width - 1) ^ lfsr(width - 2) ^ lfsr(width - 3) ^ lfsr(width - 4)
    lfsr := Cat(lfsr(width - 2, 0), feedback)
    lfsr
  }
}

// Top-Level Integration with TAGE

class TAGESCLPredictor extends Module {
  val io = IO(new Bundle {
    val predPC = Input(UInt(32.W))
    val predValid = Input(Bool())
    val predHistory = Input(UInt(64.W))

    val finalPrediction = Output(Bool())
    val confidence = Output(UInt(3.W))

    val updateValid = Input(Bool())
    val updatePC = Input(UInt(32.W))
    val updateHistory = Input(UInt(64.W))
    val actualTaken = Input(Bool())
  })

  val config = new LoopConfig

  // TAGE predictor (already implemented)
  val tage = Module(new TAGE(4, 256, 9, 2, 2))
  tage.io.predValid := io.predValid
  tage.io.predAddr := io.predPC
  tage.io.predHistory := io.predHistory

  // Loop predictor
  val loopPred = Module(new LoopPredictor(config))
  loopPred.io.predPC := io.predPC
  loopPred.io.predValid := io.predValid

  val loopInfo = loopPred.io.predictionInfo
  val tagePred = tage.io.predTaken

  // Combine predictions: Loop predictor overrides TAGE if valid and confident
  io.finalPrediction := Mux(
    loopInfo.valid && loopInfo.hitBank >= 0.S,
    loopInfo.prediction,
    tagePred
  )

  io.confidence := Mux(loopInfo.valid, 7.U, tage.io.predTageID)

  // Update logic
  loopPred.io.commitValid := io.updateValid
  loopPred.io.commitPC := io.updatePC
  loopPred.io.commitDir := io.actualTaken
  loopPred.io.tagePrediction := tagePred
  loopPred.io.commitMispred := tagePred =/= io.actualTaken

  tage.io.writeValid := io.updateValid
  tage.io.writeAddr := io.updatePC
  tage.io.writeHistory := io.updateHistory
  tage.io.writeTaken := io.actualTaken
}