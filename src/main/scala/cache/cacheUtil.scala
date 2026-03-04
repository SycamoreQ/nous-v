package cache

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage


class Transfer extends Module {
  val mmioData = Wire(31.W)
  val mmio = Wire(Bool())
  val progress =
}
