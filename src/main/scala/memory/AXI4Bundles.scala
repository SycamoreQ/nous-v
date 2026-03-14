package memory

import chisel3._
import chisel3.util._
import fetch.iFetchParams

case class AXIParams (
                     xAXI_ID_LEN : Int = 4,   //— width of transaction ID field
                     AXI_WIDTH  : Int = 64 ,   //— data bus width in bits
                     ADDR_LEN   : Int = 32   //— address width
                     )


//Slave responds with arready : Bool — slave can accept the request this cycle.

class AXI4_AR (params: AXIParams) extends Bundle{
  val arid    =  UInt(params.xAXI_ID_LEN.W)   //transaction ID, used to match response
  val araddr  = UInt(params.ADDR_LEN.W)     //byte address to read from
  val arlen   = UInt(8.W)            //burst length minus 1 (0 = single beat)
  val arsize  = UInt(3.W)            //bytes per beat encoded as log2 (2=4bytes, 3=8bytes)
  val arburst = UInt(2.W)            //burst type: FIXED=0, INCR=1, WRAP=2
  val arlock   = UInt(1.W)            //exclusive access flag, always 0 for you
  val arcache = UInt(4.W)            //cache hints, use 4'b0010 (normal non-cacheable)
  val arvalid  = Bool()                 //master is presenting a valid request
}

//Master sends this to initiate a write transaction. Same handshake — awvalid/awready.

class AXI4_AW (params: AXIParams) extends Bundle{
  val awid    =  UInt(params.xAXI_ID_LEN.W)   //transaction ID, used to match response
  val awaddr  = UInt(params.ADDR_LEN.W)     //byte address to read from
  val awlen   = UInt(8.W)            //burst length minus 1 (0 = single beat)
  val awsize  = UInt(3.W)            //bytes per beat encoded as log2 (2=4bytes, 3=8bytes)
  val awburst = UInt(2.W)            //burst type: FIXED=0, INCR=1, WRAP=2
  val awlock   = UInt(1.W)            //exclusive access flag, always 0 for you
  val awcache = UInt(4.W)            //cache hints, use 4'b0010 (normal non-cacheable)
  val awvalid  = Bool()                 //master is presenting a valid request
}

//Slave responds with wready

class AXI4_W (params: AXIParams) extends Bundle {
  val wdata  = UInt(params.AXI_WIDTH.W)        // data payload
  val wstrb   = UInt(params.AXI_WIDTH/8.W)     // byte enables, one bit per byte
  val wlast  =  Bool()                     // this is the final beat of the burst
  val wvalid = Bool()                      // data is valid this cycle
}

//Master responds with bready  — the reference hardwires this to 1.

class AXI4_B (params: AXIParams ) extends Bundle {
  val bid    = UInt(params.xAXI_ID_LEN.W)   //matches the awid of the completed transaction
  val bresp  = UInt(2.W)            //OKAY=0, EXOKAY=1, SLVERR=2, DECERR=3
  val bvalid = Bool()                 //response is valid
}

//Master responds with rready

class AXI4_R (params : AXIParams) extends Bundle {
  val rid    = UInt(params.xAXI_ID_LEN.W)   //matches the arid of the request
  val rdata  = UInt(params.AXI_WIDTH.W)    //data payload
  val rresp  = UInt(2.W)            // OKAY=0, SLVERR=2, DECERR=3
  val rlast  = Bool()                 //this is the final beat of the burst
  val rvalid = Bool()                 //data is valid this cycle
}

//ar, aw, w — master drives valid and data fields, slave drives ready.
// These are not flipped because the dominant direction (valid + data) goes master -
// slave.ar, aw, w — master drives valid and data fields, slave drives ready.

//b, r — slave drives valid and data fields, master drives only ready.
// These are flipped because the dominant direction goes slave - master.

class AXI4Bundle(params: AXIParams) extends Bundle {
  val ar = Output(new AXI4_AR(params))
  val aw = Output(new AXI4_AW(params))
  val w  = Output(new AXI4_W(params))
  val b  = Input(new AXI4_B(params))
  val r  = Input(new AXI4_R(params))
  val arready = Input(Bool())
  val awready = Input(Bool())
  val wready  = Input(Bool())
  val bready  = Output(Bool())
  val rready  = Output(Bool())
}
