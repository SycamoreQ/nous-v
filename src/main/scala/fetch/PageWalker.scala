package fetch

import chisel3._
import chisel3.util._
import _root_.circt.stage.ChiselStage
import fetch.{BTUpdate, FetchOff, PredBranch}
import branch_pred._

/*
The walker receives a VPN (virtual page number, from the faulting fetch address) and returns a PPN (physical page number).

The OS has already written two levels of page tables into DRAM when it set up the process.
The satp CSR tells you where the root table lives — specifically satp.ppn gives you the physical page number of
the root table base.
The walker knows nothing else — it has to go fetch the mappings from DRAM.


io.memc_res is the response from the memory controller, not directly from DRAM.
The PageWalker does not talk to DRAM directly. It talks to a memory controller module (MemController) which sits
between the chip and the physical DRAM chips.
 */

class PageReq extends Bundle {
  val valid = Bool()
  val vpn = UInt(20.W)
  val rqID = UInt(2.W)
  val write = Bool()
}

class PageWalk_Res_Full extends Bundle {
  val valid       = Bool()
  val busy        = Bool()
  val rqID        = UInt(2.W)
  val vpn         = UInt(20.W)
  val ppn         = UInt(22.W)
  val rwx         = UInt(3.W)
  val user        = Bool()
  val globl       = Bool()
  val isSuperPage = Bool()
  val pageFault   = Bool()
}


class PageWalker(
                  NUM_RQ: Int = 1
                ) extends Module {

  val io = IO(new Bundle {
    val in_pwReq  = Input(Vec(NUM_RQ, new PageReq))
    val satp      = Input(new SATP)
    val out_pwRes = Output(Vec(NUM_RQ, new PageWalk_Res_Full))
    val clear     = Input(Bool())
    val memc_req  = Output(new MemController_Req)   // issues PT reads to DRAM
    val memc_res  = Input(new MemController_Res)    // receives PT data from DRAM
  })

  io.memc_req       := 0.U.asTypeOf(new MemController_Req)
  for (i <- 0 until NUM_RQ) {
    io.out_pwRes(i) := 0.U.asTypeOf(new PageWalk_Res_Full)
  }

  val sIDLE   = 0.U(2.W)
  val sWAIT_L1 = 1.U(2.W)
  val sWAIT_L2 = 2.U(2.W)
  val sDONE   = 3.U(2.W)

  for (i <- 0 until NUM_RQ) {
    val state    = RegInit(sIDLE)
    val vpn_r    = RegInit(0.U(20.W))   // latched VPN from request
    val rqID_r   = RegInit(0.U)          // latched request ID

    val l1_pte_r = RegInit(0.U(32.W))   // L1 PTE held across cycles data that comes back from DRAM after you issue that read.
    // register that gets loaded when memc_res.valid is true in WAIT_L1.

    // Result registers — filled in WAIT_L1 or WAIT_L2
    val ppn_r        = RegInit(0.U(22.W))
    val rwx_r        = RegInit(0.U(3.W))
    val user_r       = RegInit(false.B)
    val globl_r      = RegInit(false.B)
    val isSuperPage_r = RegInit(false.B)
    val pageFault_r  = RegInit(false.B)

    // L1 PTE address: (satp.ppn << 12) | (VPN[19:10] << 2)
    //satp.ppn shifted left 12 bits gives the byte address of the root table

    // VPN[19:10] is the upper 10 bits of the 20-bit VPN
    // address you send to DRAM to request the PTE.
    //VPN[19:10] shifted left 2 gives the byte offset of the specific entry within that table.
    val l1_pte_addr = Cat(io.satp.ppn, 0.U(12.W)) |
      Cat(vpn_r(19, 10), 0.U(2.W))

    // L2 PTE address: (l1_pte_r.PPN << 12) | (VPN[9:0] << 2)
    // l1_pte_r[31:10] is the PPN from the L1 entry
    val l2_pte_addr = Cat(l1_pte_r(31, 10), 0.U(12.W)) |
      Cat(vpn_r(9, 0), 0.U(2.W))


    switch(state) {

      is(sIDLE) {
        when(io.in_pwReq(i).valid) {
          // Latch request
          vpn_r  := io.in_pwReq(i).vpn
          rqID_r := io.in_pwReq(i).rqID

          // Issue L1 read to memory controller
          // l1_pte_addr is computed combinationally from vpn_r but vpn_r
          // won't be updated until next cycle, so compute directly here
          val l1_addr = Cat(io.satp.ppn, 0.U(12.W)) |
            Cat(io.in_pwReq(i).vpn(19, 10), 0.U(2.W))
          io.memc_req.valid := true.B
          io.memc_req.addr  := l1_addr
          io.memc_req.we    := false.B

          state := sWAIT_L1
        }
      }

      is(sWAIT_L1) {
        // Stay here until DRAM responds
        when(io.memc_res.valid) {
          // Latch the L1 PTE data
          l1_pte_r := io.memc_res.data

          // Check V bit (bit 0)
          when(!io.memc_res.data(0)) {
            pageFault_r := true.B
            state       := sDONE

            // Check R or X bits (bits 1 and 3) — superpage leaf
            // PPN comes directly from this entry and the walk is done.
          }.elsewhen(io.memc_res.data(1) || io.memc_res.data(3)) {
            ppn_r         := io.memc_res.data(31, 10)
            rwx_r         := io.memc_res.data(3, 1)
            user_r        := io.memc_res.data(4)
            globl_r       := io.memc_res.data(5)
            isSuperPage_r := true.B
            pageFault_r   := false.B
            state         := sDONE

            // Pointer PTE — issue L2 read
          }.otherwise {
            val l2_addr = Cat(io.memc_res.data(31, 10), 0.U(12.W)) |
              Cat(vpn_r(9, 0), 0.U(2.W))
            io.memc_req.valid := true.B
            io.memc_req.addr  := l2_addr
            io.memc_req.we    := false.B
            state             := sWAIT_L2
          }
        }
      }

      is(sWAIT_L2) {
        when(io.memc_res.valid) {
          // Check V bit
          when(!io.memc_res.data(0)) {
            pageFault_r := true.B

            // Pointer at L2 is illegal in Sv32
          }.elsewhen(!io.memc_res.data(1) && !io.memc_res.data(3)) {
            pageFault_r := true.B

            // Valid leaf
          }.otherwise {
            ppn_r       := io.memc_res.data(31, 10)
            rwx_r       := io.memc_res.data(3, 1)
            user_r      := io.memc_res.data(4)
            globl_r     := io.memc_res.data(5)
            pageFault_r := false.B
          }
          state := sDONE
        }
      }

      is(sDONE) {
        // Assert result for one cycle then return to IDLE
        io.out_pwRes(i).valid       := true.B
        io.out_pwRes(i).rqID        := rqID_r
        io.out_pwRes(i).vpn         := vpn_r
        io.out_pwRes(i).ppn         := ppn_r
        io.out_pwRes(i).rwx         := rwx_r
        io.out_pwRes(i).user        := user_r
        io.out_pwRes(i).globl       := globl_r
        io.out_pwRes(i).isSuperPage := isSuperPage_r
        io.out_pwRes(i).pageFault   := pageFault_r
        state := sIDLE
      }
    }
  }
}