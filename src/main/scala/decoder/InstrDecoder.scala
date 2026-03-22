package  decoder

import branch_pred.IS_UOp
import chisel3._
import chisel3.util._
import decoder.{D_UOp, DecodeBranch, DecodeState, OpcodeConst}
import fetch.{IFetchFault, PD_Instr, iFetchParams}


class InstrDecoder(params: iFetchParams) extends Module {
  val io = IO(new Bundle {
    val in = Input(new PD_Instr)
    val decState = Input(new DecodeState)
    val inBranch = Input(new DecodeBranch)
    val out = Output(new D_UOp)
    val outBranch = Output(new DecodeBranch)
  })

  val decBranch = WireDefault(0.U.asTypeOf(new DecodeBranch))
  val invalidEnc = WireDefault(true.B)
  val instr32 = RegInit(0.U(32.W))
  val instr16 = RegInit(0.U(32.W))
  val uop = WireDefault(0.U.asTypeOf(new D_UOp))

  val instr = io.in.bits // full 32-bit instruction word
  val opcode = instr(6, 0) // 7-bit opcode field
  val funct3 = instr(14, 12)
  val funct7 = instr(31, 25)
  // upper 5 bits instr(31,27) are funct5 which identifies the atomic operation
  // and bits instr(26,25) are the aq (acquire) and rl (release)
  val funct5 = instr(31,27)
  val rs1 = instr(19, 15)
  val rs2 = instr(24, 20)
  val rd = instr(11, 7)

  uop.valid := io.in.valid && !io.inBranch.taken
  uop.fetchID := io.in.fetchID
  uop.fetchOffs := io.in.fetchID

  val imm_i = Cat(Fill(20, instr(31)), instr(31, 20))
  val imm_s = Cat(Fill(20, instr(31)), instr(31, 25), instr(11, 7))
  val imm_b = Cat(Fill(19, instr(31)), instr(31), instr(7),
    instr(30, 25), instr(11, 8), 0.U(1.W))
  val imm_u = Cat(instr(31, 12), 0.U(12.W))
  val imm_j = Cat(Fill(11, instr(31)), instr(31), instr(19, 12),
    instr(20), instr(30, 21), 0.U(1.W))

  uop.imm := MuxLookup(opcode , 0.U)(Seq(
    OpcodeConst.OPC_LUI     -> imm_u,
    OpcodeConst.OPC_AUIPC   -> imm_u,
    OpcodeConst.OPC_JAL     -> imm_j,
    OpcodeConst.OPC_JALR    -> imm_i,
    OpcodeConst.OPC_LOAD    -> imm_i,
    OpcodeConst.OPC_REG_IMM -> imm_i,
    OpcodeConst.OPC_ENV     -> imm_i,
    OpcodeConst.OPC_BRANCH  -> imm_b,
    OpcodeConst.OPC_STORE   -> imm_s
  ))

  uop.imm12 := instr(31, 20)

  // If the fetch unit detected a fault while fetching this instruction, the instruction bits themselves are meaningless.
  // You do not decode them at all. Instead you convert the fault directly into a trap micro-op.

  //Do not set decBranch here — the fetch unit already knows about the fault and has stopped fetching.
  // The trap handler will redirect the PC.

  when(io.in.fault =/= IFetchFault.IF_FAULT_NONE) {
    uop.fu        := FU_t.FU_TRAP
    uop.compressed := io.in.compressed
    invalidEnc    := false.B

    switch(io.in.fault) {
      is(IFetchFault.IF_INTERRUPT) {
        uop.opcode := TRAP_Op.TRAP_V_INTERRUPT.asUInt
      }
      is(IFetchFault.IF_ACCESS_FAULT) {
        uop.opcode := TRAP_Op.TRAP_I_ACC_FAULT.asUInt
      }
      is(IFetchFault.IF_PAGE_FAULT) {
        uop.opcode := TRAP_Op.TRAP_I_PAGE_FAULT.asUInt
      }
    }
  }

  switch(opcode) {

    is(OpcodeConst.OPC_LUI) {
      uop.fu     := FU_t.FU_INT
      uop.rs1    := 0.U
      uop.rs2    := 0.U
      uop.immB   := true.B
      uop.rd     := rd
      uop.opcode := INT_Op.INT_LUI.asUInt
      invalidEnc := false.B
    }

    is(OpcodeConst.OPC_AUIPC) {
      uop.fu     := FU_t.FU_BRANCH
      uop.rs1    := 0.U
      uop.rs2    := 0.U
      uop.rd     := rd
      uop.opcode := BR_Op.BR_AUIPC.asUInt
      invalidEnc := false.B
    }

    is(OpcodeConst.OPC_JAL) {
      uop.fu     := FU_t.FU_BRANCH
      uop.rs1    := 0.U
      uop.rs2    := 0.U
      uop.immB   := true.B
      uop.rd     := rd
      uop.opcode := BR_Op.BR_JAL.asUInt
      invalidEnc := false.B
      // Jumps that discard their result need no execution — eliminate at rename
      when(rd === 0.U) {
        uop.fu := FU_t.FU_RN
      }
    }

    is(OpcodeConst.OPC_JALR) {
      uop.fu    := FU_t.FU_BRANCH
      uop.rs1   := rs1
      uop.rs2   := 0.U
      uop.immB  := true.B
      uop.rd    := rd
      uop.imm12 := instr(31, 20)
      invalidEnc := false.B

      // Opcode selection based on rd and rs1
      // cond0: destination is a link register (ra=x1 or x5)
      // cond1: source is a link register
      // cond2: source and destination are the same register
      val cond0 = rd  === 1.U || rd  === 5.U
      val cond1 = rs1 === 1.U || rs1 === 5.U
      val cond2 = rd  === rs1

      when(!cond0 && !cond1) {
        // neither is a link reg — plain jump, no return prediction
        uop.opcode := BR_Op.BR_V_JR.asUInt
      }.elsewhen(!cond0 && cond1) {
        // source is link reg, dest is not — return
        uop.opcode := BR_Op.BR_V_RET.asUInt
      }.elsewhen(cond0 && !cond1) {
        // dest is link reg, source is not — call
        uop.opcode := BR_Op.BR_V_JALR.asUInt
      }.elsewhen(cond0 && cond1 && !cond2) {
        // both are link regs but different — return then call, treat as jump
        uop.opcode := BR_Op.BR_V_JR.asUInt
      }.otherwise {
        // both are link regs and same register — call
        uop.opcode := BR_Op.BR_V_JALR.asUInt
      }

      // imm carries the speculated target for the ALU to verify
      // If the branch predictor already predicted taken use its target
      // Otherwise use PC + 2 (compressed) or PC + 4 (regular)
      when(io.in.predTaken) {
        uop.imm := Cat(io.in.predTarget, 0.U(1.W))
      }.otherwise {
        uop.imm := Cat(io.in.pc + Mux(io.in.compressed, 1.U, 2.U), 0.U(1.W))
      }
    }

    is(OpcodeConst.OPC_LOAD) {
      uop.fu   := FU_t.FU_AGU
      uop.rs1  := rs1
      uop.rs2  := 0.U
      uop.immB := true.B
      uop.rd   := rd
      // funct3 selects load width and signedness
      switch(funct3) {
        is(0.U) { uop.opcode := LSU_Op.LSU_LB.asUInt;  invalidEnc := false.B }
        is(1.U) { uop.opcode := LSU_Op.LSU_LH.asUInt;  invalidEnc := false.B }
        is(2.U) { uop.opcode := LSU_Op.LSU_LW.asUInt;  invalidEnc := false.B }
        is(4.U) { uop.opcode := LSU_Op.LSU_LBU.asUInt; invalidEnc := false.B }
        is(5.U) { uop.opcode := LSU_Op.LSU_LHU.asUInt; invalidEnc := false.B }
        // funct3 values 3, 6, 7 are reserved — invalidEnc stays true
      }
    }

    is(OpcodeConst.OPC_STORE) {
      uop.fu   := FU_t.FU_AGU
      uop.rs1  := rs1
      uop.rs2  := rs2
      uop.immB := true.B
      uop.rd   := 0.U
      switch(funct3) {
        is(0.U) { uop.opcode := LSU_Op.LSU_SB.asUInt; invalidEnc := false.B }
        is(1.U) { uop.opcode := LSU_Op.LSU_SH.asUInt; invalidEnc := false.B }
        is(2.U) { uop.opcode := LSU_Op.LSU_SW.asUInt; invalidEnc := false.B }
        // funct3 values 3-7 are reserved — invalidEnc stays true
      }
    }

    is(OpcodeConst.OPC_BRANCH) {
      uop.fu   := FU_t.FU_BRANCH
      uop.rs1  := rs1
      uop.rs2  := rs2
      uop.immB := false.B
      uop.rd   := 0.U
      switch(funct3) {
        is(0.U) { uop.opcode := BR_Op.BR_BEQ.asUInt;  invalidEnc := false.B }
        is(1.U) { uop.opcode := BR_Op.BR_BNE.asUInt;  invalidEnc := false.B }
        is(4.U) { uop.opcode := BR_Op.BR_BLT.asUInt;  invalidEnc := false.B }
        is(5.U) { uop.opcode := BR_Op.BR_BGE.asUInt;  invalidEnc := false.B }
        is(6.U) { uop.opcode := BR_Op.BR_BLTU.asUInt; invalidEnc := false.B }
        is(7.U) { uop.opcode := BR_Op.BR_BGEU.asUInt; invalidEnc := false.B }
        // funct3 values 2 and 3 are reserved — invalidEnc stays true
      }
    }

    is(OpcodeConst.OPC_REG_IMM) {
      uop.fu   := FU_t.FU_INT
      uop.rs1  := rs1
      uop.rs2  := 0.U
      uop.rd   := rd
      uop.immB := true.B

      switch(funct3) {
        is(0.U) { uop.opcode := INT_Op.INT_ADD.asUInt;  invalidEnc := false.B }
        is(1.U) {
          // SLLI — only valid when funct7 == 0
          when(funct7 === 0.U) {
            uop.opcode := INT_Op.INT_SLL.asUInt
            invalidEnc := false.B
          }
        }
        is(2.U) { uop.opcode := INT_Op.INT_SLT.asUInt;  invalidEnc := false.B }
        is(3.U) { uop.opcode := INT_Op.INT_SLTU.asUInt; invalidEnc := false.B }
        is(4.U) { uop.opcode := INT_Op.INT_XOR.asUInt;  invalidEnc := false.B }
        is(5.U) {
          // SRLI when funct7==0, SRAI when funct7==0x20, else illegal
          when(funct7 === 0.U) {
            uop.opcode := INT_Op.INT_SRL.asUInt
            invalidEnc := false.B
          }.elsewhen(funct7 === 0x20.U) {
            uop.opcode := INT_Op.INT_SRA.asUInt
            invalidEnc := false.B
          }
        }
        is(6.U) { uop.opcode := INT_Op.INT_OR.asUInt;   invalidEnc := false.B }
        is(7.U) { uop.opcode := INT_Op.INT_AND.asUInt;  invalidEnc := false.B }
      }

      // li rd, imm elimination — addi rd, x0, small_imm needs no execution
      when(uop.opcode === INT_Op.INT_ADD.asUInt && rs1 === 0.U &&
        uop.imm(11) === uop.imm(10) && uop.imm(11) === uop.imm(9) &&
        uop.imm(11) === uop.imm(8)  && uop.imm(11) === uop.imm(7) &&
        uop.imm(11) === uop.imm(6)  && uop.imm(11) === uop.imm(5)) {
        uop.fu := FU_t.FU_RN
      }
    }

    is (OpcodeConst.OPC_REG_REG) {
      uop.rs1 := rs1
      uop.rs2 := rs2
      uop.immB := false.B
      uop.rd := rd
      uop.fu := FU_t.FU_INT

      switch(funct7) {
        is(0.U) {
          switch(funct3) {
            is(0.U) {
              uop.opcode := INT_Op.INT_ADD.asUInt; invalidEnc := false.B
            }
            is(1.U) {
              uop.opcode := INT_Op.INT_SLL.asUInt; invalidEnc := false.B
            }
            is(2.U) {
              uop.opcode := INT_Op.INT_SLT.asUInt; invalidEnc := false.B
            }
            is(3.U) {
              uop.opcode := INT_Op.INT_SLTU.asUInt; invalidEnc := false.B
            }
            is(4.U) {
              uop.opcode := INT_Op.INT_XOR.asUInt; invalidEnc := false.B
            }
            is(5.U) {
              uop.opcode := INT_Op.INT_SRL.asUInt; invalidEnc := false.B
            }
            is(6.U) {
              uop.opcode := INT_Op.INT_OR.asUInt; invalidEnc := false.B
            }
            is(7.U) {
              uop.opcode := INT_Op.INT_AND.asUInt; invalidEnc := false.B
            }
          }
        }

        is(0x20.U) {
          switch(funct3) {
            is(0.U) {
              uop.opcode := INT_Op.INT_SUB.asUInt; invalidEnc := false.B
            }
            is(5.U) {
              uop.opcode := INT_Op.INT_SRA.asUInt; invalidEnc := false.B
            }
          }
        }

        is(1.U) {
          // M extension — multiply and divide
          // funct3 < 4 → multiply, funct3 >= 4 → divide
          uop.rs1 := rs1
          uop.rs2 := rs2
          uop.rd := rd
          uop.immB := false.B

          when(funct3 < 4.U) {
            uop.fu := FU_t.FU_MUL
            switch(funct3) {
              is(0.U) {
                uop.opcode := MUL_Op.MUL_MUL.asUInt; invalidEnc := false.B
              }
              is(1.U) {
                uop.opcode := MUL_Op.MUL_MULH.asUInt; invalidEnc := false.B
              }
              is(2.U) {
                uop.opcode := MUL_Op.MUL_MULSU.asUInt; invalidEnc := false.B
              }
              is(3.U) {
                uop.opcode := MUL_Op.MUL_MULU.asUInt; invalidEnc := false.B
              }
            }
          }.otherwise {
            uop.fu := FU_t.FU_DIV
            switch(funct3) {
              is(4.U) {
                uop.opcode := DIV_Op.DIV_DIV.asUInt; invalidEnc := false.B
              }
              is(5.U) {
                uop.opcode := DIV_Op.DIV_DIVU.asUInt; invalidEnc := false.B
              }
              is(6.U) {
                uop.opcode := DIV_Op.DIV_REM.asUInt; invalidEnc := false.B
              }
              is(7.U) {
                uop.opcode := DIV_Op.DIV_REMU.asUInt; invalidEnc := false.B
              }
              invalidEnc := true.B //handled later
            }
          }
        }
      }

    is(OpcodeConst.OPC_FENCE) {
      switch(funct3) {
        is(0.U) {
          uop.fu     := FU_t.FU_RN
          invalidEnc := false.B
        }
        is(1.U) {
          // FENCE.I — serializing, must drain pipeline and flush ICache
          uop.fu        := FU_t.FU_INT
          uop.opcode    := INT_Op.INT_SYS.asUInt
          invalidEnc    := false.B
          decBranch.taken    := true.B
          decBranch.wfi      := true.B
          decBranch.fetchID  := uop.fetchID
          decBranch.fetchOffs := uop.fetchOffs
        }
      }
    }

    def takeBranch() = {
      decBranch.taken     := true.B
      decBranch.wfi       := true.B
      decBranch.fetchID   := uop.fetchID
      decBranch.fetchOffs := uop.fetchOffs
    }

    is (OpcodeConst.OPC_ENV) {
      switch (funct3) {
        is (0.U) {
          when (instr(31 , 20) === 0.U  && rs1 === 0.U && rd === 0.U) {
            uop.fu := FU_t.FU_TRAP
            uop.opcode := TRAP_Op.TRAP_ECALL_M.asUInt // transfers control to the OS or hypervisor
            uop.immB := true.B
            uop.rs1 := 0.U
            uop.rs2 := 0.U
            uop.rd := 0.U
            decBranch.taken := true.B
            decBranch.wfi := true.B
            decBranch.fetchID := uop.fetchID
            decBranch.fetchOffs := uop.fetchOffs
            invalidEnc := false.B
          }

          when (instr(31 , 20) === 1.U && rs1 === 0.U && rd === 0.U) {
            uop.fu := FU_t.FU_TRAP
            uop.opcode := TRAP_Op.TRAP_BREAK.asUInt // transfers control to a debugger
            uop.rs1 := 0.U
            uop.rs2 := 0.U
            uop.rd := 0.U
            decBranch.taken := true.B
            decBranch.wfi := true.B
            decBranch.fetchID := uop.fetchID
            decBranch.fetchOffs := uop.fetchOffs
            invalidEnc := false.B
          }

          when (funct7 === 0x18.U  && rs2 === 2.U && rs1 === 0.U && rd === 0.U ){
            uop.fu := FU_t.FU_CSR
            //. Returns from a machine-mode trap handler back to the interrupted code.
            // It restores pc from mepc and privilege level from mstatus
            uop.opcode := CSR_Op.CSR_MRET.asUInt
            decBranch.taken := true.B
            decBranch.wfi := true.B
            decBranch.fetchID := uop.fetchID
            decBranch.fetchOffs := uop.fetchOffs
            invalidEnc := false.B
          }

          // both of these do not take any input registers (rs1 and rs2) they only manipulate CSR

          when (funct7 === 0x08.U && rs2 === 2.U && rs1 === 0.U && rd === 0.U){
            uop.fu := FU_t.FU_CSR
            //Supervisor Return. Same concept as MRET but returns from supervisor-mode traps
            uop.opcode := CSR_Op.CSR_SRET.asUInt
            decBranch.taken := true.B
            decBranch.wfi := true.B
            decBranch.fetchID := uop.fetchID
            decBranch.fetchOffs := uop.fetchOffs
            invalidEnc := false.B
          }

          when (funct7 === 0x08.U && rs2 === 5.U && rs1 === 0.U && rd === 0.U){
            //Wait For Interrupt. Hints to the processor to stop executing and wait until an interrupt arrives
            when (io.decState.allowWFI === true.B) {
              uop.fu := FU_t.FU_RN
              decBranch.taken := true.B
              decBranch.wfi := true.B
              decBranch.fetchID := uop.fetchID
              decBranch.fetchOffs := uop.fetchOffs
              invalidEnc := false.B
            }
          }

          when (funct7 === 0x09.U && rd === 0.U) {
            //Supervisor Fence Virtual Memory Address.
            // Flushes TLB entries. Required after modifying page tables so the hardware TLB reflects the new mappings
            uop.rs1 := rs1
            uop.rs2 := rs2
            uop.fu := FU_t.FU_TRAP

            when (io.decState.allowSFENCE === true.B){
              uop.opcode := TRAP_Op.TRAP_V_SFENCE_VMA.asUInt
            }.otherwise{
              uop.opcode := TRAP_Op.TRAP_ILLEGAL_INSTR
            }
            decBranch.taken := true.B
            decBranch.wfi := true.B
            decBranch.fetchID := uop.fetchID
            decBranch.fetchOffs := uop.fetchOffs
            invalidEnc := false.B
          }
        }

        is (1.U) {uop.fu := FU_t.FU_CSR ; uop.opcode := CSR_Op.CSR_RW.asUInt ; uop.rs1 := rs1 ; uop.rd := rd ; uop.imm := instr(31 , 20) ; invalidEnc := false.B}
        is (2.U) {
          uop.fu := FU_t.FU_CSR
          uop.rs1 := rs1
          uop.rd := rd

          when (rs1 === 0.U) {
            uop.opcode := CSR_Op.CSR_R.asUInt
          }.otherwise{
            uop.opcode := CSR_Op.CSR_RS.asUInt
          }

          invalidEnc := false.B
        }

        is (3.U) {
          uop.fu := FU_t.FU_CSR
          uop.rs1 := rs1
          uop.rd := rd

          when (rs1 === 0.U) {
            uop.opcode := CSR_Op.CSR_R.asUInt
          }.otherwise{
            uop.opcode := CSR_Op.CSR_RC.asUInt
          }

          invalidEnc := false.B
        }

        is (5.U) {
          //atomically reads the current value of a CSR register into rd, then writes a new value into that CSR.
          // uses the imm field to read/write and not a GPR
          uop.fu := FU_t.FU_CSR
          uop.rd := rd
          uop.opcode := CSR_Op.CSR_RW_I.asUInt
          uop.imm := Cat(instr(19,15), instr(31,20))
          invalidEnc := false.B
        }

        is (6.U) {
          uop.fu := FU_t.FU_CSR
          uop.rd := rd

          when (rs1 === 0.U) {
            uop.opcode := CSR_Op.CSR_R.asUInt
          }.otherwise{
            uop.opcode := CSR_Op.CSR_RS_I.asUInt
          }

          invalidEnc := false.B
        }

        is (7.U ) {
          uop.fu := FU_t.FU_CSR
          uop.rd := rd

          when (rs1 === 0.U) {
            uop.opcode := CSR_Op.CSR_R.asUInt
          }.otherwise{
            uop.opcode := CSR_Op.CSR_RC_I.asUInt
          }
          invalidEnc := false.B
        }
      }
    }
  }

    is(OpcodeConst.OPC_ATOMIC) {
      // funct3 must be 010 for 32-bit word operations
      // any other funct3 is reserved — invalidEnc stays true
      when(funct3 === 2.U) {
        uop.rs1 := rs1
        uop.rs2 := rs2
        uop.rd  := rd

        switch(funct5) {
          is("b00010".U) {
            // LR.W — load reserved, rs2 must be zero
            when(rs2 === 0.U) {
              uop.fu     := FU_t.FU_AGU
              uop.opcode := LSU_Op.LSU_LR_W.asUInt
              uop.rs2    := 0.U
              uop.immB   := false.B
              invalidEnc := false.B
            }
          }
          is("b00011".U) {
            // SC.W — store conditional
            uop.fu     := FU_t.FU_AGU
            uop.opcode := LSU_Op.LSU_SC_W.asUInt
            uop.immB   := true.B
            invalidEnc := false.B
          }
          is("b00001".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOSWAP_W.asUInt
            invalidEnc := false.B
          }
          is("b00000".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOADD_W.asUInt
            invalidEnc := false.B
          }
          is("b00100".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOXOR_W.asUInt
            invalidEnc := false.B
          }
          is("b01100".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOAND_W.asUInt
            invalidEnc := false.B
          }
          is("b01000".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOOR_W.asUInt
            invalidEnc := false.B
          }
          is("b10000".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOMIN_W.asUInt
            invalidEnc := false.B
          }
          is("b10100".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOMAX_W.asUInt
            invalidEnc := false.B
          }
          is("b11000".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOMINU_W.asUInt
            invalidEnc := false.B
          }
          is("b11100".U) {
            uop.fu     := FU_t.FU_ATOMIC
            uop.opcode := ATOMIC_Op.ATOMIC_AMOMAXU_W.asUInt
            invalidEnc := false.B
          }
        }
      }
    }
  }

  //The pipeline needs to drain and take the illegal instruction trap before anything else executes.
  // After opcode switch — catch-all for unrecognized encodings
  when(invalidEnc) {
    uop.fu     := FU_t.FU_TRAP
    uop.opcode := TRAP_Op.TRAP_ILLEGAL_INSTR.asUInt
    uop.rd     := 0.U
    decBranch.taken     := true.B
    decBranch.wfi       := true.B
    decBranch.fetchID   := uop.fetchID
    decBranch.fetchOffs := uop.fetchOffs
  }
}
