auto CPU::Recompiler::pool(u32 address) -> Pool* {
  auto& pool = pools[address >> 8 & 0x1fffff];
  if(!pool) {
    pool = (Pool*)allocator.acquire(sizeof(Pool));
    memory::jitprotect(false);
    *pool = {};
    memory::jitprotect(true);
  }
  return pool;
}


auto CPU::Recompiler::block(u32 vaddr, u32 address, bool singleInstruction) -> Block* {
  if(auto block = pool(address)->blocks[address >> 2 & 0x3f]) return block;
  auto block = emit(vaddr, address, singleInstruction);
  pool(address)->blocks[address >> 2 & 0x3f] = block;
  memory::jitprotect(true);
  return block;
}

auto CPU::Recompiler::fastFetchBlock(u32 address) -> Block* {
  auto& pool = pools[address >> 8 & 0x1fffff];
  if(pool) return pool->blocks[address >> 2 & 0x3f];
  return nullptr;
}

#define USE_NEW_RECOMPILER 1

namespace {
  enum class InterruptEffect{
    None,
    EnableInterrupts,
    DisableInterrupts
  };
}

InterruptEffect checkIfInstructionAffectsInterrupts(CPU &cpu, u32 instruction);

#if USE_NEW_RECOMPILER
auto CPU::Recompiler::emit(u32 vaddr, u32 address, bool singleInstruction) -> Block* {
  if(unlikely(allocator.available() < 1_MiB)) {
    print("CPU allocator flush\n");
    allocator.release();
    reset();
  }

  auto block = (Block*)allocator.acquire(sizeof(Block));
  beginFunction(3); // three arguments

  Thread thread;
  bool hasBranched = 0;
  while(true) {
    u32 instruction = bus.read<Word>(address, thread, "Ares Recompiler");
    if(callInstructionPrologue) { // only for debugging: cpu.recompiler.callInstructionPrologue = tracer.instruction->enabled();
      mov32(reg(1), imm(instruction));
      call(&CPU::instructionPrologue);
    }
    InterruptEffect effect = checkIfInstructionAffectsInterrupts(cpu, instruction);
    bool mayTouchR0 = checkIfTargetIsZeroRegister(instruction);
    bool branched = emitEXECUTE(instruction);
    if(unlikely(instruction == 0x1000'ffff  //beq 0,0,<pc>
             || instruction == (2 << 26 | vaddr >> 2 & 0x3ff'ffff))) {  //j <pc>
      //accelerate idle loops
      mov32(reg(1), imm(64 * 2)); 
      call(&CPU::step); // generates a CPU::step(u32{128}) call
      // Q: How does the code above detect an idle loop (a busy loop)?
      // A: it checks if we branch back to current PC
      // Q: How does step(128) "accelerate" the loop?
      // A: "It effectively makes idle loops 128 times faster which will be enough to make them disappear from profiles" --Rasky
    }

    if (mayTouchR0) {
      //call(&CPU::instructionEpilogue);
      mov64(reg(0), imm(0));
    }
    
    // PROBLEM: CPU::branch state may change dynamically, must check it *in executed code*
    // Question: Is there a case where we definitely know branch state doesn't need to be updated?
    // The call itself is probably not an issue? It's a static branch and will be speculatively executed.
    call(&CPU::instructionEpilogueDontClearR0);

    vaddr += 4;
    address += 4;
    // Q: what is the check "(address & 0xfc) == 0"? Page boundary?
    if(hasBranched || (address & 0xfc) == 0 || singleInstruction) break;  //block boundary
    hasBranched = branched;
    testJumpEpilog(); // emits `if (CPU::instructionEpilogue() != 0) { goto epilogue; }`
    // Q: does it really emit a CMP after every instruction?
    // A: yes, confirmed by rasky on discord
  }
  jumpEpilog(); // emits `goto epilogue;`

  memory::jitprotect(false);
  block->code = endFunction(); // actually generate binary code

//print(hex(PC, 8L), " ", instructions, " ", size(), "\n");
  return block;
}
#else
auto CPU::Recompiler::emit(u32 vaddr, u32 address, bool singleInstruction) -> Block* {
  if(unlikely(allocator.available() < 1_MiB)) {
    print("CPU allocator flush\n");
    allocator.release();
    reset();
  }

  auto block = (Block*)allocator.acquire(sizeof(Block));
  beginFunction(3);

  Thread thread;
  bool hasBranched = 0;
  while(true) {
    u32 instruction = bus.read<Word>(address, thread, "Ares Recompiler");
    if(callInstructionPrologue) {
      mov32(reg(1), imm(instruction));
      call(&CPU::instructionPrologue);
    }
    bool branched = emitEXECUTE(instruction);
    if(unlikely(instruction == 0x1000'ffff  //beq 0,0,<pc>
             || instruction == (2 << 26 | vaddr >> 2 & 0x3ff'ffff))) {  //j <pc>
      //accelerate idle loops
      mov32(reg(1), imm(64 * 2));
      call(&CPU::step);
    }
    call(&CPU::instructionEpilogue);
    vaddr += 4;
    address += 4;
    if(hasBranched || (address & 0xfc) == 0 || singleInstruction) break;  //block boundary
    hasBranched = branched;
    testJumpEpilog();
  }
  jumpEpilog();

  memory::jitprotect(false);
  block->code = endFunction();

//print(hex(PC, 8L), " ", instructions, " ", size(), "\n");
  return block;
}

#endif

#define Sa  (instruction >>  6 & 31)
#define Rdn (instruction >> 11 & 31)
#define Rtn (instruction >> 16 & 31)
#define Rsn (instruction >> 21 & 31)
#define Fdn (instruction >>  6 & 31)
#define Fsn (instruction >> 11 & 31)
#define Ftn (instruction >> 16 & 31)

#define IpuBase   offsetof(IPU, r[16])
#define IpuReg(r) sreg(1), offsetof(IPU, r) - IpuBase
#define Rd        IpuReg(r[0]) + Rdn * sizeof(r64)
#define Rt        IpuReg(r[0]) + Rtn * sizeof(r64)
#define Rt32      IpuReg(r[0].u32) + Rtn * sizeof(r64)
#define Rs        IpuReg(r[0]) + Rsn * sizeof(r64)
#define Rs32      IpuReg(r[0].u32) + Rsn * sizeof(r64)
#define Lo        IpuReg(lo)
#define Hi        IpuReg(hi)

#define FpuBase   offsetof(FPU, r[16])
#define FpuReg(r) sreg(2), offsetof(FPU, r) - FpuBase
#define Fd        FpuReg(r[0]) + Fdn * sizeof(r64)
#define Fs        FpuReg(r[0]) + Fsn * sizeof(r64)
#define Ft        FpuReg(r[0]) + Ftn * sizeof(r64)

#define i16 s16(instruction)
#define n16 u16(instruction)
#define n26 u32(instruction & 0x03ff'ffff)

InterruptEffect checkIfInstructionAffectsInterrupts(CPU& cpu, u32 instruction) {
    // SCC
    if ((instruction >> 26) == 0x10) {
        // MTC0 Rt,Rd
        if ((instruction >> 21 & 0x1f) == 0x04) {

            u32 local_Rdn = (instruction >> 11 & 31);
            u32 local_Rtn = (instruction >> 16 & 31);

            //u32 Rdn_value = cpu.ipu.r[local_Rdn].u32 & 0x0f;
            u32 Rt_value = cpu.ipu.r[local_Rtn].u32;
            printf("MTC0 0x%x, 0x%x (Rtn = %u, Rdn = %u)\n", Rt_value, Rdn, local_Rtn, local_Rdn);
            if (local_Rdn == 12) {
                constexpr u32 C0_STATUS_IE = 0x00000001;
                constexpr u32 C0_STATUS_EXL = 0x00000002;
                constexpr u32 C0_STATUS_ERL = 0x00000004;

                printf("  set COP0 to %u. Interrupts: %s\n", Rt_value, (Rt_value & C0_STATUS_IE) ? "on" : "off");
                //PROBLEM: interrupt enable is done via C0_WRITE_STATUS(C0_STATUS() | (__interrupt_sr & C0_STATUS_IE))
                // so it doesn't show up as an immediate!
                // PROBLEM: interrupt disable is also dynamic (libdragon's interrupt.c)
                //         uint32_t sr = C0_STATUS();
                //         C0_WRITE_STATUS(sr & ~C0_STATUS_IE);
            }
            // lea(reg(1), Rt);
            // mov32(reg(2), imm(Rdn));
            // call(&CPU::MTC0); // (rt,rd) setControlRegister(rd, s32(rt.u32));
        }
    }
    return InterruptEffect::None;
}

auto CPU::Recompiler::emitEXECUTE(u32 instruction) -> bool {
  switch(instruction >> 26) {

  //SPECIAL
  case 0x00: {
    return emitSPECIAL(instruction);
  }

  //REGIMM
  case 0x01: {
    return emitREGIMM(instruction);
  }

  //J n26
  case 0x02: {
    mov32(reg(1), imm(n26));
    call(&CPU::J);
    return 1;
  }

  //JAL n26
  case 0x03: {
    mov32(reg(1), imm(n26));
    call(&CPU::JAL);
    return 1;
  }

  //BEQ Rs,Rt,i16
  case 0x04: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BEQ);
    return 1;
  }

  //BNE Rs,Rt,i16
  case 0x05: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BNE);
    return 1;
  }

  //BLEZ Rs,i16
  case 0x06: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLEZ);
    return 1;
  }

  //BGTZ Rs,i16
  case 0x07: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGTZ);
    return 1;
  }

  //ADDI Rt,Rs,i16
  case 0x08: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::ADDI);
    return 0;
  }

  //ADDIU Rt,Rs,i16
  case 0x09: {
    add32(reg(0), mem(Rs32), imm(i16));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rt), reg(0));
    return 0;
  }

  //SLTI Rt,Rs,i16
  case 0x0a: {
    cmp64(mem(Rs), imm(i16), set_slt);
    mov64_f(mem(Rt), flag_slt);
    return 0;
  }

  //SLTIU Rt,Rs,i16
  case 0x0b: {
    cmp64(mem(Rs), imm(i16), set_ult);
    mov64_f(mem(Rt), flag_ult);
    return 0;
  }

  //ANDI Rt,Rs,n16
  case 0x0c: {
    and64(mem(Rt), mem(Rs), imm(n16));
    return 0;
  }

  //ORI Rt,Rs,n16
  case 0x0d: {
    or64(mem(Rt), mem(Rs), imm(n16));
    return 0;
  }

  //XORI Rt,Rs,n16
  case 0x0e: {
    xor64(mem(Rt), mem(Rs), imm(n16));
    return 0;
  }

  //LUI Rt,n16
  case 0x0f: {
    mov64(mem(Rt), imm(s32(n16 << 16)));
    return 0;
  }

  //SCC
  case 0x10: {
    return emitSCC(instruction);
  }

  //FPU
  case 0x11: {
    return emitFPU(instruction);
  }

  //COP2
  case 0x12: {
    return emitCOP2(instruction);
  }

  //COP3
  case 0x13: {
    call(&CPU::COP3);
    return 1;
  }

  //BEQL Rs,Rt,i16
  case 0x14: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BEQL);
    return 1;
  }

  //BNEL Rs,Rt,i16
  case 0x15: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BNEL);
    return 1;
  }

  //BLEZL Rs,i16
  case 0x16: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLEZL);
    return 1;
  }

  //BGTZL Rs,i16
  case 0x17: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGTZL);
    return 1;
  }

  //DADDI Rt,Rs,i16
  case 0x18: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::DADDI);
    return 0;
  }

  //DADDIU Rt,Rs,i16
  case 0x19: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::DADDIU);
    return 0;
  }

  //LDL Rt,Rs,i16
  case 0x1a: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LDL);
    return 0;
  }

  //LDR Rt,Rs,i16
  case 0x1b: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LDR);
    return 0;
  }

  //INVALID
  case range4(0x1c, 0x1f): {
    call(&CPU::INVALID);
    return 1;
  }

  //LB Rt,Rs,i16
  case 0x20: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LB);
    return 0;
  }

  //LH Rt,Rs,i16
  case 0x21: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LH);
    return 0;
  }

  //LWL Rt,Rs,i16
  case 0x22: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWL);
    return 0;
  }

  //LW Rt,Rs,i16
  case 0x23: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LW);
    return 0;
  }

  //LBU Rt,Rs,i16
  case 0x24: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LBU);
    return 0;
  }

  //LHU Rt,Rs,i16
  case 0x25: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LHU);
    return 0;
  }

  //LWR Rt,Rs,i16
  case 0x26: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWR);
    return 0;
  }

  //LWU Rt,Rs,i16
  case 0x27: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWU);
    return 0;
  }

  //SB Rt,Rs,i16
  case 0x28: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SB);
    return 0;
  }

  //SH Rt,Rs,i16
  case 0x29: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SH);
    return 0;
  }

  //SWL Rt,Rs,i16
  case 0x2a: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SWL);
    return 0;
  }

  //SW Rt,Rs,i16
  case 0x2b: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SW);
    return 0;
  }

  //SDL Rt,Rs,i16
  case 0x2c: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SDL);
    return 0;
  }

  //SDR Rt,Rs,i16
  case 0x2d: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SDR);
    return 0;
  }

  //SWR Rt,Rs,i16
  case 0x2e: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SWR);
    return 0;
  }

  //CACHE op(offset),base
  case 0x2f: {
    mov32(reg(1), imm(instruction >> 16 & 31));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::CACHE);
    return 0;
  }

  //LL Rt,Rs,i16
  case 0x30: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LL);
    return 0;
  }

  //LWC1 Ft,Rs,i16
  case 0x31: {
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWC1);
    return 0;
  }

  //LWC2
  case 0x32: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //LWC3
  case 0x33: {
    call(&CPU::COP3);
    return 1;
  }

  //LLD Rt,Rs,i16
  case 0x34: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LLD);
    return 0;
  }

  //LDC1 Ft,Rs,i16
  case 0x35: {
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LDC1);
    return 0;
  }

  //LDC2
  case 0x36: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //LD Rt,Rs,i16
  case 0x37: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LD);
    return 0;
  }

  //SC Rt,Rs,i16
  case 0x38: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SC);
    return 0;
  }

  //SWC1 Ft,Rs,i16
  case 0x39: {
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SWC1);
    return 0;
  }

  //SWC2
  case 0x3a: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //SWC3
  case 0x3b: {
    call(&CPU::COP3);
    return 1;
  }

  //SCD Rt,Rs,i16
  case 0x3c: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SCD);
    return 0;
  }

  //SDC1 Ft,Rs,i16
  case 0x3d: {
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SDC1);
    return 0;
  }

  //SDC2
  case 0x3e: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //SD Rt,Rs,i16
  case 0x3f: {
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SD);
    return 0;
  }

  }

  return 0;
}

auto CPU::Recompiler::checkIfTargetIsZeroRegister(u32 instruction) -> bool {
  switch(instruction >> 26) {
  //SPECIAL
  case 0x00: {
      switch (instruction & 0x3f) {
      //MULT Rs,Rt
      case 0x18: {
        return 0; // placed in LO and HI
      }

      //MULTU Rs,Rt
      case 0x19: {
        return 0; // result placed in LO and HI
      }

      //DIV Rs,Rt
      case 0x1a: {
        return 0;
      }

      //DIVU Rs,Rt
      case 0x1b: {
        return 0;
      }

      //DMULT Rs,Rt
      case 0x1c: {
        return 0;
      }

      //DMULTU Rs,Rt
      case 0x1d: {
        return 0;
      }

      //DDIV Rs,Rt
      case 0x1e: {
        return 0;
      }

      //DDIVU Rs,Rt
      case 0x1f: {
        return 0;
      }

      //ADD Rd,Rs,Rt
      case 0x20: {
        return (Rdn == 0);
      }

      //ADDU Rd,Rs,Rt
      case 0x21: {
        return 1; // TODO uses reg(0)
      }

      //SUB Rd,Rs,Rt
      case 0x22: {
        return (Rdn == 0);
      }

      //SUBU Rd,Rs,Rt
      case 0x23: {
        return 1; //TODO uses reg(0)
      }

      //AND Rd,Rs,Rt
      case 0x24: {
        return (Rdn == 0); // TODO what is mem(Rd)?
      }

      //OR Rd,Rs,Rt
      case 0x25: {
        return (Rdn == 0); // TODO what is mem(Rd)?
      }

      //XOR Rd,Rs,Rt
      case 0x26: {
        return (Rdn == 0); // TODO what is mem(Rd)?
      }

      //NOR Rd,Rs,Rt
      case 0x27: {
        return 1; // TODO uses reg(0)
      }

      default: {
        return 1;
      }
      break;
      }
  }

  //ADDI Rt,Rs,i16
  case 0x08: 
  //ADDIU Rt,Rs,i16
  case 0x09:
  //SLTI Rt,Rs,i16
  case 0x0a: 
  //SLTIU Rt,Rs,i16
  case 0x0b: 
  //ANDI Rt,Rs,n16
  case 0x0c: 
  //ORI Rt,Rs,n16
  case 0x0d: 
  //XORI Rt,Rs,n16
  case 0x0e: 
  //LUI Rt,n16
  case 0x0f: {
    return (Rtn == 0);
  }
    default: { return 1; }
  }

  return 1;
}


auto CPU::Recompiler::emitSPECIAL(u32 instruction) -> bool {
  switch(instruction & 0x3f) {

  //SLL Rd,Rt,Sa
  case 0x00: {
    shl32(reg(0), mem(Rt32), imm(Sa));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //INVALID
  case 0x01: {
    call(&CPU::INVALID);
    return 1;
  }

  //SRL Rd,Rt,Sa
  case 0x02: {
    lshr32(reg(0), mem(Rt32), imm(Sa));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //SRA Rd,Rt,Sa
  case 0x03: {
    ashr64(reg(0), mem(Rt), imm(Sa));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //SLLV Rd,Rt,Rs
  case 0x04: {
    mshl32(reg(0), mem(Rt32), mem(Rs32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //INVALID
  case 0x05: {
    call(&CPU::INVALID);
    return 1;
  }

  //SRLV Rd,Rt,RS
  case 0x06: {
    mlshr32(reg(0), mem(Rt32), mem(Rs32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //SRAV Rd,Rt,Rs
  case 0x07: {
    and64(reg(1), mem(Rs), imm(31));
    ashr64(reg(0), mem(Rt), reg(1));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //JR Rs
  case 0x08: {
    lea(reg(1), Rs);
    call(&CPU::JR);
    return 1;
  }

  //JALR Rd,Rs
  case 0x09: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    call(&CPU::JALR);
    return 1;
  }

  //INVALID
  case range2(0x0a, 0x0b): {
    call(&CPU::INVALID);
    return 1;
  }

  //SYSCALL
  case 0x0c: {
    call(&CPU::SYSCALL);
    return 1;
  }

  //BREAK
  case 0x0d: {
    call(&CPU::BREAK);
    return 1;
  }

  //INVALID
  case 0x0e: {
    call(&CPU::INVALID);
    return 1;
  }

  //SYNC
  case 0x0f: {
    call(&CPU::SYNC);
    return 0;
  }

  //MFHI Rd
  case 0x10: {
    mov64(mem(Rd), mem(Hi));
    return 0;
  }

  //MTHI Rs
  case 0x11: {
    mov64(mem(Hi), mem(Rs));
    return 0;
  }

  //MFLO Rd
  case 0x12: {
    mov64(mem(Rd), mem(Lo));
    return 0;
  }

  //MTLO Rs
  case 0x13: {
    mov64(mem(Lo), mem(Rs));
    return 0;
  }

  //DSLLV Rd,Rt,Rs
  case 0x14: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    lea(reg(3), Rs);
    call(&CPU::DSLLV);
    return 0;
  }

  //INVALID
  case 0x15: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSRLV Rd,Rt,Rs
  case 0x16: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    lea(reg(3), Rs);
    call(&CPU::DSRLV);
    return 0;
  }

  //DSRAV Rd,Rt,Rs
  case 0x17: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    lea(reg(3), Rs);
    call(&CPU::DSRAV);
    return 0;
  }

  //MULT Rs,Rt
  case 0x18: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::MULT);
    return 0;
  }

  //MULTU Rs,Rt
  case 0x19: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::MULTU);
    return 0;
  }

  //DIV Rs,Rt
  case 0x1a: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DIV);
    return 0;
  }

  //DIVU Rs,Rt
  case 0x1b: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DIVU);
    return 0;
  }

  //DMULT Rs,Rt
  case 0x1c: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DMULT);
    return 0;
  }

  //DMULTU Rs,Rt
  case 0x1d: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DMULTU);
    return 0;
  }

  //DDIV Rs,Rt
  case 0x1e: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DDIV);
    return 0;
  }

  //DDIVU Rs,Rt
  case 0x1f: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DDIVU);
    return 0;
  }

  //ADD Rd,Rs,Rt
  case 0x20: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::ADD);
    return 0;
  }

  //ADDU Rd,Rs,Rt
  case 0x21: {
    add32(reg(0), mem(Rs32), mem(Rt32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //SUB Rd,Rs,Rt
  case 0x22: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::SUB);
    return 0;
  }

  //SUBU Rd,Rs,Rt
  case 0x23: {
    sub32(reg(0), mem(Rs32), mem(Rt32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //AND Rd,Rs,Rt
  case 0x24: {
    and64(mem(Rd), mem(Rs), mem(Rt));
    return 0;
  }

  //OR Rd,Rs,Rt
  case 0x25: {
    or64(mem(Rd), mem(Rs), mem(Rt));
    return 0;
  }

  //XOR Rd,Rs,Rt
  case 0x26: {
    xor64(mem(Rd), mem(Rs), mem(Rt));
    return 0;
  }

  //NOR Rd,Rs,Rt
  case 0x27: {
    or64(reg(0), mem(Rs), mem(Rt));
    xor64(reg(0), reg(0), imm(-1));
    mov64(mem(Rd), reg(0));
    return 0;
  }

  //INVALID
  case range2(0x28, 0x29): {
    call(&CPU::INVALID);
    return 1;
  }

  //SLT Rd,Rs,Rt
  case 0x2a: {
    cmp64(mem(Rs), mem(Rt), set_slt);
    mov64_f(mem(Rd), flag_slt);
    return 0;
  }

  //SLTU Rd,Rs,Rt
  case 0x2b: {
    cmp64(mem(Rs), mem(Rt), set_ult);
    mov64_f(mem(Rd), flag_ult);
    return 0;
  }

  //DADD Rd,Rs,Rt
  case 0x2c: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DADD);
    return 0;
  }

  //DADDU Rd,Rs,Rt
  case 0x2d: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DADDU);
    return 0;
  }

  //DSUB Rd,Rs,Rt
  case 0x2e: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DSUB);
    return 0;
  }

  //DSUBU Rd,Rs,Rt
  case 0x2f: {
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DSUBU);
    return 0;
  }

  //TGE Rs,Rt
  case 0x30: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TGE);
    return 0;
  }

  //TGEU Rs,Rt
  case 0x31: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TGEU);
    return 0;
  }

  //TLT Rs,Rt
  case 0x32: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TLT);
    return 0;
  }

  //TLTU Rs,Rt
  case 0x33: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TLTU);
    return 0;
  }

  //TEQ Rs,Rt
  case 0x34: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TEQ);
    return 0;
  }

  //INVALID
  case 0x35: {
    call(&CPU::INVALID);
    return 1;
  }

  //TNE Rs,Rt
  case 0x36: {
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TNE);
    return 0;
  }

  //INVALID
  case 0x37: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSLL Rd,Rt,Sa
  case 0x38: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa));
    call(&CPU::DSLL);
    return 0;
  }

  //INVALID
  case 0x39: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSRL Rd,Rt,Sa
  case 0x3a: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa));
    call(&CPU::DSRL);
    return 0;
  }

  //DSRA Rd,Rt,Sa
  case 0x3b: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa));
    call(&CPU::DSRA);
    return 0;
  }

  //DSLL32 Rd,Rt,Sa
  case 0x3c: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa+32));
    call(&CPU::DSLL);
    return 0;
  }

  //INVALID
  case 0x3d: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSRL32 Rd,Rt,Sa
  case 0x3e: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa+32));
    call(&CPU::DSRL);
    return 0;
  }

  //DSRA32 Rd,Rt,Sa
  case 0x3f: {
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa+32));
    call(&CPU::DSRA);
    return 0;
  }

  }

  return 0;
}

auto CPU::Recompiler::emitREGIMM(u32 instruction) -> bool {
  switch(instruction >> 16 & 0x1f) {

  //BLTZ Rs,i16
  case 0x00: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZ);
    return 0;
  }

  //BGEZ Rs,i16
  case 0x01: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZ);
    return 0;
  }

  //BLTZL Rs,i16
  case 0x02: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZL);
    return 0;
  }

  //BGEZL Rs,i16
  case 0x03: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZL);
    return 0;
  }

  //INVALID
  case range4(0x04, 0x07): {
    call(&CPU::INVALID);
    return 1;
  }

  //TGEI Rs,i16
  case 0x08: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TGEI);
    return 0;
  }

  //TGEIU Rs,i16
  case 0x09: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TGEIU);
    return 0;
  }

  //TLTI Rs,i16
  case 0x0a: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TLTI);
    return 0;
  }

  //TLTIU Rs,i16
  case 0x0b: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TLTIU);
    return 0;
  }

  //TEQI Rs,i16
  case 0x0c: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TEQI);
    return 0;
  }

  //INVALID
  case 0x0d: {
    call(&CPU::INVALID);
    return 1;
  }

  //TNEI Rs,i16
  case 0x0e: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TNEI);
    return 0;
  }

  //INVALID
  case 0x0f: {
    call(&CPU::INVALID);
    return 1;
  }

  //BLTZAL Rs,i16
  case 0x10: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZAL);
    return 0;
  }

  //BGEZAL Rs,i16
  case 0x11: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZAL);
    return 0;
  }

  //BLTZALL Rs,i16
  case 0x12: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZALL);
    return 0;
  }

  //BGEZALL Rs,i16
  case 0x13: {
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZALL);
    return 0;
  }

  //INVALID
  case range12(0x14, 0x1f): {
    call(&CPU::INVALID);
    return 1;
  }

  }

  return 0;
}

auto CPU::Recompiler::emitSCC(u32 instruction) -> bool {
  switch(instruction >> 21 & 0x1f) {

  //MFC0 Rt,Rd
  case 0x00: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::MFC0);
    return 0;
  }

  //DMFC0 Rt,Rd
  case 0x01: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::DMFC0);
    return 0;
  }

  //INVALID
  case range2(0x02, 0x03): {
    call(&CPU::INVALID);
    return 1;
  }

  //MTC0 Rt,Rd
  case 0x04: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::MTC0);
    return 0;
  }

  //DMTC0 Rt,Rd
  case 0x05: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::DMTC0);
    return 0;
  }

  //INVALID
  case range10(0x06, 0x0f): {
    call(&CPU::INVALID);
    return 1;
  }

  }

  switch(instruction & 0x3f) {

  //TLBR
  case 0x01: {
    call(&CPU::TLBR);
    return 0;
  }

  //TLBWI
  case 0x02: {
    call(&CPU::TLBWI);
    return 0;
  }

  //TLBWR
  case 0x06: {
    call(&CPU::TLBWR);
    return 0;
  }

  //TLBP
  case 0x08: {
    call(&CPU::TLBP);
    return 0;
  }

  //ERET
  case 0x18: {
    call(&CPU::ERET);
    return 1;
  }

  }

  return 0;
}

auto CPU::Recompiler::emitFPU(u32 instruction) -> bool {
  switch(instruction >> 21 & 0x1f) {

  //MFC1 Rt,Fs
  case 0x00: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Fsn));
    call(&CPU::MFC1);
    return 0;
  }

  //DMFC1 Rt,Fs
  case 0x01: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Fsn));
    call(&CPU::DMFC1);
    return 0;
  }

  //CFC1 Rt,Rd
  case 0x02: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::CFC1);
    return 0;
  }

  //DCFC1 Rt,Rd
  case 0x03: {
    call(&CPU::COP1UNIMPLEMENTED);
    return 1;
  }

  //MTC1 Rt,Fs
  case 0x04: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Fsn));
    call(&CPU::MTC1);
    return 0;
  }

  //DMTC1 Rt,Fs
  case 0x05: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Fsn));
    call(&CPU::DMTC1);
    return 0;
  }

  //CTC1 Rt,Rd
  case 0x06: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::CTC1);
    return 0;
  }

  //DCTC1 Rt,Rd
  case 0x07: {
    call(&CPU::COP1UNIMPLEMENTED);
    return 1;
  }

  //BC1 offset
  case 0x08: {
    mov32(reg(1), imm(instruction >> 16 & 1));
    mov32(reg(2), imm(instruction >> 17 & 1));
    mov32(reg(3), imm(i16));
    call(&CPU::BC1);
    return 1;
  }

  //INVALID
  case range7(0x09, 0x0f): {
    call(&CPU::INVALID);
    return 1;
  }

  }

  if((instruction >> 21 & 31) == 16)
  switch(instruction & 0x3f) {

  //FADD.S Fd,Fs,Ft
  case 0x00: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FADD_S);
    return 0;
  }

  //FSUB.S Fd,Fs,Ft
  case 0x01: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FSUB_S);
    return 0;
  }

  //FMUL.S Fd,Fs,Ft
  case 0x02: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FMUL_S);
    return 0;
  }

  //FDIV.S Fd,Fs,Ft
  case 0x03: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FDIV_S);
    return 0;
  }

  //FSQRT.S Fd,Fs
  case 0x04: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FSQRT_S);
    return 0;
  }

  //FABS.S Fd,Fs
  case 0x05: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FABS_S);
    return 0;
  }

  //FMOV.S Fd,Fs
  case 0x06: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FMOV_S);
    return 0;
  }

  //FNEG.S Fd,Fs
  case 0x07: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FNEG_S);
    return 0;
  }

  //FROUND.L.S Fd,Fs
  case 0x08: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FROUND_L_S);
    return 0;
  }

  //FTRUNC.L.S Fd,Fs
  case 0x09: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FTRUNC_L_S);
    return 0;
  }

  //FCEIL.L.S Fd,Fs
  case 0x0a: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCEIL_L_S);
    return 0;
  }

  //FFLOOR.L.S Fd,Fs
  case 0x0b: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FFLOOR_L_S);
    return 0;
  }

  //FROUND.W.S Fd,Fs
  case 0x0c: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FROUND_W_S);
    return 0;
  }

  //FTRUNC.W.S Fd,Fs
  case 0x0d: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FTRUNC_W_S);
    return 0;
  }

  //FCEIL.W.S Fd,Fs
  case 0x0e: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCEIL_W_S);
    return 0;
  }

  //FFLOOR.W.S Fd,Fs
  case 0x0f: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FFLOOR_W_S);
    return 0;
  }

  //FCVT.S.S Fd,Fs
  case 0x20: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_S_S);
    return 0;
  }

  //FCVT.D.S Fd,Fs
  case 0x21: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_D_S);
    return 0;
  }

  //FCVT.W.S Fd,Fs
  case 0x24: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_W_S);
    return 0;
  }

  //FCVT.L.S Fd,Fs
  case 0x25: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_L_S);
    return 0;
  }

  //FC.F.S Fs,Ft
  case 0x30: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_F_S);
    return 0;
  }

  //FC.UN.S Fs,Ft
  case 0x31: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_UN_S);
    return 0;
  }

  //FC.EQ.S Fs,Ft
  case 0x32: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_EQ_S);
    return 0;
  }

  //FC.UEQ.S Fs,Ft
  case 0x33: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_UEQ_S);
    return 0;
  }

  //FC.OLT.S Fs,Ft
  case 0x34: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_OLT_S);
    return 0;
  }

  //FC.ULT.S Fs,Ft
  case 0x35: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_ULT_S);
    return 0;
  }

  //FC.OLE.S Fs,Ft
  case 0x36: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_OLE_S);
    return 0;
  }

  //FC.ULE.S Fs,Ft
  case 0x37: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_ULE_S);
    return 0;
  }

  //FC.SF.S Fs,Ft
  case 0x38: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_SF_S);
    return 0;
  }

  //FC.NGLE.S Fs,Ft
  case 0x39: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGLE_S);
    return 0;
  }

  //FC.SEQ.S Fs,Ft
  case 0x3a: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_SEQ_S);
    return 0;
  }

  //FC.NGL.S Fs,Ft
  case 0x3b: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGL_S);
    return 0;
  }

  //FC.LT.S Fs,Ft
  case 0x3c: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_LT_S);
    return 0;
  }

  //FC.NGE.S Fs,Ft
  case 0x3d: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGE_S);
    return 0;
  }

  //FC.LE.S Fs,Ft
  case 0x3e: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_LE_S);
    return 0;
  }

  //FC.NGT.S Fs,Ft
  case 0x3f: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGT_S);
    return 0;
  }

  }

  if((instruction >> 21 & 31) == 17)
  switch(instruction & 0x3f) {

  //FADD.D Fd,Fs,Ft
  case 0x00: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FADD_D);
    return 0;
  }

  //FSUB.D Fd,Fs,Ft
  case 0x01: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FSUB_D);
    return 0;
  }

  //FMUL.D Fd,Fs,Ft
  case 0x02: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FMUL_D);
    return 0;
  }

  //FDIV.D Fd,Fs,Ft
  case 0x03: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    mov32(reg(3), imm(Ftn));
    call(&CPU::FDIV_D);
    return 0;
  }

  //FSQRT.D Fd,Fs
  case 0x04: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FSQRT_D);
    return 0;
  }

  //FABS.D Fd,Fs
  case 0x05: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FABS_D);
    return 0;
  }

  //FMOV.D Fd,Fs
  case 0x06: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FMOV_D);
    return 0;
  }

  //FNEG.D Fd,Fs
  case 0x07: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FNEG_D);
    return 0;
  }

  //FROUND.L.D Fd,Fs
  case 0x08: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FROUND_L_D);
    return 0;
  }

  //FTRUNC.L.D Fd,Fs
  case 0x09: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FTRUNC_L_D);
    return 0;
  }

  //FCEIL.L.D Fd,Fs
  case 0x0a: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCEIL_L_D);
    return 0;
  }

  //FFLOOR.L.D Fd,Fs
  case 0x0b: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FFLOOR_L_D);
    return 0;
  }

  //FROUND.W.D Fd,Fs
  case 0x0c: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FROUND_W_D);
    return 0;
  }

  //FTRUNC.W.D Fd,Fs
  case 0x0d: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FTRUNC_W_D);
    return 0;
  }

  //FCEIL.W.D Fd,Fs
  case 0x0e: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCEIL_W_D);
    return 0;
  }

  //FFLOOR.W.D Fd,Fs
  case 0x0f: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FFLOOR_W_D);
    return 0;
  }

  //FCVT.S.D Fd,Fs
  case 0x20: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_S_D);
    return 0;
  }

  //FCVT.D.D Fd,Fs
  case 0x21: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_D_D);
    return 0;
  }

  //FCVT.W.D Fd,Fs
  case 0x24: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_W_D);
    return 0;
  }

  //FCVT.L.D Fd,Fs
  case 0x25: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_L_D);
    return 0;
  }

  //FC.F.D Fs,Ft
  case 0x30: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_F_D);
    return 0;
  }

  //FC.UN.D Fs,Ft
  case 0x31: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_UN_D);
    return 0;
  }

  //FC.EQ.D Fs,Ft
  case 0x32: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_EQ_D);
    return 0;
  }

  //FC.UEQ.D Fs,Ft
  case 0x33: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_UEQ_D);
    return 0;
  }

  //FC.OLT.D Fs,Ft
  case 0x34: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_OLT_D);
    return 0;
  }

  //FC.ULT.D Fs,Ft
  case 0x35: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_ULT_D);
    return 0;
  }

  //FC.OLE.D Fs,Ft
  case 0x36: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_OLE_D);
    return 0;
  }

  //FC.ULE.D Fs,Ft
  case 0x37: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_ULE_D);
    return 0;
  }

  //FC.SF.D Fs,Ft
  case 0x38: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_SF_D);
    return 0;
  }

  //FC.NGLE.D Fs,Ft
  case 0x39: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGLE_D);
    return 0;
  }

  //FC.SEQ.D Fs,Ft
  case 0x3a: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_SEQ_D);
    return 0;
  }

  //FC.NGL.D Fs,Ft
  case 0x3b: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGL_D);
    return 0;
  }

  //FC.LT.D Fs,Ft
  case 0x3c: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_LT_D);
    return 0;
  }

  //FC.NGE.D Fs,Ft
  case 0x3d: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGE_D);
    return 0;
  }

  //FC.LE.D Fs,Ft
  case 0x3e: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_LE_D);
    return 0;
  }

  //FC.NGT.D Fs,Ft
  case 0x3f: {
    mov32(reg(1), imm(Fsn));
    mov32(reg(2), imm(Ftn));
    call(&CPU::FC_NGT_D);
    return 0;
  }

  }

  if((instruction >> 21 & 31) == 20)
  switch(instruction & 0x3f) {    
  case range8(0x08, 0x0f): {
    call(&CPU::COP1UNIMPLEMENTED);
    return 1;
  }

  case range2(0x24, 0x25): {
    call(&CPU::COP1UNIMPLEMENTED);
    return 1;
  }

  //FCVT.S.W Fd,Fs
  case 0x20: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_S_W);
    return 0;
  }

  //FCVT.D.W Fd,Fs
  case 0x21: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_D_W);
    return 0;
  }

  }

  if((instruction >> 21 & 31) == 21)
  switch(instruction & 0x3f) {
  case range8(0x08, 0x0f): {
    call(&CPU::COP1UNIMPLEMENTED);
    return 1;
  }
  case range2(0x24, 0x25): {
    call(&CPU::COP1UNIMPLEMENTED);
    return 1;
  }

  //FCVT.S.L
  case 0x20: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_S_L);
    return 0;
  }

  //FCVT.D.L
  case 0x21: {
    mov32(reg(1), imm(Fdn));
    mov32(reg(2), imm(Fsn));
    call(&CPU::FCVT_D_L);
    return 0;
  }

  }

  return 0;
}

auto CPU::Recompiler::emitCOP2(u32 instruction) -> bool {
  switch(instruction >> 21 & 0x1f) {

  //MFC2 Rt,Rd
  case 0x00: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::MFC2);
    return 0;
  }

  //DMFC2 Rt,Rd
  case 0x01: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::DMFC2);
    return 0;
  }

  //CFC2 Rt,Rd
  case 0x02: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::CFC2);
    return 0;
  }

  //INVALID
  case 0x03: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //MTC0 Rt,Rd
  case 0x04: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::MTC2);
    return 0;
  }

  //DMTC2 Rt,Rd
  case 0x05: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::DMTC2);
    return 0;
  }

  //CTC2 Rt,Rd
  case 0x06: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::CTC2);
    return 0;
  }

  //INVALID
  case range9(0x07, 0x0f): {
    call(&CPU::COP2INVALID);
    return 1;
  }

  }
  return 0;
}

#undef Sa
#undef Rdn
#undef Rtn
#undef Rsn
#undef Fdn
#undef Fsn
#undef Ftn
#undef IpuBase
#undef IpuReg
#undef Rd
#undef Rt
#undef Rt32
#undef Rs
#undef Rs32
#undef Lo
#undef Hi
#undef FpuBase
#undef FpuReg
#undef Fd
#undef Fs
#undef Ft
#undef i16
#undef n16
#undef n26
