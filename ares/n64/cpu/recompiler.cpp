#include <cstdint>
#include <chrono>
#include <cstdlib>

#define PROF_BEGIN(name) call(&Profiler::begin, &profiler, name, __LINE__);
#define PROF_END(name) call(&Profiler::end, &profiler, name);

namespace {
struct Profiler {
  enum Name : int {
    TOTAL=0,
    FPU,
    MATHS,
    //MULT,
    //DIV,
    LOAD,
    STORE,
    BRANCH,
    MOVE,
    SCC,
    IDLE_SKIP,
    EPILOGUE,
    COP2,
    COP3,
    TRAP,
    NUM_SECTIONS,
  };
  const char* sectionNames[NUM_SECTIONS] = {
    "TOTAL",
    "FPU",
    "MATHS",
    //"  MULT",
    //"  DIV",
    "LOAD",
    "STORE",
    "BRANCH",
    "MOVE",
    "SCC",
    "IDLE_SKIP",
    "EPILOGUE",
    "COP2",
    "COP3",
    "TRAP",
  };
  struct Section {
    const char* name;
    uint64_t numCalls;
    uint64_t nanoSum;
    std::chrono::time_point<std::chrono::high_resolution_clock> timeStart;
    bool open;
    int lastline;
  };

  Section sections[NUM_SECTIONS]{};
  //std::chrono::time_point<std::chrono::high_resolution_clock> started{};
  uint64_t totalCalls = 0;

  void begin(int section, int line) {
    sections[section].timeStart = std::chrono::high_resolution_clock::now();
    if (sections[section].open) {
      printf("Section %d = %s was already open from line %d\n", section, sectionNames[section], sections[section].lastline);
      exit(1);
    }
    sections[section].open = true;
    sections[section].lastline = line;
    totalCalls++;
  }

  void end(int section) {
    auto took = std::chrono::duration<uint64_t, std::nano>(std::chrono::high_resolution_clock::now() - sections[section].timeStart);
    sections[section].nanoSum += took.count();
    sections[section].numCalls++;
    sections[section].open = false;
    if (section == TOTAL && totalCalls >= 500'000'000ul) {
      show();
      exit(1);
    }
  }

  void show() {
    uint64_t all = 0;
    for (int i=0;i<Profiler::Name::NUM_SECTIONS;i++) {
      if (sections[i].open) {
        printf("Section %d = %s was open at end\n", i, sectionNames[i]);
        exit(1);
      }
      if (sectionNames[i][0] != ' ' && i != TOTAL) { // only high-level sections count
        all += sections[i].nanoSum;
      }
    }
    for (int i=1;i<Profiler::Name::NUM_SECTIONS;i++) {
      // double totalRatio = sections[i].nanoSum / (double)sections[TOTAL].nanoSum;
      double allRatio = sections[i].nanoSum / (double)all;
      printf("[%2d] %-16s count=%'-8lu time=%'-10lu ns (%-8.3f %%)\n", i, sectionNames[i], sections[i].numCalls, sections[i].nanoSum, allRatio*100);
    }
    uint64_t missing = (sections[TOTAL].nanoSum - all);
    double missingRatio = missing / (double)sections[TOTAL].nanoSum;
    printf("\n");
    printf("Total block execution time:    %.5f s\n", sections[TOTAL].nanoSum / 1e9);
    printf("Summed section execution time: %.5f s\n", all / 1e9);
    printf("Unaccounted for:               %.5f s (%.3f %%)\n", missing / 1e9, missingRatio * 100);
  }
} profiler;
}

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

auto CPU::Recompiler::block(u64 vaddr, u32 address, bool singleInstruction) -> Block* {
  if(auto block = pool(address)->blocks[address >> 2 & 0x3f]) return block;
  auto block = emit(vaddr, address, singleInstruction);
  pool(address)->blocks[address >> 2 & 0x3f] = block;
  memory::jitprotect(true);
  return block;
}

#define IpuBase        offsetof(IPU, r[16])
#define IpuReg(r)      sreg(1), offsetof(IPU, r) - IpuBase
#define PipelineReg(x) mem(sreg(0), offsetof(CPU, pipeline) + offsetof(Pipeline, x))

auto CPU::Recompiler::emit(u64 vaddr, u32 address, bool singleInstruction) -> Block* {
  if(unlikely(allocator.available() < 1_MiB)) {
    print("CPU allocator flush\n");
    allocator.release();
    reset();
  }

  auto block = (Block*)allocator.acquire(sizeof(Block));
  beginFunction(3);

  Thread thread;
  bool hasBranched = 0;
  constexpr u32 branchToSelf = 0x1000'ffff;  //beq 0,0,<pc>
  u32 jumpToSelf = 2 << 26 | vaddr >> 2 & 0x3ff'ffff;  //j <pc>
  while(true) {
    PROF_BEGIN(Profiler::Name::TOTAL);

    u32 instruction = bus.read<Word>(address, thread, "Ares Recompiler");
    mov32(PipelineReg(nstate), imm(0));
    mov64(reg(0), PipelineReg(nextpc));
    mov64(PipelineReg(pc), reg(0));
    add64(PipelineReg(nextpc), reg(0), imm(4));
    if(callInstructionPrologue) {
      mov64(reg(1), imm(vaddr));
      mov32(reg(2), imm(instruction));
      call(&CPU::instructionPrologue);
    }

    bool branched = emitEXECUTE(instruction);

    if(unlikely(instruction == branchToSelf || instruction == jumpToSelf)) {
      PROF_BEGIN(Profiler::Name::IDLE_SKIP);
      //accelerate idle loops
      mov32(reg(1), imm(64 * 2));
      call(&CPU::step);
      PROF_END(Profiler::Name::IDLE_SKIP);
    }
    call(&Profiler::end, &profiler, Profiler::Name::TOTAL);

    PROF_BEGIN(Profiler::Name::EPILOGUE);
    call(&CPU::instructionEpilogue<1>);
    test32(PipelineReg(state), imm(Pipeline::EndBlock), set_z);
    mov32(PipelineReg(state), PipelineReg(nstate));
    mov64(mem(IpuReg(pc)), PipelineReg(pc));
    PROF_END(Profiler::Name::EPILOGUE);
    PROF_END(Profiler::Name::TOTAL);

    vaddr += 4;
    address += 4;
    jumpToSelf += 4;
    if(hasBranched || (address & 0xfc) == 0 || singleInstruction) break;  //block boundary
    hasBranched = branched;
    jumpEpilog(flag_nz);
  }
  jumpEpilog();

  memory::jitprotect(false);
  block->code = endFunction();

//print(hex(PC, 8L), " ", instructions, " ", size(), "\n");
  return block;
}

#define Sa  (instruction >>  6 & 31)
#define Rdn (instruction >> 11 & 31)
#define Rtn (instruction >> 16 & 31)
#define Rsn (instruction >> 21 & 31)
#define Fdn (instruction >>  6 & 31)
#define Fsn (instruction >> 11 & 31)
#define Ftn (instruction >> 16 & 31)

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

auto CPU::Recompiler::emitZeroClear(u32 n) -> void {
  if(n == 0) mov64(mem(IpuReg(r[0])), imm(0));
}

auto CPU::Recompiler::emitEXECUTE(u32 instruction) -> bool {
  switch(instruction >> 26) {

  //SPECIAL
  case 0x00: {
    bool ret = emitSPECIAL(instruction);
    return ret;
  }

  //REGIMM
  case 0x01: {
    bool ret = emitREGIMM(instruction);
    return ret;
  }

  //J n26
  case 0x02: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    mov32(reg(1), imm(n26));
    call(&CPU::J);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //JAL n26
  case 0x03: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    mov32(reg(1), imm(n26));
    call(&CPU::JAL);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BEQ Rs,Rt,i16
  case 0x04: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BEQ);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BNE Rs,Rt,i16
  case 0x05: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BNE);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BLEZ Rs,i16
  case 0x06: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLEZ);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BGTZ Rs,i16
  case 0x07: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGTZ);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //ADDI Rt,Rs,i16
  case 0x08: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::ADDI);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //ADDIU Rt,Rs,i16
  case 0x09: {
    if(Rtn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    add32(reg(0), mem(Rs32), imm(i16));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rt), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //SLTI Rt,Rs,i16
  case 0x0a: {
    if(Rtn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MOVE);
    cmp64(mem(Rs), imm(i16), set_slt);
    mov64_f(mem(Rt), flag_slt);
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //SLTIU Rt,Rs,i16
  case 0x0b: {
    PROF_BEGIN(Profiler::Name::MOVE);
    if(Rtn == 0) return 0;
    cmp64(mem(Rs), imm(i16), set_ult);
    mov64_f(mem(Rt), flag_ult);
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //ANDI Rt,Rs,n16
  case 0x0c: {
    if(Rtn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    and64(mem(Rt), mem(Rs), imm(n16));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //ORI Rt,Rs,n16
  case 0x0d: {
    if(Rtn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    or64(mem(Rt), mem(Rs), imm(n16));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //XORI Rt,Rs,n16
  case 0x0e: {
    if(Rtn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    xor64(mem(Rt), mem(Rs), imm(n16));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //LUI Rt,n16
  case 0x0f: {
    if(Rtn == 0) return 0;
    PROF_BEGIN(Profiler::Name::LOAD);
    mov64(mem(Rt), imm(s32(n16 << 16)));
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //SCC
  case 0x10: {
    PROF_BEGIN(Profiler::Name::SCC);
    bool ret = emitSCC(instruction);
    PROF_END(Profiler::Name::SCC);
    return ret;
  }

  //FPU
  case 0x11: {

    PROF_BEGIN(Profiler::Name::FPU);
    bool ret = emitFPU(instruction);
    PROF_END(Profiler::Name::FPU);
    return ret;
  }

  //COP2
  case 0x12: {
    PROF_BEGIN(Profiler::Name::COP2);
    bool ret = emitCOP2(instruction);
    PROF_END(Profiler::Name::COP2);
    return ret;
  }

  //COP3
  case 0x13: {
    PROF_BEGIN(Profiler::Name::COP3);
    call(&CPU::COP3);
    PROF_END(Profiler::Name::COP3);
    return 1;
  }

  //BEQL Rs,Rt,i16
  case 0x14: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BEQL);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BNEL Rs,Rt,i16
  case 0x15: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    mov32(reg(3), imm(i16));
    call(&CPU::BNEL);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BLEZL Rs,i16
  case 0x16: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLEZL);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //BGTZL Rs,i16
  case 0x17: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGTZL);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //DADDI Rt,Rs,i16
  case 0x18: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::DADDI);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DADDIU Rt,Rs,i16
  case 0x19: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::DADDIU);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //LDL Rt,Rs,i16
  case 0x1a: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LDL);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LDR Rt,Rs,i16
  case 0x1b: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LDR);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //INVALID
  case range4(0x1c, 0x1f): {
    call(&CPU::INVALID);
    return 1;
  }

  //LB Rt,Rs,i16
  case 0x20: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LB);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LH Rt,Rs,i16
  case 0x21: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LH);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LWL Rt,Rs,i16
  case 0x22: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWL);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LW Rt,Rs,i16
  case 0x23: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LW);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LBU Rt,Rs,i16
  case 0x24: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LBU);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LHU Rt,Rs,i16
  case 0x25: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LHU);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LWR Rt,Rs,i16
  case 0x26: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWR);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LWU Rt,Rs,i16
  case 0x27: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWU);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //SB Rt,Rs,i16
  case 0x28: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SB);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SH Rt,Rs,i16
  case 0x29: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SH);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SWL Rt,Rs,i16
  case 0x2a: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SWL);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SW Rt,Rs,i16
  case 0x2b: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SW);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SDL Rt,Rs,i16
  case 0x2c: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SDL);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SDR Rt,Rs,i16
  case 0x2d: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SDR);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SWR Rt,Rs,i16
  case 0x2e: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SWR);
    PROF_END(Profiler::Name::STORE);
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
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LL);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LWC1 Ft,Rs,i16
  case 0x31: {
    PROF_BEGIN(Profiler::Name::LOAD);
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LWC1);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LWC2
  case 0x32: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //LWC3
  case 0x33: {
    PROF_BEGIN(Profiler::Name::COP3);
    call(&CPU::COP3);
    PROF_END(Profiler::Name::COP3);
    return 1;
  }

  //LLD Rt,Rs,i16
  case 0x34: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LLD);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LDC1 Ft,Rs,i16
  case 0x35: {
    PROF_BEGIN(Profiler::Name::LOAD);
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LDC1);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //LDC2
  case 0x36: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //LD Rt,Rs,i16
  case 0x37: {
    PROF_BEGIN(Profiler::Name::LOAD);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::LD);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::LOAD);
    return 0;
  }

  //SC Rt,Rs,i16
  case 0x38: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SC);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SWC1 Ft,Rs,i16
  case 0x39: {
    PROF_BEGIN(Profiler::Name::STORE);
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SWC1);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SWC2
  case 0x3a: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //SWC3
  case 0x3b: {
    PROF_BEGIN(Profiler::Name::COP3);
    call(&CPU::COP3);
    PROF_END(Profiler::Name::COP3);
    return 1;
  }

  //SCD Rt,Rs,i16
  case 0x3c: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SCD);
    emitZeroClear(Rtn);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SDC1 Ft,Rs,i16
  case 0x3d: {
    PROF_BEGIN(Profiler::Name::STORE);
    mov32(reg(1), imm(Ftn));
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SDC1);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  //SDC2
  case 0x3e: {
    call(&CPU::COP2INVALID);
    return 1;
  }

  //SD Rt,Rs,i16
  case 0x3f: {
    PROF_BEGIN(Profiler::Name::STORE);
    lea(reg(1), Rt);
    lea(reg(2), Rs);
    mov32(reg(3), imm(i16));
    call(&CPU::SD);
    PROF_END(Profiler::Name::STORE);
    return 0;
  }

  }

  return 0;
}

auto CPU::Recompiler::emitSPECIAL(u32 instruction) -> bool {
  switch(instruction & 0x3f) {

  //SLL Rd,Rt,Sa
  case 0x00: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    shl32(reg(0), mem(Rt32), imm(Sa));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //INVALID
  case 0x01: {
    call(&CPU::INVALID);
    return 1;
  }

  //SRL Rd,Rt,Sa
  case 0x02: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    lshr32(reg(0), mem(Rt32), imm(Sa));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //SRA Rd,Rt,Sa
  case 0x03: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    ashr64(reg(0), mem(Rt), imm(Sa));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //SLLV Rd,Rt,Rs
  case 0x04: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    mshl32(reg(0), mem(Rt32), mem(Rs32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //INVALID
  case 0x05: {
    call(&CPU::INVALID);
    return 1;
  }

  //SRLV Rd,Rt,RS
  case 0x06: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    mlshr32(reg(0), mem(Rt32), mem(Rs32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //SRAV Rd,Rt,Rs
  case 0x07: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    and64(reg(1), mem(Rs), imm(31));
    ashr64(reg(0), mem(Rt), reg(1));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //JR Rs
  case 0x08: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    call(&CPU::JR);
    PROF_END(Profiler::Name::BRANCH);
    return 1;
  }

  //JALR Rd,Rs
  case 0x09: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    call(&CPU::JALR);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::BRANCH);
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
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MOVE);
    mov64(mem(Rd), mem(Hi));
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //MTHI Rs
  case 0x11: {
    PROF_BEGIN(Profiler::Name::MOVE);
    mov64(mem(Hi), mem(Rs));
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //MFLO Rd
  case 0x12: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MOVE);
    mov64(mem(Rd), mem(Lo));
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //MTLO Rs
  case 0x13: {
    PROF_BEGIN(Profiler::Name::MOVE);
    mov64(mem(Lo), mem(Rs));
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //DSLLV Rd,Rt,Rs
  case 0x14: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    lea(reg(3), Rs);
    call(&CPU::DSLLV);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //INVALID
  case 0x15: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSRLV Rd,Rt,Rs
  case 0x16: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    lea(reg(3), Rs);
    call(&CPU::DSRLV);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DSRAV Rd,Rt,Rs
  case 0x17: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    lea(reg(3), Rs);
    call(&CPU::DSRAV);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //MULT Rs,Rt
  case 0x18: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::MULT);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::MULT);
    // PROF_END(Profiler::Name::MULT);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //MULTU Rs,Rt
  case 0x19: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::MULT);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::MULTU);
    // PROF_END(Profiler::Name::MULT);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DIV Rs,Rt
  case 0x1a: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::DIV);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DIV);
    // PROF_END(Profiler::Name::DIV);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DIVU Rs,Rt
  case 0x1b: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::DIV);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DIVU);
    // PROF_END(Profiler::Name::DIV);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DMULT Rs,Rt
  case 0x1c: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::MULT);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DMULT);
    // PROF_END(Profiler::Name::MULT);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DMULTU Rs,Rt
  case 0x1d: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::MULT);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DMULTU);
    // PROF_END(Profiler::Name::MULT);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DDIV Rs,Rt
  case 0x1e: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::DIV);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DDIV);
    // PROF_END(Profiler::Name::DIV);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DDIVU Rs,Rt
  case 0x1f: {
    PROF_BEGIN(Profiler::Name::MATHS);
    // PROF_BEGIN(Profiler::Name::DIV);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::DDIVU);
    // PROF_END(Profiler::Name::DIV);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //ADD Rd,Rs,Rt
  case 0x20: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::ADD);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //ADDU Rd,Rs,Rt
  case 0x21: {
    PROF_BEGIN(Profiler::Name::MATHS);
    if(Rdn == 0) return 0;
    add32(reg(0), mem(Rs32), mem(Rt32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //SUB Rd,Rs,Rt
  case 0x22: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::SUB);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //SUBU Rd,Rs,Rt
  case 0x23: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    sub32(reg(0), mem(Rs32), mem(Rt32));
    mov64_s32(reg(0), reg(0));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //AND Rd,Rs,Rt
  case 0x24: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    and64(mem(Rd), mem(Rs), mem(Rt));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //OR Rd,Rs,Rt
  case 0x25: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    or64(mem(Rd), mem(Rs), mem(Rt));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //XOR Rd,Rs,Rt
  case 0x26: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    xor64(mem(Rd), mem(Rs), mem(Rt));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //NOR Rd,Rs,Rt
  case 0x27: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MATHS);
    or64(reg(0), mem(Rs), mem(Rt));
    xor64(reg(0), reg(0), imm(-1));
    mov64(mem(Rd), reg(0));
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //INVALID
  case range2(0x28, 0x29): {
    call(&CPU::INVALID);
    return 1;
  }

  //SLT Rd,Rs,Rt
  case 0x2a: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MOVE);
    cmp64(mem(Rs), mem(Rt), set_slt);
    mov64_f(mem(Rd), flag_slt);
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //SLTU Rd,Rs,Rt
  case 0x2b: {
    if(Rdn == 0) return 0;
    PROF_BEGIN(Profiler::Name::MOVE);
    cmp64(mem(Rs), mem(Rt), set_ult);
    mov64_f(mem(Rd), flag_ult);
    PROF_END(Profiler::Name::MOVE);
    return 0;
  }

  //DADD Rd,Rs,Rt
  case 0x2c: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DADD);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DADDU Rd,Rs,Rt
  case 0x2d: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DADDU);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DSUB Rd,Rs,Rt
  case 0x2e: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DSUB);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DSUBU Rd,Rs,Rt
  case 0x2f: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rs);
    lea(reg(3), Rt);
    call(&CPU::DSUBU);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //TGE Rs,Rt
  case 0x30: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TGE);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TGEU Rs,Rt
  case 0x31: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TGEU);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TLT Rs,Rt
  case 0x32: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TLT);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TLTU Rs,Rt
  case 0x33: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TLTU);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TEQ Rs,Rt
  case 0x34: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TEQ);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //INVALID
  case 0x35: {
    call(&CPU::INVALID);
    return 1;
  }

  //TNE Rs,Rt
  case 0x36: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    lea(reg(2), Rt);
    call(&CPU::TNE);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //INVALID
  case 0x37: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSLL Rd,Rt,Sa
  case 0x38: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa));
    call(&CPU::DSLL);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //INVALID
  case 0x39: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSRL Rd,Rt,Sa
  case 0x3a: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa));
    call(&CPU::DSRL);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DSRA Rd,Rt,Sa
  case 0x3b: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa));
    call(&CPU::DSRA);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DSLL32 Rd,Rt,Sa
  case 0x3c: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa+32));
    call(&CPU::DSLL);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //INVALID
  case 0x3d: {
    call(&CPU::INVALID);
    return 1;
  }

  //DSRL32 Rd,Rt,Sa
  case 0x3e: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa+32));
    call(&CPU::DSRL);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  //DSRA32 Rd,Rt,Sa
  case 0x3f: {
    PROF_BEGIN(Profiler::Name::MATHS);
    lea(reg(1), Rd);
    lea(reg(2), Rt);
    mov32(reg(3), imm(Sa+32));
    call(&CPU::DSRA);
    emitZeroClear(Rdn);
    PROF_END(Profiler::Name::MATHS);
    return 0;
  }

  }

  return 0;
}

auto CPU::Recompiler::emitREGIMM(u32 instruction) -> bool {
  switch(instruction >> 16 & 0x1f) {

  //BLTZ Rs,i16
  case 0x00: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZ);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //BGEZ Rs,i16
  case 0x01: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZ);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //BLTZL Rs,i16
  case 0x02: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZL);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //BGEZL Rs,i16
  case 0x03: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZL);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //INVALID
  case range4(0x04, 0x07): {
    call(&CPU::INVALID);
    return 1;
  }

  //TGEI Rs,i16
  case 0x08: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TGEI);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TGEIU Rs,i16
  case 0x09: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TGEIU);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TLTI Rs,i16
  case 0x0a: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TLTI);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TLTIU Rs,i16
  case 0x0b: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TLTIU);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //TEQI Rs,i16
  case 0x0c: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TEQI);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //INVALID
  case 0x0d: {
    call(&CPU::INVALID);
    return 1;
  }

  //TNEI Rs,i16
  case 0x0e: {
    PROF_BEGIN(Profiler::Name::TRAP);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::TNEI);
    PROF_END(Profiler::Name::TRAP);
    return 0;
  }

  //INVALID
  case 0x0f: {
    call(&CPU::INVALID);
    return 1;
  }

  //BLTZAL Rs,i16
  case 0x10: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZAL);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //BGEZAL Rs,i16
  case 0x11: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZAL);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //BLTZALL Rs,i16
  case 0x12: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BLTZALL);
    PROF_END(Profiler::Name::BRANCH);
    return 0;
  }

  //BGEZALL Rs,i16
  case 0x13: {
    PROF_BEGIN(Profiler::Name::BRANCH);
    lea(reg(1), Rs);
    mov32(reg(2), imm(i16));
    call(&CPU::BGEZALL);
    PROF_END(Profiler::Name::BRANCH);
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
    emitZeroClear(Rtn);
    return 0;
  }

  //DMFC0 Rt,Rd
  case 0x01: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::DMFC0);
    emitZeroClear(Rtn);
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
    emitZeroClear(Rtn);
    return 0;
  }

  //DMFC1 Rt,Fs
  case 0x01: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Fsn));
    call(&CPU::DMFC1);
    emitZeroClear(Rtn);
    return 0;
  }

  //CFC1 Rt,Rd
  case 0x02: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::CFC1);
    emitZeroClear(Rtn);
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
    emitZeroClear(Rtn);
    return 0;
  }

  //DMFC2 Rt,Rd
  case 0x01: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::DMFC2);
    emitZeroClear(Rtn);
    return 0;
  }

  //CFC2 Rt,Rd
  case 0x02: {
    lea(reg(1), Rt);
    mov32(reg(2), imm(Rdn));
    call(&CPU::CFC2);
    emitZeroClear(Rtn);
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

#undef IpuBase
#undef IpuReg
#undef PipelineReg
#undef Sa
#undef Rdn
#undef Rtn
#undef Rsn
#undef Fdn
#undef Fsn
#undef Ftn
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
