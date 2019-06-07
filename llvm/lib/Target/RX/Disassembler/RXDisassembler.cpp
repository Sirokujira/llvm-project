//===-- RXDisassembler.cpp - Disassembler for RX --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RXDisassembler class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RXMCTargetDesc.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "rx-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {
class RXDisassembler : public MCDisassembler {

public:
  RXDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &VStream,
                              raw_ostream &CStream) const override;
};
} // end anonymous namespace

static MCDisassembler *createRXDisassembler(const Target &T,
                                               const MCSubtargetInfo &STI,
                                               MCContext &Ctx) {
  return new RXDisassembler(STI, Ctx);
}

extern "C" void LLVMInitializeRXDisassembler() {
  // Register the disassembler for each target.
  TargetRegistry::RegisterMCDisassembler(getTheRXTarget(),
                                         createRXDisassembler);
}

static const unsigned GPRDecoderTable[] = {
  RX::R0,  RX::R1,  RX::R2,  RX::R3,
  RX::R4,  RX::R5,  RX::R6,  RX::R7,
  RX::R8,  RX::R9,  RX::R10, RX::R11,
  RX::R12, RX::R13, RX::R14, RX::R15
};

static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst, uint64_t RegNo,
                                           uint64_t Address,
                                           const void *Decoder) {
  if (RegNo > sizeof(GPRDecoderTable))
    return MCDisassembler::Fail;

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  unsigned Reg = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static const unsigned FPR32DecoderTable[] = {
  RX::R0,  RX::R1,  RX::R2,  RX::R3,
  RX::R4,  RX::R5,  RX::R6,  RX::R7,
  RX::R8,  RX::R9,  RX::R10, RX::R11,
  RX::R12, RX::R13, RX::R14, RX::R15
};

static DecodeStatus DecodeFPR32RegisterClass(MCInst &Inst, uint64_t RegNo,
                                             uint64_t Address,
                                             const void *Decoder) {
  if (RegNo > sizeof(FPR32DecoderTable))
    return MCDisassembler::Fail;

  // We must define our own mapping from RegNo to register identifier.
  // Accessing index RegNo in the register class will work in the case that
  // registers were added in ascending order, but not in general.
  unsigned Reg = FPR32DecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeFPR32CRegisterClass(MCInst &Inst, uint64_t RegNo,
                                              uint64_t Address,
                                              const void *Decoder) {
  if (RegNo > 8) {
    return MCDisassembler::Fail;
  }
  unsigned Reg = FPR32DecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeGPRNoX0RegisterClass(MCInst &Inst, uint64_t RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo == 0) {
    return MCDisassembler::Fail;
  }

  return DecodeGPRRegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeGPRNoX0X2RegisterClass(MCInst &Inst, uint64_t RegNo,
                                                 uint64_t Address,
                                                 const void *Decoder) {
  if (RegNo == 2) {
    return MCDisassembler::Fail;
  }

  return DecodeGPRNoX0RegisterClass(Inst, RegNo, Address, Decoder);
}

static DecodeStatus DecodeGPRCRegisterClass(MCInst &Inst, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  if (RegNo > 8)
    return MCDisassembler::Fail;

  unsigned Reg = GPRDecoderTable[RegNo + 8];
  Inst.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

// Add implied SP operand for instructions *SP compressed instructions. The SP
// operand isn't explicitly encoded in the instruction.
static void addImplySP(MCInst &Inst, int64_t Address, const void *Decoder) {
	// Not Compress Operand(RX)
}

template <unsigned N>
static DecodeStatus decodeUImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  addImplySP(Inst, Address, Decoder);
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

template <unsigned N>
static DecodeStatus decodeSImmOperand(MCInst &Inst, uint64_t Imm,
                                      int64_t Address, const void *Decoder) {
  assert(isUInt<N>(Imm) && "Invalid immediate");
  addImplySP(Inst, Address, Decoder);
  // Sign-extend the number in the bottom N bits of Imm
  Inst.addOperand(MCOperand::createImm(SignExtend64<N>(Imm)));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCLUIImmOperand(MCInst &Inst, uint64_t Imm,
                                         int64_t Address,
                                         const void *Decoder) {
  assert(isUInt<6>(Imm) && "Invalid immediate");
  if (Imm > 31) {
    Imm = (SignExtend64<6>(Imm) & 0xfffff);
  }
  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

#include "RXGenDisassemblerTables.inc"

DecodeStatus RXDisassembler::getInstruction(MCInst &MI, uint64_t &Size,
                                               ArrayRef<uint8_t> Bytes,
                                               uint64_t Address,
                                               raw_ostream &OS,
                                               raw_ostream &CS) const {
  // TODO: This will need modification when supporting instruction set
  // extensions with instructions > 32-bits (up to 176 bits wide).
  uint32_t Insn;
  DecodeStatus Result;

  // It's a 32 bit instruction if bit 0 and 1 are 1.
  // if ((Bytes[0] & 0x3) == 0x3) {
  Insn = support::endian::read32le(Bytes.data());
  LLVM_DEBUG(dbgs() << "Trying RX table :\n");
  // SizeCheck
  // 1byte
  // DecoderTableRX8
  // Size = 1;
  // 2byte
  // DecoderTableRX16
  // Size = 2;
  // 3byte
  // DecoderTableRX24
  // Size = 3;
  // 4byte
  // DecoderTableRX32
  Result = decodeInstruction(DecoderTableRX32, MI, Insn, Address, this, STI);
  Size = 4;
  /*
  } else {
    Insn = support::endian::read16le(Bytes.data());

    if (!STI.getFeatureBits()[RX::Feature64Bit]) {
      LLVM_DEBUG(
          dbgs() << "Trying RXOnly_16 table (16-bit Instruction):\n");
      // Calling the auto-generated decoder function.
      Result = decodeInstruction(DecoderTableRX32Only_16, MI, Insn, Address,
                                 this, STI);
      if (Result != MCDisassembler::Fail) {
        Size = 2;
        return Result;
      }
    }

    LLVM_DEBUG(dbgs() << "Trying RX_C table (16-bit Instruction):\n");
    // Calling the auto-generated decoder function.
    Result = decodeInstruction(DecoderTable16, MI, Insn, Address, this, STI);
    Size = 2;
  }
  */

  return Result;
}
