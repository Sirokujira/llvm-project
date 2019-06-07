//===-- RXAsmBackend.cpp - RX Assembler Backend ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RXFixupKinds.h"
#include "MCTargetDesc/RXMCTargetDesc.h"
#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

namespace {
class RXAsmBackend : public MCAsmBackend {
  const MCSubtargetInfo &STI;
  uint8_t OSABI;
  bool Is64Bit;

public:
  RXAsmBackend(const MCSubtargetInfo &STI, uint8_t OSABI, bool Is64Bit)
      : MCAsmBackend(support::little), STI(STI), OSABI(OSABI),
        Is64Bit(Is64Bit) {}
  ~RXAsmBackend() override {}

  // Generate diff expression relocations if the relax feature is enabled,
  // otherwise it is safe for the assembler to calculate these internally.
  bool requiresDiffExpressionRelocations() const override {
    return STI.getFeatureBits()[RX::FeatureRelax];
  }
  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved,
                  const MCSubtargetInfo *STI) const override;

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;

  // If linker relaxation is enabled, always emit relocations even if the fixup
  // can be resolved. This is necessary for correctness as offsets may change
  // during relaxation.
  bool shouldForceRelocation(const MCAssembler &Asm, const MCFixup &Fixup,
                             const MCValue &Target) override {
    return STI.getFeatureBits()[RX::FeatureRelax];
  }

  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *DF,
                            const MCAsmLayout &Layout) const override {
    llvm_unreachable("Handled by fixupNeedsRelaxationAdvanced");
  }

  bool fixupNeedsRelaxationAdvanced(const MCFixup &Fixup, bool Resolved,
                                    uint64_t Value,
                                    const MCRelaxableFragment *DF,
                                    const MCAsmLayout &Layout,
                                    const bool WasForced) const override;

  unsigned getNumFixupKinds() const override {
    return RX::NumTargetFixupKinds;
  }

  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override {
    const static MCFixupKindInfo Infos[] = {
      // This table *must* be in the order that the fixup_* kinds are defined in
      // RXFixupKinds.h.
      //
      // name                      offset bits  flags
      { "fixup_rx_hi20",         12,     20,  0 },
      { "fixup_rx_lo12_i",       20,     12,  0 },
      { "fixup_rx_lo12_s",        0,     32,  0 },
      { "fixup_rx_pcrel_hi20",   12,     20,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_pcrel_lo12_i", 20,     12,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_pcrel_lo12_s",  0,     32,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_jal",          12,     20,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_branch",        0,     32,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_rvc_jump",      2,     11,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_rvc_branch",    0,     16,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_call",          0,     64,  MCFixupKindInfo::FKF_IsPCRel },
      { "fixup_rx_relax",         0,      0,  0 }
    };
    static_assert((array_lengthof(Infos)) == RX::NumTargetFixupKinds,
                  "Not all fixup kinds added to Infos array");

    if (Kind < FirstTargetFixupKind)
      return MCAsmBackend::getFixupKindInfo(Kind);

    assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
           "Invalid kind!");
    return Infos[Kind - FirstTargetFixupKind];
  }

  bool mayNeedRelaxation(const MCInst &Inst,
                         const MCSubtargetInfo &STI) const override;
  unsigned getRelaxedOpcode(unsigned Op) const;

  void relaxInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                        MCInst &Res) const override;


  bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
};


bool RXAsmBackend::fixupNeedsRelaxationAdvanced(const MCFixup &Fixup,
                                                   bool Resolved,
                                                   uint64_t Value,
                                                   const MCRelaxableFragment *DF,
                                                   const MCAsmLayout &Layout,
                                                   const bool WasForced) const {
  // Return true if the symbol is actually unresolved.
  // Resolved could be always false when shouldForceRelocation return true.
  // We use !WasForced to indicate that the symbol is unresolved and not forced
  // by shouldForceRelocation.
  if (!Resolved && !WasForced)
    return true;

  int64_t Offset = int64_t(Value);
  switch ((unsigned)Fixup.getKind()) {
  default:
    return false;
  case RX::fixup_rx_rvc_branch:
    // For compressed branch instructions the immediate must be
    // in the range [-256, 254].
    return Offset > 254 || Offset < -256;
  case RX::fixup_rx_rvc_jump:
    // For compressed jump instructions the immediate must be
    // in the range [-2048, 2046].
    return Offset > 2046 || Offset < -2048;
  }
}

void RXAsmBackend::relaxInstruction(const MCInst &Inst,
                                       const MCSubtargetInfo &STI,
                                       MCInst &Res) const {
  // TODO: replace this with call to auto generated uncompressinstr() function.
  /*
  switch (Inst.getOpcode()) {
  default:
    llvm_unreachable("Opcode not expected!");
  case RX::C_BEQZ:
    // c.beqz $rs1, $imm -> beq $rs1, R0, $imm.
    Res.setOpcode(RX::BEQ);
    Res.addOperand(Inst.getOperand(0));
    Res.addOperand(MCOperand::createReg(RX::R0));
    Res.addOperand(Inst.getOperand(1));
    break;
  case RX::C_BNEZ:
    // c.bnez $rs1, $imm -> bne $rs1, R0, $imm.
    Res.setOpcode(RX::BNE);
    Res.addOperand(Inst.getOperand(0));
    Res.addOperand(MCOperand::createReg(RX::R0));
    Res.addOperand(Inst.getOperand(1));
    break;
  // jump and link
  case RX::C_J:
    // c.j $imm -> jal R0, $imm.
    Res.setOpcode(RX::JAL);
    Res.addOperand(MCOperand::createReg(RX::R0));
    Res.addOperand(Inst.getOperand(0));
    break;
  // jump and link
  case RX::C_JAL:
    // c.jal $imm -> jal R1, $imm.
    Res.setOpcode(RX::JAL);
    Res.addOperand(MCOperand::createReg(RX::R1));
    Res.addOperand(Inst.getOperand(0));
    break;
  }
  */
}

// Given a compressed control flow instruction this function returns
// the expanded instruction.
unsigned RXAsmBackend::getRelaxedOpcode(unsigned Op) const {
  switch (Op) {
  default:
    return Op;
  }
}

bool RXAsmBackend::mayNeedRelaxation(const MCInst &Inst,
                                        const MCSubtargetInfo &STI) const {
  return getRelaxedOpcode(Inst.getOpcode()) != Inst.getOpcode();
}

bool RXAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count) const {
  unsigned MinNopLen = 1;

  if ((Count % MinNopLen) != 0)
    return false;

  // The canonical nop on RX is nop.
  OS.write("\x03", 1);
  return true;
}

static uint64_t adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
                                 MCContext &Ctx) {
  unsigned Kind = Fixup.getKind();
  switch (Kind) {
  default:
    llvm_unreachable("Unknown fixup kind!");
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
  case FK_Data_8:
    return Value;
  case RX::fixup_rx_jal: {
    if (!isInt<21>(Value))
      Ctx.reportError(Fixup.getLoc(), "fixup value out of range");
    if (Value & 0x1)
      Ctx.reportError(Fixup.getLoc(), "fixup value must be 2-byte aligned");
    // Need to produce imm[19|10:1|11|19:12] from the 21-bit Value.
    unsigned Sbit = (Value >> 20) & 0x1;
    unsigned Hi8 = (Value >> 12) & 0xff;
    unsigned Mid1 = (Value >> 11) & 0x1;
    unsigned Lo10 = (Value >> 1) & 0x3ff;
    // Inst{31} = Sbit;
    // Inst{30-21} = Lo10;
    // Inst{20} = Mid1;
    // Inst{19-12} = Hi8;
    Value = (Sbit << 19) | (Lo10 << 9) | (Mid1 << 8) | Hi8;
    return Value;
  }
  case RX::fixup_rx_branch: {
    if (!isInt<13>(Value))
      Ctx.reportError(Fixup.getLoc(), "fixup value out of range");
    if (Value & 0x1)
      Ctx.reportError(Fixup.getLoc(), "fixup value must be 2-byte aligned");
    // Need to extract imm[12], imm[10:5], imm[4:1], imm[11] from the 13-bit
    // Value.
    unsigned Sbit = (Value >> 12) & 0x1;
    unsigned Hi1 = (Value >> 11) & 0x1;
    unsigned Mid6 = (Value >> 5) & 0x3f;
    unsigned Lo4 = (Value >> 1) & 0xf;
    // Inst{31} = Sbit;
    // Inst{30-25} = Mid6;
    // Inst{11-8} = Lo4;
    // Inst{7} = Hi1;
    Value = (Sbit << 31) | (Mid6 << 25) | (Lo4 << 8) | (Hi1 << 7);
    return Value;
  }
  case RX::fixup_rx_call: {
    // Jalr will add UpperImm with the sign-extended 12-bit LowerImm,
    // we need to add 0x800ULL before extract upper bits to reflect the
    // effect of the sign extension.
    uint64_t UpperImm = (Value + 0x800ULL) & 0xfffff000ULL;
    uint64_t LowerImm = Value & 0xfffULL;
    return UpperImm | ((LowerImm << 20) << 32);
  }
  case RX::fixup_rx_rvc_jump: {
    // Need to produce offset[11|4|9:8|10|6|7|3:1|5] from the 11-bit Value.
    unsigned Bit11  = (Value >> 11) & 0x1;
    unsigned Bit4   = (Value >> 4) & 0x1;
    unsigned Bit9_8 = (Value >> 8) & 0x3;
    unsigned Bit10  = (Value >> 10) & 0x1;
    unsigned Bit6   = (Value >> 6) & 0x1;
    unsigned Bit7   = (Value >> 7) & 0x1;
    unsigned Bit3_1 = (Value >> 1) & 0x7;
    unsigned Bit5   = (Value >> 5) & 0x1;
    Value = (Bit11 << 10) | (Bit4 << 9) | (Bit9_8 << 7) | (Bit10 << 6) |
            (Bit6 << 5) | (Bit7 << 4) | (Bit3_1 << 1) | Bit5;
    return Value;
  }
  case RX::fixup_rx_rvc_branch: {
    // Need to produce offset[8|4:3], [reg 3 bit], offset[7:6|2:1|5]
    unsigned Bit8   = (Value >> 8) & 0x1;
    unsigned Bit7_6 = (Value >> 6) & 0x3;
    unsigned Bit5   = (Value >> 5) & 0x1;
    unsigned Bit4_3 = (Value >> 3) & 0x3;
    unsigned Bit2_1 = (Value >> 1) & 0x3;
    Value = (Bit8 << 12) | (Bit4_3 << 10) | (Bit7_6 << 5) | (Bit2_1 << 3) |
            (Bit5 << 2);
    return Value;
  }

  }
}

void RXAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                 const MCValue &Target,
                                 MutableArrayRef<char> Data, uint64_t Value,
                                 bool IsResolved,
                                 const MCSubtargetInfo *STI) const {
  MCContext &Ctx = Asm.getContext();
  MCFixupKindInfo Info = getFixupKindInfo(Fixup.getKind());
  if (!Value)
    return; // Doesn't change encoding.
  // Apply any target-specific value adjustments.
  Value = adjustFixupValue(Fixup, Value, Ctx);

  // Shift the value into position.
  Value <<= Info.TargetOffset;

  unsigned Offset = Fixup.getOffset();
  unsigned NumBytes = alignTo(Info.TargetSize + Info.TargetOffset, 8) / 8;

  assert(Offset + NumBytes <= Data.size() && "Invalid fixup offset!");

  // For each byte of the fragment that the fixup touches, mask in the
  // bits from the fixup value.
  for (unsigned i = 0; i != NumBytes; ++i) {
    Data[Offset + i] |= uint8_t((Value >> (i * 8)) & 0xff);
  }
}

std::unique_ptr<MCObjectTargetWriter>
RXAsmBackend::createObjectTargetWriter() const {
  return createRXELFObjectWriter(OSABI, Is64Bit);
}

} // end anonymous namespace

MCAsmBackend *llvm::createRXAsmBackend(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo &MRI,
                                          const MCTargetOptions &Options) {
  const Triple &TT = STI.getTargetTriple();
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TT.getOS());
  return new RXAsmBackend(STI, OSABI, TT.isArch64Bit());
}
