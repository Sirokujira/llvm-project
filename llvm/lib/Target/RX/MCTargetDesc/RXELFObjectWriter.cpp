//===-- RXELFObjectWriter.cpp - RX ELF Writer -----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RXFixupKinds.h"
#include "MCTargetDesc/RXMCTargetDesc.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

namespace {
class RXELFObjectWriter : public MCELFObjectTargetWriter {
public:
  RXELFObjectWriter(uint8_t OSABI, bool Is64Bit);

  ~RXELFObjectWriter() override;

  // Return true if the given relocation must be with a symbol rather than
  // section plus offset.
  bool needsRelocateWithSymbol(const MCSymbol &Sym,
                               unsigned Type) const override {
    // TODO: this is very conservative, update once RX psABI requirements
    //       are clarified.
    return true;
  }

protected:
  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};
}

RXELFObjectWriter::RXELFObjectWriter(uint8_t OSABI, bool Is64Bit)
//    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_RX,
    : MCELFObjectTargetWriter(Is64Bit, OSABI, ELF::EM_RX,   // already define ELF::EM_RX
                              /*HasRelocationAddend*/ true) {}

RXELFObjectWriter::~RXELFObjectWriter() {}

unsigned RXELFObjectWriter::getRelocType(MCContext &Ctx,
                                            const MCValue &Target,
                                            const MCFixup &Fixup,
                                            bool IsPCRel) const {
  // Determine the type of the relocation
  switch ((unsigned)Fixup.getKind()) {
  default:
    llvm_unreachable("invalid fixup kind!");
  case FK_Data_4:
    return ELF::R_RX_32;
  case FK_Data_8:
    return ELF::R_RX_64;
  case FK_Data_Add_1:
    return ELF::R_RX_ADD8;
  case FK_Data_Add_2:
    return ELF::R_RX_ADD16;
  case FK_Data_Add_4:
    return ELF::R_RX_ADD32;
  case FK_Data_Add_8:
    return ELF::R_RX_ADD64;
  case FK_Data_Sub_1:
    return ELF::R_RX_SUB8;
  case FK_Data_Sub_2:
    return ELF::R_RX_SUB16;
  case FK_Data_Sub_4:
    return ELF::R_RX_SUB32;
  case FK_Data_Sub_8:
    return ELF::R_RX_SUB64;
  case RX::fixup_rx_hi20:
    return ELF::R_RX_HI20;
  case RX::fixup_rx_lo12_i:
    return ELF::R_RX_LO12_I;
  case RX::fixup_rx_lo12_s:
    return ELF::R_RX_LO12_S;
  case RX::fixup_rx_pcrel_hi20:
    return ELF::R_RX_PCREL_HI20;
  case RX::fixup_rx_pcrel_lo12_i:
    return ELF::R_RX_PCREL_LO12_I;
  case RX::fixup_rx_pcrel_lo12_s:
    return ELF::R_RX_PCREL_LO12_S;
  case RX::fixup_rx_jal:
    return ELF::R_RX_JAL;
  case RX::fixup_rx_branch:
    return ELF::R_RX_BRANCH;
  // case RX::fixup_rx_rvc_jump:
  //   return ELF::R_RX_RXC_JUMP;
  // case RX::fixup_rx_rvc_branch:
  //   return ELF::R_RX_RXC_BRANCH;
  case RX::fixup_rx_call:
    return ELF::R_RX_CALL;
  case RX::fixup_rx_relax:
    return ELF::R_RX_RELAX;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createRXELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return llvm::make_unique<RXELFObjectWriter>(OSABI, Is64Bit);
}
