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
  case FK_Data_1:
    // TODO: R_RX_DIR8U/R_RX_DIR8S ‚Ì‘Î‰ž‚Í?
    return ELF::R_RX_DIR8;
  case FK_Data_2:
    // TODO: R_RX_DIR16U/R_RX_DIR16S ‚Ì‘Î‰ž‚Í?
    return ELF::R_RX_DIR16;
  case RX::fixup_rx_data_24:
    return ELF::R_RX_DIR24S;
  case FK_Data_4:
    return ELF::R_RX_DIR32;
  // case FK_Data_8:
  //   return ELF::R_RX_64;
  case FK_PCRel_1:
    return ELF::R_RX_DIR8S_PCREL;
  case FK_PCRel_2:
    return ELF::R_RX_DIR16S_PCREL;
  case RX::fixup_rx_pcrel_24:
    return ELF::R_RX_DIR24S_PCREL;
  case RX::fixup_rx_dir3u_pcrel:
    return ELF::R_RX_RH_3_PCREL;
  case RX::fixup_rx_rh_16_op0:
  case RX::fixup_rx_rh_24_op:
  case RX::fixup_rx_rh_32_op:
  case RX::fixup_rx_rh_24_uns:
  case RX::fixup_rx_rh_8_neg:
  case RX::fixup_rx_rh_16_neg:
  case RX::fixup_rx_rh_24_neg:
  case RX::fixup_rx_rh_32_neg:
  case RX::fixup_rx_rh_diff:
  case RX::fixup_rx_rh_gprelb:
  case RX::fixup_rx_rh_gprelw:
  case RX::fixup_rx_rh_gprell:
  case RX::fixup_rx_abs32:
  case RX::fixup_rx_abs24s:
  case RX::fixup_rx_abs16:
  case RX::fixup_rx_abs16u:
  case RX::fixup_rx_abs16s:
  case RX::fixup_rx_abs8:
  case RX::fixup_rx_abs8u:
  case RX::fixup_rx_abs8s:
  case RX::fixup_rx_abs16ul:
  case RX::fixup_rx_abs16uw:
  case RX::fixup_rx_abs8ul:
  case RX::fixup_rx_abs8uw:
  case RX::fixup_rx_abs32_rev:
  case RX::fixup_rx_abs16_rev:
  case RX::fixup_rx_sym:
  case RX::fixup_rx_opneg:
  case RX::fixup_rx_opadd:
  case RX::fixup_rx_opsub:
  case RX::fixup_rx_opmul:
  case RX::fixup_rx_opdiv:
  case RX::fixup_rx_opshla:
  case RX::fixup_rx_opshra:
  case RX::fixup_rx_opsctsize:
  case RX::fixup_rx_opscttop:
  case RX::fixup_rx_opand:
  case RX::fixup_rx_opor:
  case RX::fixup_rx_opxor:
  case RX::fixup_rx_opnot:
  case RX::fixup_rx_opmod:
  case RX::fixup_rx_opromtop:
  case RX::fixup_rx_opramtop:
    return ELF::R_RX_RH_RELAX;
  // case RX::fixup_rx_rvc_jump:
  //   return ELF::R_RX_RXC_JUMP;
  // case RX::fixup_rx_rvc_branch:
  //   return ELF::R_RX_RXC_BRANCH;
  // case RX::fixup_rx_call:
  //   return ELF::R_RX_CALL;
  case RX::fixup_rx_relax:
    return ELF::R_RX_RH_RELAX;
  }
}

std::unique_ptr<MCObjectTargetWriter>
llvm::createRXELFObjectWriter(uint8_t OSABI, bool Is64Bit) {
  return llvm::make_unique<RXELFObjectWriter>(OSABI, Is64Bit);
}
