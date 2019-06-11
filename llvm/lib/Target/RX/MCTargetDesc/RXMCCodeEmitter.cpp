//===-- RXMCCodeEmitter.cpp - Convert RX code to machine code -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RXMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RXBaseInfo.h"
#include "MCTargetDesc/RXFixupKinds.h"
#include "MCTargetDesc/RXMCExpr.h"
#include "MCTargetDesc/RXMCTargetDesc.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

STATISTIC(MCNumEmitted, "Number of MC instructions emitted");
STATISTIC(MCNumFixups, "Number of MC fixups created");

namespace {
class RXMCCodeEmitter : public MCCodeEmitter {
  RXMCCodeEmitter(const RXMCCodeEmitter &) = delete;
  void operator=(const RXMCCodeEmitter &) = delete;
  MCContext &Ctx;
  MCInstrInfo const &MCII;

public:
  RXMCCodeEmitter(MCContext &ctx, MCInstrInfo const &MCII)
      : Ctx(ctx), MCII(MCII) {}

  ~RXMCCodeEmitter() override {}

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  void expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                          SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  /// TableGen'erated function for getting the binary encoding for an
  /// instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  /// Return binary encoding of operand. If the machine operand requires
  /// relocation, record the relocation and return zero.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  unsigned getImmOpValue(const MCInst &MI, unsigned OpNo,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const;
};
} // end anonymous namespace

MCCodeEmitter *llvm::createRXMCCodeEmitter(const MCInstrInfo &MCII,
                                              const MCRegisterInfo &MRI,
                                              MCContext &Ctx) {
  return new RXMCCodeEmitter(Ctx, MCII);
}

// Expand PseudoCALL and PseudoTAIL to AUIPC and JALR with relocation types.
// We expand PseudoCALL and PseudoTAIL while encoding, meaning AUIPC and JALR
// won't go through RX MC to MC compressed instruction transformation. This
// is acceptable because AUIPC has no 16-bit form and C_JALR have no immediate
// operand field.  We let linker relaxation deal with it. When linker
// relaxation enabled, AUIPC and JALR have chance relax to JAL. If C extension
// is enabled, JAL has chance relax to C_JAL.
void RXMCCodeEmitter::expandFunctionCall(const MCInst &MI, raw_ostream &OS,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  MCInst TmpInst;
  MCOperand Func = MI.getOperand(0);
  unsigned Ra = RX::R1;
  uint32_t Binary;

  assert(Func.isExpr() && "Expected expression");

  const MCExpr *Expr = Func.getExpr();

  // Create function call expression CallExpr for AUIPC.
  // const MCExpr *CallExpr =
  //     RXMCExpr::create(Expr, RXMCExpr::VK_RX_CALL, Ctx);
  // Emit AUIPC Ra, Func with R_RX_CALL relocation type.
  // TmpInst = MCInstBuilder(RX::BRA)
  //               .addReg(Ra)
  //               .addOperand(MCOperand::createExpr(CallExpr));
  // Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  // support::endian::write(OS, Binary, support::little);

  // –ß‚è’l•s—v
  // Emit JALR R0, R6, 0
  // TmpInst = MCInstBuilder(RX::JALR).addReg(RX::R0).addReg(Ra).addImm(0);
  // Emit JALR R1, R1, 0
  // TmpInst = MCInstBuilder(RX::BRA).addReg(Ra).addReg(Ra).addImm(0);
  // Emit 
  TmpInst = MCInstBuilder(RX::BRA).addReg(Ra).addImm(0);
  Binary = getBinaryCodeForInstr(TmpInst, Fixups, STI);
  support::endian::write(OS, Binary, support::little);
}

void RXMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction.
  unsigned Size = Desc.getSize();

  switch (Size) {
  default:
    llvm_unreachable("Unhandled encodeInstruction length!");
  case 2: {
    uint16_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write<uint16_t>(OS, Bits, support::little);
    break;
  }
  case 4: {
    uint32_t Bits = getBinaryCodeForInstr(MI, Fixups, STI);
    support::endian::write(OS, Bits, support::little);
    break;
  }
  }

  ++MCNumEmitted; // Keep track of the # of mi's emitted.
}

unsigned
RXMCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {

  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());

  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  llvm_unreachable("Unhandled expression!");
  return 0;
}

unsigned
RXMCCodeEmitter::getImmOpValueAsr1(const MCInst &MI, unsigned OpNo,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isImm()) {
    unsigned Res = MO.getImm();
    assert((Res & 1) == 0 && "LSB is non-zero");
    return Res >> 1;
  }

  return getImmOpValue(MI, OpNo, Fixups, STI);
}

unsigned RXMCCodeEmitter::getImmOpValue(const MCInst &MI, unsigned OpNo,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  bool EnableRelax = STI.getFeatureBits()[RX::FeatureRelax];
  const MCOperand &MO = MI.getOperand(OpNo);

  MCInstrDesc const &Desc = MCII.get(MI.getOpcode());
  unsigned MIFrm = Desc.TSFlags & RXII::InstFormatMask;

  // If the destination is an immediate, there is nothing to do.
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr() &&
         "getImmOpValue expects only expressions or immediates");
  const MCExpr *Expr = MO.getExpr();
  MCExpr::ExprKind Kind = Expr->getKind();
  RX::Fixups FixupKind = RX::fixup_rx_invalid;
  if (Kind == MCExpr::Target) {
    const RXMCExpr *RXExpr = cast<RXMCExpr>(Expr);

    switch (RXExpr->getKind()) {
    case RXMCExpr::VK_RX_None:
    case RXMCExpr::VK_RX_Invalid:
      llvm_unreachable("Unhandled fixup kind!");
    case RXMCExpr::VK_RX_CALL:
      FixupKind = RX::fixup_rx_call;
      break;
    }
  } else if (Kind == MCExpr::SymbolRef &&
             cast<MCSymbolRefExpr>(Expr)->getKind() == MCSymbolRefExpr::VK_None) {
    if (Desc.getOpcode() == RX::JMP) {
      FixupKind = RX::fixup_rx_jal;
    }
  }

  assert(FixupKind != RX::fixup_rx_invalid && "Unhandled expression!");

  Fixups.push_back(
      MCFixup::create(0, Expr, MCFixupKind(FixupKind), MI.getLoc()));
  ++MCNumFixups;

  if (EnableRelax) {
    if (FixupKind == RX::fixup_rx_call) {
      Fixups.push_back(
      MCFixup::create(0, Expr, MCFixupKind(RX::fixup_rx_relax),
                      MI.getLoc()));
      ++MCNumFixups;
    }
  }

  return 0;
}

#include "RXGenMCCodeEmitter.inc"
