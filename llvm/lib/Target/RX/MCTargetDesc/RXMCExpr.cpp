//===-- RXMCExpr.cpp - RX specific MC expression classes ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of the assembly expression modifiers
// accepted by the RX architecture (e.g. ":lo12:", ":gottprel_g1:", ...).
//
//===----------------------------------------------------------------------===//

#include "RX.h"
#include "RXMCExpr.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "rxmcexpr"

const RXMCExpr *RXMCExpr::create(const MCExpr *Expr, VariantKind Kind,
                                       MCContext &Ctx) {
  return new (Ctx) RXMCExpr(Expr, Kind);
}

void RXMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  bool HasVariant =
      ((getKind() != VK_RX_None) && (getKind() != VK_RX_CALL));
  if (HasVariant)
    OS << '%' << getVariantKindName(getKind()) << '(';
  Expr->print(OS, MAI);
  if (HasVariant)
    OS << ')';
}

bool RXMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                            const MCAsmLayout *Layout,
                                            const MCFixup *Fixup) const {
  if (!getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup))
    return false;

  // Some custom fixup types are not valid with symbol difference expressions
  if (Res.getSymA() && Res.getSymB()) {
    switch (getKind()) {
    default:
      return true;
    case VK_RX_LO:
    case VK_RX_HI:
    case VK_RX_PCREL_LO:
    case VK_RX_PCREL_HI:
      return false;
    }
  }

  return true;
}

void RXMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

RXMCExpr::VariantKind RXMCExpr::getVariantKindForName(StringRef name) {
  return StringSwitch<RXMCExpr::VariantKind>(name)
      .Case("lo", VK_RX_LO)
      .Case("hi", VK_RX_HI)
      .Case("pcrel_lo", VK_RX_PCREL_LO)
      .Case("pcrel_hi", VK_RX_PCREL_HI)
      .Default(VK_RX_Invalid);
}

StringRef RXMCExpr::getVariantKindName(VariantKind Kind) {
  switch (Kind) {
  default:
    llvm_unreachable("Invalid ELF symbol kind");
  case VK_RX_LO:
    return "lo";
  case VK_RX_HI:
    return "hi";
  case VK_RX_PCREL_LO:
    return "pcrel_lo";
  case VK_RX_PCREL_HI:
    return "pcrel_hi";
  }
}

bool RXMCExpr::evaluateAsConstant(int64_t &Res) const {
  MCValue Value;

  if (Kind == VK_RX_PCREL_HI || Kind == VK_RX_PCREL_LO ||
      Kind == VK_RX_CALL)
    return false;

  if (!getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr))
    return false;

  if (!Value.isAbsolute())
    return false;

  Res = evaluateAsInt64(Value.getConstant());
  return true;
}

int64_t RXMCExpr::evaluateAsInt64(int64_t Value) const {
  switch (Kind) {
  default:
    llvm_unreachable("Invalid kind");
  case VK_RX_LO:
    return SignExtend64<12>(Value);
  case VK_RX_HI:
    // Add 1 if bit 11 is 1, to compensate for low 12 bits being negative.
    return ((Value + 0x800) >> 12) & 0xfffff;
  }
}
