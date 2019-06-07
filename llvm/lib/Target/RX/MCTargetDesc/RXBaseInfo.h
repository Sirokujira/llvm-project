//===-- RXBaseInfo.h - Top level definitions for RX MC ----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone enum definitions for the RX target
// useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_RX_MCTARGETDESC_RXBASEINFO_H
#define LLVM_LIB_TARGET_RX_MCTARGETDESC_RXBASEINFO_H

#include "RXMCTargetDesc.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSwitch.h"

namespace llvm {

// RXII - This namespace holds all of the target specific flags that
// instruction info tracks. All definitions must match RXInstrFormats.td.
namespace RXII {
enum {
  InstFormatPseudo    = 0,
  InstFormatRawFrm    = 1,
  InstFormatImmediate = 2,
  InstFormatRegister  = 3,
  InstFormatIndirect  = 4,
  InstFormatPostinc   = 5,
  InstFormatPredec    = 6,
  InstFormatCondition = 7,
  InstFormatFlag      = 8,
  InstFormatTwoReg    = 9,
  InstFormatOther     = 10,

  InstFormatMask = 31
};

enum {
  ImmNoImm,
  ImmSIMM8,
  ImmSIMM16,
  ImmSIMM24,
  ImmIMM32 = 0
};

enum {
  MemexB,
  MemexW,
  MemexL,
  MemexUW,
};

enum {
  RXCond_eq = 0,
  RXCond_z = 0,
  RXCond_ne = 1,
  RXCond_nz = 1,
  RXCond_geu = 2,
  RXCond_c = 2,
  RXCond_ltu = 3,
  RXCond_nc = 3,
  RXCond_gtu = 4,
  RXCond_leu = 5,
  RXCond_pz = 6,
  RXCond_n = 7,
  RXCond_ge = 8,
  RXCond_lt = 9,
  RXCond_gt = 10,
  RXCond_le = 11,
  RXCond_o = 12,
  RXCond_no = 13,
  RXCond_always = 14,
  RXCond_never  = 15,
};

enum {
  RXC_PSW = 0,
  RXC_USP = 2,
  RXC_FPSW = 3,
  RXC_BPSW = 8,
  RXC_BPC = 9,
  RXC_ISP = 10,
  RXC_FINTV = 11,
  RXC_INTB  = 12,
};

enum {
  MO_None,
  MO_LO,
  MO_HI,
  MO_PCREL_HI,
};
} // namespace RXII

// Describes the predecessor/successor bits used in the FENCE instruction.
namespace RXFenceField {
enum FenceField {
  I = 8,
  O = 4,
  R = 2,
  W = 1
};
}

// Describes the supported floating point rounding mode encodings.
namespace RXFPRndMode {
enum RoundingMode {
  RNE = 0,
  RTZ = 1,
  RDN = 2,
  RUP = 3,
  RMM = 4,
  DYN = 7,
  Invalid
};

inline static StringRef roundingModeToString(RoundingMode RndMode) {
  switch (RndMode) {
  default:
    llvm_unreachable("Unknown floating point rounding mode");
  case RXFPRndMode::RNE:
    return "rne";
  case RXFPRndMode::RTZ:
    return "rtz";
  case RXFPRndMode::RDN:
    return "rdn";
  case RXFPRndMode::RUP:
    return "rup";
  case RXFPRndMode::RMM:
    return "rmm";
  case RXFPRndMode::DYN:
    return "dyn";
  }
}

inline static RoundingMode stringToRoundingMode(StringRef Str) {
  return StringSwitch<RoundingMode>(Str)
      .Case("rne", RXFPRndMode::RNE)
      .Case("rtz", RXFPRndMode::RTZ)
      .Case("rdn", RXFPRndMode::RDN)
      .Case("rup", RXFPRndMode::RUP)
      .Case("rmm", RXFPRndMode::RMM)
      .Case("dyn", RXFPRndMode::DYN)
      .Default(RXFPRndMode::Invalid);
}
} // namespace RXFPRndMode
} // namespace llvm

#endif
