//===-- RXSubtarget.cpp - RX Subtarget Information ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the RX specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "RXSubtarget.h"
#include "RX.h"
#include "RXFrameLowering.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "rx-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "RXGenSubtargetInfo.inc"

void RXSubtarget::anchor() {}

RXSubtarget &RXSubtarget::initializeSubtargetDependencies(StringRef CPU,
                                                                StringRef FS,
                                                                bool Is64Bit) {
  // Determine default and user-specified characteristics
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "rx";
  ParseSubtargetFeatures(CPUName, FS);
  return *this;
}

RXSubtarget::RXSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, const TargetMachine &TM)
    : RXGenSubtargetInfo(TT, CPU, FS),
      FrameLowering(initializeSubtargetDependencies(CPU, FS, false)),
      InstrInfo(), RegInfo(getHwMode()), TLInfo(TM, *this) {}
