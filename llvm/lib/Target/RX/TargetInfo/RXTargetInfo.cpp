//===-- RXTargetInfo.cpp - RX Target Implementation -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

namespace llvm {
Target &getTheRXTarget() {
  static Target TheRXTarget;
  return TheRXTarget;
}
}

extern "C" void LLVMInitializeRXTargetInfo() {
  RegisterTarget<Triple::rx> X(getTheRXTarget(), "rx",
                                    "32-bit Renesas CPU", "RX");
}
