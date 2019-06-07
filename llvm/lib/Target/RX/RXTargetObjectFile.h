//===-- RXTargetObjectFile.h - RX Object Info -*- C++ ---------*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_RXTARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_RX_RXTARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {
class RXTargetMachine;

/// This implementation is used for RX ELF targets.
class RXELFTargetObjectFile : public TargetLoweringObjectFileELF {
  void Initialize(MCContext &Ctx, const TargetMachine &TM) override;
};

} // end namespace llvm

#endif
