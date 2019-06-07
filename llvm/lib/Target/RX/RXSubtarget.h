//===-- RXSubtarget.h - Define Subtarget for the RX -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the RX specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_RXSUBTARGET_H
#define LLVM_LIB_TARGET_RX_RXSUBTARGET_H

#include "RXFrameLowering.h"
#include "RXISelLowering.h"
#include "RXInstrInfo.h"
#include "llvm/CodeGen/SelectionDAGTargetInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#define GET_SUBTARGETINFO_HEADER
#include "RXGenSubtargetInfo.inc"

namespace llvm {

class RXBaseTargetMachine;
class StringRef;

class RXSubtarget : public RXGenSubtargetInfo {
protected:
  enum RXProcFamilyEnum {
    Others,

    // RX100
    // RX200
    RX600,
    RX610,
    RX61N,
    RX620,
    RX62N,
    RX621,
    RX630,
    RX63N,
    RX631,
  };
  enum RXProcClassEnum {
    None,

    RX1Class,
    RX2Class,
    RX3Class
  };
  enum RXArchEnum {
    rx100,
    rx200,
    rx600,
  };
  virtual void anchor();
  bool HasRX = false;
  bool EnableLinkerRelax = false;
  unsigned XLen = 32;
  MVT XLenVT = MVT::i32;
  RXFrameLowering FrameLowering;
  RXInstrInfo InstrInfo;
  RXRegisterInfo RegInfo;
  RXTargetLowering TLInfo;
  SelectionDAGTargetInfo TSInfo;

  /// Initializes using the passed in CPU and feature strings so that we can
  /// use initializer lists for subtarget initialization.
  RXSubtarget &initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                  bool Is64Bit);

protected:
  /// RXProcFamily - RX processor family: Cortex-A8, Cortex-A9, and others.
  RXProcFamilyEnum RXProcFamily = Others;

  /// RXProcClass - RX processor class: RX1Class, RX2Class or RX3Class.
  RXProcClassEnum RXProcClass = RX2Class;

  /// RXArch - RX architecture
  RXArchEnum RXArch = rx600;

public:
  // Initializes the data members to match that of the specified triple.
  RXSubtarget(const Triple &TT, const std::string &CPU,
                 const std::string &FS, const TargetMachine &TM);

  // Parses features string setting specified subtarget options. The
  // definition of this function is auto-generated by tblgen.
  void ParseSubtargetFeatures(StringRef CPU, StringRef FS);

  const RXFrameLowering *getFrameLowering() const override {
    return &FrameLowering;
  }
  const RXInstrInfo *getInstrInfo() const override { return &InstrInfo; }
  const RXRegisterInfo *getRegisterInfo() const override {
    return &RegInfo;
  }
  const RXTargetLowering *getTargetLowering() const override {
    return &TLInfo;
  }
  const SelectionDAGTargetInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }
  bool is64Bit() const { return HasRX; }
  bool enableLinkerRelax() const { return EnableLinkerRelax; }
  MVT getXLenVT() const { return XLenVT; }
  unsigned getXLen() const { return XLen; }
};
} // End llvm namespace

#endif
