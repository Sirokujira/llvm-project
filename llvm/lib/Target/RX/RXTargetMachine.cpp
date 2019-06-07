//===-- RXTargetMachine.cpp - Define TargetMachine for RX -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about RX target spec.
//
//===----------------------------------------------------------------------===//

#include "RX.h"
#include "RXTargetMachine.h"
#include "RXTargetObjectFile.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
using namespace llvm;

extern "C" void LLVMInitializeRXTarget() {
  RegisterTargetMachine<RXTargetMachine> X(getTheRXTarget());
}

static std::string computeDataLayout(const Triple &TT) {
  if (TT.isArch64Bit()) {
    return "e-m:e-p:64:64-i64:64-i128:128-n64-S128";
  } else {
    assert(TT.isArch32Bit() && "only RX are currently supported");
    return "e-m:e-p:32:32-i64:64-n32-S128";
  }
}

static Reloc::Model getEffectiveRelocModel(const Triple &TT,
                                           Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}

RXTargetMachine::RXTargetMachine(const Target &T, const Triple &TT,
                                       StringRef CPU, StringRef FS,
                                       const TargetOptions &Options,
                                       Optional<Reloc::Model> RM,
                                       Optional<CodeModel::Model> CM,
                                       CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, computeDataLayout(TT), TT, CPU, FS, Options,
                        getEffectiveRelocModel(TT, RM),
                        // getEffectiveCodeModel(CM), OL),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      TLOF(make_unique<RXELFTargetObjectFile>()),
      Subtarget(TT, CPU, FS, *this) {
  initAsmInfo();
}

namespace {
class RXPassConfig : public TargetPassConfig {
public:
  RXPassConfig(RXTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  RXTargetMachine &getRXTargetMachine() const {
    return getTM<RXTargetMachine>();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
};
}

TargetPassConfig *RXTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new RXPassConfig(*this, PM);
}

void RXPassConfig::addIRPasses() {
  addPass(createAtomicExpandPass());
  TargetPassConfig::addIRPasses();
}

bool RXPassConfig::addInstSelector() {
  addPass(createRXISelDag(getRXTargetMachine()));

  return false;
}

void RXPassConfig::addPreEmitPass() { addPass(&BranchRelaxationPassID); }

void RXPassConfig::addPreRegAlloc() {
  addPass(createRXMergeBaseOffsetOptPass());
}
