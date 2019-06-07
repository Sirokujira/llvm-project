//===-- RXMCTargetDesc.cpp - RX Target Descriptions -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
///
/// This file provides RX-specific target descriptions.
///
//===----------------------------------------------------------------------===//

#include "RXMCTargetDesc.h"
#include "InstPrinter/RXInstPrinter.h"
#include "RXELFStreamer.h"
#include "RXMCAsmInfo.h"
#include "RXTargetStreamer.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "RXGenInstrInfo.inc"

#define GET_REGINFO_MC_DESC
#include "RXGenRegisterInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "RXGenSubtargetInfo.inc"

using namespace llvm;

static MCInstrInfo *createRXMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitRXMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createRXMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitRXMCRegisterInfo(X, RX::R1);
  return X;
}

static MCAsmInfo *createRXMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT) {
  return new RXMCAsmInfo(TT);
}

static MCSubtargetInfo *createRXMCSubtargetInfo(const Triple &TT,
                                                   StringRef CPU, StringRef FS) {
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = TT.isArch64Bit() ? "rx" : "rx";
  return createRXMCSubtargetInfoImpl(TT, CPUName, FS);
}

static MCInstPrinter *createRXMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new RXInstPrinter(MAI, MII, MRI);
}

static MCTargetStreamer *
createRXObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  const Triple &TT = STI.getTargetTriple();
  if (TT.isOSBinFormatELF())
    return new RXTargetELFStreamer(S, STI);
  return nullptr;
}

static MCTargetStreamer *createRXAsmTargetStreamer(MCStreamer &S,
                                                      formatted_raw_ostream &OS,
                                                      MCInstPrinter *InstPrint,
                                                      bool isVerboseAsm) {
  return new RXTargetAsmStreamer(S, OS);
}

extern "C" void LLVMInitializeRXTargetMC() {
  for (Target *T : {&getTheRXTarget(), &getTheRXTarget()}) {
    TargetRegistry::RegisterMCAsmInfo(*T, createRXMCAsmInfo);
    TargetRegistry::RegisterMCInstrInfo(*T, createRXMCInstrInfo);
    TargetRegistry::RegisterMCRegInfo(*T, createRXMCRegisterInfo);
    TargetRegistry::RegisterMCAsmBackend(*T, createRXAsmBackend);
    TargetRegistry::RegisterMCCodeEmitter(*T, createRXMCCodeEmitter);
    TargetRegistry::RegisterMCInstPrinter(*T, createRXMCInstPrinter);
    TargetRegistry::RegisterMCSubtargetInfo(*T, createRXMCSubtargetInfo);
    TargetRegistry::RegisterObjectTargetStreamer(
        *T, createRXObjectTargetStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createRXAsmTargetStreamer);
  }
}
