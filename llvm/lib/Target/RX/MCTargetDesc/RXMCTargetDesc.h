//===-- RXMCTargetDesc.h - RX Target Descriptions ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides RX specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_MCTARGETDESC_RXMCTARGETDESC_H
#define LLVM_LIB_TARGET_RX_MCTARGETDESC_RXMCTARGETDESC_H

#include "llvm/Config/config.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/Support/DataTypes.h"
#include <memory>

namespace llvm {
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class StringRef;
class Target;
class Triple;
class raw_ostream;
class raw_pwrite_stream;

Target &getTheRXTarget();

MCCodeEmitter *createRXMCCodeEmitter(const MCInstrInfo &MCII,
                                        const MCRegisterInfo &MRI,
                                        MCContext &Ctx);

MCAsmBackend *createRXAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                    const MCRegisterInfo &MRI,
                                    const MCTargetOptions &Options);

std::unique_ptr<MCObjectTargetWriter> createRXELFObjectWriter(uint8_t OSABI,
                                                                 bool Is64Bit);
}

// Defines symbolic names for RX registers.
#define GET_REGINFO_ENUM
#include "RXGenRegisterInfo.inc"

// Defines symbolic names for RX instructions.
#define GET_INSTRINFO_ENUM
#include "RXGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "RXGenSubtargetInfo.inc"

#endif
