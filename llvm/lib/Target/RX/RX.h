//===-- RX.h - Top-level interface for RX -----------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// RX back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_RX_H
#define LLVM_LIB_TARGET_RX_RX_H

#include "MCTargetDesc/RXBaseInfo.h"

namespace llvm {
class RXTargetMachine;
class AsmPrinter;
class FunctionPass;
class MCInst;
class MCOperand;
class MachineInstr;
class MachineOperand;
class PassRegistry;

void LowerRXMachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    const AsmPrinter &AP);
bool LowerRXMachineOperandToMCOperand(const MachineOperand &MO,
                                         MCOperand &MCOp, const AsmPrinter &AP);

FunctionPass *createRXISelDag(RXTargetMachine &TM);

FunctionPass *createRXMergeBaseOffsetOptPass();
void initializeRXMergeBaseOffsetOptPass(PassRegistry &);
}

#endif
