//===-- RX.td - Describe the RX Target Machine -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// RX subtarget features and instruction predicates.
//===----------------------------------------------------------------------===//

def Feature64Bit : SubtargetFeature<"64bit", "HasRX", "true", "Implements RX">;
def IsRX64 : Predicate<"Subtarget->is64Bit()">,
                       AssemblerPredicate<"!Feature64Bit">;
def IsRX32 : Predicate<"!Subtarget->is64Bit()">,
                       AssemblerPredicate<"!Feature64Bit">;
def IsRX : Predicate<"!Subtarget->is64Bit()">,
                       AssemblerPredicate<"!Feature64Bit">;

def RX64           : HwMode<"+64bit">;
def RX32           : HwMode<"-64bit">;

def FeatureRelax
    : SubtargetFeature<"relax", "EnableLinkerRelax", "true",
                       "Enable Linker relaxation.">;

// subtarget -> -mattr=?
// -mattr="relax"
// v1?
// v2?
// v3?
// ïÇìÆè¨êîì_ëŒâû(RX600Å`)

//===----------------------------------------------------------------------===//
// Registers, calling conventions, instruction descriptions.
//===----------------------------------------------------------------------===//

// include "RXSchedule.td"
include "RXRegisterInfo.td"
include "RXCallingConv.td"
include "RXInstrInfo.td"

//===----------------------------------------------------------------------===//
// RX processors supported.
//===----------------------------------------------------------------------===//

def : ProcessorModel<"rx", NoSchedModel, []>;
// def : ProcessorModel<"rx", RxSchedModel, []>;

//===----------------------------------------------------------------------===//
// Define the RX target.
//===----------------------------------------------------------------------===//

def RXInstrInfo : InstrInfo {
  let guessInstructionProperties = 0;
}

def RXAsmParser : AsmParser {
  let ShouldEmitMatchRegisterAltName = 1;
  let AllowDuplicateRegisterNames = 1;
}

def RXAsmWriter : AsmWriter {
  int PassSubtarget = 1;
}

def RX : Target {
  let InstructionSet = RXInstrInfo;
  let AssemblyParsers = [RXAsmParser];
  let AssemblyWriters = [RXAsmWriter];
  let AllowRegisterRenaming = 1;
}
