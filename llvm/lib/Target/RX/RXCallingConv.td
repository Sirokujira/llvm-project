//===-- RXCallingConv.td - Calling Conventions RX ----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This describes the calling conventions for the RX architecture.
//
//===----------------------------------------------------------------------===//

// The RX calling convention is handled with custom code in
// RXISelLowering.cpp (CC_RX).
// ƒŒƒWƒXƒ^Ží—Þ
def CSR : CalleeSavedRegs<(add (sequence "R%u", 6, 13))>;

// Needed for implementation of RXRegisterInfo::getNoPreservedMask()
def CSR_NoRegs : CalleeSavedRegs<(add R5, R14, R15, BPSW, FPSW)>;

// Interrupt handler needs to save/restore all registers that are used,
// both Caller and Callee saved registers.
def CSR_Interrupt : CalleeSavedRegs<(add)>;

// Same as CSR_Interrupt, but including all 32-bit FP registers.
def CSR_XLEN_F32_Interrupt: CalleeSavedRegs<(add)>;

