//===-- RXRegisterInfo.cpp - RX Register Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RX implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "RXRegisterInfo.h"
#include "RX.h"
#include "RXSubtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_REGINFO_TARGET_DESC
#include "RXGenRegisterInfo.inc"

using namespace llvm;

RXRegisterInfo::RXRegisterInfo(unsigned HwMode)
    : RXGenRegisterInfo(RX::R1, /*DwarfFlavour*/0, /*EHFlavor*/0,
                           /*PC*/0, HwMode) {}

const MCPhysReg *
RXRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  if (MF->getFunction().hasFnAttribute("interrupt")) {
  	return CSR_Interrupt_SaveList;
  }
  return CSR_SaveList;
}

// レジスタ割り当てに使用しない、予約済みのレジスタを返す。
BitVector RXRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Use markSuperRegs to ensure any register aliases are also reserved
  markSuperRegs(Reserved, RX::R0);
  // ISP
  // USP
  markSuperRegs(Reserved, RX::BPSW);
  markSuperRegs(Reserved, RX::FPSW);
  assert(checkAllSuperRegsMarked(Reserved));
  return Reserved;
}

bool RXRegisterInfo::isConstantPhysReg(unsigned PhysReg) const {
  return PhysReg == RX::R0;
}

const uint32_t *RXRegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

void RXRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                            int SPAdj, unsigned FIOperandNum,
                                            RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj value");

  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const RXInstrInfo *TII = MF.getSubtarget<RXSubtarget>().getInstrInfo();
  DebugLoc DL = MI.getDebugLoc();

  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  unsigned FrameReg;
  int Offset =
      getFrameLowering(MF)->getFrameIndexReference(MF, FrameIndex, FrameReg) +
      MI.getOperand(FIOperandNum + 1).getImm();

  if (!isInt<32>(Offset)) {
    report_fatal_error(
        "Frame offsets outside of the signed 32-bit range not supported");
  }

  MachineBasicBlock &MBB = *MI.getParent();
  bool FrameRegIsKill = false;

  unsigned ScratchReg = MRI.createVirtualRegister(&RX::GPRRegClass);
  BuildMI(MBB, II, DL, TII->get(RX::ADD), ScratchReg)
      .addReg(FrameReg)
      .addReg(ScratchReg, RegState::Kill);
  Offset = 0;
  FrameReg = ScratchReg;
  FrameRegIsKill = true;

  MI.getOperand(FIOperandNum)
      .ChangeToRegister(FrameReg, false, false, FrameRegIsKill);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
}

unsigned RXRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  // const TargetFrameLowering *TFI = getFrameLowering(MF);
  // return TFI->hasFP(MF) ? RX::X8 : RX::X2;
  // TODO: RX に FramePointer ないけど対処はどうするのか?
  // 自前で計算する?
  return RX::R0;
}

const uint32_t *
RXRegisterInfo::getCallPreservedMask(const MachineFunction & MF,
                                        CallingConv::ID /*CC*/) const {
  if (MF.getFunction().hasFnAttribute("interrupt")) {
    return CSR_Interrupt_RegMask;
  }
  return CSR_RegMask;
}
