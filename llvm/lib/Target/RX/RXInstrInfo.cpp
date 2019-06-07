//===-- RXInstrInfo.cpp - RX Instruction Information ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the RX implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "RXInstrInfo.h"
#include "RX.h"
#include "RXSubtarget.h"
#include "RXTargetMachine.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "RXGenInstrInfo.inc"

using namespace llvm;

RXInstrInfo::RXInstrInfo()
    : RXGenInstrInfo(RX::ADJCALLSTACKDOWN, RX::ADJCALLSTACKUP) {}

unsigned RXInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                             int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  // case RX::LB:
  // case RX::LBU:
  // case RX::LH:
  // case RX::LHU:
  // case RX::LW:
  // case RX::FLW:
  // case RX::LWU:
  // case RX::LD:
  // case RX::FLD:
  //   break;
  }

  if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
      MI.getOperand(2).getImm() == 0) {
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  }

  return 0;
}

unsigned RXInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                            int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  // case RX::SB:
  // case RX::SH:
  // case RX::SW:
  // case RX::FSW:
  // case RX::SD:
  // case RX::FSD:
    break;
  }

  if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
      MI.getOperand(1).getImm() == 0) {
    FrameIndex = MI.getOperand(0).getIndex();
    return MI.getOperand(2).getReg();
  }

  return 0;
}

void RXInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MBBI,
                                 const DebugLoc &DL, unsigned DstReg,
                                 unsigned SrcReg, bool KillSrc) const {
  if (RX::GPRRegClass.contains(DstReg, SrcReg)) {
    BuildMI(MBB, MBBI, DL, get(RX::ADD), DstReg)
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0);
    return;
  }

  // TODO: RX Ç≈ÇÃëŒâûÇÕÇ«Ç§çsÇ§?
  // FPR->FPR copies
  unsigned Opc;
  // if (RX::FPR32RegClass.contains(DstReg, SrcReg))
  //   Opc = RX::FSGNJ_S;
  // else if (RX::FPR64RegClass.contains(DstReg, SrcReg))
  //   Opc = RX::FSGNJ_D;
  // else
  //   llvm_unreachable("Impossible reg-to-reg copy");
  llvm_unreachable("Impossible reg-to-reg copy");

  BuildMI(MBB, MBBI, DL, get(Opc), DstReg)
      .addReg(SrcReg, getKillRegState(KillSrc))
      .addReg(SrcReg, getKillRegState(KillSrc));
}

void RXInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                         MachineBasicBlock::iterator I,
                                         unsigned SrcReg, bool IsKill, int FI,
                                         const TargetRegisterClass *RC,
                                         const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned Opcode;

  if (RX::GPRRegClass.hasSubClassEq(RC))
    Opcode = TRI->getRegSizeInBits(RX::GPRRegClass) == 32 ?
             RX::R0 : RX::R0;
  else
    llvm_unreachable("Can't store this register to stack slot");

  BuildMI(MBB, I, DL, get(Opcode))
      .addReg(SrcReg, getKillRegState(IsKill))
      .addFrameIndex(FI)
      .addImm(0);
}

void RXInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator I,
                                          unsigned DstReg, int FI,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end())
    DL = I->getDebugLoc();

  unsigned Opcode;

  if (RX::GPRRegClass.hasSubClassEq(RC))
    Opcode = TRI->getRegSizeInBits(RX::GPRRegClass) == 32 ?
             RX::USP : RX::USP;
  else
    llvm_unreachable("Can't load this register from stack slot");

  BuildMI(MBB, I, DL, get(Opcode), DstReg).addFrameIndex(FI).addImm(0);
}

// The contents of values added to Cond are not examined outside of
// RXInstrInfo, giving us flexibility in what to push to it. For RX, we
// push BranchOpcode, Reg1, Reg2.
static void parseCondBranch(MachineInstr &LastInst, MachineBasicBlock *&Target,
                            SmallVectorImpl<MachineOperand> &Cond) {
  // Block ends with fall-through condbranch.
  assert(LastInst.getDesc().isConditionalBranch() &&
         "Unknown conditional branch");
  Target = LastInst.getOperand(2).getMBB();
  Cond.push_back(MachineOperand::CreateImm(LastInst.getOpcode()));
  Cond.push_back(LastInst.getOperand(0));
  Cond.push_back(LastInst.getOperand(1));
}

static unsigned getOppositeBranchOpcode(int Opc) {
  switch (Opc) {
  default:
    llvm_unreachable("Unrecognized conditional branch");
  case RX::BEQ:
    return RX::BNE;
  case RX::BNE:
    return RX::BEQ;
  case RX::BLT:
    return RX::BGE;
  case RX::BGE:
    return RX::BLT;
  case RX::BLTU:
    return RX::BGEU;
  case RX::BGEU:
    return RX::BLTU;
  }
}

bool RXInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *&TBB,
                                   MachineBasicBlock *&FBB,
                                   SmallVectorImpl<MachineOperand> &Cond,
                                   bool AllowModify) const {
  TBB = FBB = nullptr;
  Cond.clear();

  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end() || !isUnpredicatedTerminator(*I))
    return false;

  // Count the number of terminators and find the first unconditional or
  // indirect branch.
  MachineBasicBlock::iterator FirstUncondOrIndirectBr = MBB.end();
  int NumTerminators = 0;
  for (auto J = I.getReverse(); J != MBB.rend() && isUnpredicatedTerminator(*J);
       J++) {
    NumTerminators++;
    if (J->getDesc().isUnconditionalBranch() ||
        J->getDesc().isIndirectBranch()) {
      FirstUncondOrIndirectBr = J.getReverse();
    }
  }

  // If AllowModify is true, we can erase any terminators after
  // FirstUncondOrIndirectBR.
  if (AllowModify && FirstUncondOrIndirectBr != MBB.end()) {
    while (std::next(FirstUncondOrIndirectBr) != MBB.end()) {
      std::next(FirstUncondOrIndirectBr)->eraseFromParent();
      NumTerminators--;
    }
    I = FirstUncondOrIndirectBr;
  }

  // We can't handle blocks that end in an indirect branch.
  if (I->getDesc().isIndirectBranch())
    return true;

  // We can't handle blocks with more than 2 terminators.
  if (NumTerminators > 2)
    return true;

  // Handle a single unconditional branch.
  if (NumTerminators == 1 && I->getDesc().isUnconditionalBranch()) {
    TBB = I->getOperand(0).getMBB();
    return false;
  }

  // Handle a single conditional branch.
  if (NumTerminators == 1 && I->getDesc().isConditionalBranch()) {
    parseCondBranch(*I, TBB, Cond);
    return false;
  }

  // Handle a conditional branch followed by an unconditional branch.
  if (NumTerminators == 2 && std::prev(I)->getDesc().isConditionalBranch() &&
      I->getDesc().isUnconditionalBranch()) {
    parseCondBranch(*std::prev(I), TBB, Cond);
    FBB = I->getOperand(0).getMBB();
    return false;
  }

  // Otherwise, we can't handle this.
  return true;
}

unsigned RXInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                      int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;
  MachineBasicBlock::iterator I = MBB.getLastNonDebugInstr();
  if (I == MBB.end())
    return 0;

  if (!I->getDesc().isUnconditionalBranch() &&
      !I->getDesc().isConditionalBranch())
    return 0;

  // Remove the branch.
  I->eraseFromParent();
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);

  I = MBB.end();

  if (I == MBB.begin())
    return 1;
  --I;
  if (!I->getDesc().isConditionalBranch())
    return 1;

  // Remove the branch.
  I->eraseFromParent();
  if (BytesRemoved)
    *BytesRemoved += getInstSizeInBytes(*I);
  return 2;
}

// Inserts a branch into the end of the specific MachineBasicBlock, returning
// the number of instructions inserted.
// ì¡íËÇÃMachineBasicBlockÇÃññîˆÇ…ï™äÚÇë}ì¸ÇµÅAë}ì¸Ç≥ÇÍÇΩñΩóﬂÇÃêîÇï‘ÇµÇ‹Ç∑ÅB
unsigned RXInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                   MachineBasicBlock *TBB,
                                   MachineBasicBlock *FBB,
                                   ArrayRef<MachineOperand> Cond, 
                                   const DebugLoc &DL,
                                   int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // RXFunctionInfo *AFI = MBB.getParent()->getInfo<RXFunctionInfo>();
  // BCnd?
  int BOpc   = RX::BEQ;
  int BccOpc   = RX::BEQ;

  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 3 || Cond.size() == 0) &&
         "RX branch conditions have two components!");

  // Unconditional branch.
  if (Cond.empty()) {
    MachineInstr &MI = *BuildMI(&MBB, DL, get(BOpc)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Either a one or two-way conditional branch.
  unsigned Opc = Cond[0].getImm();
  MachineInstr &CondMI =
      *BuildMI(&MBB, DL, get(Opc)).add(Cond[1]).add(Cond[2]).addMBB(TBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);

  // One-way conditional branch.
  if (!FBB)
    return 1;

  // Two-way conditional branch.
  MachineInstr &MI = *BuildMI(&MBB, DL, get(BOpc)).addMBB(FBB);
  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(MI);
  return 2;
}

unsigned RXInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                              MachineBasicBlock &DestBB,
                                              const DebugLoc &DL,
                                              int64_t BrOffset,
                                              RegScavenger *RS) const {
  assert(RS && "RegScavenger required for long branching");
  assert(MBB.empty() &&
         "new block should be inserted for expanding unconditional branch");
  assert(MBB.pred_size() == 1);

  MachineFunction *MF = MBB.getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();
  const auto &TM = static_cast<const RXTargetMachine &>(MF->getTarget());
  const auto &STI = MF->getSubtarget<RXSubtarget>();

  if (TM.isPositionIndependent() || STI.is64Bit())
    report_fatal_error("Unable to insert indirect branch");

  if (!isInt<32>(BrOffset))
    report_fatal_error(
        "Branch offsets outside of the signed 32-bit range not supported");

  // FIXME: A virtual register must be used initially, as the register
  // scavenger won't work with empty blocks (SIInstrInfo::insertIndirectBranch
  // uses the same workaround).
  unsigned ScratchReg = MRI.createVirtualRegister(&RX::GPRRegClass);
  auto II = MBB.end();

  // MachineInstr &LuiMI = *BuildMI(MBB, II, DL, get(RX::LUI), ScratchReg)
  //                            .addMBB(&DestBB, RXII::MO_HI);
  // BuildMI(MBB, II, DL, get(RX::PseudoBRIND))
  //     .addReg(ScratchReg, RegState::Kill)
  //     .addMBB(&DestBB, RXII::MO_LO);

  RS->enterBasicBlockEnd(MBB);
  // unsigned Scav = RS->scavengeRegisterBackwards(
  //     RX::GPRRegClass, MachineBasicBlock::iterator(LuiMI), false, 0);
  // MRI.replaceRegWith(ScratchReg, Scav);
  MRI.clearVirtRegs();
  // RS->setRegUsed(Scav);
  return 8;
}

bool RXInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert((Cond.size() == 3) && "Invalid branch condition!");
  Cond[0].setImm(getOppositeBranchOpcode(Cond[0].getImm()));
  return false;
}

MachineBasicBlock *
RXInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  assert(MI.getDesc().isBranch() && "Unexpected opcode!");
  // The branch target is always the last operand.
  int NumOp = MI.getNumExplicitOperands();
  return MI.getOperand(NumOp - 1).getMBB();
}

bool RXInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                           int64_t BrOffset) const {
  // Ideally we could determine the supported branch offset from the
  // RXII::FormMask, but this can't be used for Pseudo instructions like 
  // PseudoBR.
  switch (BranchOp) {
  default:
    llvm_unreachable("Unexpected opcode!");
  case RX::BEQ:
  case RX::BNE:
  case RX::BLT:
  case RX::BGE:
  case RX::BLTU:
  case RX::BGEU:
    return isIntN(13, BrOffset);
  }
}

unsigned RXInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  default: { return get(Opcode).getSize(); }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case TargetOpcode::INLINEASM: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const auto &TM = static_cast<const RXTargetMachine &>(MF.getTarget());
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                              *TM.getMCAsmInfo());
  }
  }
}
