//===-- RXFixupKinds.h - RX Specific Fixup Entries --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_MCTARGETDESC_RXFIXUPKINDS_H
#define LLVM_LIB_TARGET_RX_MCTARGETDESC_RXFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

#undef RX

namespace llvm {
namespace RX {
enum Fixups {
  // 
  fixup_rx_data_24 = FirstTargetFixupKind,
  // 
  fixup_rx_pcrel_24,
  // fixup_rx_dir3u_pcrel
  fixup_rx_dir3u_pcrel,
  fixup_rx_rh_16_op0,
  fixup_rx_rh_24_op,
  fixup_rx_rh_32_op,
  fixup_rx_rh_24_uns,
  fixup_rx_rh_8_neg,
  fixup_rx_rh_16_neg,
  fixup_rx_rh_24_neg,
  fixup_rx_rh_32_neg,
  fixup_rx_rh_diff,
  fixup_rx_rh_gprelb,
  fixup_rx_rh_gprelw,
  fixup_rx_rh_gprell,
  fixup_rx_abs32,
  fixup_rx_abs24s,
  fixup_rx_abs16,
  fixup_rx_abs16u,
  fixup_rx_abs16s,
  fixup_rx_abs8,
  fixup_rx_abs8u,
  fixup_rx_abs8s,
  fixup_rx_abs16ul,
  fixup_rx_abs16uw,
  fixup_rx_abs8ul,
  fixup_rx_abs8uw,
  fixup_rx_abs32_rev,
  fixup_rx_abs16_rev,
  fixup_rx_sym,
  fixup_rx_opneg,
  fixup_rx_opadd,
  fixup_rx_opsub,
  fixup_rx_opmul,
  fixup_rx_opdiv,
  fixup_rx_opshla,
  fixup_rx_opshra,
  fixup_rx_opsctsize,
  fixup_rx_opscttop,
  fixup_rx_opand,
  fixup_rx_opor,
  fixup_rx_opxor,
  fixup_rx_opnot,
  fixup_rx_opmod,
  fixup_rx_opromtop,
  fixup_rx_opramtop,
  // fixup_rx_jal - 20-bit fixup for symbol references in the jal
  // instruction
  fixup_rx_jal,
  // fixup_rx_branch - 12-bit fixup for symbol references in the branch
  // instructions
  fixup_rx_branch,
  // fixup_rx_rvc_jump - 11-bit fixup for symbol references in the
  // compressed jump instruction
  fixup_rx_rvc_jump,
  // fixup_rx_rvc_branch - 8-bit fixup for symbol references in the
  // compressed branch instruction
  fixup_rx_rvc_branch,
  // fixup_rx_call - A fixup representing a call attached to the auipc
  // instruction in a pair composed of adjacent auipc+jalr instructions.
  fixup_rx_call,
  // fixup_rx_relax - Used to generate an R_RX_RELAX relocation type,
  // which indicates the linker may relax the instruction pair.
  fixup_rx_relax,

  // fixup_rx_invalid - used as a sentinel and a marker, must be last fixup
  fixup_rx_invalid,
  NumTargetFixupKinds = fixup_rx_invalid - FirstTargetFixupKind
};
} // end namespace RX
} // end namespace llvm

#endif
