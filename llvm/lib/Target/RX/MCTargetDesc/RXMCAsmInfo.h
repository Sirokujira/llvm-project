//===-- RXMCAsmInfo.h - RX Asm Info ----------------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the Company of Renesas
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the RXMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_MCTARGETDESC_RXMCASMINFO_H
#define LLVM_LIB_TARGET_RX_MCTARGETDESC_RXMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class RXMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit RXMCAsmInfo(const Triple &TargetTriple);
};

} // namespace llvm

#endif
