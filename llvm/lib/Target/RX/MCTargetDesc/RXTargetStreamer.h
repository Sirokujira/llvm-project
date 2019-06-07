//===-- RXTargetStreamer.h - RX Target Streamer ----------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_RXTARGETSTREAMER_H
#define LLVM_LIB_TARGET_RX_RXTARGETSTREAMER_H

#include "llvm/MC/MCStreamer.h"

namespace llvm {

class RXTargetStreamer : public MCTargetStreamer {
public:
  RXTargetStreamer(MCStreamer &S);

  virtual void emitDirectiveOptionRXC() = 0;
  virtual void emitDirectiveOptionNoRXC() = 0;
};

// This part is for ascii assembly output
class RXTargetAsmStreamer : public RXTargetStreamer {
  formatted_raw_ostream &OS;

public:
  RXTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);

  void emitDirectiveOptionRXC() override;
  void emitDirectiveOptionNoRXC() override;
};

}
#endif
