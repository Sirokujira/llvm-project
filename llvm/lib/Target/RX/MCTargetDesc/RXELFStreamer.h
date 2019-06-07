//===-- RXELFStreamer.h - RX ELF Target Streamer ---------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_RX_RXELFSTREAMER_H
#define LLVM_LIB_TARGET_RX_RXELFSTREAMER_H

#include "RXTargetStreamer.h"
#include "llvm/MC/MCELFStreamer.h"

namespace llvm {

class RXTargetELFStreamer : public RXTargetStreamer {
public:
  MCELFStreamer &getStreamer();
  RXTargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  virtual void emitDirectiveOptionRXC();
  virtual void emitDirectiveOptionNoRXC();
};
}
#endif
