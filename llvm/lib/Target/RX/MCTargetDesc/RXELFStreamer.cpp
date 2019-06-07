//===-- RXELFStreamer.cpp - RX ELF Target Streamer Methods ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides RX specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "RXELFStreamer.h"
#include "RXMCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

// This part is for ELF object output.
RXTargetELFStreamer::RXTargetELFStreamer(MCStreamer &S,
                                               const MCSubtargetInfo &STI)
    : RXTargetStreamer(S) {
  MCAssembler &MCA = getStreamer().getAssembler();

  const FeatureBitset &Features = STI.getFeatureBits();

  unsigned EFlags = MCA.getELFHeaderEFlags();

  MCA.setELFHeaderEFlags(EFlags);
}

MCELFStreamer &RXTargetELFStreamer::getStreamer() {
  return static_cast<MCELFStreamer &>(Streamer);
}

void RXTargetELFStreamer::emitDirectiveOptionRXC() {}
void RXTargetELFStreamer::emitDirectiveOptionNoRXC() {}
