//===-- RXTargetStreamer.cpp - RX Target Streamer Methods -----------===//
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

#include "RXTargetStreamer.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

RXTargetStreamer::RXTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

// This part is for ascii assembly output
RXTargetAsmStreamer::RXTargetAsmStreamer(MCStreamer &S,
                                               formatted_raw_ostream &OS)
    : RXTargetStreamer(S), OS(OS) {}

void RXTargetAsmStreamer::emitDirectiveOptionRXC() {
  OS << "\t.option\trvc\n";
}

void RXTargetAsmStreamer::emitDirectiveOptionNoRXC() {
  OS << "\t.option\tnorvc\n";
}
