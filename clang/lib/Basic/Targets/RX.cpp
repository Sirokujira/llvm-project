//===--- RX.cpp - Implement RX target feature support ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements RX TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "RX.h"
#include "clang/Basic/MacroBuilder.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang;
using namespace clang::targets;

ArrayRef<const char *> RXTargetInfo::getGCCRegNames() const {
  static const char *const GCCRegNames[] = {
      "r0",  "r1",  "r2",  "r3",  "r4",  "r5",  "r6",  "r7",
      "r8",  "r9",  "r10", "r11", "r12", "r13", "r14", "r15",
      "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
      "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31"};
  return llvm::makeArrayRef(GCCRegNames);
}

ArrayRef<TargetInfo::GCCRegAlias> RXTargetInfo::getGCCRegAliases() const {
  static const TargetInfo::GCCRegAlias GCCRegAliases[] = {
      {{"zero"}, "r0"}, {{"ra"}, "r1"},  {{"sp"}, "r2"},   {{"gp"}, "r3"},
      {{"tp"}, "r4"},   {{"t0"}, "r5"},  {{"t1"}, "r6"},   {{"t2"}, "r7"},
      {{"s0"}, "r8"},   {{"s1"}, "r9"},  {{"a0"}, "r10"},  {{"a1"}, "r11"},
      {{"a2"}, "r12"},  {{"a3"}, "r13"}, {{"a4"}, "r15"},  {{"a5"}, "r15"},
      {{"a6"}, "r16"},  {{"a7"}, "r17"}, {{"s2"}, "r18"},  {{"s3"}, "r19"},
      {{"s4"}, "r20"},  {{"s5"}, "r21"}, {{"s6"}, "r22"},  {{"s7"}, "r23"},
      {{"s8"}, "r24"},  {{"s9"}, "r25"}, {{"s10"}, "r26"}, {{"s11"}, "r27"},
      {{"t3"}, "r28"},  {{"t4"}, "r29"}, {{"t5"}, "r30"},  {{"t6"}, "r31"}};
  return llvm::makeArrayRef(GCCRegAliases);
}

void RXTargetInfo::getTargetDefines(const LangOptions &Opts,
                                       MacroBuilder &Builder) const {
  Builder.defineMacro("__ELF__");
  Builder.defineMacro("__rx");
  bool Is64Bit = getTriple().getArch() == llvm::Triple::rx;
  Builder.defineMacro("__rx_xlen", Is64Bit ? "32" : "32");
  // TODO: modify when more code models and ABIs are supported.
  Builder.defineMacro("__rx_cmodel_medlow");
  Builder.defineMacro("__rx_float_abi_soft");

  Builder.defineMacro("__rx_mul");
  Builder.defineMacro("__rx_div");
  Builder.defineMacro("__rx_muldiv");
  Builder.defineMacro("__rx_atomic");

  Builder.defineMacro("__rx_flen", "32");
  Builder.defineMacro("__rx_fdiv");
  Builder.defineMacro("__rx_fsqrt");
}

/// Return true if has this feature, need to sync with handleTargetFeatures.
bool RXTargetInfo::hasFeature(StringRef Feature) const {
  bool Is64Bit = getTriple().getArch() == llvm::Triple::rx;
  return llvm::StringSwitch<bool>(Feature)
      .Case("rx", true)
      .Default(false);
}

/// Perform initialization based on the user configured set of features.
bool RXTargetInfo::handleTargetFeatures(std::vector<std::string> &Features,
                                           DiagnosticsEngine &Diags) {
  for (const auto &Feature : Features) {
  }

  return true;
}
