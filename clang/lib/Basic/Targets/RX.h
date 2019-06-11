//===--- RX.h - Declare RX target feature support ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares RX TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_BASIC_TARGETS_RX_H
#define LLVM_CLANG_LIB_BASIC_TARGETS_RX_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Support/Compiler.h"

namespace clang {
namespace targets {

// RX Target
class RXTargetInfo : public TargetInfo {
protected:
  std::string ABI;

public:
  RXTargetInfo(const llvm::Triple &Triple, const TargetOptions &Opts)
      : TargetInfo(Triple) {
    TLSSupported = false;
    LongDoubleWidth = 64;
    LongDoubleAlign = 64;
    LongDoubleFormat = &llvm::APFloat::IEEEquad();
    SuitableAlign = 64;
    WCharType = SignedInt;
    WIntType = UnsignedInt;

    IntPtrType = SignedInt;
    PtrDiffType = SignedInt;
    SizeType = UnsignedInt;
    resetDataLayout("e-m:e-p:32:32-i32:32-n32-S64");
  }

  bool setABI(const std::string &Name) override {
    // TODO: support ilp32f and ilp32d ABIs.
    if (Name == "ilp32") {
      ABI = Name;
      return true;
    }
    return false;
  }

  StringRef getABI() const override { return ABI; }
  void getTargetDefines(const LangOptions &Opts,
                        MacroBuilder &Builder) const override;

  ArrayRef<Builtin::Info> getTargetBuiltins() const override { return None; }

  BuiltinVaListKind getBuiltinVaListKind() const override {
    return TargetInfo::VoidPtrBuiltinVaList;
  }

  const char *getClobbers() const override { return ""; }

  ArrayRef<const char *> getGCCRegNames() const override;

  ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override;

  bool validateAsmConstraint(const char *&Name,
                             TargetInfo::ConstraintInfo &Info) const override {
    return false;
  }

  bool hasFeature(StringRef Feature) const override;

  bool handleTargetFeatures(std::vector<std::string> &Features,
                            DiagnosticsEngine &Diags) override;
};
} // namespace targets
} // namespace clang

#endif // LLVM_CLANG_LIB_BASIC_TARGETS_RX_H
