//===-- RXAsmParser.cpp - Parse RX assembly to MCInst instructions --===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/RXBaseInfo.h"
#include "MCTargetDesc/RXMCExpr.h"
#include "MCTargetDesc/RXMCTargetDesc.h"
#include "MCTargetDesc/RXTargetStreamer.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

#include <limits>

using namespace llvm;

namespace {
struct RXOperand;

class RXAsmParser : public MCTargetAsmParser {
  SMLoc getLoc() const { return getParser().getTok().getLoc(); }
  bool isRX() const { return getSTI().hasFeature(RX::Feature64Bit); }

  RXTargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<RXTargetStreamer &>(TS);
  }

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  bool generateImmOutOfRangeError(OperandVector &Operands, uint64_t ErrorInfo,
                                  int64_t Lower, int64_t Upper, Twine Msg);

  bool MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override;

  bool ParseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  bool ParseDirective(AsmToken DirectiveID) override;

  // Helper to actually emit an instruction to the MCStreamer. Also, when
  // possible, compression of the instruction is performed.
  void emitToStreamer(MCStreamer &S, const MCInst &Inst);

  // Helper to emit pseudo instruction "lla" used in PC-rel addressing.
  void emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

  /// Helper for processing MC instructions that have been successfully matched
  /// by MatchAndEmitInstruction. Modifications to the emitted instructions,
  /// like the expansion of pseudo instructions (e.g., "li"), can be performed
  /// in this method.
  bool processInstruction(MCInst &Inst, SMLoc IDLoc, MCStreamer &Out);

// Auto-generated instruction matching functions
#define GET_ASSEMBLER_HEADER
#include "RXGenAsmMatcher.inc"

  OperandMatchResultTy parseImmediate(OperandVector &Operands);
  OperandMatchResultTy parseRegister(OperandVector &Operands,
                                     bool AllowParens = false);
  OperandMatchResultTy parseMemOpBaseReg(OperandVector &Operands);
  OperandMatchResultTy parseOperandWithModifier(OperandVector &Operands);

  bool parseOperand(OperandVector &Operands, bool ForceImmediate);

  bool parseDirectiveOption();

  void setFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (!(getSTI().getFeatureBits()[Feature])) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
    }
  }

  void clearFeatureBits(uint64_t Feature, StringRef FeatureString) {
    if (getSTI().getFeatureBits()[Feature]) {
      MCSubtargetInfo &STI = copySTI();
      setAvailableFeatures(
          ComputeAvailableFeatures(STI.ToggleFeature(FeatureString)));
    }
  }
public:
  enum RXMatchResultTy {
    Match_Dummy = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "RXGenAsmMatcher.inc"
#undef GET_OPERAND_DIAGNOSTIC_TYPES
  };

  static bool classifySymbolRef(const MCExpr *Expr,
                                RXMCExpr::VariantKind &Kind,
                                int64_t &Addend);

  RXAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
                 const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII) {
    Parser.addAliasForDirective(".half", ".2byte");
    Parser.addAliasForDirective(".hword", ".2byte");
    Parser.addAliasForDirective(".word", ".4byte");
    Parser.addAliasForDirective(".dword", ".8byte");
    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }
};

/// RXOperand - Instances of this class represent a parsed machine
/// instruction
struct RXOperand : public MCParsedAsmOperand {

  enum KindTy {
    Token,
    Register,
    Immediate,
  } Kind;

  bool IsRX;

  struct RegOp {
    unsigned RegNum;
  };

  struct ImmOp {
    const MCExpr *Val;
  };

  SMLoc StartLoc, EndLoc;
  union {
    StringRef Tok;
    RegOp Reg;
    ImmOp Imm;
  };

  RXOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}

public:
  RXOperand(const RXOperand &o) : MCParsedAsmOperand() {
    Kind = o.Kind;
    IsRX = o.IsRX;
    StartLoc = o.StartLoc;
    EndLoc = o.EndLoc;
    switch (Kind) {
    case Register:
      Reg = o.Reg;
      break;
    case Immediate:
      Imm = o.Imm;
      break;
    case Token:
      Tok = o.Tok;
      break;
    }
  }

  bool isToken() const override { return Kind == Token; }
  bool isReg() const override { return Kind == Register; }
  bool isImm() const override { return Kind == Immediate; }
  bool isMem() const override { return false; }

  bool evaluateConstantImm(int64_t &Imm, RXMCExpr::VariantKind &VK) const {
    const MCExpr *Val = getImm();
    bool Ret = false;
    if (auto *RE = dyn_cast<RXMCExpr>(Val)) {
      Ret = RE->evaluateAsConstant(Imm);
      VK = RE->getKind();
    } else if (auto CE = dyn_cast<MCConstantExpr>(Val)) {
      Ret = true;
      VK = RXMCExpr::VK_RX_None;
      Imm = CE->getValue();
    }
    return Ret;
  }

  // True if operand is a symbol with no modifiers, or a constant with no
  // modifiers and isShiftedInt<N-1, 1>(Op).
  template <int N> bool isBareSimmNLsb0() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    bool IsValid;
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isShiftedInt<N - 1, 1>(Imm);
    return IsValid && VK == RXMCExpr::VK_RX_None;
  }

  // Predicate methods for AsmOperands defined in RXInstrInfo.td

  bool isBareSymbol() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    // Must be of 'immediate' type but not a constant.
    if (!isImm() || evaluateConstantImm(Imm, VK))
      return false;
    return RXAsmParser::classifySymbolRef(getImm(), VK, Imm) &&
           VK == RXMCExpr::VK_RX_None;
  }

  /// Return true if the operand is a valid for the fence instruction e.g.
  /// ('iorw').
  bool isFenceArg() const {
    if (!isImm())
      return false;
    const MCExpr *Val = getImm();
    auto *SVal = dyn_cast<MCSymbolRefExpr>(Val);
    if (!SVal || SVal->getKind() != MCSymbolRefExpr::VK_None)
      return false;

    StringRef Str = SVal->getSymbol().getName();
    // Letters must be unique, taken from 'iorw', and in ascending order. This
    // holds as long as each individual character is one of 'iorw' and is
    // greater than the previous character.
    char Prev = '\0';
    for (char c : Str) {
      if (c != 'i' && c != 'o' && c != 'r' && c != 'w')
        return false;
      if (c <= Prev)
        return false;
      Prev = c;
    }
    return true;
  }

  /// Return true if the operand is a valid floating point rounding mode.
  bool isFRMArg() const {
    if (!isImm())
      return false;
    const MCExpr *Val = getImm();
    auto *SVal = dyn_cast<MCSymbolRefExpr>(Val);
    if (!SVal || SVal->getKind() != MCSymbolRefExpr::VK_None)
      return false;

    StringRef Str = SVal->getSymbol().getName();

    return RXFPRndMode::stringToRoundingMode(Str) != RXFPRndMode::Invalid;
  }

  bool isImmXLen() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    // Given only Imm, ensuring that the actually specified constant is either
    // a signed or unsigned 64-bit number is unfortunately impossible.
    bool IsInRange = isRX() ? true : isInt<32>(Imm) || isUInt<32>(Imm);
    return IsConstantImm && IsInRange && VK == RXMCExpr::VK_RX_None;
  }

  bool isUImm1() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<1>(Imm) && VK == RXMCExpr::VK_RX_None;
  }
  bool isUImm3() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<4>(Imm) && VK == RXMCExpr::VK_RX_None;
  }
  bool isUImm4() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<4>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isUImm5() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<5>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isImm8() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<8>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isUImm8() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<8>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isUImm16() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<16>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isUImm24() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<24>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isUImm32() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<32>(Imm) && VK == RXMCExpr::VK_RX_None;
  }

  bool isSImm1() const {
    if (!isImm())
      return false;
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<1>(Imm);
    return IsValid &&
           (VK == RXMCExpr::VK_RX_None || VK == RXMCExpr::VK_RX_LO);
  }

  bool isSImm3() const {
    if (!isImm())
      return false;
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<3>(Imm);
    return IsValid &&
           (VK == RXMCExpr::VK_RX_None || VK == RXMCExpr::VK_RX_LO);
  }

  bool isSImm4() const {
    if (!isImm())
      return false;
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<4>(Imm);
    return IsValid &&
           (VK == RXMCExpr::VK_RX_None || VK == RXMCExpr::VK_RX_LO);
  }

bool isSImm5() const {
    if (!isImm())
      return false;
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<4>(Imm);
    return IsValid &&
           (VK == RXMCExpr::VK_RX_None || VK == RXMCExpr::VK_RX_LO);
  }

  bool isSImm8() const {
    if (!isImm())
      return false;
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<8>(Imm);
    return IsValid &&
           (VK == RXMCExpr::VK_RX_None || VK == RXMCExpr::VK_RX_LO);
  }

  bool isSImm16() const {
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<16>(Imm);
    return IsValid && (VK == RXMCExpr::VK_RX_None ||
                       VK == RXMCExpr::VK_RX_LO ||
                       VK == RXMCExpr::VK_RX_PCREL_LO);
  }

  bool isImmXLenLI() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = true; //evaluateConstantImm(getImm(), Imm, VK);
    if (VK == RXMCExpr::VK_RX_LO || VK == RXMCExpr::VK_RX_PCREL_LO)
      return true;
    // Given only Imm, ensuring that the actually specified constant is either
    // a signed or unsigned 64-bit number is unfortunately impossible.
    bool IsInRange = isUInt<32>(Imm);
    return IsConstantImm && IsInRange && VK == RXMCExpr::VK_RX_None;
  }

  bool isSImm24() const {
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<24>(Imm);
    return IsValid && (VK == RXMCExpr::VK_RX_None ||
                       VK == RXMCExpr::VK_RX_LO ||
                       VK == RXMCExpr::VK_RX_PCREL_LO);
  }

  bool isSImm32() const {
    RXMCExpr::VariantKind VK;
    int64_t Imm;
    bool IsValid;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    if (!IsConstantImm)
      IsValid = RXAsmParser::classifySymbolRef(getImm(), VK, Imm);
    else
      IsValid = isInt<32>(Imm);
    return IsValid && (VK == RXMCExpr::VK_RX_None ||
                       VK == RXMCExpr::VK_RX_LO ||
                       VK == RXMCExpr::VK_RX_PCREL_LO);
  }

  bool isImm32() const {
    int64_t Imm;
    RXMCExpr::VariantKind VK;
    if (!isImm())
      return false;
    bool IsConstantImm = evaluateConstantImm(Imm, VK);
    return IsConstantImm && isUInt<32>(Imm) && VK == RXMCExpr::VK_RX_None;
  }


  /// getStartLoc - Gets location of the first token of this operand
  SMLoc getStartLoc() const override { return StartLoc; }
  /// getEndLoc - Gets location of the last token of this operand
  SMLoc getEndLoc() const override { return EndLoc; }
  /// True if this operand is for an RX instruction
  bool isRX() const { return IsRX; }

  unsigned getReg() const override {
    assert(Kind == Register && "Invalid type access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert(Kind == Immediate && "Invalid type access!");
    return Imm.Val;
  }

  StringRef getToken() const {
    assert(Kind == Token && "Invalid type access!");
    return Tok;
  }

  void print(raw_ostream &OS) const override {
    switch (Kind) {
    case Immediate:
      OS << *getImm();
      break;
    case Register:
      OS << "<register x";
      OS << getReg() << ">";
      break;
    case Token:
      OS << "'" << getToken() << "'";
      break;
    }
  }

  static std::unique_ptr<RXOperand> createToken(StringRef Str, SMLoc S,
                                                   bool IsRX) {
    auto Op = make_unique<RXOperand>(Token);
    Op->Tok = Str;
    Op->StartLoc = S;
    Op->EndLoc = S;
    Op->IsRX = IsRX;
    return Op;
  }

  static std::unique_ptr<RXOperand> createReg(unsigned RegNo, SMLoc S,
                                                 SMLoc E, bool IsRX) {
    auto Op = make_unique<RXOperand>(Register);
    Op->Reg.RegNum = RegNo;
    Op->StartLoc = S;
    Op->EndLoc = E;
    Op->IsRX = IsRX;
    return Op;
  }

  static std::unique_ptr<RXOperand> createImm(const MCExpr *Val, SMLoc S,
                                                 SMLoc E, bool IsRX) {
    auto Op = make_unique<RXOperand>(Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    Op->IsRX = IsRX;
    return Op;
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    assert(Expr && "Expr shouldn't be null!");
    int64_t Imm = 0;
    bool IsConstant = false;
    if (auto *RE = dyn_cast<RXMCExpr>(Expr)) {
      IsConstant = RE->evaluateAsConstant(Imm);
    } else if (auto *CE = dyn_cast<MCConstantExpr>(Expr)) {
      IsConstant = true;
      Imm = CE->getValue();
    }

    if (IsConstant)
      Inst.addOperand(MCOperand::createImm(Imm));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  // Used by the TableGen Code
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  void addFenceArgOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    // isFenceArg has validated the operand, meaning this cast is safe
    auto SE = cast<MCSymbolRefExpr>(getImm());

    unsigned Imm = 0;
    for (char c : SE->getSymbol().getName()) {
      switch (c) {
        default: llvm_unreachable("FenceArg must contain only [iorw]");
        case 'i': Imm |= RXFenceField::I; break;
        case 'o': Imm |= RXFenceField::O; break;
        case 'r': Imm |= RXFenceField::R; break;
        case 'w': Imm |= RXFenceField::W; break;
      }
    }
    Inst.addOperand(MCOperand::createImm(Imm));
  }

  // Returns the rounding mode represented by this RXOperand. Should only
  // be called after checking isFRMArg.
  RXFPRndMode::RoundingMode getRoundingMode() const {
    // isFRMArg has validated the operand, meaning this cast is safe.
    auto SE = cast<MCSymbolRefExpr>(getImm());
    RXFPRndMode::RoundingMode FRM =
        RXFPRndMode::stringToRoundingMode(SE->getSymbol().getName());
    assert(FRM != RXFPRndMode::Invalid && "Invalid rounding mode");
    return FRM;
  }

  void addFRMArgOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::createImm(getRoundingMode()));
  }
};
} // end anonymous namespace.

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "RXGenAsmMatcher.inc"

unsigned RXAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                    unsigned Kind) {
  RXOperand &Op = static_cast<RXOperand &>(AsmOp);
  if (!Op.isReg())
    return Match_InvalidOperand;

  unsigned Reg = Op.getReg();
  bool IsRegFPR32 = false;
  bool IsRegFPR32C = false;

  // Op.Reg.RegNum = Reg;
  return Match_Success;
  // return Match_InvalidOperand;

/*
  // As the parser couldn't differentiate an FPR32 from an FPR64, coerce the
  // register from FPR32 to FPR64 or FPR32C to FPR64C if necessary.
  if ((IsRegFPR32 && Kind == MCK_FPR64) ||
      (IsRegFPR32C && Kind == MCK_FPR64C)) {
    Op.Reg.RegNum = convert_RX_FPR32ToFPR64(Reg);
    return Match_Success;
  }
  return Match_InvalidOperand;
*/
}

bool RXAsmParser::generateImmOutOfRangeError(
    OperandVector &Operands, uint64_t ErrorInfo, int64_t Lower, int64_t Upper,
    Twine Msg = "immediate must be an integer in the range") {
  SMLoc ErrorLoc = ((RXOperand &)*Operands[ErrorInfo]).getStartLoc();
  return Error(ErrorLoc, Msg + " [" + Twine(Lower) + ", " + Twine(Upper) + "]");
}

bool RXAsmParser::MatchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                                             OperandVector &Operands,
                                             MCStreamer &Out,
                                             uint64_t &ErrorInfo,
                                             bool MatchingInlineAsm) {
  MCInst Inst;

  switch (MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm)) 
  {
  default:
    break;
  case Match_Success:
    return processInstruction(Inst, IDLoc, Out);
  case Match_MissingFeature:
    return Error(IDLoc, "instruction use requires an option to be enabled");
  case Match_MnemonicFail:
    return Error(IDLoc, "unrecognized instruction mnemonic");
  case Match_InvalidOperand: {
    SMLoc ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(ErrorLoc, "too few operands for instruction");

      ErrorLoc = ((RXOperand &)*Operands[ErrorInfo]).getStartLoc();
      if (ErrorLoc == SMLoc())
        ErrorLoc = IDLoc;
    }
    return Error(ErrorLoc, "invalid operand for instruction");
  }

  // case Match_InvalidImmXLenLI:
  //   SMLoc ErrorLoc = ((RXOperand &)*Operands[ErrorInfo]).getStartLoc();
  //   return Error(ErrorLoc, "operand must be a constant 64-bit integer");

  case Match_InvalidSImm1:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 0),
                                      (1 << 0) - 1);
  case Match_InvalidSImm3:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 2),
                                      (1 << 2) - 1);
  // case Match_InvalidSImm4:
  //   return generateImmOutOfRangeError(Operands, ErrorInfo, 0, -(1 << 3),
  //                                     (1 << 3) - 1);
  // case Match_InvalidSImm5:
  //   return generateImmOutOfRangeError(Operands, ErrorInfo, 0, -(1 << 4),
  //                                     (1 << 4) - 1);
  case Match_InvalidSImm8:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 7),
                                      (1 << 7) - 1);
  case Match_InvalidSImm16:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 15),
                                      (1 << 15) - 1);
  case Match_InvalidSImm24:
    return generateImmOutOfRangeError(Operands, ErrorInfo, -(1 << 23),
                                      (1 << 23) - 1);
  case Match_InvalidUImm3:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 3) - 1);
  case Match_InvalidUImm4:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 4) - 1);
  case Match_InvalidUImm8:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 8) - 1);
  case Match_InvalidUImm16:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 16) - 1);
  case Match_InvalidUImm24:
    return generateImmOutOfRangeError(Operands, ErrorInfo, 0, (1 << 24) - 1);
  case Match_InvalidFenceArg: {
    SMLoc ErrorLoc = ((RXOperand &)*Operands[ErrorInfo]).getStartLoc();
    return Error(
        ErrorLoc,
        "operand must be formed of letters selected in-order from 'iorw'");
  }
  // FRM?
  // case Match_InvalidFRMArg: {
  //   SMLoc ErrorLoc = ((RXOperand &)*Operands[ErrorInfo]).getStartLoc();
  //   return Error(
  //       ErrorLoc,
  //       "operand must be a valid floating point rounding mode mnemonic");
    // }
    case Match_InvalidBareSymbol: {
        SMLoc ErrorLoc = ((RXOperand &)*Operands[ErrorInfo]).getStartLoc();
        return Error(ErrorLoc, "operand must be a bare symbol name");
    }
  }

  llvm_unreachable("Unknown match type detected!");
}

bool RXAsmParser::ParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                   SMLoc &EndLoc) {
  const AsmToken &Tok = getParser().getTok();
  StartLoc = Tok.getLoc();
  EndLoc = Tok.getEndLoc();
  RegNo = 0;
  StringRef Name = getLexer().getTok().getIdentifier();

  if (!MatchRegisterName(Name) || !MatchRegisterAltName(Name)) {
    getParser().Lex(); // Eat identifier token.
    return false;
  }

  return Error(StartLoc, "invalid register name");
}

OperandMatchResultTy RXAsmParser::parseRegister(OperandVector &Operands,
                                                   bool AllowParens) {
  SMLoc FirstS = getLoc();
  bool HadParens = false;
  AsmToken Buf[2];

  // If this a parenthesised register name is allowed, parse it atomically
  if (AllowParens && getLexer().is(AsmToken::LParen)) {
    size_t ReadCount = getLexer().peekTokens(Buf);
    if (ReadCount == 2 && Buf[1].getKind() == AsmToken::RParen) {
      HadParens = true;
      getParser().Lex(); // Eat '('
    }
  }

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::Identifier:
    StringRef Name = getLexer().getTok().getIdentifier();
    unsigned RegNo = MatchRegisterName(Name);
    if (RegNo == 0) {
      RegNo = MatchRegisterAltName(Name);
      if (RegNo == 0) {
        if (HadParens)
          getLexer().UnLex(Buf[0]);
        return MatchOperand_NoMatch;
      }
    }
    if (HadParens)
      Operands.push_back(RXOperand::createToken("(", FirstS, isRX()));
    SMLoc S = getLoc();
    SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
    getLexer().Lex();
    Operands.push_back(RXOperand::createReg(RegNo, S, E, isRX()));
  }

  if (HadParens) {
    getParser().Lex(); // Eat ')'
    Operands.push_back(RXOperand::createToken(")", getLoc(), isRX()));
  }

  return MatchOperand_Success;
}

OperandMatchResultTy RXAsmParser::parseImmediate(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);
  const MCExpr *Res;

  switch (getLexer().getKind()) {
  default:
    return MatchOperand_NoMatch;
  case AsmToken::LParen:
  case AsmToken::Minus:
  case AsmToken::Plus:
  case AsmToken::Integer:
  case AsmToken::String:
    if (getParser().parseExpression(Res))
      return MatchOperand_ParseFail;
    break;
  case AsmToken::Identifier: {
    StringRef Identifier;
    if (getParser().parseIdentifier(Identifier))
      return MatchOperand_ParseFail;
    MCSymbol *Sym = getContext().getOrCreateSymbol(Identifier);
    Res = MCSymbolRefExpr::create(Sym, MCSymbolRefExpr::VK_None, getContext());
    break;
  }
  case AsmToken::Percent:
    return parseOperandWithModifier(Operands);
  }

  Operands.push_back(RXOperand::createImm(Res, S, E, isRX()));
  return MatchOperand_Success;
}

OperandMatchResultTy
RXAsmParser::parseOperandWithModifier(OperandVector &Operands) {
  SMLoc S = getLoc();
  SMLoc E = SMLoc::getFromPointer(S.getPointer() - 1);

  if (getLexer().getKind() != AsmToken::Percent) {
    Error(getLoc(), "expected '%' for operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '%'

  if (getLexer().getKind() != AsmToken::Identifier) {
    Error(getLoc(), "expected valid identifier for operand modifier");
    return MatchOperand_ParseFail;
  }
  StringRef Identifier = getParser().getTok().getIdentifier();
  RXMCExpr::VariantKind VK = RXMCExpr::getVariantKindForName(Identifier);
  if (VK == RXMCExpr::VK_RX_Invalid) {
    Error(getLoc(), "unrecognized operand modifier");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat the identifier
  if (getLexer().getKind() != AsmToken::LParen) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }
  getParser().Lex(); // Eat '('

  const MCExpr *SubExpr;
  if (getParser().parseParenExpression(SubExpr, E)) {
    return MatchOperand_ParseFail;
  }

  const MCExpr *ModExpr = RXMCExpr::create(SubExpr, VK, getContext());
  Operands.push_back(RXOperand::createImm(ModExpr, S, E, isRX()));
  return MatchOperand_Success;
}

OperandMatchResultTy
RXAsmParser::parseMemOpBaseReg(OperandVector &Operands) {
  if (getLexer().isNot(AsmToken::LParen)) {
    Error(getLoc(), "expected '('");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat '('
  Operands.push_back(RXOperand::createToken("(", getLoc(), isRX()));

  if (parseRegister(Operands) != MatchOperand_Success) {
    Error(getLoc(), "expected register");
    return MatchOperand_ParseFail;
  }

  if (getLexer().isNot(AsmToken::RParen)) {
    Error(getLoc(), "expected ')'");
    return MatchOperand_ParseFail;
  }

  getParser().Lex(); // Eat ')'
  Operands.push_back(RXOperand::createToken(")", getLoc(), isRX()));

  return MatchOperand_Success;
}

/// Looks at a token type and creates the relevant operand from this
/// information, adding to Operands. If operand was parsed, returns false, else
/// true. If ForceImmediate is true, no attempt will be made to parse the
/// operand as a register, which is needed for pseudoinstructions such as
/// call.
bool RXAsmParser::parseOperand(OperandVector &Operands,
                                  bool ForceImmediate) {
  // Attempt to parse token as register, unless ForceImmediate.
  if (!ForceImmediate && parseRegister(Operands, true) == MatchOperand_Success)
    return false;

  // Attempt to parse token as an immediate
  if (parseImmediate(Operands) == MatchOperand_Success) {
    // Parse memory base register if present
    if (getLexer().is(AsmToken::LParen))
      return parseMemOpBaseReg(Operands) != MatchOperand_Success;
    return false;
  }

  // Finally we have exhausted all options and must declare defeat.
  Error(getLoc(), "unknown operand");
  return true;
}

/// Return true if the operand at the OperandIdx for opcode Name should be
/// 'forced' to be parsed as an immediate. This is required for
/// pseudoinstructions such as tail or call, which allow bare symbols to be used
/// that could clash with register names.
static bool shouldForceImediateOperand(StringRef Name, unsigned OperandIdx) {
  // FIXME: This may not scale so perhaps we want to use a data-driven approach
  // instead.
  switch (OperandIdx) {
  case 0:
    // call imm
    // tail imm
    return Name == "tail" || Name == "call";
  case 1:
    // lla rdest, imm
    return Name == "lla";
  default:
    return false;
  }
}

bool RXAsmParser::ParseInstruction(ParseInstructionInfo &Info,
                                      StringRef Name, SMLoc NameLoc,
                                      OperandVector &Operands) {
  // First operand is token for instruction
  Operands.push_back(RXOperand::createToken(Name, NameLoc, isRX()));

  // If there are no more operands, then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse first operand
  if (parseOperand(Operands, shouldForceImediateOperand(Name, 0)))
    return true;

  // Parse until end of statement, consuming commas between operands
  unsigned OperandIdx = 1;
  while (getLexer().is(AsmToken::Comma)) {
    // Consume comma token
    getLexer().Lex();

    // Parse next operand
    if (parseOperand(Operands, shouldForceImediateOperand(Name, OperandIdx)))
      return true;

    ++OperandIdx;
  }

  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    SMLoc Loc = getLexer().getLoc();
    getParser().eatToEndOfStatement();
    return Error(Loc, "unexpected token");
  }

  getParser().Lex(); // Consume the EndOfStatement.
  return false;
}

bool RXAsmParser::classifySymbolRef(const MCExpr *Expr,
                                       RXMCExpr::VariantKind &Kind,
                                       int64_t &Addend) {
  Kind = RXMCExpr::VK_RX_None;
  Addend = 0;

  if (const RXMCExpr *RE = dyn_cast<RXMCExpr>(Expr)) {
    Kind = RE->getKind();
    Expr = RE->getSubExpr();
  }

  // It's a simple symbol reference or constant with no addend.
  if (isa<MCConstantExpr>(Expr) || isa<MCSymbolRefExpr>(Expr))
    return true;

  const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr);
  if (!BE)
    return false;

  if (!isa<MCSymbolRefExpr>(BE->getLHS()))
    return false;

  if (BE->getOpcode() != MCBinaryExpr::Add &&
      BE->getOpcode() != MCBinaryExpr::Sub)
    return false;

  // We are able to support the subtraction of two symbol references
  if (BE->getOpcode() == MCBinaryExpr::Sub &&
      isa<MCSymbolRefExpr>(BE->getRHS()))
    return true;

  // See if the addend is a constant, otherwise there's more going
  // on here than we can deal with.
  auto AddendExpr = dyn_cast<MCConstantExpr>(BE->getRHS());
  if (!AddendExpr)
    return false;

  Addend = AddendExpr->getValue();
  if (BE->getOpcode() == MCBinaryExpr::Sub)
    Addend = -Addend;

  // It's some symbol reference + a constant addend
  return Kind != RXMCExpr::VK_RX_Invalid;
}

bool RXAsmParser::ParseDirective(AsmToken DirectiveID) {
  // This returns false if this function recognizes the directive
  // regardless of whether it is successfully handles or reports an
  // error. Otherwise it returns true to give the generic parser a
  // chance at recognizing it.
  StringRef IDVal = DirectiveID.getString();

  if (IDVal == ".option")
    return parseDirectiveOption();

  return true;
}

bool RXAsmParser::parseDirectiveOption() {
  MCAsmParser &Parser = getParser();
  // Get the option token.
  AsmToken Tok = Parser.getTok();
  // At the moment only identifiers are supported.
  if (Tok.isNot(AsmToken::Identifier))
    return Error(Parser.getTok().getLoc(),
                 "unexpected token, expected identifier");

  StringRef Option = Tok.getIdentifier();

  // Unknown option.
  Warning(Parser.getTok().getLoc(),
          "unknown option, expected 'rvc' or 'norvc'");
  Parser.eatToEndOfStatement();
  return false;
}

void RXAsmParser::emitToStreamer(MCStreamer &S, const MCInst &Inst) {
  MCInst CInst;
  bool Res = false; // compressInst(CInst, Inst, getSTI(), S.getContext());
  CInst.setLoc(Inst.getLoc());
  S.EmitInstruction((Res ? CInst : Inst), getSTI());
}

void RXAsmParser::emitLoadLocalAddress(MCInst &Inst, SMLoc IDLoc,
                                          MCStreamer &Out) {
  // The local load address pseudo-instruction "lla" is used in PC-relative
  // addressing of symbols:
  //   lla rdest, symbol
  // expands to
  //   TmpLabel: AUIPC rdest, %pcrel_hi(symbol)
  //             ADDI rdest, %pcrel_lo(TmpLabel)
  MCContext &Ctx = getContext();

  MCSymbol *TmpLabel = Ctx.createTempSymbol(
      "pcrel_hi", /* AlwaysAddSuffix */ true, /* CanBeUnnamed */ false);
  Out.EmitLabel(TmpLabel);

  MCOperand DestReg = Inst.getOperand(0);
  const RXMCExpr *Symbol = RXMCExpr::create(
      Inst.getOperand(1).getExpr(), RXMCExpr::VK_RX_PCREL_HI, Ctx);

  emitToStreamer(
      Out, MCInstBuilder(RX::BRA).addOperand(DestReg).addExpr(Symbol));

  const MCExpr *RefToLinkTmpLabel =
      RXMCExpr::create(MCSymbolRefExpr::create(TmpLabel, Ctx),
                          RXMCExpr::VK_RX_PCREL_LO, Ctx);

  emitToStreamer(Out, MCInstBuilder(RX::ADD)
                          .addOperand(DestReg)
                          .addOperand(DestReg)
                          .addExpr(RefToLinkTmpLabel));
}

bool RXAsmParser::processInstruction(MCInst &Inst, SMLoc IDLoc,
                                        MCStreamer &Out) {
  Inst.setLoc(IDLoc);
/*
  if (Inst.getOpcode() == RX::PseudoLI) {
    auto Reg = Inst.getOperand(0).getReg();
    int64_t Imm = Inst.getOperand(1).getImm();
    // On RX the immediate here can either be a signed or an unsigned
    // 32-bit number. Sign extension has to be performed to ensure that Imm
    // represents the expected signed 64-bit number.
    if (!isRX())
      Imm = SignExtend64<32>(Imm);
    return false;
  } else if (Inst.getOpcode() == RX::PseudoLLA) {
    emitLoadLocalAddress(Inst, IDLoc, Out);
    return false;
  }
*/
  emitToStreamer(Out, Inst);
  return false;
}

extern "C" void LLVMInitializeRXAsmParser() {
  RegisterMCAsmParser<RXAsmParser> X(getTheRXTarget());
}
