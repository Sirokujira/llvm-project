//===-- RXInstrInfo.td - Target Description for RX ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the RX instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Instruction format superclass
//===----------------------------------------------------------------------===//

include "RXInstrFormats.td"

//===----------------------------------------------------------------------===//
// RX specific DAG Nodes.
//===----------------------------------------------------------------------===//

//  These are target-independent nodes, but have target-specific formats.
def SDT_RXCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>,
                                       SDTCisVT<1, i32>]>;
def SDT_RXCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>,
                                       SDTCisVT<1, i32>]>;
def SDT_RXCall         : SDTypeProfile<0, -1, [SDTCisVT<0, XLenVT>]>;
def SDT_RXSelectCC     : SDTypeProfile<1, 5, [SDTCisSameAs<1, 2>,
                                                 SDTCisSameAs<0, 4>,
                                                 SDTCisSameAs<4, 5>]>;


def Call         : SDNode<"RXISD::CALL", SDT_RXCall,
                          [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                           SDNPVariadic]>;
def CallSeqStart : SDNode<"ISD::CALLSEQ_START", SDT_RXCallSeqStart,
                          [SDNPHasChain, SDNPOutGlue]>;
def CallSeqEnd   : SDNode<"ISD::CALLSEQ_END", SDT_RXCallSeqEnd,
                          [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;
def RetFlag      : SDNode<"RXISD::RET_FLAG", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def URetFlag     : SDNode<"RXISD::URET_FLAG", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue]>;
def SRetFlag     : SDNode<"RXISD::SRET_FLAG", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue]>;
def MRetFlag     : SDNode<"RXISD::MRET_FLAG", SDTNone,
                          [SDNPHasChain, SDNPOptInGlue]>;
def SelectCC     : SDNode<"RXISD::SELECT_CC", SDT_RXSelectCC,
                          [SDNPInGlue]>;
def Tail         : SDNode<"RXISD::TAIL", SDT_RXCall,
                          [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue,
                           SDNPVariadic]>;

//===----------------------------------------------------------------------===//
// Operand and SDNode transformation definitions.
//===----------------------------------------------------------------------===//

class ImmXLenAsmOperand<string prefix, string suffix = ""> : AsmOperandClass {
  let Name = prefix # "ImmXLen" # suffix;
  let RenderMethod = "addImmOperands";
  let DiagnosticType = !strconcat("Invalid", Name);
}

class ImmAsmOperand<string prefix, int width, string suffix> : AsmOperandClass {
  let Name = prefix # "Imm" # width # suffix;
  let RenderMethod = "addImmOperands";
  let DiagnosticType = !strconcat("Invalid", Name);
}

class SImmAsmOperand<int width, string suffix = "">
    : ImmAsmOperand<"S", width, suffix> {
}

class UImmAsmOperand<int width, string suffix = "">
    : ImmAsmOperand<"U", width, suffix> {
}

def FenceArg : AsmOperandClass {
  let Name = "FenceArg";
  let RenderMethod = "addFenceArgOperands";
  let DiagnosticType = "InvalidFenceArg";
}

def fencearg : Operand<XLenVT> {
  let ParserMatchClass = FenceArg;
  let PrintMethod = "printFenceArg";
  let DecoderMethod = "decodeUImmOperand<4>";
}

// IMM
def imm1 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<1>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<1>;
  let DecoderMethod = "decodeSImmOperand<1>";
}

def imm3 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<3>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<3>;
  let DecoderMethod = "decodeSImmOperand<3>";
}

def imm4 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<4>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<4>;
  let DecoderMethod = "decodeSImmOperand<4>";
}

def uimm4 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<4>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<4>;
  let DecoderMethod = "decodeUImmOperand<4>";
}

def imm5 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<5>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<5>;
  let DecoderMethod = "decodeSImmOperand<5>";
}

def imm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<8>;
  let DecoderMethod = "decodeSImmOperand<8>";
}

def uimm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<8>;
  let DecoderMethod = "decodeUImmOperand<8>";
}

def simm8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<8>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<8>;
  let DecoderMethod = "decodeSImmOperand<8>";
}

def imm16 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<16>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<16>;
  let DecoderMethod = "decodeSImmOperand<16>";
}

def simm16 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<16>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<16>;
  let DecoderMethod = "decodeSImmOperand<16>";
}

def simm24 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<24>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<24>;
  let DecoderMethod = "decodeSImmOperand<24>";
}

def imm32 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isInt<32>(Imm);}]> {
  let ParserMatchClass = SImmAsmOperand<32>;
  let DecoderMethod = "decodeSImmOperand<32>";
}

def dsp3 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<3>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<3>;
  let DecoderMethod = "decodeUImmOperand<3>";
}

// mov only?
// def dsp5 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
//   let ParserMatchClass = UImmAsmOperand<8>;
//   let DecoderMethod = "decodeUImmOperand<8>";
// }
// 4:1(3)
// def dsp5hi4 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
// def dsp5lo1 : SDNodeXForm<imm, [{
//   return CurDAG->getTargetConstant(SignExtend64<1>(N->getZExtValue()),
//                                    SDLoc(N), N->getValueType(0));
// }]>;
// 1:4(3)
// def dsp5hi1 : ImmLeaf<XLenVT, [{return isShiftedInt<4, 1>(Imm);}]>;
// def dsp5lo4 : SDNodeXForm<imm, [{
//   return CurDAG->getTargetConstant(SignExtend64<4>(N->getZExtValue()),
//                                    SDLoc(N), N->getValueType(0));
// }]>;

def pcdsp3 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<3>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<3>;
  let DecoderMethod = "decodeUImmOperand<3>";
}
def pcdsp8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<8>;
  let DecoderMethod = "decodeUImmOperand<8>";
}
def pcdsp16 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<16>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<16>;
  let DecoderMethod = "decodeUImmOperand<16>";
}
def pcdsp24 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<24>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<24>;
  let DecoderMethod = "decodeUImmOperand<24>";
}

def dsp8 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<8>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<8>;
  let DecoderMethod = "decodeUImmOperand<8>";
}
def dsp16 : Operand<XLenVT>, ImmLeaf<XLenVT, [{return isUInt<16>(Imm);}]> {
  let ParserMatchClass = UImmAsmOperand<16>;
  let DecoderMethod = "decodeUImmOperand<16>";
}

def BareSymbol : AsmOperandClass {
  let Name = "BareSymbol";
  let RenderMethod = "addImmOperands";
  let DiagnosticType = "InvalidBareSymbol";
}

// A bare symbol.
def bare_symbol : Operand<XLenVT> {
  let ParserMatchClass = BareSymbol;
  let MCOperandPredicate = [{
     return MCOp.isBareSymbolRef();
  }];
}

// A parameterized register class alternative to i32imm/i64imm from Target.td.
def ixlenimm : Operand<XLenVT>;
def ixlenimm_li : Operand<XLenVT> {
  let ParserMatchClass = ImmXLenAsmOperand<"", "LI">;
}

// Addressing modes.
// Necessary because a frameindex can't be matched directly in a pattern.
def AddrFI : ComplexPattern<iPTR, 1, "SelectAddrFI", [frameindex], []>;

// Extract least significant 12 bits from an immediate value and sign extend
// them.
def LO12Sext : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(SignExtend64<12>(N->getZExtValue()),
                                   SDLoc(N), N->getValueType(0));
}]>;

// Extract the most significant 20 bits from an immediate value. Add 1 if bit
// 11 is 1, to compensate for the low 12 bits in the matching immediate addi
// or ld/st being negative.
def HI20 : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(((N->getZExtValue()+0x800) >> 12) & 0xfffff,
                                   SDLoc(N), N->getValueType(0));
}]>;

// PseudoTAIL is a pseudo instruction similar to PseudoCALL and will eventually
// expand to auipc and jalr while encoding.
// Define AsmString to print "tail" when compile with -S flag.
// let isCall = 1, isTerminator = 1, isReturn = 1, isBarrier = 1, Uses = [R2],
//     isCodeGenOnly = 0 in
// def PseudoTAIL : Pseudo<(outs), (ins bare_symbol:$dst), []> {
//   let AsmString = "tail\t$dst";
// }

//===----------------------------------------------------------------------===//
// Stack allocation
//===----------------------------------------------------------------------===//
let Defs = [R0], Uses = [R0] in {
def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                              [(CallSeqStart timm:$amt1, timm:$amt2)]>;
def ADJCALLSTACKUP   : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                              [(CallSeqEnd timm:$amt1, timm:$amt2)]>;
} // Defs = [R0], Uses = [R0]

//===----------------------------------------------------------------------===//
// Instruction Class Templates
//===----------------------------------------------------------------------===//
// class RawFrm_rri<bits<8> op, string opcodestr>
//       : I<op, (outs), (ins), opcodestr, "", []>;

class ALU_ri<bits<8> op, string opcodestr>
      : IImm<op, (outs GPR:$rd), (ins imm8:$imm8), opcodestr, "$rd, $imm8", []>;

class ALU_rri<bits<16> op, string opcodestr>
      : I2Imm<op, (outs GPR:$rd), (ins imm8:$imm8), opcodestr, "$rd, $imm8", []>;


//===----------------------------------------------------------------------===//
// Instructions
//===----------------------------------------------------------------------===//

let mayLoad = 1, mayStore = 1, hasSideEffects = 0 in {

def BRK : I<OPC_BRK, (outs), (ins), "brk", "", []>;
def NOP : I<OPC_NOP, (outs), (ins), "nop", "", []>;
def RTS : I<OPC_RTS, (outs), (ins), "rts", "", []>;

def RTE   : I2<OPC_RTE, (outs), (ins), "rte", "", []>;
def RTFI  : I2<OPC_RTFI, (outs), (ins), "rtfi", "", []>;
def SATR  : I2<OPC_SATR, (outs), (ins), "satr", "", []>;
def SCMPU : I2<OPC_SCMPU, (outs), (ins), "scmpu", "", []>;
def SMOVB : I2<OPC_SMOVB, (outs), (ins), "mov.b", "", []>;
def SMOVF : I2<OPC_SMOVF, (outs), (ins), "mov.f", "", []>;
def SMOVU : I2<OPC_SMOVU, (outs), (ins), "mov.u", "", []>;
def WAIT  : I2<OPC_WAIT, (outs), (ins), "wait", "", []>;

// IImm
// ABS
// (1)
def ABSD    : RXInstRegisterDest12<0b011111100010, (outs GPR:$rd), (ins), "abs", "$rd", []>;
// (2)
def ABSSD   : RXInstRegisterDestSrc<0b1111110000001111, (outs GPR:$rd), (ins GPR:$rs), "abs", "$rd, $rs", []>;

// // ADC
// // (1)
// def ADC1a   : RXInstIndirect12_6<0b111111010111, ImmSIMM8,  0b000010, (outs GPR:$rd), (ins simm8:$imm),  "adc", "$rd $imm", []>;
// def ADC1b   : RXInstIndirect12_6<0b111111010111, ImmSIMM16, 0b000010, (outs GPR:$rd), (ins simm16:$imm), "adc", "$rd $imm", []>;
// def ADC1c   : RXInstIndirect12_6<0b111111010111, ImmSIMM24, 0b000010, (outs GPR:$rd), (ins simm24:$imm), "adc", "$rd $imm", []>;
// def ADC1d   : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b000010, (outs GPR:$rd), (ins imm32:$imm),  "adc", "$rd $imm", []>;
// // (2)
// // def ADC2    : RXInstIndirect<0b1111110000001011, 0b11, (outs GPR:$rd), (ins GPR:$rs), "adc", "$rd $rs", []>;
// // (3)
// // def ADC3    : RXInstIndirect<0b1111110000001011, (outs GPR:$rd), (ins GPR:$rs), "adc", "$rd $rs", []>;
// 
// // ADD
// // (1)
def ADD    : RXInstImm8<0b01100010, (outs GPR:$rd), (ins uimm4:$imm), "add", "$rd $imm", []>;
// // (2)
// def ADD2a1  : RXInstIndirectDestSrc6_DspNo<0b010010, (outs GPR:$rd), (ins GPR:$rs), "add", "$rd $rs", []>;
// def ADD2a2  : RXInstIndirectDestSrc6_Dsp8<0b010010,  (outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2a3  : RXInstIndirectDestSrc6_Dsp16<0b010010, (outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2b1  : RXInstIndirectDestSrc8_4_DspNo<0b00000110, MemexB,  0b0010, (outs GPR:$rd), (ins GPR:$rs), "add", "$rd $rs", []>;
// def ADD2b2  : RXInstIndirectDestSrc8_4_DspNo<0b00000110, MemexW,  0b0010, (outs GPR:$rd), (ins GPR:$rs), "add", "$rd $rs", []>;
// def ADD2b3  : RXInstIndirectDestSrc8_4_DspNo<0b00000110, MemexL,  0b0010, (outs GPR:$rd), (ins GPR:$rs), "add", "$rd $rs", []>;
// def ADD2b4  : RXInstIndirectDestSrc8_4_DspNo<0b00000110, MemexUW, 0b0010, (outs GPR:$rd), (ins GPR:$rs), "add", "$rd $rs", []>;
// def ADD2c1  : RXInstIndirectDestSrc8_4_Dsp8<0b00000110,  MemexB,  0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2c2  : RXInstIndirectDestSrc8_4_Dsp8<0b00000110,  MemexW,  0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2c3  : RXInstIndirectDestSrc8_4_Dsp8<0b00000110,  MemexL,  0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2c4  : RXInstIndirectDestSrc8_4_Dsp8<0b00000110,  MemexUW, 0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2d1  : RXInstIndirectDestSrc8_4_Dsp16<0b00000110, MemexB,  0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2d2  : RXInstIndirectDestSrc8_4_Dsp16<0b00000110, MemexW,  0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2d3  : RXInstIndirectDestSrc8_4_Dsp16<0b00000110, MemexL,  0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp), "add", "$rd $rs $dsp", []>;
// def ADD2d4  : RXInstIndirectDestSrc8_4_Dsp16<0b00000110, MemexUW, 0b0010, (outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp), "add", "$rd $rs $dsp", []>;
// // (3)
// def ADD3a   : RXInstRegisterDestSrc6_simm8<0b011100,  (outs GPR:$rd), (ins GPR:$rs2, simm8:$imm),  "add", "$rd $rs2 $imm", []>;
// def ADD3b   : RXInstRegisterDestSrc6_simm16<0b011100, (outs GPR:$rd), (ins GPR:$rs2, simm16:$imm), "add", "$rd $rs2 $imm", []>;
// def ADD3c   : RXInstRegisterDestSrc6_simm24<0b011100, (outs GPR:$rd), (ins GPR:$rs2, simm24:$imm), "add", "$rd $rs2 $imm", []>;
// def ADD3d   : RXInstRegisterDestSrc6_imm32<0b011100,  (outs GPR:$rd), (ins GPR:$rs2, imm32:$imm),  "add", "$rd $rs2 $imm", []>;
// // (4)
// // def ADD4 : RXInstRegisterDestSrcSrc12<0b111111110010, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "add", "$rd $rs $rs2", []>;
// 
// // AND
// // (1)
// def AND1   : RXInstImm8<0b01100100, (outs GPR:$rd), (ins uimm4:$imm), "and", "$rd $imm", []>;
// // (2)
// def AND2a    : RXInstIndirect6_4<0b011101, ImmSIMM8,  0b0010, (outs GPR:$rd), (ins simm8:$imm),  "and", "$rd $imm", []>;
// def AND2b    : RXInstIndirect6_4<0b011101, ImmSIMM16, 0b0010, (outs GPR:$rd), (ins simm16:$imm), "and", "$rd $imm", []>;
// def AND2c    : RXInstIndirect6_4<0b011101, ImmSIMM24, 0b0010, (outs GPR:$rd), (ins simm24:$imm), "and", "$rd $imm", []>;
// def AND2d    : RXInstIndirect6_4<0b011101, ImmIMM32,  0b0010, (outs GPR:$rd), (ins imm32:$imm),  "and", "$rd $imm", []>;
// // (3)
// // UB
// // not UB
// // (4)
// 
// // BCLR
// // (1)
// def BCLR1a : RXInstIndirectDest6_1<0b111100, DspNo, 0b1, (outs GPR:$rd), (ins imm3:$imm), "bclr", "$rd, $imm", []>;
// def BCLR1b : RXInstIndirectDest6_1<0b111100, Dsp8,  0b1, (outs GPR:$rd), (ins imm3:$imm), "bclr", "$rd, $imm", []>;
// def BCLR1c : RXInstIndirectDest6_1<0b111100, Dsp16, 0b1, (outs GPR:$rd), (ins imm3:$imm), "bclr", "$rd, $imm", []>;
// // (2)
// def BCLR2a : RXInstIndirectSrcDest14<0b11111100011001, DspNo, (outs GPR:$rd), (ins GPR:$rs), "bclr", "$rd, $rs", []>;
// def BCLR2b : RXInstIndirectSrcDest14<0b11111100011001, Dsp8,  (outs GPR:$rd), (ins GPR:$rs), "bclr", "$rd, $rs", []>;
// def BCLR2c : RXInstIndirectSrcDest14<0b11111100011001, Dsp16, (outs GPR:$rd), (ins GPR:$rs), "bclr", "$rd, $rs", []>;
// // (3)
// def BCLR3 : InstFormatIndirect7<0b0111101, (outs GPR:$rd), (ins imm5:$imm), "bclr", "$rd, $imm", []>;
// // (4)
// def BCLR4 : RXInstIndirectSrcDest14<0b11111100011001, DspRd, (outs GPR:$rd), (ins GPR:$rs), "bclr", "$rd, $rs", []>;

// BCnd
// (1)
// def BEQ : RXInstBCnd<RXCond_EQ, (outs), (ins pcdsp3:$pcdsp), "beq.s", "", []>;
// RXCond_EQ == RXCond_Z
// def BZ  : RXInstBCnd<RXCond_Z, (outs), (ins pcdsp3:$pcdsp), "bz.s", "", []>;
// def BNE : RXInstBCnd<RXCond_NE, (outs), (ins pcdsp3:$pcdsp), "bne.s", "", []>;
// RXCond_NE == RXCond_NZ
// def BNZ : RXInstBCnd<RXCond_NZ, (outs), (ins pcdsp3:$pcdsp), "bnz.s", "", []>;
// // (2)
def BEQ : RXInstBCnd2<RXCond_EQ, (outs), (ins pcdsp8:$pcdsp), "beq.b", "", []>;
// RXCond_EQ == RXCond_Z
// def BZ  : RXInstBCnd2<RXCond_Z, (outs), (ins pcdsp8:$pcdsp), "bz.b", "", []>;

def BNE : RXInstBCnd2<RXCond_NE, (outs), (ins pcdsp8:$dsp), "bne.b", "", []>;
// RXCond_NE == RXCond_NZ
// def BNZ : RXInstBCnd<RXCond_NZ, (outs), (ins pcdsp8:$pcdsp), "bnz.b", "", []>;

// // def BC2   : RXInstBCnd2<RXCond_C, (outs), (ins pcdsp8:$pcdsp), "bc.b", "", []>;
// // def BNC2  : RXInstBCnd2<RXCond_NC, (outs), (ins pcdsp8:$pcdsp), "bnc.b", "", []>;
// // def BGTU2 : RXInstBCnd2<RXCond_GTU, (outs), (ins pcdsp8:$pcdsp), "bgtu.b", "", []>;
// // def BLEU2 : RXInstBCnd2<RXCond_LEU, (outs), (ins pcdsp8:$pcdsp), "bleu.b", "", []>;
// // def BPZ2  : RXInstBCnd2<RXCond_PZ, (outs), (ins pcdsp8:$pcdsp), "bpz.b", "", []>;
// def BN2   : RXInstBCnd2<RXCond_N, (outs), (ins pcdsp8:$pcdsp), "bn.b", "", []>;
// def BGT2  : RXInstBCnd2<RXCond_GT, (outs), (ins pcdsp8:$pcdsp), "bgt.b", "", []>;
// def BLE  : RXInstBCnd2<RXCond_LE, (outs), (ins pcdsp8:$pcdsp), "ble.b", "", []>;
def BO   : RXInstBCnd2<RXCond_O, (outs), (ins pcdsp8:$pcdsp), "bo.b", "", []>;
def BPZ  : RXInstBCnd2<RXCond_NO, (outs), (ins pcdsp8:$pcdsp), "bpz.b", "", []>;
def BGE  : RXInstBCnd2<RXCond_GE, (outs), (ins pcdsp8:$pcdsp), "bge.b", "", []>;
def BGEU : RXInstBCnd2<RXCond_GEU, (outs), (ins pcdsp8:$pcdsp), "bgeu.b", "", []>;
def BLT  : RXInstBCnd2<RXCond_LT, (outs), (ins pcdsp8:$pcdsp), "blt.b", "", []>;
def BLTU : RXInstBCnd2<RXCond_LTU, (outs), (ins pcdsp8:$pcdsp), "bltu.b", "", []>;
// // (3)
// // def BEQW3 : RXInstBCnd3<RXCond_EQ, (outs), (ins pcdsp16:$pcdsp), "beq.w", "", []>;
// // def BNEW3 : RXInstBCnd3<RXCond_NE, (outs), (ins pcdsp16:$pcdsp), "bne.w", "", []>;

// BMCnd
// // (1)
// def BMEQ   : RXInstBMCndImm3_1<RXCond_EQ,     (outs GPR:$rd), (ins imm3:$imm), "bmeq.w",  "", []>;
// // RXCond_EQ == RXCond_Z
// // def BMZ    : RXInstBMCndImm3_1<RXCond_Z,      (outs GPR:$rd), (ins imm3:$imm), "bmz.w",   "", []>;
// 
// def BMNE   : RXInstBMCndImm3_1<RXCond_NE,     (outs GPR:$rd), (ins imm3:$imm), "bmne.w",  "", []>;
// // RXCond_NE == RXCond_NZ
// // def BMNZ   : RXInstBMCndImm3_1<RXCond_NZ,     (outs GPR:$rd), (ins imm3:$imm), "bmnz.w",  "", []>;
// 
// def BMGEU  : RXInstBMCndImm3_1<RXCond_GEU,    (outs GPR:$rd), (ins imm3:$imm), "bmgeu.w", "", []>;
// // RXCond_GEU == RXCond_C
// // def BMC    : RXInstBMCndImm3_1<RXCond_C,      (outs GPR:$rd), (ins imm3:$imm), "bmc.w",   "", []>;
// 
// def BMLTU  : RXInstBMCndImm3_1<RXCond_LTU,    (outs GPR:$rd), (ins imm3:$imm), "bmltu.w", "", []>;
// // RXCond_LTU == RXCond_NC
// // def BMNC   : RXInstBMCndImm3_1<RXCond_NC,     (outs GPR:$rd), (ins imm3:$imm), "bmnc.w",  "", []>;
// 
// def BMGTU  : RXInstBMCndImm3_1<RXCond_GTU,    (outs GPR:$rd), (ins imm3:$imm), "bmgtu.w", "", []>;
// def BMLEU  : RXInstBMCndImm3_1<RXCond_LEU,    (outs GPR:$rd), (ins imm3:$imm), "bmleu.w",  "", []>;
// def BMPZ   : RXInstBMCndImm3_1<RXCond_PZ,     (outs GPR:$rd), (ins imm3:$imm), "bmpz.w",  "", []>;
// def BMN    : RXInstBMCndImm3_1<RXCond_N,      (outs GPR:$rd), (ins imm3:$imm), "bmn.w",   "", []>;
// def BMGE   : RXInstBMCndImm3_1<RXCond_GE,     (outs GPR:$rd), (ins imm3:$imm), "bmge.w",  "", []>;
// def BMLT   : RXInstBMCndImm3_1<RXCond_LT,     (outs GPR:$rd), (ins imm3:$imm), "bmlt.w",  "", []>;
// def BMGT   : RXInstBMCndImm3_1<RXCond_GT,     (outs GPR:$rd), (ins imm3:$imm), "bmgt.w",  "", []>;
// def BMLE   : RXInstBMCndImm3_1<RXCond_LE,     (outs GPR:$rd), (ins imm3:$imm), "bmle.w",  "", []>;
// def BMO    : RXInstBMCndImm3_1<RXCond_O,      (outs GPR:$rd), (ins imm3:$imm), "bmo.w",   "", []>;
// def BMNO   : RXInstBMCndImm3_1<RXCond_NO,     (outs GPR:$rd), (ins imm3:$imm), "bmno.w",  "", []>;
// // def BMEQ   : RXInstBMCndImm3_2<RXCond_EQ,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmeq.w",  "", []>;
// // def BMZ    : RXInstBMCndImm3_2<RXCond_Z,      (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmz.w",   "", []>;
// // def BMNE   : RXInstBMCndImm3_2<RXCond_NE,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmne.w",  "", []>;
// // def BMNZ   : RXInstBMCndImm3_2<RXCond_NZ,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmnz.w",  "", []>;
// // def BMGEU  : RXInstBMCndImm3_2<RXCond_GEU,    (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmgeu.w", "", []>;
// // def BMC    : RXInstBMCndImm3_2<RXCond_C,      (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmc.w",   "", []>;
// // def BMLTU  : RXInstBMCndImm3_2<RXCond_LTU,    (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmltu.w", "", []>;
// // def BMNC   : RXInstBMCndImm3_2<RXCond_NC,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmnc.w",  "", []>;
// // def BMGTU  : RXInstBMCndImm3_2<RXCond_GTU,    (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmgtu.w", "", []>;
// // def BMLEU  : RXInstBMCndImm3_2<RXCond_LEU,    (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmleu.w", "", []>;
// // def BMPZ   : RXInstBMCndImm3_2<RXCond_PZ,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmpz.w",  "", []>;
// // def BMN    : RXInstBMCndImm3_2<RXCond_N,      (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmn.w",   "", []>;
// // def BMGE   : RXInstBMCndImm3_2<RXCond_GE,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmge.w",  "", []>;
// // def BMLT   : RXInstBMCndImm3_2<RXCond_LT,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmlt.w",  "", []>;
// // def BMGT   : RXInstBMCndImm3_2<RXCond_GT,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmgt.w",  "", []>;
// // def BMLE   : RXInstBMCndImm3_2<RXCond_LE,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmle.w",  "", []>;
// // def BMO    : RXInstBMCndImm3_2<RXCond_O,      (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmo.w",   "", []>;
// // def BMNO   : RXInstBMCndImm3_2<RXCond_NO,     (outs GPR:$rd), (ins imm3:$imm, dsp8:$dsp), "bmno.w",  "", []>;
// // 
// // def BMEQ   : RXInstBMCndImm3_3<RXCond_EQ,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmeq.w",  "", []>;
// // def BMZ    : RXInstBMCndImm3_3<RXCond_Z,      (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmz.w",   "", []>;
// // def BMNE   : RXInstBMCndImm3_3<RXCond_NE,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmne.w",  "", []>;
// // def BMNZ   : RXInstBMCndImm3_3<RXCond_NZ,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmnz.w",  "", []>;
// // def BMGEU  : RXInstBMCndImm3_3<RXCond_GEU,    (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmgeu.w", "", []>;
// // def BMC    : RXInstBMCndImm3_3<RXCond_C,      (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmc.w",   "", []>;
// // def BMLTU  : RXInstBMCndImm3_3<RXCond_LTU,    (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmltu.w", "", []>;
// // def BMNC   : RXInstBMCndImm3_3<RXCond_NC,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmnc.w",  "", []>;
// // def BMGTU  : RXInstBMCndImm3_3<RXCond_GTU,    (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmgtu.w", "", []>;
// // def BMLEU  : RXInstBMCndImm3_3<RXCond_LEU,    (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmleu.w", "", []>;
// // def BMPZ   : RXInstBMCndImm3_3<RXCond_PZ,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmpz.w",  "", []>;
// // def BMN    : RXInstBMCndImm3_3<RXCond_N,      (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmn.w",   "", []>;
// // def BMGE   : RXInstBMCndImm3_3<RXCond_GE,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmge.w",  "", []>;
// // def BMLT   : RXInstBMCndImm3_3<RXCond_LT,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmlt.w",  "", []>;
// // def BMGT   : RXInstBMCndImm3_3<RXCond_GT,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmgt.w",  "", []>;
// // def BMLE   : RXInstBMCndImm3_3<RXCond_LE,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmle.w",  "", []>;
// // def BMO    : RXInstBMCndImm3_3<RXCond_O,      (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmo.w",   "", []>;
// // def BMNO   : RXInstBMCndImm3_3<RXCond_NO,     (outs GPR:$rd), (ins imm3:$imm, dsp16:$dsp), "bmno.w",  "", []>;
// // 
// // // (2)
// // // RXCond_EQ == RXCond_Z
// // def BMEQ2  : RXInstBMCndImm5<RXCond_EQ,     (outs GPR:$rd), (ins imm3:$imm), "bmeq.w",  "", []>;
// // // def BMZ2   : RXInstBMCndImm5<RXCond_Z,      (outs GPR:$rd), (ins imm3:$imm), "bmz.w",   "", []>;
// // def BMNE2  : RXInstBMCndImm5<RXCond_NE,     (outs GPR:$rd), (ins imm3:$imm), "bmne.w",  "", []>;
// // // RXCond_NE == RXCond_NZ
// // // def BMNZ2  : RXInstBMCndImm5<RXCond_NZ,     (outs GPR:$rd), (ins imm3:$imm), "bmnz.w",  "", []>;
// // def BMGEU2 : RXInstBMCndImm5<RXCond_GEU,    (outs GPR:$rd), (ins imm3:$imm), "bmgeu.w", "", []>;
// // // RXCond_GEU == RXCond_C
// // // def BMC2   : RXInstBMCndImm5<RXCond_C,      (outs GPR:$rd), (ins imm3:$imm), "bmc.w",   "", []>;
// // def BMLTU2 : RXInstBMCndImm5<RXCond_LTU,    (outs GPR:$rd), (ins imm3:$imm), "bmltu.w", "", []>;
// // // RXCond_LTU == RXCond_NC
// // // def BMNC2  : RXInstBMCndImm5<RXCond_NC,     (outs GPR:$rd), (ins imm3:$imm), "bmnc.w",  "", []>;
// // def BMGTU2 : RXInstBMCndImm5<RXCond_GTU,    (outs GPR:$rd), (ins imm3:$imm), "bmgtu.w", "", []>;
// // def BMLEU2 : RXInstBMCndImm5<RXCond_LEU,    (outs GPR:$rd), (ins imm3:$imm), "bmleu.w", "", []>;
// // def BMPZ2  : RXInstBMCndImm5<RXCond_PZ,     (outs GPR:$rd), (ins imm3:$imm), "bmpz.w",  "", []>;
// // def BMN2   : RXInstBMCndImm5<RXCond_N,      (outs GPR:$rd), (ins imm3:$imm), "bmn.w",   "", []>;
// // def BMGE2  : RXInstBMCndImm5<RXCond_GE,     (outs GPR:$rd), (ins imm3:$imm), "bmge.w",  "", []>;
// // def BMLT2  : RXInstBMCndImm5<RXCond_LT,     (outs GPR:$rd), (ins imm3:$imm), "bmlt.w",  "", []>;
// // def BMGT2  : RXInstBMCndImm5<RXCond_GT,     (outs GPR:$rd), (ins imm3:$imm), "bmgt.w",  "", []>;
// // def BMLE2  : RXInstBMCndImm5<RXCond_LE,     (outs GPR:$rd), (ins imm3:$imm), "bmle.w",  "", []>;
// // def BMO2   : RXInstBMCndImm5<RXCond_O,      (outs GPR:$rd), (ins imm3:$imm), "bmo.w",   "", []>;
// // def BMNO2  : RXInstBMCndImm5<RXCond_NO,     (outs GPR:$rd), (ins imm3:$imm), "bmno.w",  "", []>;

// BNOT
// (1) : BNOT src, dest
def BNOT1a : InstFormatIndirectDest11_4<0b11111100111, DspNo, 0b1111, (outs GPR:$rd), (ins imm3:$imm), "bnot", "$rd, $imm", []>;
def BNOT1b : InstFormatIndirectDest11_4<0b11111100111, Dsp8,  0b1111, (outs GPR:$rd), (ins imm3:$imm), "bnot", "$rd, $imm", []>;
def BNOT1c : InstFormatIndirectDest11_4<0b11111100111, Dsp16, 0b1111, (outs GPR:$rd), (ins imm3:$imm), "bnot", "$rd, $imm", []>;
// (2) : BNOT src, dest
def BNOT2a : RXInstIndirectSrcDest14<0b11111100011011, DspNo, (outs GPR:$rd), (ins GPR:$rs), "bnot", "$rd, $rs", []>;
def BNOT2b : RXInstIndirectSrcDest14<0b11111100011011, Dsp8,  (outs GPR:$rd), (ins GPR:$rs), "bnot", "$rd, $rs", []>;
def BNOT2c : RXInstIndirectSrcDest14<0b11111100011011, Dsp16, (outs GPR:$rd), (ins GPR:$rs), "bnot", "$rd, $rs", []>;
// (3) : BNOT src, dest
// def BNOT3  : RXInstIndirectDestSrc11_4<0b11111101111, 0b1111, (outs GPR:$rd), (ins imm5:$imm), "bnot",  "$rd, $imm", []>;
// (4) : BNOT src, dest
def BNOT4  : RXInstIndirectSrcDest14<0b11111100011011, DspRd, (outs GPR:$rd), (ins GPR:$rs), "bnot", "$rd, $rs", []>;

// BRA
// (1) BRA.S src
def BRA : RXInstDsp3<0b00001, (outs), (ins pcdsp3:$pcdsp), "bra.s", "$pcdsp", []>;
// (2) BRA.B src
def BRA2 : RXInstDsp8<0b00101110, (outs), (ins pcdsp8:$pcdsp), "bra.b", "$pcdsp", []>;
// (3) BRA.W src
def BRA3 : RXInstDsp16<0b00111000, (outs), (ins pcdsp16:$pcdsp), "bra.w", "$pcdsp", []>;
// (4) BRA.A src
def BRA4 : RXInstDsp24<0b00000101, (outs), (ins pcdsp24:$pcdsp), "bra.a", "$pcdsp", []>;
// (5) BRA.L src
def BSR5 : RXInstRegisterSrc12<0b011111110100, (outs), (ins GPR:$rs), "bra.l", "$rs", []>;

// // BSET
// // (1)
// def BSET1a : RXInstIndirectDest6_1<0b111100, DspNo, 0b0, (outs GPR:$rd), (ins imm3:$imm), "bset", "$rd, $imm", []>;
// def BSET1b : RXInstIndirectDest6_1<0b111100, Dsp8,  0b0, (outs GPR:$rd), (ins imm3:$imm), "bset", "$rd, $imm", []>;
// def BSET1c : RXInstIndirectDest6_1<0b111100, Dsp16, 0b0, (outs GPR:$rd), (ins imm3:$imm), "bset", "$rd, $imm", []>;
// // (2)
// def BSET2a : RXInstIndirectSrcDest14<0b11111100011000, DspNo, (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// def BSET2b : RXInstIndirectSrcDest14<0b11111100011000, Dsp8,  (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// def BSET2c : RXInstIndirectSrcDest14<0b11111100011000, Dsp16, (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// // (3)
// def BSET3 : InstFormatIndirect7<0b0111100, (outs GPR:$rd), (ins imm5:$imm), "bset", "$rd, $imm", []>;
// // (4)
// def BSET4 : RXInstIndirectSrcDest14<0b11111100011000, DspRs, (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// 
// // BSR
// // (1)
// def BSR1 : RXInstDsp16<0b00111001, (outs), (ins pcdsp16:$pcdsp), "bsr.w", "$pcdsp", []>;
// // (2)
// def BSR2 : RXInstDsp24<0b00000101, (outs), (ins pcdsp24:$pcdsp), "bsr.a", "$pcdsp", []>;
// // (3)
// def BSR3 : RXInstRegisterSrc12<0b011111110101, (outs), (ins GPR:$rs), "bsr.l", "$rs", []>;
// 
// // BTST
// // (1)
// def BTST1a : RXInstIndirectSrc6_1<0b111101, DspNo, 0b1, (outs), (ins GPR:$rs, imm3:$imm), "bset", "$rs", []>;
// def BTST1b : RXInstIndirectSrc6_1<0b111101, Dsp8,  0b1, (outs), (ins GPR:$rs, imm3:$imm), "bset", "$rs", []>;
// def BTST1c : RXInstIndirectSrc6_1<0b111101, Dsp16, 0b1, (outs), (ins GPR:$rs, imm3:$imm), "bset", "$rs", []>;
// // (2)
// def BTST2a : RXInstIndirectSrcDest14<0b11111100011000, DspNo, (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// def BTST2b : RXInstIndirectSrcDest14<0b11111100011000, Dsp8,  (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// def BTST2c : RXInstIndirectSrcDest14<0b11111100011000, Dsp16, (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// // (3)
// def BTST3a : InstFormatIndirect7<0b0111110, (outs), (ins imm5:$imm, GPR:$rd), "bset", "", []>;
// // (4)
// def BTST4a : RXInstIndirectSrcDest14<0b11111100011000, DspRd, (outs GPR:$rd), (ins GPR:$rs), "bset", "$rd, $rs", []>;
// 
// // CLRPSW
// // (1)
// def CLRPSW : RXInstPSW<0b011111111011, FlagC, (outs), (ins), "clrpsw", "", []>;
// // def CLRPSW : RXInstPSW<0b011111111011, FlagZ, (outs), (ins), "clrpsw", "", []>;
// // def CLRPSW : RXInstPSW<0b011111111011, FlagS, (outs), (ins), "clrpsw", "", []>;
// // def CLRPSW : RXInstPSW<0b011111111011, FlagO, (outs), (ins), "clrpsw", "", []>;
// // def CLRPSW : RXInstPSW<0b011111111011, FlagI, (outs), (ins), "clrpsw", "", []>;
// // def CLRPSW : RXInstPSW<0b011111111011, FlagU, (outs), (ins), "clrpsw", "", []>;
// 
// // CMP
// // (1)
// def CMP1 : InstFormatIndirect8<0b01100001, (outs), (ins imm5:$imm, GPR:$rs2), "CMP", "$rs2 $imm", []>;
// // (2)
// def CMP2 : InstFormatIndirect12<0b011101010101, (outs), (ins GPR:$rs2, uimm8:$imm), "CMP", "$rs2 $imm", []>;
// // (3)
// def CMP3a : RXInstIndirect6_4<0b011101, ImmSIMM8,  0b0000, (outs), (ins GPR:$rd, simm8:$imm),  "CMP", "$rd $imm", []>;
// def CMP3b : RXInstIndirect6_4<0b011101, ImmSIMM16, 0b0000, (outs), (ins GPR:$rd, simm16:$imm), "CMP", "$rd $imm", []>;
// def CMP3c : RXInstIndirect6_4<0b011101, ImmSIMM24, 0b0000, (outs), (ins GPR:$rd, simm24:$imm), "CMP", "$rd $imm", []>;
// def CMP3d : RXInstIndirect6_4<0b011101, ImmIMM32,  0b0000, (outs), (ins GPR:$rd, imm32:$imm),  "CMP", "$rd $imm", []>;
// // (4)
// 
// // DIV
// // (1)
// def DIV1a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b001000, (outs GPR:$rd), (ins simm8:$imm),   "div", "$rd $imm", []>;
// def DIV1b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b001000, (outs GPR:$rd), (ins simm16:$imm),  "div", "$rd $imm", []>;
// def DIV1c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b001000, (outs GPR:$rd), (ins simm24:$imm),  "div", "$rd $imm", []>;
// def DIV1d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001000, (outs GPR:$rd), (ins imm32:$imm),   "div", "$rd $imm", []>;
// // (2)
// // def DIV2a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b001000, (outs GPR:$rd), (ins GPR:$rs),  "div", "$rd $rs", []>;
// // def DIV2a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b001000, (outs GPR:$rd), (ins GPR:$rs),  "div", "$rd $rs", []>;
// // def DIV2a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b001000, (outs GPR:$rd), (ins GPR:$rs),  "div", "$rd $rs", []>;
// // def DIV2a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b001000, (outs GPR:$rd), (ins GPR:$rs),  "div", "$rd $rs", []>;
// 
// // DIVU
// // (1)
// def DIVU1a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b001001, (outs GPR:$rd), (ins simm8:$imm),   "divu", "$rd $imm", []>;
// def DIVU1b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b001001, (outs GPR:$rd), (ins simm16:$imm),  "divu", "$rd $imm", []>;
// def DIVU1c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b001001, (outs GPR:$rd), (ins simm24:$imm),  "divu", "$rd $imm", []>;
// def DIVU1d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001001, (outs GPR:$rd), (ins imm32:$imm),   "divu", "$rd $imm", []>;
// // (2)
// 
// // EMUL
// // (1)
// def EMUL1a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b000110, (outs GPR:$rd), (ins simm8:$imm),   "emul", "$rd $imm", []>;
// def EMUL1b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b000110, (outs GPR:$rd), (ins simm16:$imm),  "emul", "$rd $imm", []>;
// def EMUL1c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b000110, (outs GPR:$rd), (ins simm24:$imm),  "emul", "$rd $imm", []>;
// def EMUL1d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b000110, (outs GPR:$rd), (ins imm32:$imm),   "emul", "$rd $imm", []>;
// // (2)
//  
// // EMULU
// // (1)
// def EMULU1a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b000111, (outs GPR:$rd), (ins simm8:$imm),   "emulu", "$rd $imm", []>;
// def EMULU1b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b000111, (outs GPR:$rd), (ins simm16:$imm),  "emulu", "$rd $imm", []>;
// def EMULU1c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b000111, (outs GPR:$rd), (ins simm24:$imm),  "emulu", "$rd $imm", []>;
// def EMULU1d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b000111, (outs GPR:$rd), (ins imm32:$imm),   "emulu", "$rd $imm", []>;
// // (2)


// INT
def INT : RXInstImm16<0b0111010101100000, (outs), (ins imm8:$imm), "INT", "$imm", []>;

// JMP
def JMP : RXInstRegisterDest12<0b011111110000, (outs), (ins GPR:$rs), "jmp", "$rs", []>;

// JSR
def JSR : RXInstRegisterDest12<0b011111110001, (outs), (ins GPR:$rs), "JSR", "$rs", []>;

// // MACHI
// def MACHI : RXInstRegisterSrc16<0b1111110100000100, (outs), (ins GPR:$rs, GPR:$rs2), "MACHI", "$rs, $rs2", []>;
// 
// // MACLO
// def MACLO : RXInstRegisterSrc16<0b1111110100000101, (outs), (ins GPR:$rs, GPR:$rs2), "MACLO", "$rs, $rs2", []>;
// 
// // MAX
// // (1)
// def MAX1a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b000110, (outs GPR:$rd), (ins simm8:$imm),   "max", "$rd $imm", []>;
// def MAX1b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b000110, (outs GPR:$rd), (ins simm16:$imm),  "max", "$rd $imm", []>;
// def MAX1c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b000110, (outs GPR:$rd), (ins simm24:$imm),  "max", "$rd $imm", []>;
// def MAX1d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b000110, (outs GPR:$rd), (ins imm32:$imm),   "max", "$rd $imm", []>;
// // (2)
// 
// // MIN
// // (1)
// def MIN1a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b000101, (outs GPR:$rd), (ins simm8:$imm),   "min", "$rd $imm", []>;
// def MIN1b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b000101, (outs GPR:$rd), (ins simm16:$imm),  "min", "$rd $imm", []>;
// def MIN1c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b000101, (outs GPR:$rd), (ins simm24:$imm),  "min", "$rd $imm", []>;
// def MIN1d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b000101, (outs GPR:$rd), (ins imm32:$imm),   "min", "$rd $imm", []>;
// // (2)
// 
// // MOV
// // (1) MOV.size src, dest
// 
// // (2) MOV.size src, dest
// // (3)
// // (4)
// // (5)
// // (6)
// // (7)
// // (8)
// // (9)
// // (10)
// // (11)
// // (12)
// // (13)
// // (14)
// // (15)
// 
// // MOVU
// // (1)
// // (2)
// def MOVU2a : RXInstMOVU5<0b01011, 0b0, DspNo, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2b : RXInstMOVU5<0b01011, 0b0, Dsp8,  (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2c : RXInstMOVU5<0b01011, 0b0, Dsp16, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2d : RXInstMOVU5<0b01011, 0b0, DspRs, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2e : RXInstMOVU5<0b01011, 0b1, DspNo, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2f : RXInstMOVU5<0b01011, 0b1, Dsp8,  (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2g : RXInstMOVU5<0b01011, 0b1, Dsp16, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// def MOVU2h : RXInstMOVU5<0b01011, 0b1, DspRs, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd, $rs", []>;
// // (3)
// def MOVU3a : RXInstMOVU11<0b11111110110, 0b0, (outs GPR:$rd), (ins GPR:$ri, GPR:$rb), "movu", "$rd, $ri, $rb", []>;
// def MOVU3b : RXInstMOVU11<0b11111110110, 0b1, (outs GPR:$rd), (ins GPR:$ri, GPR:$rb), "movu", "$rd, $ri, $rb", []>;
// // (4)
// def MOVU4a : RXInstPostinc<0b0, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd $rs", []>;
// def MOVU4b : RXInstPostinc<0b1, (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd $rs", []>;
// def MOVU4c : RXInstPredec<0b0,  (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd $rs", []>;
// def MOVU4d : RXInstPredec<0b1,  (outs GPR:$rd), (ins GPR:$rs), "movu", "$rd $rs", []>;
// 
// // MUL
// // (1)
// // def MUL1a : RXInstImme<0b111111010111, ImmSIMM8,   0b000101, (outs GPR:$rd), (ins imm4:$imm),    "mul", "$rd $imm", []>;
// // (2)
// def MUL2a : RXInstIndirect12_6<0b111111010111, ImmSIMM8,   0b000101, (outs GPR:$rd), (ins simm8:$imm),   "mul", "$rd $imm", []>;
// def MUL2b : RXInstIndirect12_6<0b111111010111, ImmSIMM16,  0b000101, (outs GPR:$rd), (ins simm16:$imm),  "mul", "$rd $imm", []>;
// def MUL2c : RXInstIndirect12_6<0b111111010111, ImmSIMM24,  0b000101, (outs GPR:$rd), (ins simm24:$imm),  "mul", "$rd $imm", []>;
// def MUL2d : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b000101, (outs GPR:$rd), (ins imm32:$imm),   "mul", "$rd $imm", []>;
// // (3)
// 
// // (4)
// 
// // MULHI
// // (1)
// def MULHI : RXInstRegisterSrc16<0b1111110100000000, (outs), (ins GPR:$rs, GPR:$rs2), "mul.hi", "$rs, $rs2", []>;
// 
// // MULLO
// // (1)
// def MULLO : RXInstRegisterSrc16<0b1111110100000001, (outs), (ins GPR:$rs, GPR:$rs2), "mul.lo", "$rs, $rs2", []>;
// 
// // MVFACHI
// // (1)
// def MVFACHI : RXInstRegisterSrc20<0b11111101000111110000, (outs GPR:$rd), (ins), "mvfachi", "$rd", []>;
// 
// // MVFACMI
// // (1)
// def MVFACMI : RXInstRegisterSrc20<0b11111101000111110010, (outs GPR:$rd), (ins), "mvfacmi", "$rd", []>;
// 
// // MVFC
// // (1) MVFC src, dest
// def MVFC1a : RXInstIndirectCR16<0b1111110101101010, RXC_PSW,   (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1b : RXInstIndirectCR16<0b1111110101101010, RXC_PC,    (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1c : RXInstIndirectCR16<0b1111110101101010, RXC_USP,   (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1d : RXInstIndirectCR16<0b1111110101101010, RXC_FPSW,  (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1e : RXInstIndirectCR16<0b1111110101101010, RXC_BPSW,  (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1f : RXInstIndirectCR16<0b1111110101101010, RXC_BPC,   (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1g : RXInstIndirectCR16<0b1111110101101010, RXC_ISP,   (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1h : RXInstIndirectCR16<0b1111110101101010, RXC_FINTV, (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// def MVFC1i : RXInstIndirectCR16<0b1111110101101010, RXC_INTB,  (outs GPR:$rd), (ins), "mvfc", "$rd", []>;
// 
// // MVTACHI
// // (1)
// def MVTACHI : RXInstRegisterSrc20<0b11111101000101110000, (outs), (ins GPR:$rs), "MVTAC.HI", "$rs", []>;
// 
// // MVTACLO
// // (1)
// def MVTACLO : RXInstRegisterSrc20<0b11111101000101110001, (outs), (ins GPR:$rs), "MVTAC.LO", "$rs", []>;
// 
// // MVTC
// // (1)
// def MVTC1a1 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_PSW,   (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a3 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_USP,   (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a4 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_FPSW,  (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a5 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_BPSW,  (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a6 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_BPC,   (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a7 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_ISP,   (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a8 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_FINTV, (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1a9 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM8,  0b110000, RXC_INTB,  (outs), (ins simm8:$imm),  "mvtc", "$imm", []>;
// def MVTC1b1 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_PSW,   (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b3 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_USP,   (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b4 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_FPSW,  (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b5 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_BPSW,  (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b6 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_BPC,   (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b7 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_ISP,   (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b8 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_FINTV, (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1b9 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM16, 0b110000, RXC_INTB,  (outs), (ins simm16:$imm), "mvtc", "$imm", []>;
// def MVTC1c1 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_PSW,   (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c3 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_USP,   (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c4 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_FPSW,  (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c5 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_BPSW,  (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c6 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_BPC,   (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c7 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_ISP,   (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c8 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_FINTV, (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1c9 : RXInstIndirectCR12_6<0b111111010111, ImmSIMM24, 0b110000, RXC_INTB,  (outs), (ins simm24:$imm), "mvtc", "$imm", []>;
// def MVTC1d1 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_PSW,   (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d3 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_USP,   (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d4 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_FPSW,  (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d5 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_BPSW,  (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d6 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_BPC,   (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d7 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_ISP,   (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d8 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_FINTV, (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// def MVTC1d9 : RXInstIndirectCR12_6<0b111111010111, ImmIMM32,  0b110000, RXC_INTB,  (outs), (ins imm32:$imm),  "mvtc", "$imm", []>;
// // (2)
// def MVFC2a1 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_PSW,   (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a3 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_USP,   (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a4 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_FPSW,  (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a5 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_BPSW,  (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a6 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_BPC,   (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a7 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_ISP,   (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a8 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_FINTV, (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// def MVFC2a9 : RXInstIndirectSrcCR16<0b1111110101101000, RXC_INTB,  (outs), (ins GPR:$rs), "mvfc", "$rs", []>;
// 
// // NEG
// // (1)
// def NEG1 : RXInstRegisterDest12<0b011111100001, (outs GPR:$rd), (ins), "neg", "$rd", []>;
// // (2)
// def NEG2 : RXInstRegisterDestSrc<0b1111110000000111, (outs GPR:$rd), (ins GPR:$rs), "neg", "$rd", []>;
// 
// // NOT
// // (1)
// def NOT1 : RXInstRegisterDest12<0b011111100000, (outs GPR:$rd), (ins), "not", "$rd", []>;
// // (2)
// def NOT2 : RXInstRegisterDestSrc<0b1111110000111011, (outs GPR:$rd), (ins GPR:$rs), "not", "$rd", []>;
// 
// // OR
// // (1)
// // (2)
// def OR2a : RXInstIndirect6_4<0b011101, ImmSIMM8,  0b0011, (outs GPR:$rd), (ins simm8:$imm),   "or", "$rd $imm", []>;
// def OR2b : RXInstIndirect6_4<0b011101, ImmSIMM16, 0b0011, (outs GPR:$rd), (ins simm16:$imm),  "or", "$rd $imm", []>;
// def OR2c : RXInstIndirect6_4<0b011101, ImmSIMM24, 0b0011, (outs GPR:$rd), (ins simm24:$imm),  "or", "$rd $imm", []>;
// def OR2d : RXInstIndirect6_4<0b011101, ImmIMM32,  0b0011, (outs GPR:$rd), (ins imm32:$imm),   "or", "$rd $imm", []>;
// // (3)
// // (4)
// 
// // POP
// def POP : RXInstRegisterDest12<0b011111101011, (outs GPR:$rd), (ins), "pop", "$rd", []>;
// 
// // POPC
// // def POPC : RXInstRegisterDest12<0b0011111101110, (outs GPR:$rd), (ins), "pop", "$rd", []>;
// // POPM
// // def POPM : RXInstRegisterDest12<0b01101111, (outs), (ins GPR:$rs, GPR:$rs2), "pop", "$rs, $rs2", []>;
// 
// // PUSH
// // (1)
// // def PUSH1 : RXInstRegisterDest12<0b011111101011, (outs GPR:$rd), (ins), "pop", "$rd", []>;
// // (2)
// // def PUSH2 : RXInstRegisterDest12<0b011111101011, (outs GPR:$rd), (ins), "pop", "$rd", []>;
// 
// // PUSHC
// // def PUSHC : RXInstRegisterDest12<0b011111101011, (outs GPR:$rd), (ins), "pop", "$rd", []>;
// 
// // PUSHM
// // def PUSHM : RXInstRegisterDest12<0b011111101011, (outs GPR:$rd), (ins), "pop", "$rd", []>;
// 
// // RACW
// 
// // RAVL
// def RAVL : RXInstRegisterDestSrc<0b1111110101100111, (outs GPR:$rd), (ins GPR:$rs), "RAV.L", "$rd, $rs", []>;
// 
// // RAVW
// def RAVW : RXInstRegisterDestSrc<0b1111110101100101, (outs GPR:$rd), (ins GPR:$rs), "RAV.W", "$rd, $rs", []>;
// 
// // RMPA
// 
// // ROLC
// def ROLC : RXInstRegisterDest12<0b011111100101, (outs GPR:$rd), (ins), "ROLC", "$rd", []>;
// 
// // RORC
// def RORC : RXInstRegisterDest12<0b011111100100, (outs GPR:$rd), (ins), "RORC", "$rd", []>;
// 
// // ROTL
// // (1)
// def ROTL1 : RXInstRegisterDest15<0b111111010110111, (outs GPR:$rd), (ins imm5:$imm), "ROTL", "$rd, $imm", []>;
// // (2)
// def ROTL2 : RXInstRegisterDestSrc<0b1111110101100110, (outs GPR:$rd), (ins GPR:$rs), "ROTL", "$rd, $rs", []>;
// 
// // ROTR
// // (1)
// def ROTR1 : RXInstRegisterDest15<0b111111010110110, (outs GPR:$rd), (ins imm5:$imm), "ROTR", "$rd, $imm", []>;
// // (2)
// def ROTR2 : RXInstRegisterDestSrc<0b1111110101100100, (outs GPR:$rd), (ins GPR:$rs), "ROTR", "$rd, $rs", []>;

// RTSD
// (1)
def RTSD1 : RXInst8Imm8<0b01100111, (outs), (ins uimm8:$imm), "RTSD", "$imm", []>;
// (2)
def RTSD2 : RXInstIndirectDestDest8<0b00111111, (outs GPR:$rd, GPR:$rd2), (ins), "RTSD", "$rd, $rd2", []>;

// SAT
// (1)
def SAT : RXInstRegisterDest12<0b011111100011, (outs GPR:$rd), (ins), "SAT", "$rd", []>;

// SBB
// (1)
def SBB1 : RXInstIndirectSrcDest14<0b11111100000000, DspRd, (outs GPR:$rd), (ins GPR:$rs), "SBB", "$rd, $rs", []>;
// (2)
def SBB2a : RXInstIndirectDestSrc14_8<0b00000110101000, DspNo, 0b00000000, (outs GPR:$rd), (ins GPR:$rs), "SBB", "$rd, $rs", []>;
def SBB2b : RXInstIndirectDestSrc14_8<0b00000110101000, Dsp8,  0b00000000, (outs GPR:$rd), (ins GPR:$rs), "SBB", "$rd, $rs", []>;
def SBB2c : RXInstIndirectDestSrc14_8<0b00000110101000, Dsp16, 0b00000000, (outs GPR:$rd), (ins GPR:$rs), "SBB", "$rd, $rs", []>;

// SCCnd
// def SCEQ : RXInstSCCnd<SizeB, DspNo, RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.B", "", []>;
// def SCEQ1b2 : RXInstSCCnd<SizeB, Dsp8,  RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.B", "", []>;
// def SCEQ1b3 : RXInstSCCnd<SizeB, Dsp16, RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.B", "", []>;
// def SCEQ1w1 : RXInstSCCnd<SizeW, DspNo, RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.W", "", []>;
// def SCEQ1w2 : RXInstSCCnd<SizeW, Dsp8,  RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.W", "", []>;
// def SCEQ1w3 : RXInstSCCnd<SizeW, Dsp16, RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.W", "", []>;
// def SCEQ1l1 : RXInstSCCnd<SizeL, DspNo, RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.L", "", []>;
// def SCEQ1l2 : RXInstSCCnd<SizeL, Dsp8,  RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.L", "", []>;
// def SCEQ1l3 : RXInstSCCnd<SizeL, Dsp16, RXCond_EQ, (outs GPR:$rd), (ins), "SCEQ.L", "", []>;
// RXCond_EQ == RXCond_Z
// def SCZ  : RXInstSCCnd<SizeB, DspNo, RXCond_Z,  (outs), (ins), "SCZ.B", "", []>;
// :

// def SCNE : RXInstSCCnd<SizeB, DspNo, RXCond_NE, (outs), (ins), "SCNE.B", "", []>;
// :
// RXCond_NE == RXCond_NZ
// def SCNZ : RXInstSCCnd<SizeB, DspNo, RXCond_NZ, (outs), (ins), "SCNZ.B", "", []>;
// :

// def SCGEU : RXInstSCCnd<SizeB, DspNo, RXCond_GEU, (outs), (ins), "SCGEU.B", "", []>;
// :
// def SCC   : RXInstSCCnd<SizeB, DspNo, RXCond_C,   (outs), (ins), "SCNE.B", "", []>;
// :
// def SCLTU : RXInstSCCnd<SizeB, DspNo, RXCond_LTU, (outs), (ins), "SCLTU.v", "", []>;
// :
// def SCNC  : RXInstSCCnd<SizeB, DspNo, RXCond_NC,  (outs), (ins), "SCNE.B", "", []>;
// :
// def SCGTU : RXInstSCCnd<SizeB, DspNo, RXCond_GTU, (outs), (ins), "SCGTU.B", "", []>;
// def SCLEU : RXInstSCCnd<SizeB, DspNo, RXCond_LEU, (outs), (ins), "SCLEU.B", "", []>;
// def SCPZ  : RXInstSCCnd<SizeB, DspNo, RXCond_PZ,  (outs), (ins), "SCPZ.B", "", []>;
// def SCN   : RXInstSCCnd<SizeB, DspNo, RXCond_N,   (outs), (ins), "SCN.B", "", []>;
// def SCGE  : RXInstSCCnd<SizeB, DspNo, RXCond_GE, (outs), (ins), "SCGE.B", "", []>;
// def SCLT  : RXInstSCCnd<SizeB, DspNo, RXCond_LT, (outs), (ins), "SCLT.B", "", []>;
// def SCGT  : RXInstSCCnd<SizeB, DspNo, RXCond_GT,  (outs), (ins), "SCGT.B", "", []>;
// def SCLE  : RXInstSCCnd<SizeB, DspNo, RXCond_LE,  (outs), (ins), "SCLE.B", "", []>;
// def SCO   : RXInstSCCnd<SizeB, DspNo, RXCond_O,   (outs), (ins), "SCO.B", "", []>;
// def SCNO  : RXInstSCCnd<SizeB, DspNo, RXCond_NO,  (outs), (ins), "SCNO.B", "", []>;

// SETPSW
// def SETPSW : RXInstPSW<0b011111111010, FlagC, (outs), (ins), "setpsw", "", []>;
// def SETPSW : RXInstPSW<0b011111111010, FlagZ, (outs), (ins), "setpsw", "", []>;
// def SETPSW : RXInstPSW<0b011111111010, FlagS, (outs), (ins), "setpsw", "", []>;
// def SETPSW : RXInstPSW<0b011111111010, FlagO, (outs), (ins), "setpsw", "", []>;
// def SETPSW : RXInstPSW<0b011111111010, FlagI, (outs), (ins), "setpsw", "", []>;
// def SETPSW : RXInstPSW<0b011111111010, FlagU, (outs), (ins), "setpsw", "", []>;

// SHAR
// (1)
def SHAR : InstFormatIndirect7<0b0110101, (outs GPR:$rd), (ins imm5:$imm), "SHAR", "", []>;
// (2)
// def SHAR : InstFormatIndirect7<0b0110101, (outs GPR:$rd), (ins imm5:$imm), "SHAR", "", []>;
// (3)
// def SHAR : InstFormatIndirect7<0b0110101, (outs GPR:$rd), (ins imm5:$imm), "SHAR", "", []>;

// SHLL
// (1)
def SHLL : InstFormatIndirect7<0b0110110, (outs GPR:$rd), (ins imm5:$imm), "SHLL", "", []>;
// (2)
// def SHLL : InstFormatIndirect7<0b0110110, (outs GPR:$rd), (ins imm5:$imm), "SHLL", "", []>;
// (3)
// def SHLL : InstFormatIndirect7<0b0110110, (outs GPR:$rd), (ins imm5:$imm), "SHLL", "", []>;

// SHLR
// (1)
def SHLR : InstFormatIndirect7<0b0110100, (outs GPR:$rd), (ins imm5:$imm), "SHLL", "", []>;
// (2)
// def SHLR : InstFormatIndirect7<0b0110100, (outs GPR:$rd), (ins imm5:$imm), "SHLL", "", []>;
// (3)
// def SHLR : InstFormatIndirect7<0b0110100, (outs GPR:$rd), (ins imm5:$imm), "SHLL", "", []>;

// SSTR
// (1)

// STNZ
// (1)
def STNZ1a    : RXInstIndirect12_6<0b111111010111, ImmSIMM8,  0b001111, (outs GPR:$rd), (ins simm8:$imm),  "STNZ", "$rd $imm", []>;
def STNZ1b    : RXInstIndirect12_6<0b111111010111, ImmSIMM16, 0b001111, (outs GPR:$rd), (ins simm16:$imm), "STNZ", "$rd $imm", []>;
def STNZ1c    : RXInstIndirect12_6<0b111111010111, ImmSIMM24, 0b001111, (outs GPR:$rd), (ins simm24:$imm), "STNZ", "$rd $imm", []>;
def STNZ1d    : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001111, (outs GPR:$rd), (ins imm32:$imm),  "STNZ", "$rd $imm", []>;

// STZ
// (1)
def STZ1a    : RXInstIndirect12_6<0b111111010111, ImmSIMM8,  0b001110, (outs GPR:$rd), (ins simm8:$imm),  "STZ", "$rd $imm", []>;
def STZ1b    : RXInstIndirect12_6<0b111111010111, ImmSIMM16, 0b001110, (outs GPR:$rd), (ins simm16:$imm), "STZ", "$rd $imm", []>;
def STZ1c    : RXInstIndirect12_6<0b111111010111, ImmSIMM24, 0b001110, (outs GPR:$rd), (ins simm24:$imm), "STZ", "$rd $imm", []>;
def STZ1d    : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001110, (outs GPR:$rd), (ins imm32:$imm),  "STZ", "$rd $imm", []>;


// SUB
// (1) src, dest
def SUB1   : RXInstImm8<0b01100000, (outs GPR:$rd), (ins uimm4:$imm), "sub", "$rd $imm", []>;
// (2) src, dest
// def SUB2a   : RXInstRegisterDestSrcSrc12<0b111111110000, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "sub", "$rd, $rs, $rs2", []>;
// def SUB2b   : RXInstRegisterDestSrcSrc12<0b111111110000, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "sub", "$rd, $rs, $rs2", []>;
// def SUB2c   : RXInstRegisterDestSrcSrc12<0b111111110000, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "sub", "$rd, $rs, $rs2", []>;
// def SUB2d   : RXInstRegisterDestSrcSrc12<0b111111110000, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "sub", "$rd, $rs, $rs2", []>;
// (3) src, src2, dest
def SUB3   : RXInstRegisterDestSrcSrc12<0b111111110000, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "sub", "$rd, $rs, $rs2", []>;

// SUNTIL
// (1)
def SUNTIL1a : I14<0b01111111100000, SizeB, (outs), (ins), "suntil.B", "", []>;
def SUNTIL1b : I14<0b01111111100000, SizeW, (outs), (ins), "suntil.W", "", []>;
def SUNTIL1c : I14<0b01111111100000, SizeL, (outs), (ins), "suntil.L", "", []>;

// SWHILE
// (1)
def SWHILE1a : I14<0b01111111100001, SizeB, (outs), (ins), "suntil.B", "", []>;
def SWHILE1b : I14<0b01111111100001, SizeW, (outs), (ins), "suntil.W", "", []>;
def SWHILE1c : I14<0b01111111100001, SizeL, (outs), (ins), "suntil.L", "", []>;

// TST
// (1)
def TST1a    : RXInstIndirect12_6<0b111111010111, ImmSIMM8,  0b001100, (outs GPR:$rd), (ins GPR:$rs2, simm8:$imm),  "tst", "$rd $imm", []>;
def TST1b    : RXInstIndirect12_6<0b111111010111, ImmSIMM16, 0b001100, (outs GPR:$rd), (ins GPR:$rs2, simm16:$imm), "tst", "$rd $imm", []>;
def TST1c    : RXInstIndirect12_6<0b111111010111, ImmSIMM24, 0b001100, (outs GPR:$rd), (ins GPR:$rs2, simm24:$imm), "tst", "$rd $imm", []>;
def TST1d    : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001100, (outs GPR:$rd), (ins GPR:$rs2, imm32:$imm),  "tst", "$rd $imm", []>;
// (2)
// def TST2a    : RXInstIndirect12_6<0b111111010111, ImmSIMM8,  0b001100, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "tst", "$rd $rs $rs2", []>;
// def TST2b    : RXInstIndirect12_6<0b111111010111, ImmSIMM16, 0b001100, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "tst", "$rd $rs $rs2", []>;
// def TST2c    : RXInstIndirect12_6<0b111111010111, ImmSIMM24, 0b001100, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "tst", "$rd $rs $rs2", []>;
// def TST2d    : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001100, (outs GPR:$rd), (ins GPR:$rs, GPR:$rs2), "tst", "$rd $rs $rs2", []>;

// XCHG
// (1)

// XOR
// (1) src, dest
def XOR1a    : RXInstIndirect12_6<0b111111010111, ImmSIMM8,  0b001101, (outs GPR:$rd), (ins simm8:$imm),  "XOR", "$rd $imm", []>;
def XOR1b    : RXInstIndirect12_6<0b111111010111, ImmSIMM16, 0b001101, (outs GPR:$rd), (ins simm16:$imm), "XOR", "$rd $imm", []>;
def XOR1c    : RXInstIndirect12_6<0b111111010111, ImmSIMM24, 0b001101, (outs GPR:$rd), (ins simm24:$imm), "XOR", "$rd $imm", []>;
def XOR1d    : RXInstIndirect12_6<0b111111010111, ImmIMM32,  0b001101, (outs GPR:$rd), (ins imm32:$imm),  "XOR", "$rd $imm", []>;
// (2) src, dest
// def XOR2a : RXInstIndirectDestSrc14_XOR_DspNo<(outs GPR:$rd), (ins GPR:$rs),  "XOR", "$rd $rs", []>;
// def XOR2b : RXInstIndirectDestSrc14_XOR_Dsp8<(outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp),  "XOR", "$rd $rs $dsp", []>;
// def XOR2c : RXInstIndirectDestSrc14_XOR_Dsp16<(outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp),  "XOR", "$rd $rs $dsp", []>;
// memex != UB
// def XOR2a : RXInstIndirectDestSrc14_XOR_DspNo<(outs GPR:$rd), (ins GPR:$rs),  "XOR", "$rd $rs", []>;
// def XOR2b : RXInstIndirectDestSrc14_XOR_Dsp8<(outs GPR:$rd), (ins GPR:$rs, dsp8:$dsp),  "XOR", "$rd $rs $dsp", []>;
// def XOR2c : RXInstIndirectDestSrc14_XOR_Dsp16<(outs GPR:$rd), (ins GPR:$rs, dsp16:$dsp),  "XOR", "$rd $rs $dsp", []>;
}
