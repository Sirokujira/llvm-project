//===-- RXInstrFormats.td - RX Instruction Formats ---*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//
//  These instruction format definitions are structured to match the
//  description in the RX User-Level ISA specification as closely as
//  possible. For instance, the specification describes instructions with the
//  MSB (31st bit) on the left and the LSB (0th bit) on the right. This is
//  reflected in the order of parameters to each instruction class.
//
//  One area of divergence is in the description of immediates. The
//  specification describes immediate encoding in terms of bit-slicing
//  operations on the logical value represented. The immediate argument to
//  these instruction formats instead represents the bit sequence that will be
//  inserted into the instruction. e.g. although JAL's immediate is logically
//  a 21-bit value (where the LSB is always zero), we describe it as an imm20
//  to match how it is encoded.
//
//===----------------------------------------------------------------------===//

// http://msyksphinz.hatenablog.com/entry/2019/01/23/040000
// 命令タイプ : RX の場合、命令コードでバイトが変わる(可変長)。
// X86 等を参考に記述する
// http://llvm.org/docs/WritingAnLLVMBackend.html
// https://github.com/Broadcom/stblinux-2.6.37/tree/master/uclinux-rootfs/user/gdb/include/opcode/rx.h
// の RX_Operand_Type 参照?
// Format specifies the encoding used by the instruction. This is used by
// RXMCCodeEmitter to determine which form of fixup to use. These
// definitions must be kept in-sync with MCTargetDesc/RXBaseInfo.h.
class InstFormat<bits<5> val> {
  bits<5> Value = val;
}
def InstFormatPseudo        : InstFormat<0>;
def InstFormatRawFrm        : InstFormat<1>;
def InstFormatImmediate     : InstFormat<2>; /* #addend */ // addend?(imm系?)(li が存在?)
def InstFormatRegister      : InstFormat<3>; /* Rn */ // Rs, Rd only?
def InstFormatIndirect      : InstFormat<4>; /* [Rn + addend] */ // Rs, Rd + #Imm?, ADC, ADD?
def InstFormatPostinc       : InstFormat<5>; /* [Rn+] */ // MOVU.size (4)
def InstFormatPredec        : InstFormat<6>; /* [-Rn] */ // MOVU.size (4)
def InstFormatCondition     : InstFormat<7>; /* eq, gtu, etc */ // BCnd/BMCnd or SCCnd
def InstFormatFlag          : InstFormat<8>; /* [UIOSZC] */ // CLRPSW, SETPSW
def InstFormatTwoReg        : InstFormat<9>; /* [Rn + scale*R2] */ // 加算、減算??? Rd = R1 + R2?
def InstFormatOther         : InstFormat<10>;


// https://github.com/Broadcom/stblinux-2.6.37/tree/master/uclinux-rootfs/user/gdb/include/opcode/rx.h
// の RX_Size 参照

// cb[3:0] に影響するフラグ
class FlagType<bits<4> val> {
  bits<4> Value = val;
}
def FlagC     : FlagType<0>;
def FlagZ     : FlagType<1>;
def FlagS     : FlagType<2>;
def FlagO     : FlagType<3>;
def FlagI     : FlagType<8>;
def FlagU     : FlagType<9>;


// 2.2.1 IMM 範囲 をチェック
// ImmType - This specifies the immediate type used by an instruction. This is
// part of the ad-hoc solution used to emit machine instruction encodings by our
// machine code emitter.
// li[1:0] に影響するフラグ
class ImmType<bits<2> val> {
  bits<2> Value = val;
}
def ImmNoImm      : ImmType<0>;
def ImmSIMM8      : ImmType<1>;
def ImmSIMM16     : ImmType<2>;
def ImmSIMM24     : ImmType<3>;
def ImmIMM32      : ImmType<0>;
// 
// def IMM1       : ImmType<1>;
// def IMM3       : ImmType<2>;
// def IMM4       : ImmType<3>;
// def UIMM4      : ImmType<4>;
// def IMM5       : ImmType<31>; // BSET, BTST 0-31
// def IMM8       : ImmType<6>;
// def UIMM8      : ImmType<7>;
// def IMM16      : ImmType<0>;

// ld[1:0] に影響するフラグ
class DspType<bits<2> val> {
  bits<2> Value = val;
}
def DspNo      : DspType<0>;
def Dsp8       : DspType<1>;
def Dsp16      : DspType<2>;
def DspRs      : DspType<3>;
def DspRd      : DspType<3>;

// mi[1:0] に影響するフラグ
class MemexType<bits<2> val> {
  bits<2> Value = val;
}
def MemexB     : MemexType<0>;
def MemexW     : MemexType<1>;
def MemexL     : MemexType<2>;
def MemexUW    : MemexType<3>;

// sz[1:0] に影響するフラグ
class SizeType<bits<2> val> {
  bits<2> Value = val;
}
def SizeB     : SizeType<0>;
def SizeW     : SizeType<1>;
def SizeL     : SizeType<2>;

// Condition bitpatterns, as registers. 
// cr[1:0] に影響するフラグ
class RXCondition<bits<4> val> {
  bits<4> Value = val;
}
def RXCond_EQ     : RXCondition<0>;
def RXCond_Z      : RXCondition<0>;
def RXCond_NE     : RXCondition<1>;
def RXCond_NZ     : RXCondition<1>;
def RXCond_GEU    : RXCondition<2>;
def RXCond_C      : RXCondition<2>;
def RXCond_LTU    : RXCondition<3>;
def RXCond_NC     : RXCondition<3>;
def RXCond_GTU    : RXCondition<4>;
def RXCond_LEU    : RXCondition<5>;
def RXCond_PZ     : RXCondition<6>;
def RXCond_N      : RXCondition<7>;
def RXCond_GE     : RXCondition<8>;
def RXCond_LT     : RXCondition<9>;
def RXCond_GT     : RXCondition<10>;
def RXCond_LE     : RXCondition<11>;
def RXCond_O      : RXCondition<12>;
def RXCond_NO     : RXCondition<13>;
def RXCond_ALWAYS : RXCondition<14>;
def RXCond_NEVER  : RXCondition<15>;

// target Register? : MVTC
// def RXC_receive : RXControl<1>;
// def RXC_receive : RXControl<4-7>;
// def RXC_receive : RXControl<13-15>;
class RXControl<bits<4> val> {
  bits<4> Value = val;
}
def RXC_PSW     : RXControl<0>;
def RXC_PC      : RXControl<1>;
def RXC_USP     : RXControl<2>;
def RXC_FPSW    : RXControl<3>;
def RXC_BPSW    : RXControl<8>;
def RXC_BPC     : RXControl<9>;
def RXC_ISP     : RXControl<10>;
def RXC_FINTV   : RXControl<11>;
def RXC_INTB    : RXControl<12>;


// 命令コード のうち、固定値で対応している命令を定義
class RXOpcode<bits<8> val> {
  bits<8> Value = val;
}
def OPC_BRK      : RXOpcode<0b00000000>;
def OPC_NOP      : RXOpcode<0b00000011>;
def OPC_RTS      : RXOpcode<0b00000010>;

class RXOpcode2<bits<16> val> {
  bits<16> Value = val;
}
def OPC_RTE       : RXOpcode2<0b0111111110010101>;
def OPC_RTFI      : RXOpcode2<0b0111111110010100>;
def OPC_SATR      : RXOpcode2<0b0111111110010011>;
def OPC_SCMPU     : RXOpcode2<0b0111111110000011>;
def OPC_SMOVB     : RXOpcode2<0b0111111110001011>;
def OPC_SMOVF     : RXOpcode2<0b0111111110001111>;
def OPC_SMOVU     : RXOpcode2<0b0111111110000111>;
def OPC_WAIT      : RXOpcode2<0b0000001100000011>;


// http://msyksphinz.hatenablog.com/entry/2019/01/23/040000
// P167 からの命令コードの対応が入る?(命令コードが可変[CISC]のため注意。X86 の定義を見る?)
// https://www.renesas.com/jp/ja/doc/products/mpumcu/doc/rx_family/r01us0032jj0120_rxsm.pdf
// Base 命令?
// Generic RX Format
class RXInst<InstFormat format, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : Instruction {
  // Inst and Size: for tablegen(... -gen-emitter) and
  // tablegen(... -gen-disassembler) in CMakeLists.txt
  InstFormat Form = format;
  let Namespace = "RX";
  let DecoderNamespace = "RX";

  // SoftFail is a field the disassembler can use to provide a way for
  // instructions to not match without killing the whole decode process. It is
  // mainly used for ARM, but Tablegen expects this field to exist or it fails
  // to build the decode table.
  field bits<32> SoftFail = 0;

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = opcodestr # "\t" # argstr;
  let Pattern = pattern;
}

// Pseudo instructions
class Pseudo<dag outs, dag ins, list<dag> pattern, string opcodestr = "", string argstr = "">
    : RXInst<InstFormatPseudo, outs, ins, opcodestr, argstr, pattern> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

// 固定命令長コード(1byte)
// BRK, NOP, RTS
class I<RXOpcode opcode, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<8> Inst;
  let Size = 1;

  let Inst = opcode.Value;
}

// 固定命令長コード(2byte)
// RTE, RTFI, SATR, SCMPU, SMOVF, SMOVU, WAIT
class I2<RXOpcode2 opcode, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;

  let Inst = opcode.Value;
}

class I14<bits<14> o, SizeType sz, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;

  let Inst{15-2} = o;
  let Inst{1-0} = sz.Value;
}

// Fix opcode + imm?
// INT(Imm8), ...
class IImm<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<8> imm;
  let Size = 2;

  let Inst{15-8} = o;
  let Inst{7-0} = imm;
}
class I2Imm<bits<16> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<8> imm;
  let Size = 3;

  let Inst{23-8} = o;
  let Inst{7-0} = imm;
}


// InstFormatImmediate
// ADD(1)
class RXInstImm8<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatImmediate, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<4> imm;
  bits<4> rd;
  let Size = 2;

  let Inst{15-8} = o;
  let Inst{7-4} = imm;
  let Inst{3-0} = rd;
}

class RXInst8Imm8<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatImmediate, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<8> imm;

  let Inst{15-8} = o;
  let Inst{7-0} = imm;
}

// INT?(0b0111010101100000)
class RXInstImm16<bits<16> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatImmediate, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<8> imm;
  let Size = 3;

  let Inst{23-8} = o;
  let Inst{7-0} = imm;
}

// InstFormatRegister
// (Register Dest only?)
class RXInstRegisterDest12<bits<12> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;
  bits<4> rd;

  let Inst{15-4} = o;
  let Inst{3-0} = rd;
}

class RXInstRegisterDest15<bits<15> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<5> imm;
  bits<4> rd;

  let Inst{23-9} = o;
  let Inst{8-4} = imm;
  let Inst{3-0} = rd;
}

class RXInstRegisterSrc12<bits<12> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;
  bits<4> rs;

  let Inst{15-4} = o;
  let Inst{3-0} = rs;
}

class RXInstRegisterSrc16<bits<16> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rs;
  bits<4> rs2;

  let Inst{23-8} = o;
  let Inst{7-4} = rs;
  let Inst{3-0} = rs;
}

class RXInstRegisterSrc20<bits<20> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rs;

  let Inst{23-4} = o;
  let Inst{3-0} = rs;
}


// (Register Dest/Src)
class RXInstRegisterDestSrc<bits<16> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rs;
  bits<4> rd;

  let Inst{23-8} = o;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}

class RXInstRegisterDestSrcSrc12<bits<12> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rd;
  bits<4> rs;
  bits<4> rs2;

  let Inst{23-12} = o;
  let Inst{11-8} = rd;
  let Inst{7-4} = rs;
  let Inst{3-0} = rs2;
}


// InstFormatIndirect
// 汎用レジスタ?
class RXInstIndirect12_6<bits<12> of, ImmType li, bits<6> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst; // 4-7(src 1-4)
  bits<4> rd;

  let Inst{23-12} = of;
  let Inst{11-10} = li.Value;
  let Inst{9-4} = oe;
  // let Inst{3-0} = rd;
}

class RXInstIndirect6_4<bits<6> of, ImmType li, bits<4> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst; // 3-6(src 1-4)
  bits<4> rd;

  let Inst{15-10} = of;
  let Inst{9-8} = li.Value;
  let Inst{7-4} = oe;
  let Inst{3-0} = rd;
}


class RXInstIndirectSrc6_1<bits<6> of, DspType ld, bits<1> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<4> rs;
  bits<3> imm;

  let Inst{15-10} = of;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3} = oe;
  let Inst{2-0} = imm;
}

// oe = 0b1
class RXInstIndirectDest6_1<bits<6> of, DspType ld, bits<1> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<4> rs;
  bits<3> imm;

  let Inst{15-10} = of;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3} = oe;
  let Inst{2-0} = imm;
}

class InstFormatIndirect7<bits<7> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<5> imm;
  bits<4> rd;

  let Inst{15-9} = o;
  let Inst{8-4} = imm;
  let Inst{3-0} = rd;
}

class InstFormatIndirect8<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<4> imm;
  bits<4> rs2;

  let Inst{15-8} = o;
  let Inst{7-4} = imm;
  let Inst{3-0} = rs2;
}

class InstFormatIndirect12<bits<12> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<4> rs2;
  bits<8> imm;

  let Inst{23-12} = o;
  let Inst{11-8} = rs2;
  let Inst{7-0} = imm;
}

// BNOT
class InstFormatIndirectDest11_4<bits<11> of, DspType ld, bits<4> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<3> imm;
  bits<4> rd;

  let Inst{23-13} = of;
  let Inst{12-10} = imm;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rd;
  let Inst{3-0} = oe;
}

class RXInstIndirectDestSrc14_8<bits<14> of, DspType ld, bits<8> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst; // 4-7(src 0-2)
  bits<4> rs;
  bits<4> rd;

  let Inst{31-18} = of;
  let Inst{17-16} = ld.Value;
  let Inst{15-8} = oe;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}

// Dest*2 + uimm8
class RXInstIndirectDestDest8<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rd;
  bits<4> rd2;
  bits<8> imm;

  let Inst{23-16} = o;
  let Inst{15-12} = rd;
  let Inst{11-8} = rd2;
  let Inst{7-0} = imm;
}

// SBB
// BSET
class RXInstIndirectSrcDest14<bits<14> o, DspType ld, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rs;
  bits<4> rd;

  let Inst{23-10} = o;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}

// BSET
class RXInstIndirectDestSrc11_4<bits<11> of, bits<4> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatRegister, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<5> imm;
  bits<4> rd;

  let Inst{23-13} = of;
  let Inst{12-8} = imm;
  let Inst{7-4} = oe;
  let Inst{3-0} = rd;
}


// 制御レジスタ
class RXInstIndirectCR12_6<bits<12> of, ImmType li, bits<6> oe, RXControl cr, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;

  let Inst{23-12} = of;
  let Inst{11-10} = li.Value;
  let Inst{9-4} = oe;
  let Inst{3-0} = cr.Value;
}

class RXInstIndirectSrcCR16<bits<16> o, RXControl cr, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rd;

  let Inst{23-8} = o;
  let Inst{7-4} = cr.Value;
  let Inst{3-0} = rd;
}

class RXInstIndirectCR16<bits<16> o, RXControl cr, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatIndirect, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rd;

  let Inst{23-8} = o;
  let Inst{7-4} = cr.Value;
  let Inst{3-0} = rd;
}

// InstFormatPostinc
class RXInstPostinc<bits<1> sz, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatPostinc, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<2> ad; // 0b10 +, 0b11 -
  // bits<1> sz;
  bits<4> rd;
  bits<4> rs;
  let ad = 0b10;

  let Inst{23-12} = 0b111111010011;
  let Inst{11-10} = ad;
  let Inst{9} = 0b0;
  let Inst{8} = sz;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}
// InstFormatPredec
class RXInstPredec<bits<1> sz, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatPredec, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<2> ad; // 0b10 +, 0b11 -
  // bits<1> sz;
  bits<4> rd;
  bits<4> rs;
  let ad = 0b11;

  let Inst{23-12} = 0b111111010011;
  let Inst{11-10} = ad;
  let Inst{9} = 0b0;
  let Inst{8} = sz;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}

// 
// InstFormatCondition
// 

// BCnd?
class RXInstBCnd<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
    field bits<8> Inst;
    bits<1> cd;
    let cd = 0b0; // cond.Value & 0x1;
    bits<3> pcdsp;

    let Inst{7-4} = 0b0001;
    let Inst{3} = cd;
    let Inst{2-0} = pcdsp;
}

class RXInstBCnd2<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
    field bits<16> Inst;
    bits<4> cd;
    let cd = cond.Value;
    bits<8> pcdsp;

    let Inst{15-12} = 0b0001;
    let Inst{11-8} = cd;
    let Inst{7-0} = pcdsp;
}

class RXInstBCnd3<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
    field bits<24> Inst;
    bits<1> cd;
    // let cd = cond.Value;
    bits<16> pcdsp;

    let Inst{23-17} = 0b0011101;
    let Inst{16} = cd;
    let Inst{15-0} = pcdsp;
}

// BMCnd
// src : IMM3
class RXInstBMCndImm3_1<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<3> imm;
  bits<2> ld;
  bits<4> rd;
  bits<4> cd;
  let Size = 3; // 3-5
  let cd = cond.Value;
  let ld = DspNo.Value;

  let Inst{23-13} = 0b11111100111;
  let Inst{12-10} = imm;
  let Inst{9-8} = ld;
  let Inst{7-4} = rd;
  let Inst{3-0} = cd;
}
class RXInstBMCndImm3_2<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst;
  bits<3> imm;
  bits<2> ld;
  bits<4> rd;
  bits<4> cd;
  bits<8> dsp;
  let Size = 4; // 3-5
  let cd = cond.Value;
  let ld = Dsp8.Value;

  let Inst{31-21} = 0b11111100111;
  let Inst{20-18} = imm;
  let Inst{17-16} = ld;
  let Inst{15-12} = rd;
  let Inst{11-8} = cd;
  let Inst{7-0} = dsp;
}
class RXInstBMCndImm3_3<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
  field bits<40> Inst;
  bits<3> imm;
  bits<2> ld;
  bits<4> rd;
  bits<4> cd;
  bits<16> dsp;
  let Size = 5;
  let cd = cond.Value;
  let ld = Dsp16.Value;

  let Inst{39-29} = 0b11111100111;
  let Inst{28-26} = imm;
  let Inst{25-24} = ld;
  let Inst{23-20} = rd;
  let Inst{19-16} = cd;
  let Inst{15-0} = dsp;
}

// src : IMM5
class RXInstBMCndImm5<RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<5> imm;
  bits<4> cd;
  bits<4> rd;
  let Size = 3;
  let cd = cond.Value;

  let Inst{23-13} = 0b11111101111;
  let Inst{12-8} = imm;
  let Inst{7-4} = cd;
  let Inst{3-0} = rd;
}

// SCCnd
class RXInstSCCnd<SizeType size, DspType dspld, RXCondition cond, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatCondition, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  bits<2> sz;
  bits<2> ld;
  bits<4> rd;
  bits<4> cd;
  let Size = 3; // 3-5
  let sz = size.Value;
  let ld = dspld.Value;
  let cd = cond.Value;

  let Inst{23-12} = 0b111111001101;
  let Inst{11-10} = sz;
  let Inst{9-8} = ld;
  let Inst{7-4} = rd;
  let Inst{3-0} = cd;
}

///
// InstFormatFlag
///

// CLRPSW : 0b011111111011
// SETPSW : 0b011111111010
class RXInstPSW<bits<12> o, FlagType cb, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatFlag, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;

  let Inst{15-4} = o;
  let Inst{3-0} = cb.Value;
}

// 
// InstFormatTwoReg
// 


//
// InstFormatOther
// 

// BRA?
// BRA.S
class RXInstDsp3<bits<5> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<8> Inst;
  let Size = 1;
  bits<3> pcdsp;

  let Inst{7-3} = 0b00001;
  let Inst{2-0} = pcdsp;
}

// BRA.B
// 0b00101110
class RXInstDsp8<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;
  bits<8> pcdsp;

  let Inst{15-8} = o;
  let Inst{7-0} = pcdsp;
}

// BSR?
// BRA.W
// 0b00111000
// BSR.W
// 0b00111001
class RXInstDsp16<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<16> pcdsp;

  let Inst{23-16} = o;
  let Inst{15-0} = pcdsp;
}

// BRA.A
// 0b00000100
// BSR.A
// 0b00000101
class RXInstDsp24<bits<8> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst;
  let Size = 4;
  bits<24> pcdsp;

  let Inst{31-24} = o;
  let Inst{23-0} = pcdsp;
}

// 0b011111110101
class RXInstBSRL<bits<12> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  bits<4> rs;
  let Size = 2;

  let Inst{15-4} = o;
  let Inst{3-0} = rs;
}

// MOVU(2)
// 0b01011
class RXInstMOVU5<bits<5> o, bits<1> sz, DspType ld, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 3;
  bits<4> rs;
  bits<4> rd;

  let Inst{15-11} = o;
  let Inst{10} = sz;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}

// MOVU(3)
// 0b11111110110
class RXInstMOVU11<bits<11> o, bits<1> sz, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> ri;
  bits<4> rb;
  bits<4> rd;

  let Inst{23-13} = o;
  let Inst{12} = sz;
  let Inst{11-8} = ri;
  let Inst{7-4} = rb;
  let Inst{3-0} = rd;
}

// ADD(2) - memex == UB or Src==Rs
class RXInstIndirectDestSrc6_DspNo<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2; // 2-4
  bits<4> rs;
  bits<4> rd;
  DspType ld;
  let ld = DspNo;

  let Inst{15-10} = o;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}
class RXInstIndirectDestSrc6_Dsp8<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3; // 2-4
  bits<4> rs;
  bits<4> rd;
  bits<8> dsp;
  DspType ld;
  let ld = Dsp8;

  let Inst{23-18} = o;
  let Inst{17-16} = ld.Value;
  let Inst{15-12} = rs;
  let Inst{11-8} = rd;
  let Inst{7-0} = dsp;
}
class RXInstIndirectDestSrc6_Dsp16<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst;
  let Size = 4; // 2-4
  bits<4> rs;
  bits<4> rd;
  bits<16> dsp;
  DspType ld;
  let ld = Dsp16;

  let Inst{31-26} = o;
  let Inst{25-24} = ld.Value;
  let Inst{23-20} = rs;
  let Inst{19-16} = rd;
  let Inst{15-0} = dsp;
}


// ADD(2) - memex != UB
class RXInstIndirectDestSrc8_4_DspNo<bits<8> of, MemexType mi, bits<4> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3; // 3-5
  bits<4> rs;
  bits<4> rd;
  DspType ld;
  let ld = DspNo;

  let Inst{23-16} = of;
  let Inst{15-14} = mi.Value;
  let Inst{13-10} = oe;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}
class RXInstIndirectDestSrc8_4_Dsp8<bits<8> of, MemexType mi, bits<4> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst;
  let Size = 4; // 3-5
  bits<4> rs;
  bits<4> rd;
  bits<8> dsp;
  DspType ld;
  let ld = Dsp8;

  let Inst{31-24} = of;
  let Inst{23-22} = mi.Value;
  let Inst{21-18} = oe;
  let Inst{17-16} = ld.Value;
  let Inst{15-12} = rs;
  let Inst{11-8} = rd;
  let Inst{7-0} = dsp;
}
class RXInstIndirectDestSrc8_4_Dsp16<bits<8> of, MemexType mi, bits<4> oe, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<40> Inst;
  let Size = 5; // 3-5
  bits<4> rs;
  bits<4> rd;
  bits<16> dsp;
  DspType ld;
  let ld = Dsp16;

  let Inst{39-32} = of;
  let Inst{31-30} = mi.Value;
  let Inst{29-26} = oe;
  let Inst{25-24} = ld.Value;
  let Inst{23-20} = rs;
  let Inst{19-16} = rd;
  let Inst{15-0} = dsp;
}

// ADD(3)
class RXInstRegisterDestSrc6_simm8<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3;
  bits<4> rs2;
  bits<4> rd;
  bits<8> imm;
  ImmType li;
  let li = ImmSIMM8;

  let Inst{23-18} = o;
  let Inst{17-16} = li.Value;
  let Inst{15-12} = rs2;
  let Inst{11-8} = rd;
  let Inst{7-0} = imm;
}
class RXInstRegisterDestSrc6_simm16<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst;
  let Size = 4;
  bits<4> rs2;
  bits<4> rd;
  bits<16> imm;
  ImmType li;
  let li = ImmSIMM16;

  let Inst{31-26} = o;
  let Inst{25-24} = li.Value;
  let Inst{23-20} = rs2;
  let Inst{19-16} = rd;
  let Inst{15-0} = imm;
}
class RXInstRegisterDestSrc6_simm24<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<40> Inst;
  let Size = 5;
  bits<4> rs2;
  bits<4> rd;
  bits<24> imm;
  ImmType li;
  let li = ImmSIMM24;

  let Inst{39-34} = o;
  let Inst{33-32} = li.Value;
  let Inst{31-28} = rs2;
  let Inst{27-24} = rd;
  let Inst{23-0} = imm;
}
class RXInstRegisterDestSrc6_imm32<bits<6> o, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<48> Inst;
  let Size = 3;
  bits<4> rs2;
  bits<4> rd;
  bits<32> imm;
  ImmType li;
  let li = ImmIMM32;

  let Inst{47-42} = o;
  let Inst{41-40} = li.Value;
  let Inst{39-36} = rs2;
  let Inst{35-32} = rd;
  let Inst{31-0} = imm;
}

// XOR(2) - memex == UB or Src==Rs
// 0b11111100001101 : bits<14> o
class RXInstIndirectDestSrc14_XOR_DspNo<dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<24> Inst;
  let Size = 3; // 3-5
  bits<4> rs;
  bits<4> rd;
  DspType ld;
  let ld = DspNo;

  let Inst{23-10} = 0b11111100001101;
  let Inst{9-8} = ld.Value;
  let Inst{7-4} = rs;
  let Inst{3-0} = rd;
}
class RXInstIndirectDestSrc14_XOR_Dsp8<dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<32> Inst;
  let Size = 4; // 3-5
  bits<4> rs;
  bits<4> rd;
  bits<8> dsp;
  DspType ld;
  let ld = Dsp8;

  let Inst{31-18} = 0b11111100001101;
  let Inst{17-16} = ld.Value;
  let Inst{15-12} = rs;
  let Inst{11-8} = rd;
  let Inst{7-0} = dsp;
}
class RXInstIndirectDestSrc14_XOR_Dsp16<dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<40> Inst;
  let Size = 5; // 3-5
  bits<4> rs;
  bits<4> rd;
  bits<16> dsp;
  DspType ld;
  let ld = Dsp16;

  let Inst{39-26} = 0b11111100001101;
  let Inst{25-24} = ld.Value;
  let Inst{23-20} = rs;
  let Inst{19-16} = rd;
  let Inst{15-0} = dsp;
}
// XOR(2) - memex != UB
class RXInstIndirectDestSrc_XOR_DspNo<MemexType mi, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
}
class RXInstIndirectDestSrc_XOR_Dsp8<MemexType mi, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
}
class RXInstIndirectDestSrc_XOR_Dsp16<MemexType mi, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
    : RXInst<InstFormatOther, outs, ins, opcodestr, argstr, pattern> {
  field bits<48> Inst;
  let Size = 6; // 4-6
  bits<4> rs;
  bits<4> rd;
  bits<16> dsp;
  DspType ld;
  let ld = Dsp16;

  let Inst{39-26} = 0b11111100001101;
  let Inst{25-24} = ld.Value;
  let Inst{23-20} = rs;
  let Inst{19-16} = rd;
  let Inst{15-0} = dsp;
}