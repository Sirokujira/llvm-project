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
// ���߃^�C�v : RX �̏ꍇ�A���߃R�[�h�Ńo�C�g���ς��(�ϒ�)�B
// X86 �����Q�l�ɋL�q����
// http://llvm.org/docs/WritingAnLLVMBackend.html
// https://github.com/Broadcom/stblinux-2.6.37/tree/master/uclinux-rootfs/user/gdb/include/opcode/rx.h
// �� RX_Operand_Type �Q��?
// Format specifies the encoding used by the instruction. This is used by
// RXMCCodeEmitter to determine which form of fixup to use. These
// definitions must be kept in-sync with MCTargetDesc/RXBaseInfo.h.
class InstFormat<bits<5> val> {
  bits<5> Value = val;
}
def InstFormatPseudo        : InstFormat<0>;
def InstFormatRawFrm        : InstFormat<1>;
def InstFormatImmediate     : InstFormat<2>; /* #addend */ // addend?(imm�n?)(li ������?)
def InstFormatRegister      : InstFormat<3>; /* Rn */ // Rs, Rd only?
def InstFormatIndirect      : InstFormat<4>; /* [Rn + addend] */ // Rs, Rd + #Imm?, ADC, ADD?
def InstFormatPostinc       : InstFormat<5>; /* [Rn+] */ // MOVU.size (4)
def InstFormatPredec        : InstFormat<6>; /* [-Rn] */ // MOVU.size (4)
def InstFormatCondition     : InstFormat<7>; /* eq, gtu, etc */ // BCnd/BMCnd or SCCnd
def InstFormatFlag          : InstFormat<8>; /* [UIOSZC] */ // CLRPSW, SETPSW
def InstFormatTwoReg        : InstFormat<9>; /* [Rn + scale*R2] */ // ���Z�A���Z??? Rd = R1 + R2?
def InstFormatOther         : InstFormat<10>;

// gdb �� RX_Size �\���̎Q��

// cb[3:0] �ɉe������t���O
class FlagType<bits<4> val> {
  bits<4> Value = val;
}
def FlagC     : FlagType<0>;
def FlagZ     : FlagType<1>;
def FlagS     : FlagType<2>;
def FlagO     : FlagType<3>;
def FlagI     : FlagType<8>;
def FlagU     : FlagType<9>;


// 2.2.1 IMM �͈� ���`�F�b�N
// ImmType - This specifies the immediate type used by an instruction. This is
// part of the ad-hoc solution used to emit machine instruction encodings by our
// machine code emitter.
// li[1:0] �ɉe������t���O
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

// ld[1:0] �ɉe������t���O
class DspType<bits<2> val> {
  bits<2> Value = val;
}
def DspNo      : DspType<0>;
def Dsp8       : DspType<1>;
def Dsp16      : DspType<2>;
def DspRs      : DspType<3>;
def DspRd      : DspType<3>;

// mi[1:0] �ɉe������t���O
class MemexType<bits<2> val> {
  bits<2> Value = val;
}
def MemexB     : MemexType<0>;
def MemexW     : MemexType<1>;
def MemexL     : MemexType<2>;
def MemexUW    : MemexType<3>;

// sz[1:0] �ɉe������t���O
class SizeType<bits<2> val> {
  bits<2> Value = val;
}
def SizeB     : SizeType<0>;
def SizeW     : SizeType<1>;
def SizeL     : SizeType<2>;

// Condition bitpatterns, as registers. 
// cr[1:0] �ɉe������t���O
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


// gdb �� rx-decode.c �� RX�t�@�~�����[�U�[�Y�}�j���A�� ����
// ���߃R�[�h �̂����A����8bit�l�őΉ����Ă��閽�߂��`
class RXOpcode<bits<8> val> {
  bits<8> Value = val;
}
def OPC_BRK            : RXOpcode<0b00000000>;
def OPC_DBT            : RXOpcode<0b00000001>;
def OPC_NOP            : RXOpcode<0b00000011>;
def OPC_RTS            : RXOpcode<0b00000010>;
def OPC_BSAA           : RXOpcode<0b00000100>;
def OPC_BSRA           : RXOpcode<0b00000101>;
def OPC_NOTUB          : RXOpcode<0b00000110>;  // not UB�n?(add/and/cmp/div/divu/emul/emulu/itof/max/min/mul/or/sub/tst/xchg/xor) - 400�s-3600�s�܂ł��邽�߉�͒���
def OPC_BSRS           : RXOpcode<0b00000111>;
def OPC_BRAS           : RXOpcode<0b00001000>;  // 0000 0 dsp : ���� 3bit �� dsp �Ƃ��đΉ�(8-f)
def OPC_BCndS          : RXOpcode<0b00011111>;  // 0001 n dsp : ���� 3bit �� dsp �Ƃ��đΉ�(10-1f)
def OPC_BRAB           : RXOpcode<0b00101110>;  // bra.b
def OPC_BCndB          : RXOpcode<0b00101111>;  // 0001 n dsp : ���� 3bit �� dsp �Ƃ��đΉ�(20-2f)
def OPC_BRAW           : RXOpcode<0b00111000>;  // bra.w
def OPC_BSRW           : RXOpcode<0b00111001>;  // bsr.w
def OPC_BCndW_C0       : RXOpcode<0b00111010>;  // 0011 101c
def OPC_BCndW_C1       : RXOpcode<0b00111011>;  // 0011 101c
def OPC_MOV4B          : RXOpcode<0b00111100>;  // 0011 11sz d dst sppp, sz(0b00-0b10)
def OPC_MOV4W          : RXOpcode<0b00111101>;  // 0x3d
def OPC_MOV4L          : RXOpcode<0b00111110>;  // 0x3e
def OPC_RRSD2          : RXOpcode<0b00111111>;  // 0011 1111 rega regb      rtsd    #%1, %2-%0
def OPC_SUB2_UB        : RXOpcode<0b01000000>;  // 0100 00ss rsrc rdst      sub %2%S2, %1
def OPC_SUB2_UB_DSP8   : RXOpcode<0b01000001>;  // dsp8
def OPC_SUB2_UB_DSP16  : RXOpcode<0b01000010>;  // dsp16
def OPC_CMP2_IMM32     : RXOpcode<0b01000100>;  // 0100 01ss rsrc rdst      cmp %2%S2, %1
def OPC_CMP2_SIMM8     : RXOpcode<0b01000101>;
def OPC_CMP2_SIMM16    : RXOpcode<0b01000110>;
def OPC_CMP2_SIMM24    : RXOpcode<0b01000111>;
def OPC_ADD2_UB        : RXOpcode<0b01001000>;  // 0100 10ss rsrc rdst      add %1%S1, %0
def OPC_ADD2_UB_DSP8   : RXOpcode<0b01001001>;  // 
def OPC_ADD2_UB_DSP16  : RXOpcode<0b01001010>;  // 
def OPC_MUL2_IMM32     : RXOpcode<0b01001100>;  // 0100 11ss rsrc rdst      mul %1%S1, %0
def OPC_MUL2_SIMM8     : RXOpcode<0b01001101>;
def OPC_MUL2_SIMM16    : RXOpcode<0b01001110>;
def OPC_MUL2_SIMM24    : RXOpcode<0b01001111>;
def OPC_AND2_IMM32     : RXOpcode<0b01010000>;  // 0101 00ss rsrc rdst      and %1%S1, %0
def OPC_AND2_SIMM8     : RXOpcode<0b01010001>;
def OPC_AND2_SIMM16    : RXOpcode<0b01010010>;
def OPC_AND2_SIMM24    : RXOpcode<0b01010011>;
def OPC_OR3            : RXOpcode<0b01010100>;  // 0101 01ss rsrc rdst      or  %1%S1, %0
def OPC_OR3_DSP8       : RXOpcode<0b01010101>;
def OPC_OR3_DSP16      : RXOpcode<0b01010110>;
def OPC_OR3_MOVU2_0    : RXOpcode<0b01011000>;  // 0101 1 s ss rsrc rdst	movu%s	%1, %0
def OPC_OR3_MOVU2_1    : RXOpcode<0b01011001>;
def OPC_OR3_MOVU2_2    : RXOpcode<0b01011010>;
def OPC_OR3_MOVU2_3    : RXOpcode<0b01011011>;
def OPC_OR3_MOVU2_W_0  : RXOpcode<0b01011100>;  // 0101 1 s ss rsrc rdst	movu%s	%1, %0
def OPC_OR3_MOVU2_W_1  : RXOpcode<0b01011101>;
def OPC_OR3_MOVU2_W_2  : RXOpcode<0b01011110>;
def OPC_OR3_MOVU2_W_3  : RXOpcode<0b01011111>;
def OPC_SUB1           : RXOpcode<0b01100000>;  // 0110 0000 immm rdst	sub	#%2, %0
def OPC_CMP1           : RXOpcode<0b01100001>;  // 0110 0001 immm rdst	cmp	#%2, %1
def OPC_ADD1           : RXOpcode<0b01100010>;  // 0110 0010 immm rdst	add	#%1, %0
def OPC_MUL1           : RXOpcode<0b01100011>;  // 0110 0011 immm rdst	mul	#%1, %0
def OPC_AND1           : RXOpcode<0b01100100>;  // 0110 0100 immm rdst	and	#%1, %0
def OPC_OR1            : RXOpcode<0b01100101>;  // 0110 0101 immm rdst	or	#%1, %0
def OPC_MOV1           : RXOpcode<0b01100110>;  // 0110 0110 immm rdst	mov%s	#%1, %0
def OPC_RTSD1          : RXOpcode<0b01100111>;  // 0110 0111 rtsd	#%1
def OPC_SHLR1          : RXOpcode<0b01101000>;  // 0110 100i mmmm rdst	shlr	#%2, %0
def OPC_SHAR1          : RXOpcode<0b01101010>;  // 0110 101i mmmm rdst	shar	#%2, %0
def OPC_SHLL1          : RXOpcode<0b01101100>;  // 0110 110i mmmm rdst	shll	#%2, %0
def OPC_PUSHM          : RXOpcode<0b01101110>;  // 0110 1110 dsta dstb	pushm	%1-%2
def OPC_POPM           : RXOpcode<0b01101111>;  // 0110 1111 dsta dstb	popm	%1-%2
def OPC_ADD3_IMM32     : RXOpcode<0b01110000>;  // 0111 00im rsrc rdst	add	#%1, %2, %0
def OPC_ADD3_SIMM8     : RXOpcode<0b01110001>;
def OPC_ADD3_SIMM16    : RXOpcode<0b01110010>;
def OPC_ADD3_SIMM24    : RXOpcode<0b01110011>;
def OPC_CMP3_IMM32     : RXOpcode<0b01110100>;  // 0111 01im 0000 rsrc	cmp	#%2, %1%S1
def OPC_CMP3_SIMM8     : RXOpcode<0b01110101>;
def OPC_CMP3_SIMM16    : RXOpcode<0b01110110>;
def OPC_CMP3_SIMM24    : RXOpcode<0b01110111>;
def OPC_BSET3          : RXOpcode<0b01110110>;  // 0111 100i mmmm rdst	bset	#%1, %0
def OPC_BCLR3          : RXOpcode<0b01111010>;  // 0111 101i mmmm rdst	bclr	#%1, %0
def OPC_BTST3          : RXOpcode<0b01111100>;  // 0111 110i mmmm rdst	btst	#%2, %1
def OPC_MOV1_B         : RXOpcode<0b10000000>;  // 10sz 0dsp a dst b src	mov%s	%1, %0
def OPC_MOV1_W         : RXOpcode<0b10010000>;
def OPC_MOV1_L         : RXOpcode<0b10100000>;
def OPC_MOV2_B         : RXOpcode<0b10001000>;  // 10sz 1dsp a src b dst	mov%s	%1, %0
def OPC_MOV2_W         : RXOpcode<0b10011000>;
def OPC_MOV2_L         : RXOpcode<0b10101000>;
def OPC_movu           : RXOpcode<0b10110000>;  // 1011 wdsp asrc bdst	movu%s	%1, %0
def OPC_mov5           : RXOpcode<0b11000000>;  // 11sz sdss rsrc rdst	mov%s	%1, %0
def OPC_mov6           : RXOpcode<0b11111000>;  // 1111 10sd rdst imsz	mov%s	#%1, %0



// ���߃R�[�h �̂����A16 bit�l�őΉ����Ă��閽�߂��`
// switch (op[1] & 0xff) ���܂�
class RXOpcode2<bits<8> val1, bits<8> val2, bits<8> mask2> {
  bits<8> Value1 = val1;
  bits<8> Value2 = val2;
  bits<8> Mask2 = mask2;
}
def OPC_RTE       : RXOpcode2<0b01111111, 0b10010101, 0b11111111>;
def OPC_RTFI      : RXOpcode2<0b01111111, 0b10010100, 0b11111111>;
def OPC_SATR      : RXOpcode2<0b01111111, 0b10010011, 0b11111111>;
def OPC_SCMPU     : RXOpcode2<0b01111111, 0b10000011, 0b11111111>;
def OPC_SMOVB     : RXOpcode2<0b01111111, 0b10001011, 0b11111111>;
def OPC_SMOVF     : RXOpcode2<0b01111111, 0b10001111, 0b11111111>;
def OPC_SMOVU     : RXOpcode2<0b01111111, 0b10000111, 0b11111111>;
def OPC_WAIT      : RXOpcode2<0b00000011, 0b00000011, 0b11111111>;
def OPC_mov       : RXOpcode2<0b00000011, 0b00000011, 0b11111111>; // 0111 0101 0100 rdst	mov%s	#%1, %0
// def OPC_X76		// 0x76 : etc
// def OPC_x77		// 0x00/0x10/0x20/0x30

def OPC_NOT1      : RXOpcode2<0b01111110, 0b00000000, 0b11110000>;  // 0111 1110 0000 rdst	not	%0
def OPC_NEG1      : RXOpcode2<0b01111110, 0b00010000, 0b11110000>;  // 0111 1110 0001 rdst	neg	%0
def OPC_ABS1      : RXOpcode2<0b01111110, 0b00100000, 0b11110000>;  // 0111 1110 0010 rdst	abs	%0
def OPC_SAT1      : RXOpcode2<0b01111110, 0b00110000, 0b11110000>;  // 0111 1110 0011 rdst	sat	%0
def OPC_RORC1     : RXOpcode2<0b01111110, 0b01000000, 0b11110000>;  // 0111 1110 0100 rdst	rorc	%0
def OPC_ROLC1     : RXOpcode2<0b01111110, 0b01010000, 0b11110000>;  // 0111 1110 0101 rdst	rolc	%0
def OPC_PUSH1     : RXOpcode2<0b01111110, 0b10000000, 0b11000000>;  // 0111 1110 10sz rsrc	push%s %1

def OPC_POP       : RXOpcode2<0b01111110, 0b10110000, 0b11110000>;  // 0111 1110 1011 rdst	pop	%0
def OPC_PUSH2     : RXOpcode2<0b01111110, 0b11000000, 0b11100000>;  // 0111 1110 110c rsrc	pushc	%1
def OPC_POPC      : RXOpcode2<0b01111110, 0b11100000, 0b11100000>;  // 0111 1110 111c rdst	popc	%0
def OPC_JMP       : RXOpcode2<0b01111111, 0b00000000, 0b11110000>;  // 0111 1111 0000 rsrc	jmp	%0
def OPC_JSR       : RXOpcode2<0b01111111, 0b00010000, 0b11110000>;  // 0111 1111 0001 rsrc	jsr	%0
def OPC_BRAL      : RXOpcode2<0b01111111, 0b01000000, 0b11110000>;  // 0111 1111 0100 rsrc	bra.l	%0
def OPC_BSRL      : RXOpcode2<0b01111111, 0b01010000, 0b11110000>;  // 0111 1111 0101 rsrc	bsr.l	%0
def OPC_SUNTIL    : RXOpcode2<0b01111111, 0b10000000, 0b11111100>;  // 0111 1111 1000 00sz	suntil%s
def OPC_SSTR      : RXOpcode2<0b01111111, 0b10001000, 0b11111100>;  // 0111 1111 1000 10sz	sstr
def OPC_RMPA      : RXOpcode2<0b01111111, 0b10001100, 0b11111100>;  // 0111 1111 1000 11sz	rmpa%s
def OPC_SETPSW    : RXOpcode2<0b01111111, 0b10100000, 0b11110000>;  // 0111 1111 1010 rdst	setpsw	%0
def OPC_CLRPSW    : RXOpcode2<0b01111111, 0b10110000, 0b11110000>;  // 0111 1111 1011 rdst	clrpsw	%0
def OPC_BSET      : RXOpcode2<0b11110000, 0b10110000, 0b11110000>;  // 1111 00sd rdst 0bit	bset	#%1, %0%S0
def OPC_BCLR      : RXOpcode2<0b11110000, 0b10110000, 0b11110000>;  // 1111 00sd rdst 1bit	bclr	#%1, %0%S0
def OPC_BTST      : RXOpcode2<0b11110100, 0b10110000, 0b11110000>;  // 1111 01sd rdst 0bit	btst	#%2, %1%S1
def OPC_PUSH3     : RXOpcode2<0b11110100, 0b10110000, 0b11110000>;  // 1111 01ss rsrc 10sz	push%s	%1

// 1111 1100 0000 0011 rsrc rdst	sbb	%1, %0
// 1111 1100 0000 0111 rsrc rdst	neg	%2, %0
// 1111 1100 0000 1011 rsrc rdst	adc	%1, %0
// 1111 1100 0000 1111 rsrc rdst	abs	%1, %0
// 1111 1100 0001 00ss rsrc rdst	max	%1%S1, %0
// 1111 1100 0001 01ss rsrc rdst	min	%1%S1, %0
// 1111 1100 0001 10ss rsrc rdst	emul	%1%S1, %0
// 1111 1100 0001 11ss rsrc rdst	emulu	%1%S1, %0
// 1111 1100 0010 00ss rsrc rdst	div	%1%S1, %0
// 1111 1100 0010 01ss rsrc rdst	divu	%1%S1, %0
// 1111 1100 0011 00ss rsrc rdst	tst	%1%S1, %2
// 1111 1100 0011 01ss rsrc rdst	xor	%1%S1, %0
// 1111 1100 0011 1011 rsrc rdst	not	%1, %0
// 1111 1100 0100 00ss rsrc rdst	xchg	%1%S1, %0
// 1111 1100 0100 01sd rsrc rdst	itof	%1%S1, %0
// 1111 1100 0100 1011 rsrc rdst	stz	%1, %0
// 1111 1100 0100 1111 rsrc rdst	stnz	%1, %0
// 1111 1100 0101 01sd rsrc rdst	utof	%1%S1, %0
// 1111 1100 0101 1010 rsrc rdst	bfmovz	%bf 
// 1111 1100 0101 1110 rsrc rdst	bfmov	%bf
// 1111 1100 0110 00sd rdst rsrc	bset	%1, %0%S0
// 1111 1100 0110 01sd rdst rsrc	bclr	%1, %0%S0
// 1111 1100 0110 10sd rdst rsrc	btst	%2, %1%S1
// 1111 1100 0110 11sd rdst rsrc	bnot	%1, %0%S0
// 1111 1100 0111 10sz rdst 1000	dmov.d	%1, %0
// 1111 1100 1000 00sd rsrc rdst	fsub	%1%S1, %0
// 1111 1100 1000 01sd rsrc rdst	fcmp	%1%S1, %0
// 1111 1100 1000 10sd rsrc rdst	fadd	%1%S1, %0
// 1111 1100 1000 11sd rsrc rdst	fmul	%1%S1, %0
// 1111 1100 1001 00sd rsrc rdst	fdiv	%1%S1, %0
// 1111 1100 1001 01sd rsrc rdst	ftoi	%1%S1, %0
// 1111 1100 1001 10sd rsrc rdst	round	%1%S1, %0
// 1111 1100 1010 00sd rsrc rdst	fsqrt	%1%S1, %0
// 1111 1100 1010 01sd rsrc rdst	ftou	%1%S1, %0
// 1111 1100 1100 10sz rsrc 1000	dmov.d	%1, %0
// 1111 1100 1101 sz sd rdst cond	sc%1%s	%0

// ���߃R�[�h �̂����A24 bit�l�őΉ����Ă��閽�߂��`
class RXOpcode3<bits<8> val1, bits<8> val2, bits<8> val3> {
  bits<8> Value1 = val1;
  bits<8> Value2 = val2;
  bits<8> Value3 = val3;
}
// 0xf9
// 1111 1001 0000 0011 rdst 0000 dmov.l	#%1, %0
// 1111 1001 0000 0011 rdst 001s dmov%s	#%1, %0

// 1111 1100 111bit sd rdst cond	bm%2	#%1, %0%S0
// 1111 1100 111bit sd rdst 1111	bnot	#%1, %0%S0

// 1111 1101 0000 a000 srca srcb	mulhi	%1, %2, %0
// 1111 1101 0000 a001 srca srcb	mullo	%1, %2, %0
// 1111 1101 0000 a010 srca srcb	mullh	%1, %2, %0
// 1111 1101 0000 a011 srca srcb 	emula	%1, %2, %0
// 1111 1101 0000 a100 srca srcb	machi	%1, %2, %0
// 1111 1101 0000 a101 srca srcb	maclo	%1, %2, %0
// 1111 1101 0000 a110 srca srcb	maclh	%1, %2, %0
// 1111 1101 0000 a111 srca srcb 	emaca	%1, %2, %0
// 1111 1101 0001 0111 a000 rsrc	mvtachi	%1, %0
// 1111 1101 0001 0111 a001 rsrc	mvtaclo	%1, %0
// 1111 1101 0001 0111 a011 rdst	mvtacgu	%0, %1
// 1111 1101 0001 1000 a00i 0000	racw	#%1, %0
// 1111 1101 0001 1000 a10i 0000	rdacw	#%1, %0
// 1111 1101 0001 1001 a00i 0000	racl	#%1, %0
// 1111 1101 0001 1001 a10i 0000	rdacl	#%1, %0
// 1111 1101 0001 111i a m01 rdst	mvfaclo	#%2, %1, %0
// 1111 1101 0001 111i a m11 rdst	mvfacgu	#%2, %1, %0
// 1111 1101 0010 0p sz rdst rsrc	mov%s	%1, %0
// 1111 1101 0010 0111 rdst rsrc	movco	%1, [%0]
// 1111 1101 0010 1p sz rsrc rdst	mov%s	%1, %0
// 1111 1101 0010 1111 rsrc rdst	movli	[%1], %0
// 1111 1101 0011 1p sz rsrc rdst	movu%s	%1, %0
// 1111 1101 0100 a100 srca srcb	msbhi	%1, %2, %0
// 1111 1101 0100 a101 srca srcb	msblo	%1, %2, %0
// 1111 1101 0100 a110 srca srcb	msblh	%1, %2, %0
// 1111 1101 0100 a111 srca srcb 	emsba	%1, %2, %0
// 1111 1101 0110 0000 rsrc rdst	shlr	%2, %0
// 1111 1101 0110 0001 rsrc rdst	shar	%2, %0
// 1111 1101 0110 0010 rsrc rdst	shll	%2, %0
// 1111 1101 0110 0100 rsrc rdst	rotr	%1, %0
// 1111 1101 0110 0101 rsrc rdst	revw	%1, %0
// 1111 1101 0110 0110 rsrc rdst	rotl	%1, %0
// 1111 1101 0110 0111 rsrc rdst	revl	%1, %0

// 1111 1101 0110 100c rsrc rdst	mvtc	%1, %0
// 1111 1101 0110 101s rsrc rdst	mvfc	%1, %0
// 1111 1101 0110 110i mmmm rdst	rotr	#%1, %0
// 1111 1101 0110 111i mmmm rdst	rotl	#%1, %0
// 1111 1101 0111 im00 0010rdst	adc	#%1, %0
// 1111 1101 0111 im00 0100rdst	max	#%1, %0
// 1111 1101 0111 im00 0101rdst	min	#%1, %0
// 1111 1101 0111 im00 0110rdst	emul	#%1, %0
// 1111 1101 0111 im00 0111rdst	emulu	#%1, %0
// 1111 1101 0111 im00 1000rdst	div	#%1, %0
// 1111 1101 0111 im00 1001rdst	divu	#%1, %0
// 1111 1101 0111 im00 1100rdst	tst	#%1, %2
// 1111 1101 0111 im00 1101rdst	xor	#%1, %0
// 1111 1101 0111 im00 1110rdst	stz	#%1, %0
// 1111 1101 0111 im00 1111rdst	stnz	#%1, %0
// 1111 1101 0111 0010 0000 rdst	fsub	#%1, %0
// 1111 1101 0111 0010 0001 rdst	fcmp	#%1, %0
// 1111 1101 0111 0010 0010 rdst	fadd	#%1, %0
// 1111 1101 0111 0010 0011 rdst	fmul	#%1, %0
// 1111 1101 0111 0010 0100 rdst	fdiv	#%1, %0
// 1111 1101 0111 im11 000crdst	mvtc	#%1, %0
// 1111 1101 0111 0101 1000 rdst rsrc 0000	dmov.l	%1, %0
// 1111 1101 0111 0101 1000 rdst rsrc 0010	dmov.l	%1, %0
// 1111 1101 0111 0101 1000 rdst rsrc 0100	mvfdc	%1, %0
// 1111 1101 0111 0110 1100 rsrc 0000 0000 	save	%1
// 1111 1101 0111 0110 1101 rsrc 0000 0000 	rstr	%1
// 1111 1101 0111 0110 1110 0000	save	#%1
// 1111 1101 0111 0110 1111 0000 	rstr	#%1
// 1111 1101 0111 0111 1000 rsrc rdst 0000	dmov.l	%1, %0
// 1111 1101 0111 0111 1000 rsrc rdst 001s	dmov%s	%1, %0
// 1111 1101 0111 0111 1000 rdst rsrc 0100	mvtdc	%1, %0
// 1111 1101 0111 0111 1000 rsrc rdst 1001	itod	%1, %0
// 1111 1101 0111 0111 1000 rsrc rdst 1010	ftod	%1, %0
// 1111 1101 0111 0111 1000 rsrc rdst 1101	utod	%1, %0
// 1111 1101 100immmm rsrc rdst	shlr	#%2, %1, %0
// 1111 1101 101immmm rsrc rdst	shar	#%2, %1, %0
// 1111 1101 110immmm rsrc rdst	shll	#%2, %1, %0
// 1111 1101 111 bittt cond rdst	bm%2	#%1, %0%S0
// 1111 1101 111bittt 1111 rdst	bnot	#%1, %0
// 1111 1110 00sz isrc bsrc rdst	mov%s	%0, [%1, %2]
// 1111 1110 01sz isrc bsrc rdst	mov%s	[%1, %2], %0
// 1111 1110 11sz isrc bsrc rdst	movu%s	[%1, %2], %0
// 1111 1111 0000 rdst srca srcb	sub	%2, %1, %0
// 1111 1111 0010 rdst srca srcb	add	%2, %1, %0
// 1111 1111 0011 rdst srca srcb	mul 	%2, %1, %0
// 1111 1111 0100 rdst srca srcb	and	%2, %1, %0
// 1111 1111 0101 rdst srca srcb	or	%2, %1, %0
// 1111 1111 0110 rdst srca srcb	xor	%2, %1, %0
// 1111 1111 1000 rdst srca srcb	fsub	%2, %1, %0
// 1111 1111 1010 rdst srca srcb	fadd	%2, %1, %0
// 1111 1111 1011 rdst srca srcb	fmul	%2, %1, %0


// http://msyksphinz.hatenablog.com/entry/2019/01/23/040000
// P167 ����̖��߃R�[�h�̑Ή�������?(���߃R�[�h����[CISC]�̂��ߒ��ӁBX86 �̒�`������?)
// https://www.renesas.com/jp/ja/doc/products/mpumcu/doc/rx_family/r01us0032jj0120_rxsm.pdf
// Base ����?
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

// �Œ薽�ߒ��R�[�h(1byte)
// BRK, NOP, RTS
class I<RXOpcode opcode, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<8> Inst;
  let Size = 1;

  let Inst = opcode.Value;
}

// �Œ薽�ߒ��R�[�h(2byte)
// RTE, RTFI, SATR, SCMPU, SMOVF, SMOVU, WAIT
class I2<RXOpcode2 opcode, dag outs, dag ins, string opcodestr, string argstr, list<dag> pattern>
  : RXInst<InstFormatRawFrm, outs, ins, opcodestr, argstr, pattern> {
  field bits<16> Inst;
  let Size = 2;

  let Inst{15-8} = opcode.Value1;
  let Inst{7-0} = opcode.Value2;
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
// �ėp���W�X�^?
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


// ���䃌�W�X�^
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