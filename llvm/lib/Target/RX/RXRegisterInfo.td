//===-- RXRegisterInfo.td - RX Register defs --------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the RX register files
//===----------------------------------------------------------------------===//

// We have banks of 16+9 registers each.
let Namespace = "RX" in {
class RXReg<bits<5> Enc, string n, list<string> alt = []> : Register<n> {
  let HWEncoding{4-0} = Enc;
  let AltNames = alt;
}

// RX CPU Registers?
// https://www.aps-web.jp/academy/risc-v/03/
// CSR の設定?
// class RXReg<bits<5> Enc, string n, list<string> alt> : RXReg<Enc, n>;

  def ABIRegAltName : RegAltNameIndex;
} // Namespace = "RX"

// Integer registers
// CostPerUse is set higher for registers that may not be compressible as they
// are not part of GPRC, the most restrictive register class used by the
// compressed instruction set. This will influence the greedy register
// allocator to reduce the use of registers that can't be encoded in 16 bit
// instructions. This affects register allocation even when compressed
// instruction isn't targeted, we see no major negative codegen impact.

// CPU レジスタセットを見て対応?
// Arm 側のソースコードの対応も見ておいた方がよい?(レジスタの切り替えの点から)
// https://www.renesas.com/jp/ja/doc/products/mpumcu/doc/rx_family/r01us0032jj0120_rxsm.pdf
// 32bit 汎用レジスタ*16 + 32bit 制御レジスタ*9 + アキュムレータ(64bit?)
// def ACC0  : RXReg<24, "fpsw", ["acc0"]>, DwarfRegNum<[24]>;
// def ACC1  : RXReg<25, "fpsw", ["acc1"]>, DwarfRegNum<[25]>;
// https://github.com/Broadcom/stblinux-2.6.37/blob/master/uclinux-rootfs/user/gdb/opcodes/rx-dis.c [register_names] 参照?
let Namespace = "RX" in {
  def R0    : RXReg<0,  "sp",  ["sp"]>, DwarfRegNum<[0]>;
  def R1    : RXReg<1,  "r1",  ["R1"]>, DwarfRegNum<[1]>;
  def R2    : RXReg<2,  "r2",  ["R2"]>, DwarfRegNum<[2]>;
  def R3    : RXReg<3,  "r3",  ["R3"]>, DwarfRegNum<[3]>;
  def R4    : RXReg<4,  "r4",  ["R4"]>, DwarfRegNum<[4]>;
  def R5    : RXReg<5,  "r5",  ["R5"]>, DwarfRegNum<[5]>;
  def R6    : RXReg<6,  "r6",  ["R6"]>, DwarfRegNum<[6]>;
  def R7    : RXReg<7,  "r7",  ["R7"]>, DwarfRegNum<[7]>;
  def R8    : RXReg<8,  "r8",  ["R8"]>, DwarfRegNum<[8]>;
  def R9    : RXReg<9,  "r9",  ["R9"]>, DwarfRegNum<[9]>;
  def R10   : RXReg<10, "r10", ["R10"]>, DwarfRegNum<[10]>;
  def R11   : RXReg<11, "r11", ["R11"]>, DwarfRegNum<[11]>;
  def R12   : RXReg<12, "r12", ["R12"]>, DwarfRegNum<[12]>;
  def R13   : RXReg<13, "r13", ["R13"]>, DwarfRegNum<[13]>;
  def R14   : RXReg<14, "r14", ["R14"]>, DwarfRegNum<[14]>;
  def R15   : RXReg<15, "r15", ["R15"]>, DwarfRegNum<[15]>;
  def PSW   : RXReg<16, "psw",   ["psw"]>,   DwarfRegNum<[16]>;
  def PC    : RXReg<17, "pc",    ["pc"]>,    DwarfRegNum<[17]>;
  def USP   : RXReg<18, "usp",   ["usp"]>,   DwarfRegNum<[18]>;
  def FPSW  : RXReg<19, "fpsw",  ["fpsw"]>,  DwarfRegNum<[19]>;
  def WR    : RXReg<23, "wr",    ["wr"]>,    DwarfRegNum<[23]>;
  def BPSW  : RXReg<24, "bpsw",  ["bpsw"]>,  DwarfRegNum<[24]>;
  def BPC   : RXReg<25, "bpc",   ["bpc"]>,   DwarfRegNum<[25]>;
  def ISP   : RXReg<26, "isp",   ["isp"]>,   DwarfRegNum<[26]>;
  def FINTV : RXReg<27, "fintv", ["fintv"]>, DwarfRegNum<[27]>;
  def INTB  : RXReg<28, "intb",  ["intb"]>,  DwarfRegNum<[28]>;
}

def XLenVT : ValueTypeByHwMode<[DefaultMode], [i32]>;

// R15 : 戻り値が構造体等の場合に使用される？
// Page 724 of 853
// https://www.renesas.com/us/en/doc/products/tool/doc/016/r20ut3248ej0108-ccrx.pdf
// Page 718 of 880(JP)
// ---
// The order of registers represents the preferred allocation sequence.
// Registers are listed in the order caller-save, callee-save, specials.
def GPR : RegisterClass<"RX", [XLenVT], 32, (add
  // Reserved
  R0,
  // Return Values and Arguments
  R1, R2, R3, R4, 
  // Not preserved across procedure calls
  R5, R14, R15, BPSW, FPSW,
  // Callee save(None?)
  // Caller save
  R6, R7, R8, R9, R10, R11, R12, R13,
  // Reserved
  PC, INTB, BPC, FINTV
)>;

// 汎用レジスタは 16
def GPRC : RegisterClass<"RX", [XLenVT], 32, (add
  (sequence "R%u", 6, 13)
)>;

// For indirect tail calls, we can't use callee-saved registers, as they are
// restored to the saved value before the tail call, which would clobber a call
// address.
def GPRTC : RegisterClass<"RX", [XLenVT], 32, (add
  R5,
  (sequence "R%u", 14, 15)
)>;

def SP : RegisterClass<"RX", [XLenVT], 32, (add R0, ISP, USP)> {
}

// 浮動小数点レジスタ(RX は存在しない?)(ACC?)
// The order of registers represents the preferred allocation sequence,
// meaning caller-save regs are listed before callee-save.
// def FPR32 : RegisterClass<"RX", [f32], 32, (add
//     (sequence "F%u_32", 0, 7),
//     (sequence "F%u_32", 10, 17),
//     (sequence "F%u_32", 28, 31),
//     (sequence "F%u_32", 8, 9),
//     (sequence "F%u_32", 18, 27)
// )>;
// 
// def FPR32C : RegisterClass<"RX", [f32], 32, (add
//   (sequence "F%u_32", 10, 15),
//   (sequence "F%u_32", 8, 9)
// )>;
// def FPR32 : RegisterClass<"RX", [f32], 32, (add
//   (sequence "R%u", 2, 3)
// )>;

// def FPR32C : RegisterClass<"RX", [f32], 32, (add
//   (sequence "R%u", 0, 1)
// )>;
