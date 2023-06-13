# -*- coding: utf-8 -*-
"""
Created on Thu Jan 26 11:02:38 2023
@author: mdifeder
"""


from tprocv2_assembler import Converter, Logger
asm = """
//TEST program for ALL instructions

// DIRECTIVES
// Assign new names to the registers.
.ALIAS repeat_cnt r0
.ALIAS addr_aux r1
.ALIAS data_aux r2
.ALIAS time_aux r3
INIT:
// CONF INTRUCTIONS
/////////////////////////////////////////////////
TIME rst

TEST -op(r3 - #3)
TEST -op(r4 AND #b11)
COND set
COND clear
// REGISTER Instructions
/////////////////////////////////////////////////
// GENERAL REGITSERS
REG_WR repeat_cnt imm #255
REG_WR repeat_cnt op -op(s0-#255)
REG_WR addr_aux imm #b1
REG_WR r4 imm #4
REG_WR r3 op -op(r4-#1) 
REG_WR data_aux op -op(r4 ASR #1)
REG_WR r5 op -op(r4 + #1)
REG_WR s12 op -op(s7 + #1)

END: 
    JUMP END
"""
                  
# Show ALL
Logger.level = 0 

print('-----\n Get Executable from Program List Structure ')
prog_list, Dict_Label    = Converter.str_asm2list(asm)
p_txt, p_bin   = Converter.str_asm2bin(asm)

print('-----\n Get ASM from Program List Structure ')
p_asm         = Converter.list2asm(prog_list, Dict_Label)
#print(p_asm)

