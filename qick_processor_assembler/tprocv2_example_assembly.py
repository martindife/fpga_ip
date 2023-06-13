"""
FERMILAB
"""

from tprocv2_assembler import Assembler, Logger

import pickle


filenames = ['Test_0.asm','Test_LOOPS.asm', 'Test_OUT.asm', 'Test_ISA.asm', 'Examples_ISA.asm','prog_WAV.txt']

## LOGGER SHOW 
INFO = 0
WARNING = 1
ERROR = 2
Logger.setLevel(1)

print('-----\n Get Program List Structure from ASM')
p_list        = Assembler.file_asm2list(filenames[0])

print('-----\n Get ASM from Program List Structure ')
if p_list[0]:
    p_asm         = Assembler.list2asm(p_list[0], p_list[1])
#    print(p_asm)

print('-----\n Get Binary Files from ASM')
p_txt, p_bin  = Assembler.file_asm2bin(filenames[0])


#print('-----\n Print Binary executable')
#for val in p_txt:
#    print (val)
 

with open('program_mem.bin', 'w') as f:
    f.write('// TPROC Program Memory File\n')
    f.write('//HEADER & COND  &   CONF|____|     ADDRESS     |__|      DATA SOURCE (Reg and Imm)      |_| DEST|\n')
    for line_bin in p_txt:
        f.write(line_bin)
        f.write('\n')

#print ("Storing Data to Download to FPGA in file program_mem.pkl")
#data_to_file = p_bin
#pickle.dump(data_to_file, open('program_mem.pkl', 'wb')) 
#print(p_asm)
#print(p_txt)


