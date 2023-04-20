"""
FERMILAB
"""

from tprocv2_assembler import Converter, Logger

import pickle


filenames = ['Test_ISA.asm', 'Examples_ISA.asm','prog_WAV.txt']

## LOGGER SHOW 
INFO = 0
WARNING = 1
ERROR = 2
Logger.setLevel(0)

print('-----\n Get Program List Structure from ASM')
p_list        = Converter.asm2list(filenames[0])

print('-----\n Get ASM from Program List Structure ')
if p_list[0]:
    p_asm         = Converter.list2asm(p_list[0], p_list[1])


print('-----\n Get Binary Files from ASM')
p_txt, p_bin  = Converter.asm2bin(filenames[1])


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