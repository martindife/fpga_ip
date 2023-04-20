# -*- coding: utf-8 -*-
"""
Created on Thu Jan 26 11:02:38 2023
@author: mdifeder
"""


from tprocv2_assembler import Converter, Logger

# Show ALL
Logger.level = 0 

prog_list = []
Dict_Label = { 'r_addr':'s15'   }

## Create the WAIT_EC label on this Memory address
Dict_Label['WAIT_EC'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'START', 'IF':'EC' } )
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'WAIT_EC' } )

## Create the START label on this Memory address
Dict_Label['START'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':"REG_WR" , 'DST':'r1'    ,'SRC':'op'   ,'OP'   : 'MSH s7'  } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'r2'    ,'SRC':'op'   ,'OP'   : 'LSH s7'  } )
prog_list.append( {'CMD':'CALL'     , 'LABEL':'F_CREATE_WAVES' } )
prog_list.append( {'CMD':'TIME'     , 'DST':'rst' } )
prog_list.append( {'CMD':'CALL'     , 'LABEL':'F_SHOW_WAVES' } )

## Create the WAIT_NEC label on this Memory address
Dict_Label['WAIT_NEC'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'WAIT_NEC' } )
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'WAIT_EC' } )

## Create the F_SHOW_WAVES label on this Memory address
Dict_Label['F_SHOW_WAVES'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':'TIME'     , 'DST':'set_ref', 'LIT':'100' } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'r0'    ,'SRC':'op'   ,'OP'   : 'r1-#1', 'UF':'1'  } )


## Create the SHOW_LOOP label on this Memory address
Dict_Label['SHOW_LOOP'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':"REG_WR" , 'DST':'s14'    ,'SRC':'op'   ,'OP'   : 's14+r2'  } )
prog_list.append( {'CMD':'WPORT_WR' , 'DST':'4'    ,'SRC':'wmem'   ,'ADDR':'r0' } )
prog_list.append( {'CMD':'DPORT_WR' , 'DST':'0'    ,'SRC':'op'   ,'OP'  :'r0' } )
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'SHOW_LOOP', 'IF':'NZ', 'WR':'r0 op', 'OP':'r0-#1', 'UF':'1' } )
prog_list.append( {'CMD':'RET'  } )


## Create the F_CREATE_WAVES label on this Memory address
Dict_Label['F_CREATE_WAVES'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':"REG_WR" , 'DST':'r0'    ,'SRC':'op'   ,'OP'   : 'r1-#1', 'UF':'1'  } )
## Create the CREATE_LOOP label on this Memory address
Dict_Label['CREATE_LOOP'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':'CALL'     , 'LABEL':'F_RAND_WAVE' } )
prog_list.append( {'CMD':'WMEM_WR'  , 'DST':'r0'    } )
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'CREATE_LOOP', 'IF':'NZ', 'WR':'r0 op', 'OP':'r0-#1', 'UF':'1' } )
prog_list.append( {'CMD':'RET'  } )

## Create the F_RAND_WAVE label on this Memory address
Dict_Label['F_RAND_WAVE'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':"REG_WR" , 'DST':'w0'    ,'SRC':'op'   ,'OP'   : 's1>>#1'  } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'w1'    ,'SRC':'op'   ,'OP'   : 's1>>#1'  } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'w2'    ,'SRC':'imm'  ,'LIT'  : '0'  } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'w3'    ,'SRC':'op'   ,'OP'   : 's1 AND #32767'  } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'w4'    ,'SRC':'op'   ,'OP'   : 's1 AND #63'  } )
prog_list.append( {'CMD':"REG_WR" , 'DST':'w5'    ,'SRC':'imm'  ,'LIT'  : '5'  } )
prog_list.append( {'CMD':'RET'  } )


## JUST IN CASE -Not Necesary
Dict_Label['END'] = '&' + str(len(prog_list)+1)
prog_list.append( {'CMD':'JUMP'     , 'LABEL':'END' } )

print('-----\n Get Executable from Program List Structure ')
p_bin         = Converter.list2bin(prog_list, Dict_Label)

print('-----\n Get ASM from Program List Structure ')
p_asm         = Converter.list2asm(prog_list, Dict_Label)
print(p_asm)

