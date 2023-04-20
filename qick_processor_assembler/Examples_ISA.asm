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
NOP

REG_WR s_time label CASA
REG_WR r_time label CASA
REG_WR s_addr label CASA
REG_WR r_addr label CASA

TEST -op(r3 - #3)
TEST -op(r4 AND #b11)
COND set
COND clear

// REGISTER Instructions
/////////////////////////////////////////////////
// GENERAL REGITSERS
REG_WR r1 imm #b1
REG_WR r4 imm #4
REG_WR r3 op -op(r4-#1) 
REG_WR r2 op -op(r4 ASR #1)
REG_WR r5 op -op(r4 + #1)
REG_WR r6 op -op(r5 + #1)
REG_WR r7 op -op(r5 + #2)
REG_WR r8 op -op(r4 SL #1)
REG_WR r9 op -op(r8 OR #1)
REG_WR r10 imm #-10
REG_WR r10 op -op(ABS r10)
REG_WR r11 imm #11
REG_WR r12 op -op(r3 SL #2)
REG_WR r13 imm #13
REG_WR r14 op -op(r12 OR #2)
REG_WR r15 op -op(rand)
REG_WR r15 label INIT
REG_WR r15 dmem [&0]
REG_WR r15 op -op(r14 XOR #1)



// SPECIAL FUNCTION REGITSERS
REG_WR s0  imm #0  // Does Nothing (UPDATE LFRS if CFG='11')
REG_WR s1  imm #1  // Update LFSR Value (Load Seed)
REG_WR s2  imm #2  // Write CFG Register
REG_WR s3  imm #3  // Does Nothing (Status Register)
REG_WR s4  imm #4  // Does Nothing (div_quotient READ ONLY register)
REG_WR s5  imm #5  // Does Nothing (div_remainder READ ONLY register)
REG_WR s6  imm #6  // Does Nothing (arith_low READ ONLY register)
REG_WR s7  imm #7  // Does Nothing (core_dt1 READ ONLY register)
REG_WR s8  imm #8  // Does Nothing (core_dt2 READ ONLY register)
REG_WR s9  imm #9  // Does Nothing (port_lsw READ ONLY register)
REG_WR s10 imm #10 // Does Nothing (port_msw READ ONLY register)
REG_WR s11 imm #11 // Does Nothing (time_usr READ ONLY register)
REG_WR s12 imm #12 // Write core_w1 Register
REG_WR s13 imm #13 // Write core_w2 Register
REG_WR s14 imm #14 // Write s_time Register
REG_WR s15 imm #15 // Write s_addr Register

REG_WR r15 dmem [&0]

REG_WR s_time label CASA
REG_WR s15 label CASA
REG_WR s15 op -op(s1)
REG_WR s_addr op -op(s1+s0)


// WAVE-PARAM REGITSERS
REG_WR w0 imm #-1
REG_WR w1 imm #-1
REG_WR w2 imm #-1
REG_WR w3 imm #-1
REG_WR w4 imm #-1
REG_WR w5 imm #-1
REG_WR w_freq   imm   #104857600
REG_WR w_phase  imm   #0
REG_WR w_env    imm   #1
REG_WR w_gain   imm   #30000
REG_WR w_lenght imm   #430
REG_WR w_conf   imm   #17 //PHRST-NoPeriodic-DDS
//REG_WR r_conf   imm   #21 //PHRST-Periodic-DDS
//REG_WR r_conf   imm   #5 //NoPHRST-Periodic-DDS
//REG_WR r_conf   imm   #1 //NoPHRST-NoPeriodic-DDS

REG_WR r_wave wmem [&0]


// MEMORY Instructions
/////////////////////////////////////////////////

// DATA MEMORY
DMEM_WR [&0] imm #0 
DMEM_WR [r1] op -op(r2 ASR #1) -uf
DMEM_WR [r1+&1] imm #2 -wr(r0 op) -op(r4-#2) -uf
DMEM_WR [&3] imm #65535 -if(Z)
DMEM_WR [r1+r2] op -op(r0+r1) -wr(r0 imm) #0 -if(NZ)
DMEM_WR [r0+&4] imm #4 -wr(r1 op) -op(r1+#4)
DMEM_WR [r1] imm #5 -wr(r1 op) -op(r3-r2) 
DMEM_WR [r1+&5] op -wr(r3 op) -op(r7 AND #6)  -uf
DMEM_WR [r1+r6] op -op(r3+r1) -wr(r3 imm) #3 -if(NZ) -uf
DMEM_WR [r8] op -op(r3+r5) -wr(r8 imm) #0 -if(NZ)
DMEM_WR [&8] imm #8 -wr(r8 imm) -if(NZ)


// WAVE-PARAM  MEMORY
WMEM_WR [&0]



// PORT Instructions
/////////////////////////////////////////////////







REG_WR r0 op -op(s0+#10)
REG_WR s_time imm #100

CASA:



CALL F_CREATE_WAVES
CALL F_SHOW_WAVES


// CASE STATMENT EXAMPLES
/////////////////////////////////////////////////

// CASE STATMENT 0
//Compare and Jump.
   REG_WR r_addr label CMD_OTHERS         //OTHERS
	TEST -op(core_r1 - #0)
      CALL CMD_0 -if(Z)    // core_r1 = #0
 	TEST -op(core_r1 - #1)
      CALL CMD_1 -if(Z)    // core_r1 = #1
	TEST -op(core_r1 - #2)
      CALL CMD_2 -if(Z)    // core_r1 = #2
	TEST -op(core_r1 - #3)
      CALL CMD_3 -if(Z)    // core_r1 = #3
END_OF_CASE_0:


// CASE STATMENT 1
//Compare all posible values, and call the function (Address stored in r_addr)
   REG_WR r_addr label CMD_OTHERS         //OTHERS
	TEST -op(core_r1 - #0)
      REG_WR r_addr label CMD_0 -if(Z)    // core_r1 = #0
 	TEST -op(core_r1 - #1)
      REG_WR r_addr label CMD_1 -if(Z)    // core_r1 = #1
	TEST -op(core_r1 - #2)
      REG_WR r_addr label CMD_2 -if(Z)    // core_r1 = #2
	TEST -op(core_r1 - #3)
      REG_WR r_addr label CMD_3 -if(Z)    // core_r1 = #3
   CALL [r_addr]                          //CALL FUNCTION
END_OF_CASE_1:


// CASE STATMENT 2
//Check for the Value and jump to CALL
	TEST -op(core_r1 - #0)
      REG_WR r_addr label CMD_0 -if(Z)    // core_r1 = #0
      JUMP END_OF_CASE_2 -if(Z)
 	TEST -op(core_r1 - #1)
      REG_WR r_addr label CMD_1 -if(Z)    // core_r1 = #1
      JUMP END_OF_CASE_2 -if(Z)
	TEST -op(core_r1 - #2)
      REG_WR r_addr label CMD_2 -if(Z)    // core_r1 = #2
      JUMP END_OF_CASE_2 -if(Z)
	TEST -op(core_r1 - #3)
      REG_WR r_addr label CMD_3 -if(Z)    // core_r1 = #3
      JUMP END_OF_CASE_2 -if(Z)
   REG_WR r_addr label CMD_OTHERS         //OTHERS
END_OF_CASE_2:
   CALL [r_addr]                          //CALL FUNCTION



/////////////////////////////////////////////////
CMD_0:
// Write in DMEM Addres i1 Data i2
   // WAIT FOR EXTERNAL CONDITION TO FALL
	JUMP HERE -if(EC)
	// Get Address & Data
	REG_WR addr_aux op -op(core_r1)
	REG_WR data_aux op -op(core_r2)
	// Write Data to memory
   DMEM_WR [addr_aux] op -op(data_aux)
	// Write Data to Python
	REG_WR core_w1 op -op(tuser)
	REG_WR core_w2 imm #10
RET
/////////////////////////////////////////////////
CMD_1:
// Write Data core_r1 in PORT0 @ TIME core_r2
   // WAIT FOR EXTERNAL CONDITION TO FALL
	JUMP HERE -if(EC)
	// Get Data & Time
	REG_WR data_aux op -op(core_r1)
	REG_WR r_time op -op(core_r2)
	// Write Data to PORT
   DPORT_WR p0 op -op(data_aux)
	// Write Data to Python
	REG_WR core_w1 op -op(tuser)
	REG_WR core_w2 imm #11
RET
/////////////////////////////////////////////////
CMD_2:
   // WAIT FOR EXTERNAL CONDITION TO FALL
	JUMP HERE -if(EC)
	// Write Data to Python
	REG_WR core_w1 op -op(tuser)
	REG_WR core_w2 imm #12
RET
/////////////////////////////////////////////////
CMD_3:
   // WAIT FOR EXTERNAL CONDITION TO FALL
	JUMP HERE -if(EC)
	// Write Data to Python
	REG_WR core_w1 op -op(tuser)
	REG_WR core_w2 imm #13
RET
/////////////////////////////////////////////////
CMD_OTHERS:
	// Write Data to Python
	REG_WR core_w1 op -op(tuser)
	REG_WR core_w2 op -op(core_r2)
   // WAIT FOR EXTERNAL CONDITION TO FALL
	JUMP HERE -if(EC)
RET








// FUNCTIONS
F_SHOW_WAVES:
   TIME set_ref #100
   REG_WR repeat_cnt op -op(data_aux-#1) -uf
   SHOW_LOOP:
      REG_WR s_time op -op(s_time + time_aux) // Increase Time
      WPORT_WR p4 wmem [repeat_cnt]
      DPORT_WR p0 op -op(repeat_cnt)
   JUMP SHOW_LOOP -wr(repeat_cnt op) -op(repeat_cnt-#1) -if(NZ) -uf
RET

// Create data_aux number of frecuency random waves and stored on WMEM
F_CREATE_WAVES:
   REG_WR repeat_cnt op -op(data_aux-#1) -uf
   CREATE_LOOP:
      CALL F_RAND_WAVE           // Create Random Waveform Function
      WMEM_WR [repeat_cnt]       // Copy to Memory
   JUMP CREATE_LOOP -wr(repeat_cnt op) -op(repeat_cnt-#1) -if(NZ) -uf
RET

F_RAND_WAVE:
   REG_WR w_freq   op   -op(s1 >> #4 )
   REG_WR w_freq   op   -op(w_freq +  #163840 )
   REG_WR w_phase  imm  #0
   REG_WR w_env    imm  #0
   REG_WR w_gain   op   -op(s1 AND #4095) // 12 Bits
   REG_WR w_gain   op   -op(w_gain +  #20000)
   REG_WR w_lenght imm #430
   // REG_WR w_lenght op   -op(s1 AND #430) // Max Value 430
   REG_WR w_conf   imm  #17 // PHRST   - NoPeriodic - DDS
   //REG_WR w_conf   imm  #5  // NoPHRST - Periodic   - DDS
   //REG_WR w_conf   imm  #21 // PHRST   - Periodic   - DDS
   //REG_WR w_conf   imm  #1  // NoPHRST - NoPeriodic - DDS
RET


JUMP CASA
