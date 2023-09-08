// TEST WRITE READ AND LOOPS

INIT:
// REGITSERS
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
DMEM_WR [&9] op -op(r9) -wr(r10 imm) #11 
DMEM_WR [&10] op -op(r10 AND #14) -wr(r10 op)

REG_WR r1 imm #-1
REG_WR r1 op -op(ABS r1)

// REGITSERS & MEMORY
REG_WR r11 dmem [&10] 
REG_WR r11 op -op(r11 + #1)
DMEM_WR [&11] op -op(r11)
REG_WR r12 dmem [r11] 
REG_WR r12 op -op(r12 + #1)
DMEM_WR [r12] imm #12
DMEM_WR [r12+&1] op -op(r11+#2)

REG_WR r13 imm #13
REG_WR r14 dmem [r13+&1] 

REG_WR r14 op -op(r12 OR #2)
REG_WR r15 op -op(rand)
REG_WR r15 label INIT
REG_WR r15 dmem [&0]
REG_WR r15 op -op(r14 XOR #1)




// CYCLES 

////////////////////////////////////////////
REG_WR r0 imm #1
REG_WR r15 imm #10 
// This LOOP takes 3 clock per cycle (1 to execute and 2 to FLUSH and Update FLAG)
REPEAT_1:
   JUMP REPEAT_1 -if(NZ) -wr( r15 op ) -op(r15-#1) -uf
//Update Flag, but not execute instruction FLAG HERE ENDS IN 0..

////////////////////////////////////////////
REG_WR r0 imm #2
REG_WR r15 imm #10
// This LOOP takes 6 clock per cycle (1ex Substract 2stall FlagUpdate 1ex JUMP and 2 to FLUSH)
REPEAT_2:
   REG_WR r15 op -op(r15-#1) -uf
   JUMP REPEAT_2 -if(NZ) 
//Flag was Updated instruction FLAG HERE ENDS IN 1..

////////////////////////////////////////////
REG_WR r0 imm #3
REG_WR r15 op -op(s0+#10) -uf
// This LOOP takes 7 clock per cycle (1ex Substract 2stall FlagUpdate 1ex JUMP1 1ex JUMP2 and 2 to FLUSH)
REPEAT_3:
   REG_WR r15 op -op(r15-#1) -uf
   JUMP CONT_3 -if(Z)
   JUMP REPEAT_3
CONT_3:


////////////////////////////////////////////
REG_WR r0 imm #4
REG_WR r15 op -op(s0+#10) -uf
// This LOOP takes 4 clock cycle (1ex JUMP1 1ex JUMP2 and 2 to FLUSH and Update FLAG)
REPEAT_4:
   JUMP CONT_4 -if(Z) 
   JUMP REPEAT_4 -wr(r15 op) -op(r15-#1) -uf 
CONT_4:


REG_WR r0 imm #5

JUMP HERE
