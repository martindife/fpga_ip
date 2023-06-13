//    TEST WRITE READ AND LOOPS

INIT:

.ALIAS hazard_dest s12
.ALIAS hazard_source s13

DMEM_WR [r0] imm #3

DPORT_WR p0 imm #1 -wr(r1 imm)
DPORT_WR p1 op -op(r1+#1) -wr(r2 op)
DPORT_WR p2 imm #3 -wr(r3 op) -op(r1+r2)
DPORT_WR p3 op -op(r1+r3) -wr(r4 imm) #4
REG_WR r1 imm #1
DPORT_WR p0 imm #1 -wr(r2 op) -op(r1+#1)
DPORT_WR p1 op -op(r2) -wr(r3 imm) #3
DPORT_WR p2 imm #3 -wr(r4 op) -op(r1+r3)
DPORT_WR p3 op -op(r1+r3) -wr(r5 imm) #5


CALL F_FILL_MEM
CALL F_CHECK_MEM
CALL F_FILL_REG
CALL F_CHECK_REG
CALL F_CLEAR_REG

// HAZARD SPREADSHEET

/// DEST > D_S_REG
CALL F_CLEAR_REG

REG_WR hazard_dest imm #1
REG_WR hazard_source imm #1
REG_WR r1 imm #1
REG_WR r2 op -op(r1+#1)
REG_WR r3 op -op(r2+r1)
REG_WR r4 dmem [&4]
REG_WR r5 op -op(r4+r1)
REG_WR r6 op -op(r4+r2)

REG_WR hazard_source imm #2
REG_WR r_freq imm #100
REG_WR r7 op -op(r_freq-#93)
REG_WR r_gain op -op(r7-r1)
REG_WR r8 op -op(r_gain+#2)
REG_WR r_phase dmem [&3]
REG_WR r9 op -op(r_phase+r_gain)

REG_WR hazard_source imm #3
REG_WR r_wave wmem [&1]
REG_WR r10 op -op(r_freq+#9)
REG_WR r_wave wmem [r2]
REG_WR r11 op -op(r_freq+r9)
REG_WR r_wave wmem [&3] -wr(w1 imm) #123
REG_WR r12 op -op(r_freq+#9)

REG_WR hazard_source imm #4
DMEM_WR [&13] imm #13
REG_WR r13 dmem [&13]
DMEM_WR [r13+&1] op -op(r12+#2)
REG_WR r14 dmem [&14]
DMEM_WR [&15] op -op(r10+r5) -wr(r15 imm) #5
REG_WR r15 op -op(r15+#10)

REG_WR hazard_source imm #5
WMEM_WR [&9] -wr(r1 imm) #15
REG_WR r1 op -op(r1 AND #1)
WMEM_WR [r10] -wr(r2 imm) #-1
REG_WR r2 op -op(r1 SL #1)

CALL F_CHECK_REG

/// DEST > W_REG
CALL F_CLEAR_REG

REG_WR hazard_dest imm #2

REG_WR hazard_source imm #1
REG_WR hazard_source imm #2
REG_WR r_freq imm #-1
REG_WR r_freq op -op(r_freq+#1)
REG_WR r_freq op -op(r_freq+#1)
REG_WR r_freq op -op(r1+r1)
REG_WR r_freq op -op(r_freq+#1)
REG_WR r_freq dmem [&4]
REG_WR r_freq op -op(r_freq+#1)

REG_WR hazard_source imm #3
REG_WR r_wave wmem [&0]
REG_WR r_freq op -op(r_freq+#1)
REG_WR r_wave wmem [r2]
REG_WR r_freq op -op(r_freq+#1)
REG_WR r_wave wmem [&9] -wr(r_freq op) -op(r_freq+#1)
REG_WR r_freq op -op(r_freq+#1)

REG_WR hazard_source imm #4
DMEM_WR [&10] imm #10
REG_WR r_freq dmem [&10]
DMEM_WR [r11] op -op(r10+#1)
REG_WR r_freq dmem [&11]
DMEM_WR [&12] op -op(r10+r2) -wr(r_freq imm) #12
REG_WR r_freq dmem [&13]

REG_WR hazard_source imm #5
WMEM_WR [&10] -wr(r_freq imm) #1000
REG_WR r_freq op -op(r_freq-#1)
WMEM_WR [r10] -wr(r_freq op) -op(r_freq-#1)
REG_WR r_freq op -op(r_freq-#1)
WMEM_WR [r10] -wr(r_freq op) -op(r_freq-#1)
REG_WR r_freq op -op(r_freq-#1)

CALL F_CHECK_REG


/// DEST > R_WAVE
REG_WR hazard_dest imm #3

/// DEST > DMEM
REG_WR hazard_dest imm #4

REG_WR hazard_source imm #1
REG_WR r1 imm #5
DMEM_WR [&5] op -op(r1)
REG_WR r1 op -op(r1+#1)
DMEM_WR [r6] op -op(r1) -wr(hazard_source imm) #2
REG_WR r2 dmem [&0] -wr(r_gain imm) #7
DMEM_WR [r7] op -op(r_gain) -wr(r2 imm) #2
REG_WR r1 op -op(r2 ASR #1)


REG_WR hazard_source imm #3
REG_WR r_wave wmem [&0]
DMEM_WR [r7] op -op(r_freq)
REG_WR r_wave wmem [r1]
DMEM_WR [r7] op -op(r_freq)
REG_WR r_wave wmem [r2]
DMEM_WR [r7] op -op(r_freq)
DMEM_WR [r7] imm #7

REG_WR hazard_source imm #4

/// DEST > WMEM
REG_WR hazard_dest imm #5

REG_WR hazard_source imm #1

REG_WR hazard_source imm #2
REG_WR r_freq imm #-1
WMEM_WR [&10]
REG_WR r_freq op -op(r1+r2)
WMEM_WR [&11]
REG_WR r_freq dmem [&11]
WMEM_WR [r11]

REG_WR hazard_source imm #3
REG_WR hazard_source imm #4
REG_WR hazard_source imm #5

/// DEST > DPORT
REG_WR hazard_dest imm #6

REG_WR hazard_source imm #1
DPORT_WR p0 imm #1 -wr(r1 imm)
DPORT_WR p1 op -op(r1+#1) -wr(r2 op)
DPORT_WR p2 imm #3 -wr(r3 op) -op(r1+r2)
DPORT_WR p3 op -op(r1+r3) -wr(r4 imm) #4
REG_WR r1 imm #1
DPORT_WR p0 imm #1 -wr(r2 op) -op(r1+#1)
DPORT_WR p1 op -op(r2) -wr(r3 imm) #3
DPORT_WR p2 imm #3 -wr(r4 op) -op(r1+r3)
DPORT_WR p3 op -op(r1+r3) -wr(r5 imm) #5

REG_WR r1 imm #5
DPORT_WR p0 op -op(r1)
REG_WR r1 op -op(r1+#1)
DPORT_WR p0 op -wr(r1 op) -op(r1+#1)
REG_WR r2 dmem [&0] -wr(r_gain imm) #7
DPORT_WR p0 op -op(r_gain) -wr(hazard_source imm) #2
REG_WR r1 op -op(r2 SR #1)

REG_WR hazard_source imm #3
REG_WR hazard_source imm #4
DMEM_WR [&10] imm #5 -wr(r1 op) -op(r5+r4)
DPORT_WR p0 op -op(r3)

REG_WR hazard_source imm #5
WMEM_WR [&10] -wr(r1 op) -op(r5-r4)
DPORT_WR p0 op -op(r1)

/// DEST > WPORT
REG_WR hazard_dest imm #7

REG_WR hazard_source imm #1

REG_WR hazard_source imm #2
REG_WR r_freq imm #-1
WPORT_WR p0 r_wave
REG_WR r_freq op -op(r1+r2)
WPORT_WR p0 r_wave
REG_WR r_freq dmem [&11]
WPORT_WR p0 r_wave

REG_WR hazard_source imm #3
REG_WR r_wave wmem [&0]
WPORT_WR p0 r_wave
REG_WR r_wave wmem [r1]
WPORT_WR p0 r_wave
REG_WR r_wave wmem [r2]
WPORT_WR p0 r_wave




REG_WR r7 imm #0
REG_WR r7 op -op(r5+r6)
REG_WR r7 dmem [&0]



REG_WR hazard_source imm #255
START:
REG_WR r1 imm #-1
REG_WR r1 imm #0
REG_WR r1 op -op(r1+#1)
REG_WR r1 op -op(r1+#2)
REG_WR r1 op -op(r1+#4)
REG_WR r1 op -op(r1+#8)
REG_WR r1 op -op(r1+#16)
REG_WR r1 op -op(r1+#32)

DMEM:
REG_WR r15 imm #-1
DMEM_WR [&0] imm #-1 
DMEM_WR [&0] op -wr(r15 op) -op(s0)
DMEM_WR [&1] op -wr(r15 op) -op(r15 + #1)
DMEM_WR [&2] op -wr(r15 op) -op(r15 + #2)
DMEM_WR [&3] op -wr(r15 op) -op(r15 + #4)
DMEM_WR [&4] op -wr(r15 op) -op(r15 + #8)
DMEM_WR [&5] op -wr(r15 op) -op(r15 + #16)
DMEM_WR [&5] op -wr(r15 op) -op(r15 + #32)

WREG:
REG_WR r15 imm #-1
REG_WR r_wave wmem [&1] -wr(r15 op) -op(s0)
REG_WR r_wave wmem [&2] -wr(r15 op) -op(r15 + #1)
REG_WR r_wave wmem [&3] -wr(r15 op) -op(r15 + #2)
REG_WR r_wave wmem [&4] -wr(r15 op) -op(r15 + #4)
REG_WR r_wave wmem [&5] -wr(r15 op) -op(r15 + #8)
REG_WR r_wave wmem [&6] -wr(r15 op) -op(r15 + #16)
REG_WR r_wave wmem [&7] -wr(r15 op) -op(r15 + #32)
REG_WR r_wave wmem [&8] -wr(r14 op) -op(r15)
REG_WR r_wave wmem [&9] -wr(r13 op) -op(r15)
REG_WR r_wave wmem [&10] -wr(r12 op) -op(r15)




JUMP HERE

///// FUNCTIONS

F_FILL_REG:
   REG_WR r0 imm #255
   REG_WR r0 op -op(r0-#255)
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

   REG_WR s12 imm #12 // Write core_w1 Register
   REG_WR s13 imm #13 // Write core_w2 Register
   REG_WR s14 imm #14 // Write s_time Register
   REG_WR s15 imm #15 // Write s_addr Register

   REG_WR w_freq   imm   #1
   REG_WR w_phase  imm   #2
   REG_WR w_env    imm   #3
   REG_WR w_gain   imm   #4
   REG_WR w_length imm   #5
   REG_WR w_conf   imm   #6
   RET

F_CLEAR_REG:
   REG_WR r0  imm #0
   REG_WR r1  imm #0
   REG_WR r2  imm #0
   REG_WR r3  imm #0
   REG_WR r4  imm #4
   REG_WR r5  imm #0
   REG_WR r6  imm #0
   REG_WR r7  imm #0
   REG_WR r8  imm #0
   REG_WR r9  imm #0
   REG_WR r10 imm #0
   REG_WR r11 imm #0
   REG_WR r12 imm #0
   REG_WR r13 imm #0
   REG_WR r14 imm #0
   REG_WR r15 imm #0
   RET
   
F_FILL_MEM:
   REG_WR r1 op -op(s0+#15) -uf
   LOOP_FILL_MEM:
      DMEM_WR [r1]  op -op(r1)
   JUMP LOOP_FILL_MEM -if(NZ) -wr( r1 op ) -op(r1-#1) -uf
   RET

F_CHECK_MEM:
   REG_WR r1 op -op(s0+#15) -uf
   LOOP_CHECK_MEM:
      REG_WR r2 dmem [r1]
      TEST -op(r1 - r2)
      JUMP HERE -if(NZ)
      REG_WR r1 op -op(r1-#1) -uf
   JUMP LOOP_CHECK_MEM -if(NZ)
   RET   

F_CHECK_REG:
   REG_WR r0 imm #1 -op(r1-#1) -uf
   REG_WR r0 imm #2 -op(r2-#2) -uf -if(Z)
   REG_WR r0 imm #3 -op(r3-#3) -uf -if(Z)
   REG_WR r0 imm #4 -op(r4-#4) -uf -if(Z)
   REG_WR r0 imm #5 -op(r5-#5) -uf -if(Z)
   REG_WR r0 imm #6 -op(r6-#6) -uf -if(Z)
   REG_WR r0 imm #7 -op(r7-#7) -uf -if(Z)
   REG_WR r0 imm #8 -op(r8-#8) -uf -if(Z)
   REG_WR r0 imm #9 -op(r9-#9) -uf -if(Z)
   REG_WR r0 imm #10 -op(r10-#10) -uf -if(Z)
   REG_WR r0 imm #11 -op(r11-#11) -uf -if(Z)
   REG_WR r0 imm #12 -op(r12-#12) -uf -if(Z)
   REG_WR r0 imm #13 -op(r13-#13) -uf -if(Z)
   REG_WR r0 imm #14 -op(r14-#14) -uf -if(Z)
   REG_WR r0 imm #15 -op(r15-#15) -uf -if(Z)
   JUMP HERE -if(NZ)
   RET
   