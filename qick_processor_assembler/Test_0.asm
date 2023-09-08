//TEST program
.ALIAS repeat_cnt r0
.ALIAS pulse_inc  r5
.ALIAS aux       r15

.ALIAS pulse_start  r4
.ALIAS pulse_moving_pos   r3

.CONST pulse_envelope_w  #100
.CONST pulse_central_pos  #50
.CONST pulse_central_w    #5
.CONST pulse_moving_w     #5

.CONST total_repeat #1000

.CONST velocity #1

DPORT_WR p0 imm 0
DPORT_WR p1 imm 1 @100
DPORT_WR p2 imm 2  -wr(r1 op) -op(r1+#2)
DPORT_WR p0 reg r3
DPORT_WR p1 reg r4 -wr(r1 imm) #2
DPORT_WR p2 reg r5  -wr(r1 op) -op(r1+r2)

TRIG p1 set -wr(r1 imm) #2
TRIG p1 clr -wr(r1 op) -op(r1+#2)
TRIG p1 set @100
TRIG p1 clr @150
TRIG p1 set

WPORT_WR p0 r_wave
DPORT_WR p3 reg r5
DPORT_WR p0 imm 5

REG_WR r0 dmem [r0+&10]
DMEM_WR [r0+&10] imm #5

INIT:
   REG_WR pulse_start imm #0

REG_WR repeat_cnt op -op(zero + total_repeat) -uf



TRIG_LOOP:
   // ENVELOPE RISE
    REG_WR out_usr_time op -op(pulse_start)
        TRIG p0 set
        TRIG p0 clr
        REG_WR out_usr_time op -op(out_usr_time + #1)
        TRIG p1 set
        TRIG p1 clr
        REG_WR out_usr_time op -op(out_usr_time + #1)
        TRIG p7 set
        TRIG p7 clr
        REG_WR out_usr_time op -op(out_usr_time+pulse_moving_w)
        TRIG p2 clr
   // ENVELOPE FALL
    REG_WR out_usr_time op -op(pulse_start+pulse_envelope_w)
    TRIG p0 clr -wr(pulse_start op) -op(out_usr_time+pulse_envelope_w)
   JUMP TRIG_LOOP -wr(repeat_cnt op) -op(repeat_cnt-#1) -if(NZ) -uf

END: 
    JUMP END