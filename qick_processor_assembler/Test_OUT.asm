//TEST program
.ALIAS repeat_cnt r0
.ALIAS pulse_env r1
.ALIAS pulse_w   r2
.ALIAS pulse_p   r3

INIT:
   REG_WR pulse_env imm #1000
   REG_WR pulse_w imm #100
   REG_WR pulse_p op -op(pulse_env SR #1)

REG_WR repeat_cnt op -op(zero+#100) -uf
TRIG_LOOP:
   // ENVELOPE
    REG_WR r_time imm #100
    TRIG p0 set
    REG_WR r_time op -op(r_time+pulse_env)
    TRIG p0 clr
   // CENTRAL PULSE
    REG_WR r_time op -op(pulse_p+#100)
    TRIG p1 set
    REG_WR r_time op -op(r_time+pulse_w)
    TRIG p1 clr

    //REG_WR pulse1_time op -op(pulse1_time+#250)

   JUMP TRIG_LOOP -wr(repeat_cnt op) -op(repeat_cnt-#1) -if(NZ) -uf

TEST -op(tuser - r_time)
JUMP PREV -if(S)

TIME inc_ref r_time

JUMP INIT
END: 
    JUMP END