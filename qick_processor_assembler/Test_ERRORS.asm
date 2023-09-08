//TEST program

.ALIAS pulse_cnt      r0
.ALIAS number_waves   r1
.ALIAS time_btw_waves r2
.ALIAS mem_addr       r3
.ALIAS ind_rep        r4

REG_WR r_time imm #100

DPORT_WR p0 op -op(zero) 
DPORT_WR p0 imm #0
DPORT_WR p0 imm #0 @100
DPORT_WR p0 op -op(zero) @100
WPORT_WR p1 r_wave @100
