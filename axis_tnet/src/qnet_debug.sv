`include "_qnet_defines.svh"

module qnet_cmd_dbg (
   input  wire             st_clk_i      ,
   input  wire             st_rst_ni     ,
   input  TYPE_QNET_CMD    current_st_i  ,
   input  TYPE_QNET_CMD    next_st_i     ,
   output reg  [31:0]      debug_dt_o    
);

// Command Execution
///////////////////////////////////////////////////////////////////////////////
reg [5:0] param_LT, param_LT_r, param_LT_r2, param_LT_r3, param_LT_r4 ;
reg [5:0] debug_dt;

assign st_change = current_st_i != next_st_i ;

// COMMAND STATE OUT
always_comb begin
   case (next_st_i)
      NOT_READY        : debug_dt = 0 ;
      IDLE             : debug_dt = 1 ;
      LOC_GNET         : debug_dt = 2 ;
      LOC_SNET         : debug_dt = 3 ;
      LOC_SYNC         : debug_dt = 4 ;
      LOC_GET_OFF      : debug_dt = 5 ;
      LOC_UPDT_OFF     : debug_dt = 6 ;
      LOC_SET_DT       : debug_dt = 7 ;
      LOC_GET_DT       : debug_dt = 8 ;
      LOC_RST_PROC     : debug_dt = 9 ;
      LOC_START_CORE   : debug_dt = 10 ;
      LOC_STOP_CORE    : debug_dt = 11 ;
      NET_GNET_P       : debug_dt = 12 ;
      NET_SNET_P       : debug_dt = 13 ;
      NET_SYNC_P       : debug_dt = 14 ;
      NET_GET_OFF_P    : debug_dt = 15 ;
      NET_UPDT_OFF_P   : debug_dt = 16 ;
      NET_SET_DT_P     : debug_dt = 17 ;
      NET_GET_DT_P     : debug_dt = 18 ;
      NET_RST_PROC_P   : debug_dt = 20 ;
      NET_START_CORE_P : debug_dt = 20 ;
      NET_STOP_CORE_P  : debug_dt = 21 ;
      NET_GNET_R       : debug_dt = 22 ;
      NET_SNET_R       : debug_dt = 23 ;
      NET_SYNC_R       : debug_dt = 24 ;
      NET_UPDT_OFF_R   : debug_dt = 25 ;
      NET_SET_DT_R     : debug_dt = 26 ;
      NET_GET_DT_R     : debug_dt = 27 ;
      NET_RST_PROC_R   : debug_dt = 28 ;
      NET_START_CORE_R : debug_dt = 29 ;
      NET_STOP_CORE_R  : debug_dt = 30 ;
      NET_GET_OFF_A    : debug_dt = 31 ;
      NET_GET_DT_A     : debug_dt = 32 ;
      WAIT_TX_ACK      : debug_dt = 33 ;
      WAIT_TX_nACK     : debug_dt = 34 ;
      WAIT_CMD_nACK    : debug_dt = 35 ;
      ST_ERROR         : debug_dt = 63 ;
   endcase
end

// DEBUG
always_ff @(posedge st_clk_i)
   if (!st_rst_ni) begin
      param_LT     <=  6'd0;
      param_LT_r   <=  6'd62;
      param_LT_r2  <=  6'd62;
      param_LT_r3  <=  6'd62;
      param_LT_r4  <=  6'd62;
   end else begin
      if (st_change ) begin
         param_LT     <= debug_dt; 
         param_LT_r   <= param_LT; 
         param_LT_r2  <= param_LT_r; 
         param_LT_r3  <= param_LT_r2 ;
         param_LT_r4  <= param_LT_r3;
      end
   end

assign debug_dt_o  = {param_LT, param_LT_r, param_LT_r2, param_LT_r3, param_LT_r4 } ;
   
endmodule



