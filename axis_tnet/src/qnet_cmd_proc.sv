`include "_qnet_defines.svh"

module qnet_cmd_proc # (
   parameter DEBUG     = 1
)(
   input  wire             t_clk_i          ,
   input  wire             t_rst_ni         ,
   input  wire             ctrl_rst_i       ,
// External Signaling
   input  wire             aurora_ready_i   ,
   input  wire             t_ready_t01      ,
   input TYPE_QPARAM       param_i          ,
   input wire [31:0]       qnet_T_LINK      ,
  
   input wire [31:0]       qnet_dt_i[2]     ,
   input  wire [31:0]      t_time_abs       ,
   output  wire            c_ready_o        ,
// Command Control
   input   wire            loc_cmd_req_i    ,
   input   wire            net_cmd_req_i    ,
   input  wire [63:0]      cmd_header_i     ,
   input  wire [31:0]      cmd_dt_i [2]     ,
   output  wire            loc_cmd_ack_o    ,
   output  wire            net_cmd_ack_o    ,
  
// Update Parameter   
   output                 tx_req_set_o      ,
   output TYPE_PARAM_WE   param_we          ,
   output reg  [31:0]     param_64_dt [2]   ,
   output reg  [31:0]     param_32_dt       ,
   output reg  [ 9:0]     param_10_dt       ,
// Transmit Info
   output  reg            tx_req_o          ,
   output  reg [63:0]     tx_cmd_header_o   ,
   output  reg [31:0]     tx_cmd_dt_o[2]    ,
   input   wire           tx_ack_i          ,
// Control tProc
   output  reg            time_reset_o       ,
   output  reg            time_init_o        ,
   output  reg            time_updt_o        ,
// Debug
   output  wire [31:0]    cmd_st_do 
  );


wire net_ctrl_ack ;

wire t_ready_t3rd;
wire proc_step_3;

reg cmd_end        ;
reg cmd_error      ;



// Network Information
///////////////////////////////////////////////////////////////////////////////



// Command Decoding
///////////////////////////////////////////////////////////////////////////////

wire [ 1:0] cmd_type;
wire [ 4:0] cmd_id;
wire [ 2:0] cmd_flg;
wire [ 9:0] cmd_dst, cmd_src, cmd_step;
wire [23:0] cmd_hdt;

assign cmd_type = cmd_header_i[63:62];
assign cmd_id   = cmd_header_i[61:57];
assign cmd_flg  = cmd_header_i[56:54];
assign cmd_dst  = cmd_header_i[53:44];
assign cmd_src  = cmd_header_i[43:34];
assign cmd_step = cmd_header_i[33:24];
assign cmd_hdt  = cmd_header_i[23: 0];

assign net_src_hit = (cmd_src == param_i.ID) ;
assign net_dst_hit = (cmd_dst == param_i.ID) ;
assign is_ret      = net_src_hit &  cmd_flg[1] ;
assign is_ans      = net_dst_hit & ~cmd_flg[0];

reg id_get_net, id_set_net ;
reg id_sync_1, id_sync_2, id_sync_3, id_sync_4, id_sync_5 ;
reg id_updt_off, id_get_dt, id_set_dt ;
reg id_rst_time, id_start_core, id_stop_core ;
reg id_get_cond, id_set_cond ;

always_comb begin
   id_get_net    = 1'b0;
   id_set_net    = 1'b0;
   id_sync_1     = 1'b0;
   id_sync_2     = 1'b0;
   id_sync_3     = 1'b0;
   id_sync_4     = 1'b0;
   id_sync_5     = 1'b0;
   id_updt_off   = 1'b0;
   id_get_dt     = 1'b0;
   id_set_dt     = 1'b0;
   id_rst_time   = 1'b0;
   id_start_core = 1'b0;
   id_stop_core  = 1'b0;
   id_get_cond   = 1'b0;
   id_set_cond   = 1'b0;
   if      (cmd_id == _get_net     ) id_get_net     = 1'b1 ;
   else if (cmd_id == _set_net     ) id_set_net     = 1'b1 ;
   else if (cmd_id == _sync1_net   ) id_sync_1      = 1'b1 ;
   else if (cmd_id == _sync2_net   ) id_sync_2      = 1'b1 ;
   else if (cmd_id == _sync3_net   ) id_sync_3      = 1'b1 ;
   else if (cmd_id == _sync4_net   ) id_sync_4      = 1'b1 ;
   else if (cmd_id == _sync5_net   ) id_sync_5      = 1'b1 ;
   else if (cmd_id == _updt_off    ) id_updt_off    = 1'b1 ;
   else if (cmd_id == _get_dt      ) id_get_dt      = 1'b1 ;
   else if (cmd_id == _set_dt      ) id_set_dt      = 1'b1 ;
   else if (cmd_id == _rst_time    ) id_rst_time   = 1'b1 ;
   else if (cmd_id == _start_core  ) id_start_core  = 1'b1 ;
   else if (cmd_id == _stop_core   ) id_stop_core   = 1'b1 ;
   else if (cmd_id == _set_cond    ) id_set_cond    = 1'b1 ;
end  


// Comunicacion Syncronization
reg loc_cmd_ack, loc_cmd_ack_set, loc_cmd_ack_clr  ;
reg net_cmd_ack, net_cmd_ack_set, net_cmd_ack_clr  ; 

always_ff @(posedge t_clk_i) 
   if (!t_rst_ni) begin
      loc_cmd_ack   <= 1'b0;
      net_cmd_ack   <= 1'b0;
   end else begin 
      if (loc_cmd_ack_clr | time_out | ctrl_rst_i ) loc_cmd_ack <= 1'b0;
      else if (loc_cmd_ack_set)                     loc_cmd_ack <= 1'b1;
      if (net_cmd_ack_clr | time_out | ctrl_rst_i ) net_cmd_ack <= 1'b0;
      else if (net_cmd_ack_set)                     net_cmd_ack <= 1'b1;
   end

wire cmd_ack;
assign cmd_ack = loc_cmd_ack | net_cmd_ack;


reg [63:0] tx_cmd_header_r, tx_cmd_dt_r;



// TIMEOUT
///////////////////////////////////////////////////////////////////////////////
wire      time_out_cnt_en    ;
reg [9:0] time_out_cnt;

always_ff @ (posedge t_clk_i, negedge t_rst_ni) begin
   if (!t_rst_ni) begin
      time_out_cnt      <= 0 ;
   end else
      if ( time_out_cnt_en ) begin
         if ( t_ready_t01  ) 
            time_out_cnt  <= time_out_cnt + 1'b1; 
      end else
            time_out_cnt  <= 0;
end

assign time_out_cnt_en  = ~main_idle;
assign time_out         = time_out_cnt[9];







///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///// MAIN STATE
///////////////////////////////////////////////////////////////////////////////
enum {
   M_NOT_READY = 0, 
   M_IDLE      = 1,
   LOC_CMD     = 2,
   NET_CMD     = 3,
   M_WRESP     = 4,
   M_WACK      = 5,
   NET_RESP    = 6,
   NET_ACK     = 7,
   M_ERROR     = 8 
} main_st_nxt, main_st;

always_ff @(posedge t_clk_i)
   if      ( !t_rst_ni   )  main_st  <= M_NOT_READY;
   else if (  ctrl_rst_i )  main_st  <= M_NOT_READY;
   else                     main_st  <= main_st_nxt;

// State Change
always_comb begin
   main_st_nxt  = main_st; // Default Current
   if (!aurora_ready_i | time_out)  
      main_st_nxt = M_NOT_READY;
   else
      case (main_st)
         M_NOT_READY :  if (aurora_ready_i)        main_st_nxt  = M_IDLE;
         M_IDLE      :  if       (loc_cmd_req_i )  main_st_nxt  = LOC_CMD;
                        else if  (net_cmd_req_i )  main_st_nxt  = NET_CMD;
         LOC_CMD     :  if (cmd_end     ) begin
                           if      ( cmd_error  )  main_st_nxt = M_ERROR;
                           else if ( cmd_flg[2] )  main_st_nxt = M_WRESP;
                           else if ( cmd_flg[1] )  main_st_nxt = M_WACK ;
                           else                    main_st_nxt = M_IDLE ;
                        end
         NET_CMD     :  if ( cmd_end        )      main_st_nxt = M_IDLE ;
         M_WRESP     :  if ( net_cmd_req_i  )      main_st_nxt = NET_RESP;
         M_WACK      :  if ( net_cmd_req_i  )      main_st_nxt = M_IDLE ;
         NET_RESP    :  if ( cmd_end        )      main_st_nxt = M_IDLE ;
         NET_ACK     :  if ( cmd_end        )      main_st_nxt = M_IDLE ;
         M_ERROR     :                             main_st_nxt = M_IDLE ;
      endcase
end

// State Outputs
assign wfr        = (main_st == NET_RESP);
assign main_idle  = (main_st == M_IDLE);



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///// TASK STATE
///////////////////////////////////////////////////////////////////////////////
enum {
   T_NOT_READY =0, 
   T_IDLE      =1, 
   T_LOC_CMD   =2,
   T_LOC_WSYNC =3,
   T_LOC_SEND  =4,
   T_LOC_WnREQ =5, 
   T_NET_CMD   =6, 
   T_NET_WSYNC =7, 
   T_NET_SEND  =8, 
   T_NET_WnREQ =9 
}task_st_nxt, task_st;

always_ff @(posedge t_clk_i)
   if      ( !t_rst_ni   )  task_st  <= T_NOT_READY;
   else if (  ctrl_rst_i )  task_st  <= T_NOT_READY;
   else                     task_st  <= task_st_nxt;

// Outputs and State Change
always_comb begin
   task_st_nxt      = task_st; // Default Current
   loc_cmd_ack_set  = 1'b0 ;
   loc_cmd_ack_clr  = 1'b0 ;
   net_cmd_ack_set  = 1'b0 ; 
   net_cmd_ack_clr  = 1'b0 ;

   if (!aurora_ready_i | time_out)  
      task_st_nxt = T_NOT_READY;
      case (task_st)
         T_NOT_READY : begin
            if (aurora_ready_i)  task_st_nxt = T_IDLE;
         end
         T_IDLE      : begin
            if (loc_cmd_req_i) begin
               loc_cmd_ack_set  = 1'b1 ;
               task_st_nxt      = T_LOC_CMD;
            end
            else if (net_cmd_req_i) begin
               net_cmd_ack_set  = 1'b1 ; 
               task_st_nxt      = T_NET_CMD;
            end
         end
   // LOCAL COMMAND
         T_LOC_CMD : begin
            if      ( tx_req_o  )     task_st_nxt = T_LOC_SEND;
            else if ( cmd_end )     task_st_nxt = T_LOC_WnREQ;
         end
         T_LOC_SEND : begin
            if      ( cmd_end )     task_st_nxt = T_LOC_WnREQ;
         end
         T_LOC_WnREQ : begin
            if ( !loc_cmd_req_i )  begin
               loc_cmd_ack_clr = 1'b1;
               task_st_nxt = T_IDLE;
            end
         end
   // NETWORK COMMAND
         T_NET_CMD : begin
            if      ( tx_req_o  )     task_st_nxt = T_NET_SEND;
            else if ( cmd_end )     task_st_nxt = T_NET_WnREQ;
         end
         T_NET_WSYNC : begin
            if ( t_ready_t01 )      task_st_nxt  = T_NET_SEND;
         end
         T_NET_SEND : begin
            if (!tx_ack_i & !tx_req_o) task_st_nxt = T_NET_WnREQ;
         end
         T_NET_WnREQ : begin
            if (!net_cmd_req_i) begin
               net_cmd_ack_clr = 1'b1;
               task_st_nxt = T_IDLE;
            end
         end
      endcase
end






///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///// COMMAND EXECUTION STATE
///////////////////////////////////////////////////////////////////////////////


TYPE_QNET_CMD cmd_st_nxt, cmd_st;

always_ff @(posedge t_clk_i)
   if      (!t_rst_ni)   cmd_st  <= NOT_READY;
   else if ( ctrl_rst_i) cmd_st  <= NOT_READY;
   else                  cmd_st  <= cmd_st_nxt;

// COMMAND STATE CHANGE
always_comb begin
   cmd_st_nxt      = cmd_st; // Default Current
   if (!aurora_ready_i )
      cmd_st_nxt    = NOT_READY;
   else if ( time_out)
      cmd_st_nxt    = ST_ERROR;
   else
      case (cmd_st)
         NOT_READY: if (aurora_ready_i)  cmd_st_nxt = IDLE;
         IDLE: begin
            // GET_NET
            if      (id_get_net & loc_cmd_ack                 )  cmd_st_nxt  = LOC_GNET;
            else if (id_get_net & net_cmd_ack & !wfr          )  cmd_st_nxt  = NET_GNET_P;
            else if (id_get_net & net_cmd_ack & wfr & is_ret  )  cmd_st_nxt  = NET_GNET_R;
            else if (id_get_net & net_cmd_ack & wfr & !is_ret )  cmd_st_nxt  = ST_ERROR;
            // SET_NET
            else if (id_set_net & loc_cmd_ack                 )  cmd_st_nxt = LOC_SNET;
            else if (id_set_net & net_cmd_ack & !wfr          )  cmd_st_nxt = NET_SNET_P;
            else if (id_set_net & net_cmd_ack & wfr & is_ret  )  cmd_st_nxt = NET_SNET_R;
            else if (id_set_net & net_cmd_ack & wfr & !is_ret )  cmd_st_nxt = ST_ERROR;
            // SYNC1
            else if (id_sync_1 & loc_cmd_ack                  ) cmd_st_nxt = LOC_SYNC1;
            else if (id_sync_1 & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_SYNC1_P;
            else if (id_sync_1 & net_cmd_ack & wfr & is_ret   ) cmd_st_nxt = NET_SYNC1_R;
            else if (id_sync_1 & net_cmd_ack & wfr & !is_ret  ) cmd_st_nxt = ST_ERROR;
            // SYNC2
            else if (id_sync_2 & loc_cmd_ack                  ) cmd_st_nxt = LOC_SYNC2;
            else if (id_sync_2 & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_SYNC2_P;
            else if (id_sync_2 & net_cmd_ack & wfr & is_ret   ) cmd_st_nxt = NET_SYNC2_R;
            else if (id_sync_2 & net_cmd_ack & wfr & !is_ret  ) cmd_st_nxt = ST_ERROR;
            // SYNC3
            else if (id_sync_3 & loc_cmd_ack                   ) cmd_st_nxt = LOC_SYNC3;
            else if (id_sync_3 & net_cmd_ack & !wfr            ) cmd_st_nxt = NET_SYNC3_P;
            else if (id_sync_3 & net_cmd_ack & wfr & is_ret    ) cmd_st_nxt = NET_SYNC3_R;
            else if (id_sync_3 & net_cmd_ack & wfr & !is_ret   ) cmd_st_nxt = ST_ERROR;
            // SYNC4
            else if (id_sync_4 & loc_cmd_ack                   ) cmd_st_nxt = LOC_SYNC4;
            else if (id_sync_4 & net_cmd_ack & !wfr            ) cmd_st_nxt = NET_SYNC4_P;
            else if (id_sync_4 & net_cmd_ack & wfr & is_ret    ) cmd_st_nxt = NET_SYNC4_R;
            else if (id_sync_4 & net_cmd_ack & wfr & !is_ret   ) cmd_st_nxt = ST_ERROR;
            // UPDATE_OFFSET  
            else if (id_updt_off & loc_cmd_ack                 ) cmd_st_nxt = LOC_UPDT_OFF;
            else if (id_updt_off & net_cmd_ack & !wfr          ) cmd_st_nxt = NET_UPDT_OFF_P;
            else if (id_updt_off & net_cmd_ack & wfr & is_ret  ) cmd_st_nxt = NET_UPDT_OFF_R;
            else if (id_updt_off & net_cmd_ack & wfr & !is_ret ) cmd_st_nxt = ST_ERROR;
            // SET_DT   
            else if (id_set_dt & loc_cmd_ack                   ) cmd_st_nxt = LOC_SET_DT;
            else if (id_set_dt & net_cmd_ack & !wfr            ) cmd_st_nxt = NET_SET_DT_P;
            else if (id_set_dt & net_cmd_ack & wfr & is_ret    ) cmd_st_nxt = NET_SET_DT_R;
            else if (id_set_dt & net_cmd_ack & wfr & !is_ret   ) cmd_st_nxt = ST_ERROR;
            // GET_DT   
            else if (id_get_dt & loc_cmd_ack                   ) cmd_st_nxt = LOC_GET_DT;
            else if (id_get_dt & net_cmd_ack & !wfr            ) cmd_st_nxt = NET_GET_DT_P;
            else if (id_get_dt & net_cmd_ack & wfr & is_ret    ) cmd_st_nxt = NET_GET_DT_R;
            else if (id_get_dt & net_cmd_ack & wfr & is_ans    ) cmd_st_nxt = NET_GET_DT_A;
            else if (id_get_dt & net_cmd_ack & wfr & !is_ret   ) cmd_st_nxt = ST_ERROR;
            // RESET_TIME
            else if (id_rst_time & loc_cmd_ack & ~net_ctrl_ack ) cmd_st_nxt = LOC_RST_PROC;
            else if (id_rst_time & net_cmd_ack & !wfr          ) cmd_st_nxt = NET_RST_PROC_P;
            else if (id_rst_time & net_cmd_ack & wfr & is_ret  ) cmd_st_nxt = NET_RST_PROC_R;
            else if (id_rst_time & net_cmd_ack & wfr & !is_ret ) cmd_st_nxt = ST_ERROR;
            // START_CORE
            else if (id_start_core & loc_cmd_ack & ~net_ctrl_ack ) cmd_st_nxt = LOC_START_CORE;
            else if (id_start_core & net_cmd_ack & !wfr          ) cmd_st_nxt = NET_START_CORE_P;
            else if (id_start_core & net_cmd_ack & wfr & is_ret  ) cmd_st_nxt = NET_START_CORE_R;
            else if (id_start_core & net_cmd_ack & wfr & !is_ret ) cmd_st_nxt = ST_ERROR;
            // STOP_CORE   
            else if (id_stop_core & loc_cmd_ack & ~net_ctrl_ack  ) cmd_st_nxt = LOC_STOP_CORE;
            else if (id_stop_core & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_STOP_CORE_P;
            else if (id_stop_core & net_cmd_ack & wfr & is_ret   ) cmd_st_nxt = NET_STOP_CORE_R;
            else if (id_stop_core & net_cmd_ack & wfr & !is_ret  ) cmd_st_nxt = ST_ERROR;
            // OTHER 
            else if (loc_cmd_ack | net_cmd_ack) cmd_st_nxt  = ST_ERROR;
         end
         ST_ERROR: cmd_st_nxt    = WAIT_TX_nACK;
   // LOCAL COMMAND
         LOC_GNET       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SNET       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SYNC1      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SYNC2      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SYNC3      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SYNC4      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_GET_OFF    : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_UPDT_OFF   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SET_DT     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_GET_DT     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_RST_PROC   : if (t_ready_t01) begin if (net_ctrl_ack) cmd_st_nxt = WAIT_TX_ACK; else cmd_st_nxt =  ST_ERROR; end
         LOC_START_CORE : if (t_ready_t01) begin if (net_ctrl_ack) cmd_st_nxt = WAIT_TX_ACK; else cmd_st_nxt =  ST_ERROR; end
         LOC_STOP_CORE  : if (t_ready_t01) begin if (net_ctrl_ack) cmd_st_nxt = WAIT_TX_ACK; else cmd_st_nxt =  ST_ERROR; end
   // NET Command Process
         NET_GNET_P       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SNET_P       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SYNC1_P      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SYNC2_P      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SYNC3_P      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SYNC4_P      : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_GET_OFF_P    : if (t_ready_t3rd) cmd_st_nxt = WAIT_TX_ACK;
         NET_UPDT_OFF_P   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SET_DT_P     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_GET_DT_P     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_RST_PROC_P   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_START_CORE_P : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_STOP_CORE_P  : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
   // NET Command Return Process
         NET_GNET_R       :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SNET_R       :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SYNC1_R      :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SYNC2_R      :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SYNC3_R      :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SYNC4_R      :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_UPDT_OFF_R   :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SET_DT_R     :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_GET_DT_R     :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_RST_PROC_R   :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_START_CORE_R :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_STOP_CORE_R  :                  cmd_st_nxt =  WAIT_CMD_nACK;
   // NET ANSWER Process
         NET_GET_OFF_A    :  if (proc_step_3)    cmd_st_nxt =  WAIT_CMD_nACK;
         NET_GET_DT_A     :                  cmd_st_nxt =  WAIT_CMD_nACK;
   // WAIT FOR SYNC
         WAIT_TX_ACK   : if ( tx_ack_i )    cmd_st_nxt = WAIT_TX_nACK;
         WAIT_TX_nACK  : if (!tx_ack_i )    cmd_st_nxt = WAIT_CMD_nACK;
         WAIT_CMD_nACK : if (!cmd_ack)    cmd_st_nxt = IDLE;
      endcase
end




///////////////////////////////////////////////////////////////////////////////



reg get_time_lcs   ;
reg tx_req_set     ;

reg time_reset     ;
reg time_init      ;
reg time_updt      ;
reg [31: 0]   net_ctrl_time  ;
reg [2:0] net_ctrl_id;

reg [63:0] tx_cmd_header;
reg[31:0] tx_cmd_dt [2];

// assign get_time_lcs  = t_ready_t01 & 

//OUTPUTS

always_comb begin
   param_we       = '{default:'0};
   param_64_dt    = '{default:'0};
   param_32_dt    = 32'd0;
   param_10_dt    = 10'd0;
   get_time_lcs   = 1'b0;

// Processor Control
   time_reset     = 1'b0;
   time_init      = 1'b0;
   time_updt      = 1'b0;
   net_ctrl_id    = 0 ; // Network Command (rst, start, stop)
   net_ctrl_time  = 0 ; // Network Command Time to be executed

   cmd_end        = 1'b0;
   cmd_error      = 1'b0;

   tx_req_set     = 1'b0 ;
   tx_cmd_dt      = '{default:'0};
   tx_cmd_header  = 0;

   case (cmd_st)
      NOT_READY: begin
      end
      IDLE: begin
         if (id_get_net & net_cmd_ack & !wfr )  begin // (cmd_st_nxt  == NET_GNET_P) 
            time_reset  = 1'b1 ;
         end
         if (id_sync_1 & net_cmd_ack & !wfr ) begin //  (cmd_st_nxt  == NET_SYNC_P)
            time_init      = 1'b1 ;
            param_we.OFF   = 1'b1 ;
            param_32_dt    = cmd_dt_i[0] + cmd_dt_i[1] ;
         end
      end
      ST_ERROR: begin
         cmd_error     = 1'b1;
         cmd_end = 1'b1 ;
      end
// COMMAND DATA      
      LOC_GNET: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = 0;
         param_we.ID     = 1'b1;
         param_10_dt  = 9'd1;
         // Command wait for answer
         if (t_ready_t01) begin
            time_reset  = 1'b1 ;
            tx_req_set  = 1'b1 ;
         end
      end
      LOC_SNET: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt[1]  = param_i.RTD;
         tx_cmd_dt[0]  = qnet_T_LINK;
         // Command wait for answer
         if (t_ready_t01) begin
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_SYNC1: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = qnet_T_LINK;
         // Command wait for answer
         if (t_ready_t01) begin
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_SYNC2: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = qnet_T_LINK;
         // Command wait for answer
         if (t_ready_t01) begin
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      
      
      LOC_GET_OFF: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i;
         if (t_ready_t01) begin
            get_time_lcs  = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_UPDT_OFF: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i;
         if (t_ready_t01) begin
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_SET_DT: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
         end
      end
      LOC_GET_DT: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = '{default:'0};
         if (t_ready_t01) begin
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_RST_PROC: begin
         net_ctrl_id    = 3'b001;
         net_ctrl_time  = {cmd_dt_i[0][15:0] , cmd_dt_i[1] };
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end
      LOC_START_CORE: begin
         net_ctrl_id    = 3'b010;
         net_ctrl_time  = {cmd_dt_i[0][15:0] , cmd_dt_i[1] };
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end
      LOC_STOP_CORE: begin
         net_ctrl_id    = 3'b100;
         net_ctrl_time  = {cmd_dt_i[0][15:0] , cmd_dt_i[1] };
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end

// PROCESS AND PROPAGATE COMMAND
      NET_GNET_P: begin 
         param_we.ID      = 1'b1  ;
         param_10_dt   = cmd_header_i[9:0];
         tx_cmd_header = cmd_header_i + 1'b1;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = 0;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
         end
      end
         
      NET_SNET_P: begin   // PROCESS AND PROPAGATE COMMAND
         param_we.RTD   = 1'b1  ;
         param_32_dt    = cmd_dt_i[1];
         param_we.NN    = 1'b1  ;
         param_10_dt    = cmd_header_i[9:0];
         tx_cmd_header  = cmd_header_i;
         tx_cmd_dt      =  cmd_dt_i ;
         if (t_ready_t01) begin 
            tx_req_set = 1'b1 ;
         end
      end
      NET_SYNC1_P: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = cmd_dt_i[0]+qnet_T_LINK;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
         end            
      end
      NET_SYNC2_P: begin
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = cmd_dt_i[0]+qnet_T_LINK;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
         end            
      end
      NET_GET_OFF_P: begin
         tx_cmd_header   = {14'b100_00101_100001, cmd_header_i[39:30] , param_i.ID, 30'b0000000000_0000000000_0000000000};
         tx_cmd_dt[1]  = param_i.T_NCR ;
         tx_cmd_dt[0]  = t_time_abs;
         if (t_ready_t3rd) begin
            tx_req_set     = 1'b1 ;
         end
      end
      NET_UPDT_OFF_P: begin
         param_we.OFF     = 1'b1 ;
         param_32_dt  = cmd_dt_i[0];
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i ;
         if (t_ready_t01) begin
            time_updt      = 1'b1 ;         
            tx_req_set     = 1'b1 ;
         end
      end
      NET_SET_DT_P: begin
         cmd_end        = 1'b1;
         param_we      = 1'b1 ;
         param_64_dt   = cmd_dt_i;
         tx_cmd_header  = cmd_header_i;
         tx_cmd_dt      = cmd_dt_i ;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
         end
      end
      NET_GET_DT_P: begin
         tx_cmd_header   = {14'b100_01011_000001, cmd_header_i[39:30] , param_i.ID, 30'b0000000000_0000000000_0000000000};
         tx_cmd_dt        = qnet_dt_i;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
         end
      end
      NET_RST_PROC_P: begin
         net_ctrl_id    = 3'b001;
         net_ctrl_time  = {cmd_dt_i[0][15:0] , cmd_dt_i[1] };
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end
      NET_START_CORE_P: begin
         net_ctrl_id    = 3'b010;
         net_ctrl_time  = {cmd_dt_i[0][15:0] , cmd_dt_i[1] };
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end
      NET_STOP_CORE_P: begin
         net_ctrl_id    = 3'b100;
         net_ctrl_time  = {cmd_dt_i[0][15:0] , cmd_dt_i[1] };
         tx_cmd_header = cmd_header_i;
         tx_cmd_dt     = cmd_dt_i;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end


// COmmand RETURN  
      NET_GNET_R: begin
                  param_we.RTD     = 1'b1  ;
            param_32_dt  = t_time_abs[31:0];
            param_we.NN      = 1'b1  ;
            param_10_dt   = cmd_header_i[9:0];
         cmd_end        = 1'b1;
      end
      NET_SNET_R     : begin
         cmd_end        = 1'b1;
      end
      NET_SYNC1_R     : begin 
         cmd_end        = 1'b1;
      end
      NET_SYNC2_R     : begin 
         cmd_end        = 1'b1;
      end
      NET_UPDT_OFF_R : begin 
         cmd_end        = 1'b1;
      end
      NET_SET_DT_R   : begin 
         cmd_end        = 1'b1;
      end
      NET_GET_DT_R: begin
         cmd_end        = 1'b1;
      end
      NET_RST_PROC_R: begin
         cmd_end        = 1'b1;
      end
      NET_START_CORE_R: begin
         cmd_end        = 1'b1;
      end
      NET_STOP_CORE_R: begin
         cmd_end        = 1'b1;
      end

// ANSWERS
      NET_GET_OFF_A: begin
         if (proc_step_3) begin
            cmd_end        = 1'b1;
         end
      
         cmd_end         = 1'b1;
         param_we.DT       = 1'b1 ;
         param_64_dt[0]   = 0;
         param_64_dt[1]   = 0;
         
      end
      NET_GET_DT_A: begin
         cmd_end        = 1'b1;
         param_we      = 1'b1 ;
         param_64_dt   = cmd_dt_i;
      end
      WAIT_TX_nACK: begin
         if (!tx_ack_i) begin 
            cmd_end = 1'b1;
         end
      end
//DEBUG      
      WAIT_TX_ACK   : begin
         if ( tx_ack_i ) begin
         end
      end
      WAIT_CMD_nACK : begin
         if (!cmd_ack) begin
         end
      end
      
   endcase
end


    


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// DEBUG
///////////////////////////////////////////////////////////////////////////////
wire [31:0] dbg_dt ;
generate
   if (DEBUG) begin : DEBUG_BLOCK
      qnet_cmd_dbg debug_inst (
      .st_clk_i     ( t_clk_i ) ,
      .st_rst_ni    ( t_rst_ni ) ,
      .current_st_i ( cmd_st ) ,
      .next_st_i    ( cmd_st_nxt ) ,
      .debug_dt_o   ( dbg_dt ) 
      );
   end else
   assign dbg_dt = 0;

endgenerate



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// OUTPUTS
///////////////////////////////////////////////////////////////////////////////

always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      tx_req_o            <= 1'b0;
      tx_cmd_header_o   <= 64'd0;
      tx_cmd_dt_o       <= '{default:'0};;
   end else begin 
      if (tx_req_set ) begin   
         tx_req_o          <= 1'b1;
         tx_cmd_header_o <= tx_cmd_header;
         tx_cmd_dt_o     <= { tx_cmd_dt[1], tx_cmd_dt[0] };
      end 
      if (tx_req_o & tx_ack_i) begin
         tx_req_o <= 1'b0;
      end
   end

assign c_ready_o   = (main_st == M_IDLE) ;
assign loc_cmd_ack_o = loc_cmd_ack ;
assign net_cmd_ack_o = net_cmd_ack ;
assign tx_req_set_o = tx_req_set ;

assign time_reset_o = time_reset ;
assign time_init_o  = time_init  ;
assign time_updt_o  = time_updt  ;

endmodule

