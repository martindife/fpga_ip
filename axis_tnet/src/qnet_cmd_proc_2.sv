`include "_qnet_defines.svh"

module qnet_cmd_proc (
   input  wire             t_clk_i             ,
   input  wire             t_rst_ni            ,
   input  wire  [9:0]           param_NN            ,
   input  wire  [9:0]           param_ID            ,
   input  wire  [31:0]           param_CD            ,
   input  wire  [31:0]           param_RTD            ,
   input  wire  [31:0]          param_DT [2]            ,
   input  wire  [31:0]           param_ID            ,
   input  wire  [31:0]     cmd_header_r           ,




   input   wire            net_cmd_ack_i             
   
   );




wire [9:0]cmd_src, cmd_dst;
assign cmd_id  = cmd_header_r[60:56];
assign cmd_flg = cmd_header_r[55:50];
assign cmd_dst = cmd_header_r[49:40];
assign cmd_src = cmd_header_r[39:30];

// LOCAL, or EXTERNAL commands and external command
// Core and Python COMMANDS are generated in this NET_NODE. 
// Net can be a external command or an answer of a current command of other NET_NODE.
// If local Command needs answer or ACK, it should wait for an external command for the answer or ACK



reg loc_cmd_ack, net_cmd_ack;

always_ff @(posedge t_clk_i) 
   if (!t_rst_ni) begin
      loc_cmd_ack   <= 1'b0;
      net_cmd_ack   <= 1'b0;
   end else begin 
      if (loc_cmd_ack_set)                         loc_cmd_ack <= 1'b1;
      if (loc_cmd_ack_clr | time_out | ctrl_rst )  loc_cmd_ack <= 1'b0;
      if (net_cmd_ack_set)                         net_cmd_ack <= 1'b1;
      if (net_cmd_ack_clr | time_out | ctrl_rst )  net_cmd_ack <= 1'b0;
   end
   





reg tready_cnt_rst;
reg [1:0] tready_cnt;

always_ff @(posedge t_clk_i)
      if (tready_cnt_rst) 
         tready_cnt    <= 0;
      else if (t_ready_t01)
         tready_cnt    <=    tready_cnt + 1'b1;

assign t_ready_t3rd  = (tready_cnt == 2'b11);

wire loc_cmd_req, net_cmd_req;

// Network Information
///////////////////////////////////////////////////////////////////////////////
reg [31:0] param_OFF_new, param_RTD_new; 
reg [9:0] param_NN_new, param_ID_new;
reg [31:0] param_DT_new [2];
reg update_OFF, update_RTD, update_CD, update_NN, update_ID, update_DT;
reg time_reset, time_init, time_updt;

// Command Control 
///////////////////////////////////////////////////////////////////////////////
enum {M_NOT_READY=0, M_IDLE=1, LOC_CMD=2, NET_CMD=3, M_WRESP=4, M_WACK=5 , NET_RESP=6, NET_ACK=7, M_ERROR=8 } main_st_nxt, main_st;
always_ff @(posedge t_clk_i)
   if (!t_rst_ni)      main_st  <= M_NOT_READY;
   else if ( ctrl_rst)  main_st  <= M_NOT_READY;
   else                 main_st  <= main_st_nxt;

// AXI Command Register
sync_reg # (.DW ( 1 ) )       sync_pcmd_op (
   .dt_i      ( TNET_CTRL[5] ) ,
   .clk_i     ( t_clk_i      ) ,
   .rst_ni    ( t_rst_ni  ) ,
   .dt_o      ( {ctrl_rst}  ) );
   
assign c_ready_o = (main_st == M_IDLE);


///// MAIN STATE
always_comb begin
   main_st_nxt  = main_st; // Default Current
   if (!aurora_ready | time_out)  
      main_st_nxt = M_NOT_READY;
   else
      case (main_st)
         M_NOT_READY :  if (aurora_ready)          main_st_nxt  = M_IDLE;
         M_IDLE      :  if       (loc_cmd_req )          main_st_nxt  = LOC_CMD;
                        else if  (net_cmd_req )          main_st_nxt  = NET_CMD;
         LOC_CMD     :  if (cmd_end     ) begin
                           if      ( cmd_error  )  main_st_nxt = M_ERROR;
                           else if ( cmd_flg[2] )  main_st_nxt = M_WRESP;
                           else if ( cmd_flg[1] )  main_st_nxt = M_WACK ;
                           else                    main_st_nxt = M_IDLE ;
                        end
         NET_CMD     :  if ( cmd_end      )        main_st_nxt = M_IDLE ;
         M_WRESP     :  if ( net_cmd_req  )        main_st_nxt = NET_RESP;
         M_WACK      :  if ( net_cmd_req  )        main_st_nxt = M_IDLE ;
         NET_RESP    :  if ( cmd_end      )        main_st_nxt = M_IDLE ;
         NET_ACK     :  if ( cmd_end      )        main_st_nxt = M_IDLE ;
         M_ERROR     :           main_st_nxt = M_IDLE ;
      endcase
end

enum {T_NOT_READY=0, T_IDLE=1, T_LOC_CMD=2,T_LOC_WSYNC=3,T_LOC_SEND=4,T_LOC_WnREQ=5, T_NET_CMD=6, T_NET_WSYNC=7, T_NET_SEND=8, T_NET_WnREQ=9 } task_st_nxt, task_st;
always_ff @(posedge t_clk_i)
   if      (!t_rst_ni) task_st  <= T_NOT_READY;
   else if ( ctrl_rst)  task_st  <= T_NOT_READY;
   else                 task_st  <= task_st_nxt;
reg tx_ch;
reg task_time_ok, tx_ch_dt ;
///// TASK STATE
always_comb begin
   task_st_nxt  = task_st; // Default Current
   loc_cmd_ack_set  = 1'b0 ;
   loc_cmd_ack_clr  = 1'b0 ;
   net_cmd_ack_set  = 1'b0 ; 
   net_cmd_ack_clr  = 1'b0 ;
   task_time_ok     = 1'b0 ;
   tx_ch_dt         = 1'b0 ;
   if (!aurora_ready | time_out)  
      task_st_nxt = T_NOT_READY;

      case (task_st)
         T_NOT_READY : begin
            task_time_ok     = 1'b1 ;
            if (aurora_ready)  task_st_nxt = T_IDLE;
         end
         T_IDLE      : begin
            task_time_ok     = 1'b1 ;
         // LOCAL COMMANDS
            if (loc_cmd_req) begin
               loc_cmd_ack_set = 1'b1 ;
               task_st_nxt      = T_LOC_CMD;
            end
         // NEWORK COMMANDS
            else if (net_cmd_req) begin
               net_cmd_ack_set  = 1'b1 ; 
               task_st_nxt      = T_NET_CMD;
            end
         end
   // LOCAL COMMAND
         T_LOC_CMD : begin
            tx_ch_dt      = 1'b1 ;
            if (tx_req)             task_st_nxt     = T_LOC_SEND;
            else if (cmd_end)       task_st_nxt     = T_LOC_WnREQ;
         end
         T_LOC_SEND : begin
            if (cmd_end)            task_st_nxt = T_LOC_WnREQ;
         end
         T_LOC_WnREQ : begin
            if (!loc_cmd_req)  begin
               loc_cmd_ack_clr = 1'b1;
               task_st_nxt = T_IDLE;
            end
         end
   // NETWORK COMMAND
         T_NET_CMD : begin
            tx_ch_dt      = 1'b1 ;
            if (tx_req)             task_st_nxt     = T_NET_SEND;
            else if (cmd_end)       task_st_nxt     = T_NET_WnREQ;
         end
         T_NET_WSYNC : begin
            if (t_ready_t01)        task_st_nxt     = T_NET_SEND;
         end
         T_NET_SEND : begin
            if (!tx_ack & !tx_req)  task_st_nxt = T_NET_WnREQ;
         end
         T_NET_WnREQ : begin
            if (!net_cmd_req) begin
               net_cmd_ack_clr = 1'b1;
               task_st_nxt = T_IDLE;
            end
         end
      endcase
end





// Command Execution
///////////////////////////////////////////////////////////////////////////////
/*
enum {NOT_READY, IDLE, ST_ERROR, NET_CMD_RT, 
      LOC_GNET      , NET_GNET_P      , NET_GNET_R      ,
      LOC_SNET      , NET_SNET_P      , NET_SNET_R      ,
      LOC_SYNC      , NET_SYNC_P      , NET_SYNC_R      ,
      LOC_GET_OFF   , NET_GET_OFF_P   , NET_GET_OFF_A      ,
      LOC_UPDT_OFF  , NET_UPDT_OFF_P  , NET_UPDT_OFF_R  ,
      LOC_SET_DT    , NET_SET_DT_P    , NET_SET_DT_R    ,
      LOC_GET_DT    , NET_GET_DT_P    , NET_GET_DT_R    , NET_GET_DT_A      ,
      LOC_RST_PROC  , NET_RST_PROC_P  , NET_RST_PROC_R  , 
      LOC_START_CORE, NET_START_CORE_P, NET_START_CORE_R,
      LOC_STOP_CORE , NET_STOP_CORE_P , NET_STOP_CORE_R ,
      WAIT_TX_ACK, WAIT_TX_nACK, WAIT_CMD_nACK
      } 
*/


TYPE_QNET_CMD cmd_st_nxt, cmd_st; 
always_ff @(posedge t_clk_i)
   if (!t_rst_ni)      cmd_st  <= NOT_READY;
   else if ( ctrl_rst)  cmd_st  <= NOT_READY;
   else                 cmd_st  <= cmd_st_nxt;



always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin   
      cmd_error_code  <= 0;
      cmd_error_cnt   <= 0;
   end else if (cmd_error) begin             
      cmd_error_code  <= {net_ctrl_id, cmd_id} ;
      cmd_error_cnt   <= cmd_error_cnt+1'b1;
   end else if (time_out) begin             
      cmd_error_code  <= 8'b11111111 ;
      cmd_error_cnt   <= cmd_error_cnt+1'b1;
   end

// Is response if 
// 1) For dst = 111111111 > net_src_hit & cmd_flg[4-WAIT_REQ]
// 2) For dst = DST   > net_dst_hit & & cmd_flg[0-ANS]





//wire [63:0] cmd_header_ans;
//assign cmd_header_ans = { cmd_header_r[63:51],1'b1,cmd_src, cmd_dst, cmd_header_r[29:0] };

assign net_src_hit = (cmd_src == param_ID) ;
assign net_dst_hit = (cmd_dst == param_ID) ;

assign is_resp  = net_src_hit & cmd_flg[2] ;
assign is_answ  = net_dst_hit & cmd_flg[0];


reg [63:0] tx_cmd_header;
reg[31:0] tx_cmd_dt [2];

reg get_net, set_net, sync_net, updt_off, get_off ;
reg set_dt, get_dt, rst_tproc, start_core, stop_core;
reg set_cond, clear_cond, custom;

// NET COMMAND OPERATON DECODER
always_comb begin
   get_net       = 1'b0;
   set_net       = 1'b0;
   sync_net      = 1'b0;
   get_off       = 1'b0;
   updt_off      = 1'b0;
   set_dt        = 1'b0;
   get_dt        = 1'b0;
   rst_tproc     = 1'b0;
   start_core    = 1'b0;
   stop_core     = 1'b0;
   set_cond      = 1'b0;
   clear_cond    = 1'b0;
   custom        = 1'b0;
   if      (cmd_id == _get_net     ) get_net       = 1'b1 ;
   else if (cmd_id == _set_net     ) set_net       = 1'b1 ;
   else if (cmd_id == _sync_net    ) sync_net      = 1'b1 ;
   else if (cmd_id == _get_off     ) get_off       = 1'b1 ;
   else if (cmd_id == _updt_off    ) updt_off      = 1'b1 ;
   else if (cmd_id == _set_dt      ) set_dt        = 1'b1 ;
   else if (cmd_id == _get_dt      ) get_dt        = 1'b1 ;
   else if (cmd_id == _rst_tproc   ) rst_tproc     = 1'b1 ;
   else if (cmd_id == _start_core  ) start_core    = 1'b1 ;
   else if (cmd_id == _stop_core   ) stop_core     = 1'b1 ;
   else if (cmd_id == _set_cond    ) set_cond      = 1'b1 ;
   else if (cmd_id == _clear_cond  ) clear_cond    = 1'b1 ;
end  

// Waiting for RETURN 
assign wfr = (main_st == NET_RESP);

// COMMAND STATE CHANGE
always_comb begin
   cmd_st_nxt      = cmd_st; // Default Current
   if (!aurora_ready )
      cmd_st_nxt    = NOT_READY;
   else if ( time_out)
      cmd_st_nxt    = ST_ERROR;
   else
      case (cmd_st)
         NOT_READY: if (aurora_ready)  cmd_st_nxt = IDLE;
         IDLE: begin
            // GET_NET
            if      (get_net & loc_cmd_ack                  )  cmd_st_nxt  = LOC_GNET;
            else if (get_net & net_cmd_ack & !wfr           )  cmd_st_nxt  = NET_GNET_P;
            else if (get_net & net_cmd_ack & wfr & is_resp  )  cmd_st_nxt  = NET_GNET_R;
            else if (get_net & net_cmd_ack & wfr & !is_resp )  cmd_st_nxt  = ST_ERROR;
            // SET_NET
            else if (set_net & loc_cmd_ack                  )  cmd_st_nxt = LOC_SNET;
            else if (set_net & net_cmd_ack & !wfr           )  cmd_st_nxt = NET_SNET_P;
            else if (set_net & net_cmd_ack & wfr & is_resp  )  cmd_st_nxt = NET_SNET_R;
            else if (set_net & net_cmd_ack & wfr & !is_resp )  cmd_st_nxt = ST_ERROR;
            // SYNC_NET 
            else if (sync_net & loc_cmd_ack                  ) cmd_st_nxt = LOC_SYNC;
            else if (sync_net & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_SYNC_P;
            else if (sync_net & net_cmd_ack & wfr & is_resp  ) cmd_st_nxt = NET_SYNC_R;
            else if (sync_net & net_cmd_ack & wfr & !is_resp ) cmd_st_nxt = ST_ERROR;
            // GET OFFSET  
            else if (get_off & loc_cmd_ack                  ) cmd_st_nxt = LOC_GET_OFF;
            else if (get_off & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_GET_OFF_P;
            else if (get_off & net_cmd_ack & wfr & is_answ  ) cmd_st_nxt = NET_GET_OFF_A;
            else if (get_off & net_cmd_ack & wfr & !is_answ ) cmd_st_nxt = ST_ERROR;
            // UPDATE_OFFSET  
            else if (updt_off & loc_cmd_ack                  ) cmd_st_nxt = LOC_UPDT_OFF;
            else if (updt_off & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_UPDT_OFF_P;
            else if (updt_off & net_cmd_ack & wfr & is_resp  ) cmd_st_nxt = NET_UPDT_OFF_R;
            else if (updt_off & net_cmd_ack & wfr & !is_resp ) cmd_st_nxt = ST_ERROR;
            // SET_DT   
            else if (set_dt & loc_cmd_ack                    ) cmd_st_nxt = LOC_SET_DT;
            else if (set_dt & net_cmd_ack & !wfr             ) cmd_st_nxt = NET_SET_DT_P;
            else if (set_dt & net_cmd_ack & wfr & is_resp    ) cmd_st_nxt = NET_SET_DT_R;
            else if (set_dt & net_cmd_ack & wfr & !is_resp   ) cmd_st_nxt = ST_ERROR;
            // GET_DT   
            else if (get_dt & loc_cmd_ack                    )   cmd_st_nxt = LOC_GET_DT;
            else if (get_dt & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_GET_DT_P;
            else if (get_dt & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_GET_DT_R;
            else if (get_dt & net_cmd_ack & wfr & is_answ    )   cmd_st_nxt = NET_GET_DT_A;
            else if (get_dt & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // RESET_TPROC   
            else if (rst_tproc & loc_cmd_ack & ~net_ctrl_ack  )   cmd_st_nxt = LOC_RST_PROC;
            else if (rst_tproc & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_RST_PROC_P;
            else if (rst_tproc & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_RST_PROC_R;
            else if (rst_tproc & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // START_CORE
            else if (start_core & loc_cmd_ack & ~net_ctrl_ack  )   cmd_st_nxt = LOC_START_CORE;
            else if (start_core & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_START_CORE_P;
            else if (start_core & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_START_CORE_R;
            else if (start_core & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // STOP_CORE   
            else if (stop_core & loc_cmd_ack & ~net_ctrl_ack  )   cmd_st_nxt = LOC_STOP_CORE;
            else if (stop_core & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_STOP_CORE_P;
            else if (stop_core & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_STOP_CORE_R;
            else if (stop_core & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // OTHER 
            else if (loc_cmd_ack | net_cmd_ack) cmd_st_nxt  = ST_ERROR;
         end
         ST_ERROR: cmd_st_nxt    = WAIT_TX_nACK;
   // LOCAL COMMAND
         LOC_GNET       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SNET       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SYNC       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
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
         NET_SYNC_P       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_GET_OFF_P    : if (t_ready_t3rd) cmd_st_nxt = WAIT_TX_ACK;
         NET_UPDT_OFF_P   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SET_DT_P     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_GET_DT_P     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_RST_PROC_P   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_START_CORE_P : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_STOP_CORE_P  : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
   // NET Command Return Process
         NET_GNET_R       : if (div_end)     cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SNET_R       :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SYNC_R       :                  cmd_st_nxt =  WAIT_CMD_nACK;
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
         WAIT_TX_ACK   : if ( tx_ack )    cmd_st_nxt = WAIT_TX_nACK;
         WAIT_TX_nACK  : if (!tx_ack )    cmd_st_nxt = WAIT_CMD_nACK;
         WAIT_CMD_nACK : if (!cmd_ack)    cmd_st_nxt = IDLE;
      endcase
end


reg get_time_lcs   ;
reg proc_en        ;
reg tx_req_set     ;
reg tready_cnt_rst ;
reg cmd_end        ;
reg wt_rst         ;
reg wt_loc_inc     ;
reg wt_ext_inc     ;
reg wt_init        ;
reg cmd_error      ;
reg time_reset     ;
reg time_init      ;
reg time_updt      ;
reg div_start      ;
reg update_NN      ;
reg update_ID      ;
reg update_CD      ;
reg update_RTD     ;
reg update_OFF     ;    
reg update_DT      ;
reg [63: 0]   tx_cmd_header  ;
reg [ 9: 0]   param_NN_new   ;
reg [ 9: 0]   param_ID_new   ;
reg [31: 0]   param_RTD_new  ;
reg [31: 0]   param_OFF_new  ;
reg [31: 0]   param_DT_new[2];
reg [31: 0]   tx_cmd_dt      ;
reg [31: 0]   cmd_time_ok    ;
reg [31: 0]   net_ctrl_time  ;
reg [31: 0]   net_ctrl_id    ;
reg [31: 0]   wt_init_dt     ;

//OUTPUTS
always_comb begin
   get_time_lcs = 1'b0;
   proc_en = 1'b0;
   tx_req_set = 1'b0 ;
   net_ctrl_id = 0 ;
   net_ctrl_time = 0;
   tready_cnt_rst = 1'b1;
   cmd_end        = 1'b0;
   wt_rst         = 1'b0;
   wt_loc_inc     = 1'b0;
   wt_ext_inc     = 1'b0;
   wt_init        = 1'b0 ;
   wt_init_dt     = 0;
   cmd_error      = 1'b0;
   time_reset     = 1'b0;
   time_init      = 1'b0;
   time_updt      = 1'b0;
   tx_cmd_dt      = '{default:'0};
   tx_cmd_header  = 0;
   div_start      = 1'b0;
   update_NN  = 1'b0 ;
   update_ID  = 1'b0 ;
   update_CD  = 1'b0 ;
   update_RTD = 1'b0 ;
   update_OFF = 1'b0 ;    
   update_DT  = 1'b0 ;
   param_NN_new   = 0;
   param_ID_new   = 0;
   param_RTD_new  = 0;
   param_OFF_new  = 0;
   param_DT_new   = '{default:'0};;
   cmd_time_ok = 1'b0;
   case (cmd_st)
      NOT_READY:    cmd_time_ok = 1'b1;
      IDLE: begin
         cmd_time_ok = 1'b1;
         if (get_net & net_cmd_ack & !wfr )  begin // (cmd_st_nxt  == NET_GNET_P) 
            wt_init     = 1'b1 ;
            time_reset  = 1'b1 ;
            wt_init_dt  = cmd_dt_r[1] ;
         end
         if (sync_net & net_cmd_ack & !wfr ) begin //  (cmd_st_nxt  == NET_SYNC_P)
            wt_init        = 1'b1 ;
            wt_init_dt     = cmd_dt_r[1] ;
            time_init      = 1'b1 ;
            update_OFF     = 1'b1 ;
            param_OFF_new  = cmd_dt_r[0] + cmd_dt_r[1] ;
         end
         if (get_net & net_cmd_ack & wfr & is_resp  ) begin  // (cmd_st_nxt  == NET_GNET_R) begin 
            update_RTD     = 1'b1  ;
            param_RTD_new  = t_time_abs[31:0];
            update_NN      = 1'b1  ;
            param_NN_new   = cmd_header_r[9:0];
            div_start      = 1'b1;
         end
      end
      ST_ERROR: begin
         cmd_error     = 1'b1;
         cmd_end = 1'b1 ;
      end
// COMMAND DATA      
      LOC_GNET: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = 0;
         update_ID     = 1'b1;
         param_ID_new  = 9'd1;
         // Command wait for answer
         if (t_ready_t01) begin
            get_time_lcs  = 1'b1 ;
            time_reset  = 1'b1 ;
            tx_req_set  = 1'b1 ;
         end
      end
      LOC_SNET: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = param_RTD;
         tx_cmd_dt[0]  = param_CD;
         // Command wait for answer
         if (t_ready_t01) begin
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_SYNC: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = param_CD;
         // Command wait for answer
         if (t_ready_t01) begin
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_GET_OFF: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) begin
            get_time_lcs  = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_UPDT_OFF: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) begin
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_SET_DT: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
         end
      end
      LOC_GET_DT: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = '{default:'0};
         if (t_ready_t01) begin
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_RST_PROC: begin
         net_ctrl_id    = 3'b001;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end
      LOC_START_CORE: begin
         net_ctrl_id    = 3'b010;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end
      LOC_STOP_CORE: begin
         net_ctrl_id    = 3'b100;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end

// PROCESS AND PROPAGATE COMMAND
      NET_GNET_P: begin 
         wt_loc_inc    = 1'b1;
         tx_cmd_header = cmd_header_r + 1'b1;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = 0;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
         end
      end
      NET_SNET_P: begin   // PROCESS AND PROPAGATE COMMAND
         update_RTD     = 1'b1  ;
         param_RTD_new  = cmd_dt_r[1];
         update_NN      = 1'b1  ;
         param_NN_new   = cmd_header_r[19:10];
         update_ID      = 1'b1  ;
         param_ID_new   = cmd_header_r[9:0];
         tx_cmd_header  = cmd_header_r + 1'b1;
         tx_cmd_dt      =  cmd_dt_r ;
         if (t_ready_t01) begin 
            tx_req_set = 1'b1 ;
         end
      end
      NET_SYNC_P: begin
         wt_loc_inc    = 1'b1;
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = cmd_dt_r[0]+param_CD;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
         end            
      end
      NET_GET_OFF_P: begin
         tready_cnt_rst = 1'b0;
         tx_cmd_header   = {14'b100_00101_100001, cmd_header_r[39:30] , param_ID, 30'b0000000000_0000000000_0000000000};
         tx_cmd_dt[1]  = time_ncr ;
         tx_cmd_dt[0]  = t_time_abs;
         if (t_ready_t3rd) begin
            tx_req_set     = 1'b1 ;
         end
      end
      NET_UPDT_OFF_P: begin
         update_OFF     = 1'b1 ;
         param_OFF_new  = cmd_dt_r[0];
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r ;
         if (t_ready_t01) begin
            time_updt      = 1'b1 ;         
            tx_req_set     = 1'b1 ;
         end
      end
      NET_SET_DT_P: begin
         cmd_end        = 1'b1;
         update_DT      = 1'b1 ;
         param_DT_new   = cmd_dt_r;
         tx_cmd_header  = cmd_header_r;
         tx_cmd_dt      = cmd_dt_r ;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
         end
      end
      NET_GET_DT_P: begin
         tx_cmd_header   = {14'b100_01011_000001, cmd_header_r[39:30] , param_ID, 30'b0000000000_0000000000_0000000000};
         tx_cmd_dt        = param_DT;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
         end
      end

      NET_RST_PROC_P: begin
         net_ctrl_id    = 3'b001;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end
      NET_START_CORE_P: begin
         net_ctrl_id    = 3'b010;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end
      NET_STOP_CORE_P: begin
         net_ctrl_id    = 3'b100;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end


// COmmand RETURN  
      NET_GNET_R: begin
         if (div_end) begin
            cmd_end        = 1'b1;
         end
      end
      NET_SNET_R     : begin
         cmd_end        = 1'b1;
         end
      NET_SYNC_R     : begin 
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
         proc_en = 1'b1;
         if (proc_step_3) begin
            cmd_end        = 1'b1;
         end
      
         cmd_end         = 1'b1;
         update_DT       = 1'b1 ;
         param_DT_new[0]   = node_offset;
         param_DT_new[1]   = node_offset;
         
      end
      NET_GET_DT_A: begin
         cmd_end        = 1'b1;
         update_DT      = 1'b1 ;
         param_DT_new   = cmd_dt_r;
         
      end
      WAIT_TX_nACK: begin
         if (!tx_ack) begin 
            cmd_end = 1'b1;
         end
      end
//DEBUG      
      WAIT_TX_ACK   : begin
         if ( tx_ack ) begin
         end
      end
      WAIT_CMD_nACK : begin
         if (!cmd_ack) begin
         end
      end
      
   endcase
end
assign cmd_ack = loc_cmd_ack | net_cmd_ack;


endmodule

