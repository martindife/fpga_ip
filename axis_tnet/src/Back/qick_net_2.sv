`include "_qnet_defines.svh"

module qick_net # (
   parameter SIM_LEVEL = 1 ,
   parameter DEBUG     = 1
   
)(
// Core, Time and AXI CLK & RST.
   input  wire             gt_refclk1_p    ,
   input  wire             gt_refclk1_n    ,
   input  wire             t_clk_i         ,
   input  wire             t_rst_ni        ,
   input  wire             c_clk_i         ,
   input  wire             c_rst_ni        ,
   input  wire             ps_clk_i        ,
   input  wire             ps_rst_ni       ,
   input  wire  [47:0]     t_time_abs      ,
// TPROC CONTROL
   input  wire             c_cmd_i         ,
   input  wire  [4:0]      c_op_i          ,
   input  wire  [31:0]     c_dt1_i         ,
   input  wire  [31:0]     c_dt2_i         ,
   input  wire  [31:0]     c_dt3_i         ,
   output reg              c_ready_o       ,
   output reg              core_start_o    ,
   output reg              core_stop_o     ,
   output reg              time_rst_o      ,
   output reg              time_init_o     ,
   output reg              time_updt_o     ,
   output wire  [31:0]     time_off_dt_o   ,
   output reg  [31:0]      tnet_dt1_o      ,
   output reg  [31:0]      tnet_dt2_o      ,
///////////////// SIMULATION    
   input  wire             rxn_A_i        ,
   input  wire             rxp_A_i        ,
   output wire             txn_A_o        ,
   output  wire            txp_A_o        ,
   input  wire             rxn_B_i        ,
   input  wire             rxp_B_i        ,
   output wire             txn_B_o        ,
   output  wire            txp_B_o        ,
////////////////   CHANNEL A LINK
   input  wire             axi_rx_tvalid_A_RX_i  ,
   input  wire  [63:0]     axi_rx_tdata_A_RX_i   ,
   input  wire             axi_rx_tlast_A_RX_i   ,
   output reg   [63:0]     axi_tx_tdata_A_TX_o   ,
   output reg              axi_tx_tvalid_A_TX_o  ,
   output reg              axi_tx_tlast_A_TX_o   ,
   input  wire             axi_tx_tready_A_TX_i  ,
////////////////   CHANNEL B LINK
   input  wire             axi_rx_tvalid_B_RX_i  ,
   input  wire  [63:0]     axi_rx_tdata_B_RX_i   ,
   input  wire             axi_rx_tlast_B_RX_i   ,
   output reg   [63:0]     axi_tx_tdata_B_TX_o   ,
   output reg              axi_tx_tvalid_B_TX_o  ,
   output reg              axi_tx_tlast_B_TX_o   ,
   input  wire             axi_tx_tready_B_TX_i  ,

// AXI-Lite DATA Slave I/F.   
   input  wire [5:0]       s_axi_awaddr         ,
   input  wire [2:0]       s_axi_awprot         ,
   input  wire             s_axi_awvalid        ,
   output wire             s_axi_awready        ,
   input  wire [31:0]      s_axi_wdata          ,
   input  wire [3:0]       s_axi_wstrb          ,
   input  wire             s_axi_wvalid         ,
   output wire             s_axi_wready         ,
   output wire  [1:0]      s_axi_bresp          ,
   output wire             s_axi_bvalid         ,
   input  wire             s_axi_bready         ,
   input  wire [5:0]       s_axi_araddr         ,
   input  wire [2:0]       s_axi_arprot         ,
   input  wire             s_axi_arvalid        ,
   output wire             s_axi_arready        ,
   output wire  [31:0]     s_axi_rdata          ,
   output wire  [1:0]      s_axi_rresp          ,
   output wire             s_axi_rvalid         ,
   input  wire             s_axi_rready         );


///// AXI LITE PORT /////
///////////////////////////////////////////////////////////////////////////////
TYPE_IF_AXI_REG        IF_s_axireg()   ;
assign IF_s_axireg.axi_awaddr  = s_axi_awaddr ;
assign IF_s_axireg.axi_awprot  = s_axi_awprot ;
assign IF_s_axireg.axi_awvalid = s_axi_awvalid;
assign IF_s_axireg.axi_wdata   = s_axi_wdata  ;
assign IF_s_axireg.axi_wstrb   = s_axi_wstrb  ;
assign IF_s_axireg.axi_wvalid  = s_axi_wvalid ;
assign IF_s_axireg.axi_bready  = s_axi_bready ;
assign IF_s_axireg.axi_araddr  = s_axi_araddr ;
assign IF_s_axireg.axi_arprot  = s_axi_arprot ;
assign IF_s_axireg.axi_arvalid = s_axi_arvalid;
assign IF_s_axireg.axi_rready  = s_axi_rready ;
assign s_axi_awready = IF_s_axireg.axi_awready;
assign s_axi_wready  = IF_s_axireg.axi_wready ;
assign s_axi_bresp   = IF_s_axireg.axi_bresp  ;
assign s_axi_bvalid  = IF_s_axireg.axi_bvalid ;
assign s_axi_arready = IF_s_axireg.axi_arready;
assign s_axi_rdata   = IF_s_axireg.axi_rdata  ;
assign s_axi_rresp   = IF_s_axireg.axi_rresp  ;
assign s_axi_rvalid  = IF_s_axireg.axi_rvalid ;


wire [31:0] TNET_CTRL, TNET_CFG, REG_AXI_DT1, REG_AXI_DT2, REG_AXI_DT3;
wire [15:0] TNET_ADDR,TNET_LEN; 
reg  [63 :0]      net_cmd [2]   ;
reg cmd_end;
wire             net_tx_ack ;
reg wt_loc_inc;
reg tx_req;
reg loc_cmd_ack_set, loc_cmd_ack_clr  ;
reg net_cmd_ack_set, net_cmd_ack_clr  ; 
reg net_cmd_ack;
reg [31:0] param_OFF, param_RTD;
wire [31:0] param_CD;
reg [9 :0] param_NN, param_ID;
reg [31:0] param_DT [2];
wire [31:0] div_result_r;
wire div_end;
wire tx_ack;
reg  [4 :0] cmd_id;
wire [5 :0] cmd_flg;
reg loc_cmd_ack;
reg [3:0] cmd_error_cnt;
reg [7:0] cmd_error_code;

reg cmd_error ;


reg [47:0] net_ctrl_time;
reg [2:0]  net_ctrl_id;
reg div_start;
reg proc_en;

reg [47:0] net_ctrl_time_r;
reg [2:0]  net_ctrl_id_r, net_ctrl_exec_r;
reg net_ctrl_ack; // A command will be executed anytime....
reg ctrl_sign_time_r;


reg wt_rst, wt_init;
reg [31:0] wt_init_dt;
wire wt_inc;
reg wt_ext_inc;


reg get_time_lcs;
wire  get_time_ncr, get_time_lcc, get_time_ltr;
reg [31:0] param_T_LCS; //Time Local Command Send 
reg [31:0] param_T_NCR; //Time Network Command Received
reg [31:0] param_T_LCC; //Time clock Compensation
reg [31:0] param_T_LTR; //Time aurora Ready



assign get_time_ltr = t_ready_t01; 

wire [63:0]  cmd_header_r ;
wire [31:0]  cmd_dt_r [2] ;


qnet_cmd_dec CMD_DEC (
   .c_clk_i       ( c_clk_i       ) ,
   .c_rst_ni      ( c_rst_ni      ) ,
   .t_clk_i       ( t_clk_i       ) ,
   .t_rst_ni      ( t_rst_ni      ) ,
   .param_NN      ( param_NN      ) ,
   .param_ID      ( param_ID      ) ,
   .c_cmd_i       ( c_cmd_i       ) ,
   .c_op_i        ( c_op_i        ) ,
   .c_dt_i        ( {c_dt1_i, c_dt2_i, c_dt3_i}  ) ,
   .p_op_i        ( TNET_CTRL[4:0]        ) ,
   .p_dt_i        ( {REG_AXI_DT1, REG_AXI_DT2, REG_AXI_DT3} ) ,
   .net_cmd_i     ( net_cmd_hit     ) ,
   .net_cmd_h_i   ( net_cmd[0]  ) ,
   .net_cmd_dt_i  ( net_cmd[1]  ) ,
   .header_o      ( cmd_header_r      ) ,
   .data_o        ( cmd_dt_r        ) ,
   .loc_cmd_req_o ( loc_cmd_req ) ,
   .loc_cmd_ack_i ( loc_cmd_ack ) ,
   .net_cmd_req_o ( net_cmd_req ) ,
   .net_cmd_ack_i ( net_cmd_ack ) 
);

wire [63:0]  tx_cmd_header_s ;
wire [31:0]  tx_cmd_dt_s [2] ;

qnet_cmd_proc CMD_PROCESSING (
   .t_clk_i         ( t_clk_i         ) ,
   .t_rst_ni        ( t_rst_ni        ) ,
   .aurora_ready_i  ( aurora_ready        ) ,
   .t_ready_t01     ( t_ready_t01        ) ,
   .time_out     ( time_out        ) ,
   
   
   .param_ID_i      ( param_ID      ) ,
   .param_CD_i      ( param_CD      ) ,
   .param_NN_i      ( param_NN      ) ,
   .param_DT_i      ( param_DT      ) ,
   .param_RTD_i     ( param_RTD     ) ,
   .param_T_NCR_i   ( param_T_NCR      ) ,
   .ctrl_rst_i      ( ctrl_rst      ) ,
   .t_time_abs      ( t_time_abs      ) ,
   .loc_cmd_req_i   ( loc_cmd_req   ) ,
   .net_cmd_req_i   ( net_cmd_req   ) ,
   .cmd_header_i    ( cmd_header_r    ) ,
   .cmd_dt_i        ( cmd_dt_r        ) ,
   .loc_cmd_ack_o   ( loc_cmd_ack_s   ) ,
   .net_cmd_ack_o   ( net_cmd_ack_s   ) ,
   .tx_req_o        ( tx_req_s        ) ,
   .tx_ch_o         ( tx_ch_s         ) ,
   .tx_cmd_header_o ( tx_cmd_header_s ) ,
   .tx_cmd_dt_o     ( tx_cmd_dt_s     ) ,
   .tx_ack_i        ( tx_ack        ) ,
   .cmd_st_do       ( cmd_st_do       ) );
   



aurora_ctrl # (
   .SIM_LEVEL ( SIM_LEVEL )
) QNET_LINK (
   .gt_refclk1_p         ( gt_refclk1_p         ),      
   .gt_refclk1_n         ( gt_refclk1_n         ),
   .t_clk_i              ( t_clk_i              ),
   .t_rst_ni             ( t_rst_ni             ),
   .ps_clk_i             ( ps_clk_i             ),
   .ps_rst_ni            ( ps_rst_ni            ),
   .ID_i                 ( param_ID                ),
   .NN_i                 ( param_NN                ),
   .tx_req_i             ( tx_req             ),
   .tx_ch_i              ( tx_ch              ),
   .tx_header_i          ( tx_cmd_header_r ),
   .tx_data_i            ( tx_cmd_dt_r ),
   .tx_ack_o             ( tx_ack  ),
   .sync_tx_A        (link_A_rdy_01),
   .sync_tx_B        (link_B_rdy_01),
   .sync_cc_A        (link_A_rdy_cc),
   .sync_cc_B        (link_B_rdy_cc),
   .param_CD         (param_CD),
   .cmd_net_o            ( net_cmd_hit           ),
   .cmd_ch_o             ( net_cmd_ch            ),
   .cmd_o                ( net_cmd                ),
   .ready_o              ( aurora_ready         ),
   .rxn_A_i              ( rxn_A_i              ),
   .rxp_A_i              ( rxp_A_i              ),
   .txn_A_o              ( txn_A_o              ),
   .txp_A_o              ( txp_A_o              ),
   .rxn_B_i              ( rxn_B_i              ),
   .rxp_B_i              ( rxp_B_i              ),
   .txn_B_o              ( txn_B_o              ),
   .txp_B_o              ( txp_B_o              ),
   .axi_rx_tvalid_A_RX_i ( axi_rx_tvalid_A_RX_i ),
   .axi_rx_tdata_A_RX_i  ( axi_rx_tdata_A_RX_i  ),
   .axi_rx_tlast_A_RX_i  ( axi_rx_tlast_A_RX_i  ),
   .axi_tx_tdata_A_TX_o  ( axi_tx_tdata_A_TX_o  ),
   .axi_tx_tvalid_A_TX_o ( axi_tx_tvalid_A_TX_o ),
   .axi_tx_tlast_A_TX_o  ( axi_tx_tlast_A_TX_o  ),
   .axi_tx_tready_A_TX_i ( axi_tx_tready_A_TX_i ),
   .axi_rx_tvalid_B_RX_i ( axi_rx_tvalid_B_RX_i ),
   .axi_rx_tdata_B_RX_i  ( axi_rx_tdata_B_RX_i  ),
   .axi_rx_tlast_B_RX_i  ( axi_rx_tlast_B_RX_i  ),
   .axi_tx_tdata_B_TX_o  ( axi_tx_tdata_B_TX_o  ),
   .axi_tx_tvalid_B_TX_o ( axi_tx_tvalid_B_TX_o ),
   .axi_tx_tlast_B_TX_o  ( axi_tx_tlast_B_TX_o  ),
   .axi_tx_tready_B_TX_i ( axi_tx_tready_B_TX_i ),
   //.s_axi_rx_tdata_RX    ( s_axi_rx_tdata_RX    ),
   //.s_axi_rx_tvalid_RX   ( s_axi_rx_tvalid_RX   ),
   //.s_axi_rx_tlast_RX    ( s_axi_rx_tlast_RX    ),
   //.m_axi_tx_tdata_TX    ( m_axi_tx_tdata_TX    ),
   //.m_axi_tx_tvalid_TX   ( m_axi_tx_tvalid_TX   ),
   //.m_axi_tx_tlast_TX    ( m_axi_tx_tlast_TX    ),
   //.m_axi_tx_tready_TX   ( m_axi_tx_tready_TX   ),
   .aurora_do            ( aurora_do            ),
   .channel_A_up       ( channelA_ok_do       ),
   .channel_B_up       ( channelB_ok_do       ),
   .pack_cnt_do          ( aurora_cnt     ),
   .last_op_do           ( aurora_op      ),
   .state_do             ( aurora_st      ) 
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
            cmd_end        = 1'b1;
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

reg tx_req_set;

reg [63:0] tx_cmd_header_r, tx_cmd_dt_r;
//TX Communication Control
reg tx_ch_dt;

always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      tx_req            <= 1'b0;
      tx_ch             <= 1'b0;
      tx_cmd_header_r   <= 64'd0;
      tx_cmd_dt_r       <= 64'd0;
   end else begin 
      if (tx_req_set ) begin   
         tx_req          <= 1'b1;
         tx_ch           <= tx_ch_dt ;
         tx_cmd_header_r <= tx_cmd_header;
         tx_cmd_dt_r     <= { tx_cmd_dt[1], tx_cmd_dt[0] };
      end 
      if (tx_req & tx_ack) begin
         tx_req <= 1'b0;
      end
   end







wire [31:0] div_num; 
wire [9:0] div_den;

assign div_num        = t_time_abs[31:0] - cmd_dt_r[1]; // Total Round Time minus Accumulated Wait Time
assign div_den        = cmd_header_r[9:0];

// Pipelined Divider
net_div_r #(
   .DW (32) ,
   .N_PIPE (32)
) div_r_inst (
   .clk_i           ( t_clk_i      ) ,
   .rst_ni          ( t_rst_ni  ) ,
   .start_i         ( div_start  ) ,
   .end_o           ( div_end  ) ,
   .A_i             ( div_num    ) ,
   .B_i             ( {22'd0, div_den } ) ,
   .ready_o         ( div_rdy    ) ,
   .div_remainder_o (  ) ,
   .div_quotient_o  ( div_result_r ) );

wire [31:0] slave_time, master_time, offset_time, node_offset;

addsub_32 node_time_inst (
  .A(cmd_dt_r[0]),      // input wire [31 : 0] A
  .B(cmd_dt_r[1]),      // input wire [31 : 0] B
  .CLK(t_clk_i),  // input wire CLK
  .ADD(1),  // input wire ADD
  .CE(proc_en),    // input wire CE
  .S(slave_time)      // output wire [31 : 0] S
  );

addsub_32 master_time_inst (
  .A(time_lcs),      // input wire [31 : 0] A
  .B(time_ncr),      // input wire [31 : 0] B
  .CLK(t_clk_i),  // input wire CLK
  .ADD(1),  // input wire ADD
  .CE(proc_en),    // input wire CE
  .S(master_time)      // output wire [31 : 0] S
);

addsub_32 offset_inst (
  .A(slave_time),      // input wire [31 : 0] A
  .B(master_time),      // input wire [31 : 0] B
  .CLK(t_clk_i),  // input wire CLK
  .ADD(0),  // input wire ADD
  .CE(proc_en),    // input wire CE
  .S(offset_time)      // output wire [31 : 0] S
);

assign node_offset = {offset_time[31], offset_time[31:1]} ;

assign t_ready_t01 = link_A_rdy_01 | link_A_rdy_01 ;

// TIMEOUT
reg cmd_time_ok, task_time_ok;
reg [9:0] time_cnt;
always_ff @ (posedge ps_clk_i, negedge ps_rst_ni) begin
   if (!ps_rst_ni) begin
      time_cnt      <= 1 ;
   end else begin
      if (cmd_time_ok & task_time_ok)   time_cnt  <= 0;
      else           time_cnt      <= time_cnt + 1'b1;
   end
end
// assign time_out = time_cnt[9];
assign time_out = 0;


   


/*
aurora_ctrl QNET_LINK (
   .user_clk_i          ( user_clk_i            ) , 
   .user_rst_i          ( user_rst_i            ) , 
   .t_clk_i             ( t_clk                 ),  
   .t_rstn_i            ( t_aresetn             ) , 
   .tx_req_ti           ( tx_req                ) , 
   .tx_header_ti        ( tx_cmd_header_r       ) , 
   .tx_data_ti          ( tx_cmd_dt_r           ) , 
   .tx_ack_o            ( net_tx_ack            ) , 
   .cmd_req_o           ( net_cmd_hit           ) , 
   .cmd_o               ( net_cmd               ) , 
   .ID                  ( param_ID              ) , 
   .channel_ok_i        ( channel_ok            ) , 
   .ready_o             ( aurora_ready          ) , 
   .s_axi_rx_tdata_RX   ( s_axi_rx_tdata_RX     ) , 
   .s_axi_rx_tvalid_RX  ( s_axi_rx_tvalid_RX    ) , 
   .s_axi_rx_tlast_RX   ( s_axi_rx_tlast_RX     ) , 
   .m_axi_tx_tdata_TX   ( m_axi_tx_tdata_TX   ) , 
   .m_axi_tx_tvalid_TX  ( m_axi_tx_tvalid_TX  ) , 
   .m_axi_tx_tlast_TX   ( m_axi_tx_tlast_TX   ) , 
   .m_axi_tx_tready_TX  ( m_axi_tx_tready_TX    ) ,
   .pack_cnt_do         ( aurora_cnt ) ,
   .last_op_do          ( aurora_op    ) ,
   .state_do            ( aurora_st) );
*/




wire [31:0] TNET_DEBUG, TNET_STATUS;  
// AXI Slave.
tnet_axi_reg TNET_xREG (
   .ps_aclk        ( ps_clk_i       ) , 
   .ps_aresetn     ( ps_rst_ni      ) , 
   .IF_s_axireg    ( IF_s_axireg    ) ,
   .TNET_CTRL      ( TNET_CTRL      ) ,
   .TNET_CFG       ( TNET_CFG       ) ,
   .TNET_ADDR      ( TNET_ADDR      ) ,
   .TNET_LEN       ( TNET_LEN       ) ,
   .REG_AXI_DT1    ( REG_AXI_DT1    ) ,
   .REG_AXI_DT2    ( REG_AXI_DT2    ) ,
   .REG_AXI_DT3    ( REG_AXI_DT3    ) ,
   .NN             ( param_NN       ) ,
   .ID             ( param_ID       ) ,
   .CDELAY         ( param_CD       ) ,
   .RTD            ( param_RTD      ) ,
   .VERSION        ( TNET_VER  ) ,
   .TNET_W_DT1     ( param_DT[0]    ) ,
   .TNET_W_DT2     ( param_DT[1]    ) ,
   .TNET_STATUS    ( TNET_STATUS    ) ,
   .TNET_DEBUG     ( TNET_DEBUG     ) );

wire [4:0]  aurora_cnt;
wire [3:0]  aurora_op;
wire [2:0]  aurora_st;
   
/*
wire  [31:0]TNET_VER;
assign TNET_VER[31 : 27]    = param_LT_5r ;
assign TNET_VER[26 : 22]    = param_LT_4r ;
assign TNET_VER[21 : 17]    = param_LT_3r ;
assign TNET_VER[16 : 12]    = param_LT_2r ;
assign TNET_VER[11 :  7]    = param_LT_r ;
assign TNET_VER[6  :  2]    = param_LT ;
*/


assign TNET_DEBUG[31 : 29]    = cmd_src[2:0] ;
assign TNET_DEBUG[28 : 26]    = cmd_dst[2:0] ;
assign TNET_DEBUG[25 : 18]    = main_st[7:0] ;
assign TNET_DEBUG[17 : 14]    = task_st[3:0] ;
assign TNET_DEBUG[13 : 11]    = aurora_st[2:0] ;
assign TNET_DEBUG[10 :  7]    = aurora_op[3:0] ;
assign TNET_DEBUG[ 6 :  0]    = aurora_cnt[6:0] ;


assign TNET_STATUS[31 : 30]    = { clear_cond, set_cond } ;
assign TNET_STATUS[29 : 27]    = { stop_core, start_core, rst_tproc} ;
assign TNET_STATUS[26 : 25]    = { get_dt, set_dt } ;
assign TNET_STATUS[24 : 21]    = { updt_off, sync_net, set_net, get_net } ;
assign TNET_STATUS[20 : 18]    = net_ctrl_id_r;
assign TNET_STATUS[17 : 10]    = cmd_error_code ;
assign TNET_STATUS[ 9 :  6]    = cmd_error_cnt;
assign TNET_STATUS[ 5 ]        = aurora_ready;
assign TNET_STATUS[ 4 :  0]    = 5'b00000 ;


   
  
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      time_rst_o  <= 1'b0;
      time_init_o <= 1'b0;
      time_updt_o <= 1'b0;
   end else begin
      time_rst_o  <=  time_reset | net_ctrl_exec_r[0];
      time_init_o <=  time_init;
      time_updt_o <=  time_updt;
   end


assign net_ctrl_req_set = |net_ctrl_id  ;
assign net_ctrl_ack_set = |net_ctrl_id_r & ctrl_time[47]; // ACK if RTD time is OK 

reg net_ctrl_req;

always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      net_ctrl_req   <= 1'b0;
      net_ctrl_ack   <= 1'b0;
      net_ctrl_exec_r  <= 1'b0;
      net_ctrl_id_r    <= 3'b0;
      net_ctrl_time_r  <= 47'b0111111111111111_1111111111111111_111111111111111;
   end else begin
      ctrl_sign_time_r <= ctrl_left_time[47];

      if ( net_ctrl_req_set )   net_ctrl_req   <= 1'b1;
      else if (cmd_error)       net_ctrl_req   <= 1'b0;

      if ( net_ctrl_ack_set ) begin
         net_ctrl_ack   <= 1'b1;
      end
      if ( net_ctrl_req & ~net_ctrl_ack) begin
         net_ctrl_id_r    <= net_ctrl_id ;
         net_ctrl_time_r  <= net_ctrl_time;
      end

      if (ctrl_execute )   net_ctrl_exec_r  <= net_ctrl_id_r;
      else                 net_ctrl_exec_r  <= 1'b0;

      if (net_ctrl_ack & ctrl_left_time[47] ) begin
         net_ctrl_id_r    <= 0;
         if ( !net_ctrl_ack_set ) begin
            net_ctrl_ack   <= 1'b0;
            net_ctrl_req   <= 1'b0;
         end
      end
      
   end

wire ctrl_execute;
assign ctrl_execute = ~ctrl_sign_time_r & ctrl_left_time[47] ;

wire [47:0] ctrl_left_time;

ADDSUB_MACRO #(
      .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
      .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
      .WIDTH      ( 48  )             // Input / output bus width, 1-48
   ) TIME_CMP_inst (
      .CARRYOUT   (                     ),  // 1-bit carry-out output signal
      .RESULT     ( ctrl_left_time    ),  // A-B
      .B          ( t_time_abs   ),  // Input A bus, width defined by WIDTH parameter
      .ADD_SUB    ( 1'b0              ),  // 1-bit add/sub input, high selects add, low selects subtract
      .A          ( net_ctrl_time_r   ),  // Input B bus, width defined by WIDTH parameter
      .CARRYIN    ( 1'b0              ),    // 1-bit carry-in input
      .CE         ( 1'b1              ),    // 1-bit clock enable input
      .CLK        ( t_clk_i             ),    // 1-bit clock input
      .RST        ( ~t_rst_ni        )     // 1-bit active high synchronous reset
   );

reg [47:0] ctrl_time;

ADDSUB_MACRO #(
      .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
      .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
      .WIDTH      ( 48  )             // Input / output bus width, 1-48
   ) CMD_TIME (
      .CARRYOUT   (               ),    // 1-bit carry-out output signal
      .RESULT     ( ctrl_time     ), // Add/sub result output, width defined by WIDTH parameter
      .B          ( ctrl_left_time     ),    // Input A bus, width defined by WIDTH parameter
      .ADD_SUB    ( 1'b0          ),    // 1-bit add/sub input, high selects add, low selects subtract
      .A          ( {16'd0, param_RTD}     ),   // Input B bus, width defined by WIDTH parameter
      .CARRYIN    ( 1'b0          ),    // 1-bit carry-in input
      .CE         ( net_ctrl_req  ),    // 1-bit clock enable input
      .CLK        ( t_clk_i         ),    // 1-bit clock input
      .RST        ( ~t_rst_ni    )     // 1-bit active high synchronous reset
   );



// Processing-Wait Time
///////////////////////////////////////////////////////////////////////////////

assign get_time_lcc = link_A_rdy_cc ;
assign get_time_ltr = link_A_rdy_01 ;
assign get_time_ncr =  net_cmd_hit ; //Single Pulse net_req
reg [31:0] cnt_CD ;


// Parameters
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      param_OFF   <= 32'd0;
      param_RTD   <= 32'd0;
      param_NN    <=  9'd0;
      param_ID    <=  9'd0;
      param_DT    <=  '{default:'0};
   end else begin
      if (update_OFF)     param_OFF  <= param_OFF_new;
      if (update_RTD)     param_RTD  <= param_RTD_new; 
      if (update_NN )     param_NN   <= param_NN_new; 
      if (update_ID )     param_ID   <= param_ID_new; 
      if (update_DT )     param_DT   <= param_DT_new; 

   end
   
/// Capture Reception Time
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin    
      param_T_LCS <= 32'd0; // Time Last Local Command Sent
      param_T_NCR <= 32'd0; // Time Last Network Command Received
      param_T_LCC <= 32'd0; // Time Last Clock Compensation Starts
      param_T_LTR <= 32'd0; // Time Last Rising TREADY
      cnt_CD   <= 32'd0; //Communication Delay Measured
   end else begin               
      if ( get_time_lcs  ) param_T_LCS  <= t_time_abs[31:0];
      if ( get_time_ncr  ) param_T_NCR  <= t_time_abs[31:0];
      if ( get_time_lcc  ) param_T_LCC  <= t_time_abs[31:0];
      if ( get_time_ltr  ) begin 
            cnt_CD <= 0;
            param_T_LTR    <= t_time_abs[31:0];
      end else
            cnt_CD = cnt_CD + 1;
   end



/// Processing Time
reg proc_step_1, proc_step_2, proc_step_3, proc_step_4 ;
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin    
      proc_step_1 <= 1'b0 ;
      proc_step_2 <= 1'b0 ;
      proc_step_3 <= 1'b0 ;
      proc_step_4 <= 1'b0 ;
   end else begin               
      proc_step_1 <= proc_en  ;
      proc_step_2 <= proc_step_1 ;
      proc_step_3 <= proc_step_2 ;
      proc_step_4 <= proc_step_3 ;
   end
   

assign core_start_o  = net_ctrl_exec_r[1];
assign core_stop_o   = net_ctrl_exec_r[2];
      
assign time_off_dt_o = param_OFF;

assign tnet_dt1_o = param_DT[0];
assign tnet_dt2_o = param_DT[1];


generate
   wire [31:0] dbg_dt ;
   wire dbg_en;
   
   if (DEBUG) begin : DEBUG_BLOCK
      qnet_cmd_dbg debug_inst (
      .st_clk_i     ( t_clk_i ) ,
      .st_rst_ni    ( t_rst_ni ) ,
      .current_st_i ( cmd_st ) ,
      .next_st_i    ( cmd_st_nxt ) ,
      .debug_en_o   ( dbg_en ) , 
      .debug_dt_o   ( dbg_dt ) 
      );
   end

endgenerate
   
    
endmodule

