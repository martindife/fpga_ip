`include "_qnet_defines.svh"

module qick_net_simplex # (
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
   input  wire             net_sync_i      ,
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
   output reg   [31:0]     tnet_dt1_o      ,
   output reg   [31:0]     tnet_dt2_o      ,
///////////////// SIMULATION    
   input  wire             rxn_A_i        ,
   input  wire             rxp_A_i        ,
   output wire             txn_B_o        ,
   output  wire            txp_B_o        ,
////////////////   LINK CHANNEL A
   input  wire             axi_rx_tvalid_A_RX_i   ,
   input  wire  [63:0]     axi_rx_tdata_A_RX_i    ,
   input  wire             axi_rx_tlast_A_RX_i   ,
////////////////   LINK CHANNEL B
   output reg              axi_tx_tvalid_B_TX_o   ,
   output reg   [63:0]     axi_tx_tdata_B_TX_o    ,
   output reg              axi_tx_tlast_B_TX_o   ,
   input  wire             axi_tx_tready_B_TX_i   ,
// AXI-Lite DATA Slave I/F.   
   input  wire  [ 5:0]     s_axi_awaddr         ,
   input  wire  [ 2:0]     s_axi_awprot         ,
   input  wire             s_axi_awvalid        ,
   output wire             s_axi_awready        ,
   input  wire  [31:0]     s_axi_wdata          ,
   input  wire  [ 3:0]     s_axi_wstrb          ,
   input  wire             s_axi_wvalid         ,
   output wire             s_axi_wready         ,
   output wire  [ 1:0]     s_axi_bresp          ,
   output wire             s_axi_bvalid         ,
   input  wire             s_axi_bready         ,
   input  wire  [ 5:0]     s_axi_araddr         ,
   input  wire  [ 2:0]     s_axi_arprot         ,
   input  wire             s_axi_arvalid        ,
   output wire             s_axi_arready        ,
   output wire  [31:0]     s_axi_rdata          ,
   output wire  [ 1:0]     s_axi_rresp          ,
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

reg loc_cmd_ack_set, loc_cmd_ack_clr  ;
reg net_cmd_ack_set, net_cmd_ack_clr  ; 
reg net_cmd_ack;

reg [31:0] qnet_DT [2];


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

reg wt_rst, wt_init;
reg [31:0] wt_init_dt;
wire wt_inc;
reg wt_ext_inc;


wire  get_time_lcs, get_time_ncr;



 

wire [63:0]  cmd_header_r ;
wire [31:0]  cmd_dt_r [2] ;

TYPE_PARAM_WE       param_we;
wire  [31:0]       param_32_dt    ;
wire  [9 :0]       param_10_dt     ;
wire  [31:0]       param_64_dt [2] ;


qnet_cmd_cod CMD_COD (
   .c_clk_i       ( c_clk_i       ) ,
   .c_rst_ni      ( c_rst_ni      ) ,
   .t_clk_i       ( t_clk_i       ) ,
   .t_rst_ni      ( t_rst_ni      ) ,
   .param_NN      ( param.NN      ) ,
   .param_ID      ( param.ID      ) ,
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
   .loc_cmd_ack_i ( loc_cmd_ack_s ) ,
   .net_cmd_req_o ( net_cmd_req ) ,
   .net_cmd_ack_i ( net_cmd_ack_s ) 
);

wire [63:0]  tx_cmd_header_s ;
wire [31:0]  tx_cmd_dt_s [2] ;
wire tx_req_s, tx_ack_s;

TYPE_CTRL_REQ      ctrl_cmd_req_s ;
TYPE_CTRL_OP       ctrl_cmd_op_s  ;
wire [47:0]        ctrl_cmd_dt_s  ;

qnet_cmd_proc CMD_PROCESSING (
   .t_clk_i         ( t_clk_i         ) ,
   .t_rst_ni        ( t_rst_ni        ) ,
   .ctrl_rst_i      ( 0      ) ,
   .aurora_ready_i  ( aurora_ready        ) ,
   .t_ready_t01     ( t_ready_t01        ) ,
   .net_sync_t01     ( net_sync_t01        ) ,
   .param_i         ( param      ) ,
   .qnet_T_LINK     ( qnet_T_LINK      ) ,
   .qnet_dt_i       ( qnet_DT    ),
   .t_time_abs      ( t_time_abs      ) ,
   .c_ready_o       ( c_ready_o      ) ,
   .loc_cmd_req_i   ( loc_cmd_req   ) ,
   .net_cmd_req_i   ( net_cmd_req   ) ,
   .cmd_header_i    ( cmd_header_r    ) ,
   .cmd_dt_i        ( cmd_dt_r        ) ,
   .loc_cmd_ack_o   ( loc_cmd_ack_s   ) ,
   .net_cmd_ack_o   ( net_cmd_ack_s   ) ,
   .tx_req_set_o    ( get_time_lcs      ) ,
   .param_we        ( param_we      ) ,
   .param_64_dt     ( param_64_dt     ) ,
   .param_32_dt     ( param_32_dt      ) ,
   .param_10_dt     ( param_10_dt      ) ,
   .tx_req_o        ( tx_req_s        ) ,
   .tx_cmd_header_o ( tx_cmd_header_s ) ,
   .tx_cmd_dt_o     ( tx_cmd_dt_s     ) ,
   .tx_ack_i        ( tx_ack_s        ) ,
   .ctrl_cmd_req_o  ( ctrl_cmd_req_s        ) ,
   .ctrl_cmd_op_o   ( ctrl_cmd_op_s        ) ,
   .ctrl_cmd_dt_o   ( ctrl_cmd_dt_s        ) ,
   .cmd_st_do       ( cmd_st_do       ) );



      
qnet_qick_cmd TPROC_CTRL (
   .c_clk_i        ( c_clk_i        ) ,
   .c_rst_ni       ( c_rst_ni       ) ,
   .t_clk_i        ( t_clk_i        ) ,
   .t_rst_ni       ( t_rst_ni       ) ,
   .net_sync_t01     ( net_sync_t01     ) ,
   .RTD_i          ( param.RTD      ),
   .t_time_abs     ( t_time_abs     ) ,
   .ctrl_cmd_req_i ( ctrl_cmd_req_s ) ,
   .ctrl_cmd_op_i  ( ctrl_cmd_op_s  ) ,
   .ctrl_cmd_dt_i  ( ctrl_cmd_dt_s  ) ,
   .core_start_o   ( core_start_o   ) ,
   .core_stop_o    ( core_stop_o    ) ,
   .time_reset_o   ( time_rst_o   ) ,
   .time_init_o    ( time_init_o    ) ,
   .time_updt_o    ( time_updt_o    ) ,    
   .time_off_dt_o  ( time_off_dt_o  )
);


// Core Start and Core Stop are c_clk Sync
// TIme commandas are t_clk Sync

reg net_sync_r, net_sync_r2 ;
(* ASYNC_REG = "TRUE" *) reg net_sync_cdc ;
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      net_sync_cdc   <= 0;
      net_sync_r     <= 0;
      net_sync_r2    <= 0;
   end else begin
      net_sync_cdc     <= net_sync_i;
      net_sync_r       <= net_sync_cdc;
      net_sync_r2      <= net_sync_r;
   end

wire net_sync_t01 ;
assign net_sync_t01  = !net_sync_r2 & net_sync_r ;


wire [31:0] qnet_T_LINK ;   
wire        link_A_rdy_01, link_B_rdy_01, link_A_rdy_cc, link_B_rdy_cc ;

aurora_ctrl_simplex # (
   .SIM_LEVEL ( SIM_LEVEL )
) QNET_LINK_INST (
   .gt_refclk1_p         ( gt_refclk1_p         ),      
   .gt_refclk1_n         ( gt_refclk1_n         ),
   .t_clk_i              ( t_clk_i              ),
   .t_rst_ni             ( t_rst_ni             ),
   .ps_clk_i             ( ps_clk_i             ),
   .ps_rst_ni            ( ps_rst_ni            ),
   .ID_i                 ( param.ID                ),
   .NN_i                 ( param.NN                ),
   .tx_req_i             ( tx_req_s             ),
   .tx_header_i          ( tx_cmd_header_s   ),
   .tx_data_i            ( tx_cmd_dt_s       ),
   .tx_ack_o             ( tx_ack_s          ),
   .sync_tx          ( link_B_rdy_01     ),
   .sync_cc          ( link_B_rdy_cc     ),
   .qnet_LINK_o          ( qnet_T_LINK       ),
   .cmd_net_o            ( net_cmd_hit           ),
   .cmd_o                ( net_cmd               ),
   .ready_o              ( aurora_ready         ),
   .rxn_A_i              ( rxn_A_i              ),
   .rxp_A_i              ( rxp_A_i              ),
   .txn_B_o              ( txn_B_o              ),
   .txp_B_o              ( txp_B_o              ),
// Channel A
   .axi_rx_tvalid_A_RX_i ( axi_rx_tvalid_A_RX_i ),
   .axi_rx_tdata_A_RX_i  ( axi_rx_tdata_A_RX_i  ),
   .axi_rx_tlast_A_RX_i  ( axi_rx_tlast_A_RX_i  ),
// Channel B
   .axi_tx_tvalid_B_TX_o ( axi_tx_tvalid_B_TX_o ),
   .axi_tx_tdata_B_TX_o  ( axi_tx_tdata_B_TX_o  ),
   .axi_tx_tlast_B_TX_o  ( axi_tx_tlast_B_TX_o  ),
   .axi_tx_tready_B_TX_i ( axi_tx_tready_B_TX_i ),
   .aurora_do            ( aurora_do            ),
   .channel_RX_up       ( channelA_ok_do       ),
   .channel_TX_up       ( channelB_ok_do       ),
   .pack_cnt_do          ( aurora_cnt     ),
   .last_op_do           ( aurora_op      ),
   .state_do             ( aurora_st      ) 
   );
   








assign t_ready_t01 = link_B_rdy_01 ;

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
   .NN             ( param.NN       ) ,
   .ID             ( param.ID       ) ,
   .CDELAY         ( qnet_T_LINK       ) ,
   .RTD            ( param.RTD      ) ,
   .VERSION        ( TNET_VER  ) ,
   .TNET_W_DT1     ( qnet_DT[0]    ) ,
   .TNET_W_DT2     ( qnet_DT[1]    ) ,
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

*/
   


/*  
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
*/




 

// Processing-Wait Time
///////////////////////////////////////////////////////////////////////////////

assign get_time_ncr = net_cmd_hit ; //Single Pulse net_req


TYPE_QPARAM param ;
    
// Parameters
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      param    <= '{default:'0};
      qnet_DT  <= '{default:'0};
   end else begin
      if ( param_we.DT  )  qnet_DT      <= param_64_dt; 
      if ( param_we.OFF )  param.OFF    <= param_32_dt;
      if ( param_we.RTD )  param.RTD    <= param_32_dt; 
      if ( param_we.NN  )  param.NN     <= param_10_dt; 
      if ( param_we.ID  )  param.ID     <= param_10_dt; 
      if ( get_time_ncr )  param.T_NCR  <= t_time_abs[31:0];
      if ( get_time_lcs )  param.T_LCS  <= t_time_abs[31:0];
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
   

//assign core_start_o  = net_ctrl_exec_r[1];
//assign core_stop_o   = net_ctrl_exec_r[2];
      
assign time_off_dt_o = param.OFF;

assign tnet_dt1_o = qnet_DT[0];
assign tnet_dt2_o = qnet_DT[1];


    
endmodule

