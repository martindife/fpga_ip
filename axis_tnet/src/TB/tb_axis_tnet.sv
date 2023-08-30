///////////////////////////////////////////////////////////////////////////////
//  FERMI RESEARCH LAB
///////////////////////////////////////////////////////////////////////////////
//  Author         : mdife
///////////////////////////////////////////////////////////////////////////////

import axi_vip_pkg::*;
import axi_mst_0_pkg::*;

//`define T_TCLK         1.953125  // Half Clock Period for Simulation
`define T_TCLK         1.162574  // Half Clock Period for Simulation
`define T_CCLK         1.66 // Half Clock Period for Simulation
`define T_SCLK         5  // Half Clock Period for Simulation
`define T_ICLK         10  // Half Clock Period for Simulation
`define T_GTCLK         3.2  // Half Clock Period GT CLK 156.25


`define LFSR             1
`define DIVIDER          1
`define ARITH            1
`define TIME_READ        1
`define PMEM_AW          7 
`define DMEM_AW          6 
`define WMEM_AW          5 
`define REG_AW           4 
`define IN_PORT_QTY      2 
`define OUT_DPORT_QTY    2 
`define OUT_WPORT_QTY    2 


module tb_axis_tnet ();

///////////////////////////////////////////////////////////////////////////////
// Signals
reg   t_clk, c_clk, ps_clk, init_clk, rst_ni, gt_clk;
// VIP Agent
axi_mst_0_mst_t 	axi_mst_0_agent;
xil_axi_prot_t  prot        = 0;
reg[31:0]       data_wr     = 32'h12345678;
xil_axi_resp_t  resp;
//AXI-LITE
wire [7:0]             s_axi_awaddr     ;
wire [2:0]             s_axi_awprot     ;
wire                   s_axi_awvalid    ;
wire                   s_axi_awready    ;
wire [31:0]            s_axi_wdata      ;
wire [3:0]             s_axi_wstrb      ;
wire                   s_axi_wvalid     ;
wire                   s_axi_wready     ;
wire  [1:0]            s_axi_bresp      ;
wire                   s_axi_bvalid     ;
wire                   s_axi_bready     ;
wire [7:0]             s_axi_araddr     ;
wire [2:0]             s_axi_arprot     ;
wire                   s_axi_arvalid    ;
wire                   s_axi_arready    ;
wire  [31:0]           s_axi_rdata      ;
wire  [1:0]            s_axi_rresp      ;
wire                   s_axi_rvalid     ;
wire                   s_axi_rready     ;

// Aurora COnection
reg             channel_up_A      ;
reg  [127:0]     s_axi_rx_tdata_A  ;
reg             s_axi_rx_tvalid_A ;
reg             m_axi_tx_tready_A ;
reg             channel_up_B      ;
reg  [127:0]     s_axi_rx_tdata_B  ;
reg             s_axi_rx_tvalid_B ;
reg             m_axi_tx_tready_B ;

wire [127:0] m_axi_tx_tdata_A_1, m_axi_tx_tdata_A_2, m_axi_tx_tdata_A_3 ;
wire [127:0] m_axi_tx_tdata_B_1, m_axi_tx_tdata_B_2, m_axi_tx_tdata_B_3 ; 
  

//////////////////////////////////////////////////////////////////////////
//  CLK Generation
initial begin
  t_clk = 1'b0;
  forever # (`T_TCLK) t_clk = ~t_clk;
end
initial begin
  c_clk = 1'b0;
  forever # (`T_CCLK) c_clk = ~c_clk;
end
initial begin
  ps_clk = 1'b0;
  forever # (`T_SCLK) ps_clk = ~ps_clk;
end
initial begin
  init_clk = 1'b0;
  forever # (`T_ICLK) init_clk = ~init_clk;
end
initial begin
  gt_clk = 1'b0;
  forever # (`T_GTCLK) gt_clk = ~gt_clk;
end
wire gt_refclk1_p, gt_refclk1_n;
assign gt_refclk1_p = gt_clk;
assign gt_refclk1_n = ~gt_clk;

reg ready;

 initial
      forever begin
         ready = 1'b0;
         #100 ready = 1'b1;
         #1000 ;
      end
      


assign axi_rx_tready_RX   = 1;


reg       t_ready;

initial begin
   t_ready = 0;
   forever begin
      if (t_ready) # 200;
      @ (posedge c_clk) #1; 
      t_ready  = ~t_ready;
   end
end   


reg [31:0] axi_dt;
// Register ADDRESS
parameter TNET_CTRL     = 0  * 4 ;
parameter TNET_CFG      = 1  * 4 ;
parameter TNET_ADDR     = 2  * 4 ;
parameter TNET_LEN      = 3  * 4 ;
parameter REG_AXI_DT1   = 4  * 4 ;
parameter REG_AXI_DT2   = 5  * 4 ;
parameter REG_AXI_DT3   = 6  * 4 ;
parameter NN            = 7  * 4 ;
parameter ID            = 8  * 4 ;
parameter CD            = 9  * 4 ;
parameter RTD           = 10  * 4 ;
parameter VERSION       = 11 * 4 ;
parameter TNET_W_DT1    = 12 * 4 ;
parameter TNET_W_DT2    = 13 * 4 ;
parameter TNET_STATUS   = 14 * 4 ;
parameter TNET_DEBUG    = 15 * 4 ;


axi_mst_0 axi_mst_0_i
	(
		.aclk			   (ps_clk		),
		.aresetn		   (rst_ni	),
		.m_axi_araddr	(s_axi_araddr	),
		.m_axi_arprot	(s_axi_arprot	),
		.m_axi_arready	(s_axi_arready	),
		.m_axi_arvalid	(s_axi_arvalid	),
		.m_axi_awaddr	(s_axi_awaddr	),
		.m_axi_awprot	(s_axi_awprot	),
		.m_axi_awready	(s_axi_awready	),
		.m_axi_awvalid	(s_axi_awvalid	),
		.m_axi_bready	(s_axi_bready	),
		.m_axi_bresp	(s_axi_bresp	),
		.m_axi_bvalid	(s_axi_bvalid	),
		.m_axi_rdata	(s_axi_rdata	),
		.m_axi_rready	(s_axi_rready	),
		.m_axi_rresp	(s_axi_rresp	),
		.m_axi_rvalid	(s_axi_rvalid	),
		.m_axi_wdata	(s_axi_wdata	),
		.m_axi_wready	(s_axi_wready	),
		.m_axi_wstrb	(s_axi_wstrb	),
		.m_axi_wvalid	(s_axi_wvalid	)
	);


reg         c_cmd_i  ;
reg [4 :0]  c_op_i;
reg [31:0]  c_dt_1_i, c_dt_2_i, c_dt_3_i ;


reg [47:0] t_time_abs1, t_time_abs2, t_time_abs3;

wire time_rst_1, time_rst_2, time_rst_3    ;
wire time_init_1, time_init_2, time_init_3 ;
wire time_updt_1, time_updt_2, time_updt_3;
wire [31:0] time_dt_1, time_dt_2, time_dt_3;
wire core_start_1, core_start_2, core_start_3    ;
wire core_stop_1, core_stop_2, core_stop_3     ;

always_ff @(posedge t_clk) 
   if (!rst_ni) begin
      t_time_abs1 <= ($random %1024)+1023;;
      t_time_abs2 <= ($random %1024)+1023;;
      t_time_abs3 <= ($random %1024)+1023;;
   end else begin 
      if       (time_rst_1)      t_time_abs1   <=  0;
      else if  ( time_init_1)      t_time_abs1   <= time_dt_1;
      else if  ( time_updt_1) t_time_abs1   <= t_time_abs1 + time_dt_1;
      else                    t_time_abs1   <= t_time_abs1 + 1'b1;

      if       (time_rst_2)      t_time_abs2   <= 0;
      else if  ( time_init_2)      t_time_abs2   <= time_dt_2;
      else if  ( time_updt_2) t_time_abs2   <= t_time_abs2 + time_dt_2;
      else                    t_time_abs2   <= t_time_abs2 + 1'b1;

      if       ( time_rst_3)      t_time_abs3   <= 0;
      else if  ( time_init_3)      t_time_abs3   <= time_dt_3;
      else if  ( time_updt_3) t_time_abs3   <= t_time_abs3 + time_dt_3;
      else                    t_time_abs3   <= t_time_abs3 + 1'b1;
   end

wire ready_1;
reg reset_i, start_i, stop_i, init_i;
reg time_updt_i;
wire [63:0] axi_tx_tdata_TX_1, axi_tx_tdata_TX_2, axi_tx_tdata_TX_3;
wire txn_1, txn_2, txn_3;
wire txp_1, txp_2, txp_3;

// 0 SIM_LEVEL -> NO SIMULATION > SYNTH
// 1 SIM_LEVEL -> SIMULATION NO AURORA
// 2 SIM_LEVEL -> SIMULATION YES AURORA

localparam SIM_LEVEL = 1;
wire [63:0] axi_tx_tdata_TX_1d;
wire  [63:0] axi_tx_tdata_TX_2d;
wire axi_tx_tvalid_TX_1d, axi_tx_tlast_TX_1d;
wire axi_tx_tvalid_TX_2d, axi_tx_tlast_TX_2d;

assign #10 axi_tx_tdata_TX_1d  = axi_tx_tdata_TX_1  ;
assign #10 axi_tx_tvalid_TX_1d = axi_tx_tvalid_TX_1 ;
assign #10 axi_tx_tlast_TX_1d  = axi_tx_tlast_TX_1  ;
assign #5 axi_tx_tdata_TX_2d  = axi_tx_tdata_TX_2  ;
assign #5 axi_tx_tvalid_TX_2d = axi_tx_tvalid_TX_2 ;
assign #5 axi_tx_tlast_TX_2d  = axi_tx_tlast_TX_2  ;

      
axis_tnet  # ( 
   .SIM_LEVEL (SIM_LEVEL) 
) TNET_1 (
   .gt_refclk1_p       (  gt_refclk1_p           ) ,
   .gt_refclk1_n       (  gt_refclk1_n           ) ,
   .t_clk              (  t_clk           ) ,
   .t_aresetn          (  rst_ni          ) ,
   .c_clk              (  c_clk           ) ,
   .c_aresetn          (  rst_ni          ) ,
   .ps_clk             (  ps_clk          ) ,
   .ps_aresetn         (  rst_ni          ) ,
   .t_time_abs         (  t_time_abs1    )  ,

   ////////////////   SIMULATION
   .rxn_i         (  txn_3    )  ,
   .rxp_i         (  txp_3    )  ,
   .txn_o         (  txn_1    )  ,
   .txp_o         (  txp_1    )  ,
   ////////////////   LINK CHANNEL A
   .axi_rx_tvalid_RX_i  ( axi_tx_tvalid_TX_3  ) ,
   .axi_rx_tdata_RX_i   ( axi_tx_tdata_TX_3 ) ,
   .axi_rx_tlast_RX_i   ( axi_tx_tlast_TX_3  ) ,
   ////////////////   LINK CHANNEL B
   .axi_tx_tvalid_TX_o  ( axi_tx_tvalid_TX_1  ) ,
   .axi_tx_tdata_TX_o   ( axi_tx_tdata_TX_1 ) ,
   .axi_tx_tlast_TX_o   ( axi_tx_tlast_TX_1  ) ,
   .axi_tx_tready_TX_i  ( ready ) ,
   

   .c_cmd_i            ( c_cmd_i        ) ,
   .c_op_i             ( c_op_i        ) ,
   .c_dt1_i            ( c_dt_1_i ) ,
   .c_dt2_i            ( c_dt_2_i ) ,
   .c_dt3_i            ( c_dt_3_i ) ,
   .c_ready_o          ( ready_1) ,
   .core_start_o    ( core_start_1     ) ,
   .core_stop_o     ( core_stop_1     ) ,
   .time_rst_o      ( time_rst_1     ) ,
   .time_init_o     ( time_init_1      ) ,
   .time_updt_o     ( time_updt_1 ) ,
   .time_dt_o       ( time_dt_1 ) ,
   .tnet_dt1_o      ( tnet_dt1_1 ) ,
   .tnet_dt2_o      ( tnet_dt2_1 ) ,
//AXI   
   .s_axi_awaddr       (  s_axi_awaddr        ) ,
   .s_axi_awprot       (  s_axi_awprot        ) ,
   .s_axi_awvalid      (  s_axi_awvalid       ) ,
   .s_axi_awready      (  s_axi_awready       ) ,
   .s_axi_wdata        (  s_axi_wdata         ) ,
   .s_axi_wstrb        (  s_axi_wstrb         ) ,
   .s_axi_wvalid       (  s_axi_wvalid        ) ,
   .s_axi_wready       (  s_axi_wready        ) ,
   .s_axi_bresp        (  s_axi_bresp         ) ,
   .s_axi_bvalid       (  s_axi_bvalid        ) ,
   .s_axi_bready       (  s_axi_bready        ) ,
   .s_axi_araddr       (  s_axi_araddr        ) ,
   .s_axi_arprot       (  s_axi_arprot        ) ,
   .s_axi_arvalid      (  s_axi_arvalid       ) ,
   .s_axi_arready      (  s_axi_arready       ) ,
   .s_axi_rdata        (  s_axi_rdata         ) ,
   .s_axi_rresp        (  s_axi_rresp         ) ,
   .s_axi_rvalid       (  s_axi_rvalid        ) ,
   .s_axi_rready       (  s_axi_rready        ) );

axis_tnet  # ( 
   .SIM_LEVEL (SIM_LEVEL) 
) TNET_2 (
   .gt_refclk1_p       (  gt_refclk1_p           ) ,
   .gt_refclk1_n       (  gt_refclk1_n           ) ,
   .t_clk              (  t_clk           ) ,
   .t_aresetn          (  rst_ni          ) ,
   .c_clk              (  c_clk           ) ,
   .c_aresetn          (  rst_ni          ) ,
   .ps_clk             (  ps_clk          ) ,
   .ps_aresetn         (  rst_ni          ) ,
   .t_time_abs         (  t_time_abs2      )  ,

   ////////////////   SIMULATION
   .rxn_i         (  txn_1    )  ,
   .rxp_i         (  txp_1    )  ,
   .txn_o         (  txn_2    )  ,
   .txp_o         (  txp_2    )  ,
////////////////   LINK CHANNEL A
.axi_rx_tvalid_RX_i  ( axi_tx_tvalid_TX_1d  ) ,
.axi_rx_tdata_RX_i   ( axi_tx_tdata_TX_1d  )  ,
.axi_rx_tlast_RX_i   ( axi_tx_tlast_TX_1d  ) ,
////////////////   LINK CHANNEL B
.axi_tx_tvalid_TX_o  ( axi_tx_tvalid_TX_2  ) ,
.axi_tx_tdata_TX_o   ( axi_tx_tdata_TX_2 ) ,
.axi_tx_tlast_TX_o   ( axi_tx_tlast_TX_2  ) ,
.axi_tx_tready_TX_i  ( ready ) ,

   .c_cmd_i            ( 0        ) ,
   .c_op_i             ( 0        ) ,
   .c_dt1_i            ( 0 ) ,
   .c_dt2_i            ( 0 ) ,
   .c_dt3_i            ( 0 ) ,
   .c_ready_o          ( ready_2) ,
   .core_start_o    ( core_start_2     ) ,
   .core_stop_o     ( core_stop_2     ) ,
   .time_rst_o      ( time_rst_2     ) ,
   .time_init_o     ( time_init_2      ) ,
   .time_updt_o     ( time_updt_2 ) ,
   .time_dt_o       ( time_dt_2 ) ,
   .tnet_dt1_o      ( tnet_dt1_2 ) ,
   .tnet_dt2_o      ( tnet_dt2_2 ) ,
   .s_axi_awaddr       (  0      ) ,
   .s_axi_awprot       (  0      ) ,
   .s_axi_awvalid      (  0      ) ,
   .s_axi_awready      (         ) ,
   .s_axi_wdata        (  0      ) ,
   .s_axi_wstrb        (  0      ) ,
   .s_axi_wvalid       (  0      ) ,
   .s_axi_wready       (         ) ,
   .s_axi_bresp        (         ) ,
   .s_axi_bvalid       (         ) ,
   .s_axi_bready       (  0      ) ,
   .s_axi_araddr       (  0      ) ,
   .s_axi_arprot       (  0      ) ,
   .s_axi_arvalid      (  0      ) ,
   .s_axi_arready      (         ) ,
   .s_axi_rdata        (         ) ,
   .s_axi_rresp        (         ) ,
   .s_axi_rvalid       (         ) ,
   .s_axi_rready       (         ) );

axis_tnet  # ( 
   .SIM_LEVEL ( SIM_LEVEL ) 
) TNET_3 (
   .gt_refclk1_p       (  gt_refclk1_p           ) ,
   .gt_refclk1_n       (  gt_refclk1_n           ) ,
   .t_clk              (  t_clk           ) ,
   .t_aresetn          (  rst_ni          ) ,
   .c_clk              (  c_clk           ) ,
   .c_aresetn          (  rst_ni          ) ,
   .ps_clk             (  ps_clk          ) ,
   .ps_aresetn         (  rst_ni          ) ,
   .t_time_abs         (  t_time_abs2      )  ,

   ////////////////   SIMULATION
   .rxn_i         (  txn_2    )  ,
   .rxp_i         (  txp_2    )  ,
   .txn_o         (  txn_3    )  ,
   .txp_o         (  txp_3    )  ,
   ////////////////   LINK CHANNEL A
   .axi_rx_tvalid_RX_i  ( axi_tx_tvalid_TX_2d  ) ,
   .axi_rx_tdata_RX_i   ( axi_tx_tdata_TX_2d ) ,
   .axi_rx_tlast_RX_i   ( axi_tx_tlast_TX_2d  ) ,
   ////////////////   LINK CHANNEL B
   .axi_tx_tvalid_TX_o  ( axi_tx_tvalid_TX_3  ) ,
   .axi_tx_tdata_TX_o   ( axi_tx_tdata_TX_3 ) ,
   .axi_tx_tlast_TX_o   ( axi_tx_tlast_TX_3  ) ,
   .axi_tx_tready_TX_i  ( ready ) ,
   


   .c_cmd_i            ( 0        ) ,
   .c_op_i             ( 0        ) ,
   .c_dt1_i            ( 0 ) ,
   .c_dt2_i            ( 0 ) ,
   .c_dt3_i            ( 0 ) ,
   .c_ready_o          ( ready_3) ,
   .core_start_o    ( core_start_3     ) ,
   .core_stop_o     ( core_stop_3     ) ,
   .time_rst_o      ( time_rst_3     ) ,
   .time_init_o     ( time_init_3      ) ,
   .time_updt_o     ( time_updt_3 ) ,
   .time_dt_o       ( time_dt_3 ) ,
   .tnet_dt1_o      ( tnet_dt1_3 ) ,
   .tnet_dt2_o      ( tnet_dt2_3 ) ,
   .s_axi_awaddr       (  0      ) ,
   .s_axi_awprot       (  0      ) ,
   .s_axi_awvalid      (  0      ) ,
   .s_axi_awready      (         ) ,
   .s_axi_wdata        (  0      ) ,
   .s_axi_wstrb        (  0      ) ,
   .s_axi_wvalid       (  0      ) ,
   .s_axi_wready       (         ) ,
   .s_axi_bresp        (         ) ,
   .s_axi_bvalid       (         ) ,
   .s_axi_bready       (  0      ) ,
   .s_axi_araddr       (  0      ) ,
   .s_axi_arprot       (  0      ) ,
   .s_axi_arvalid      (  0      ) ,
   .s_axi_arready      (         ) ,
   .s_axi_rdata        (         ) ,
   .s_axi_rresp        (         ) ,
   .s_axi_rvalid       (         ) ,
   .s_axi_rready       (         ) );
  


reg [ 2:0] h_type  ;
reg [ 5:0] h_cmd   ;
reg [ 4:0] h_flags ;
reg [8:0] h_src  ;
reg [8:0] h_dst  ;


initial begin
   $display("START SIMULATION");

  	// Create agents.
	axi_mst_0_agent 	= new("axi_mst_0 VIP Agent",tb_axis_tnet.axi_mst_0_i.inst.IF);
	// Set tag for agents.
	axi_mst_0_agent.set_agent_tag	("axi_mst_0 VIP");
	// Start agents.
	axi_mst_0_agent.start_master();

   rst_ni            = 1'b0;
h_type   = 3'd0 ;
h_cmd    = 6'd0 ;
h_flags  = 5'd0 ;
h_src    = 9'd0 ;
h_dst    = 9'd0 ;

s_axi_rx_tdata_A   = 127'd0;
s_axi_rx_tvalid_A  = 1'b0;
//m_axi_tx_tready_A  = 1'b0;
channel_up_B       = 1'b0;
//m_axi_tx_tready_B  = 1'b0;

   c_cmd_i   = 1'b0 ;
   c_op_i   = 0 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = 31'd0 ;

   axi_dt            = 0 ;
   reset_i           = 1'b0 ;
   init_i            = 1'b0 ;
   start_i           = 1'b0 ;
   stop_i            = 1'b0 ;
   time_updt_i       =  1'b0 ;
   #10 ;

   @ (posedge ps_clk); #0.1;
   rst_ni            = 1'b1;
   #10 ;

/*
// GET_NET COMMAND
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b00001;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   #100;

// SET_NET 
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b00010 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   #100;
*/


   //GET NET
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 1); // GET_NET (NO PARAMETER)
   //SET NET
   #100;
   wait (ready_1==1'b1);
   #1000;
   WRITE_AXI( TNET_CTRL, 2); // SET_NET  (NO PARAMETER / Automatic > RTD - CD - NN - ID) 
   // SYNC_NET
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 8); // SYNC_NET (NO PARAMETER / Automatic > Delay, TimeWait)

//   WRITE_AXI( TNET_CTRL, 32); // RESET

   //UPDF_OFF
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 2); // DATA
   WRITE_AXI( REG_AXI_DT3    , {16'd0, 16'd2}); // NODE
   WRITE_AXI( TNET_CTRL, 9); // UPDT_OFFSET (NODE - DATA)

//UPDF_OFF
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 4); // DATA
   WRITE_AXI( REG_AXI_DT3    , {16'd0, 16'd3}); // NODE
   WRITE_AXI( TNET_CTRL, 9); // UPDT_OFFSET (NODE - DATA)

//RST_PROC
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 32'd0); // TIME
   WRITE_AXI( REG_AXI_DT2 , 32'd6000); // TIME
   WRITE_AXI( TNET_CTRL, 16); // RESET

//START_CORE
   #5000;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 32'd0); // TIME
   WRITE_AXI( REG_AXI_DT2 , 32'd2000); // TIME
   WRITE_AXI( TNET_CTRL, 17); // RESET

//STOP_CORE
   #5000;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 32'd0); // TIME
   WRITE_AXI( REG_AXI_DT2 , 32'd3000); // TIME
   WRITE_AXI( TNET_CTRL, 18); // RESET8



   //SET_DT
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 15); // DATA1
   WRITE_AXI( REG_AXI_DT2 , 255); // DATA2
   WRITE_AXI( REG_AXI_DT3    , {16'd2, 16'd0}); // NODE
   WRITE_AXI( TNET_CTRL, 10); // SET_DT (NODE - DATA)

   //SET_DT
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 15); // DATA1
   WRITE_AXI( REG_AXI_DT2 , 255); // DATA2
   WRITE_AXI( TNET_CTRL, 10); // SET_DT (NODE - DATA)

   //SET_DT
   #100;
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT1 , 7); // DATA1
   WRITE_AXI( REG_AXI_DT2 , 127); // DATA2
   WRITE_AXI( REG_AXI_DT3    , {16'd3, 16'd0}); // NODE
   WRITE_AXI( TNET_CTRL, 10); // SET_DT (NODE - DATA)

   //GET_DT
   wait (ready_1==1'b1);
   WRITE_AXI( REG_AXI_DT3    , {16'd2, 16'd0}); // NODE
   WRITE_AXI( TNET_CTRL, 11); // GET_DT (NODE - DATA)



// GET_NET COMMAND
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b00001;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   #100;

// SET_NET 
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b00010 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   #100;


// SYNC_NET 
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b01000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = 31'd0 ;
   #100;


// OFFSET UPDATE 
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b01001 ;
   c_dt_1_i  = 31'd2 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = {16'd0, 16'd2} ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = 31'd0 ;
   #100;
   

// SET DT 
   wait (ready_1==1'b1);
   #100;
   @ (posedge c_clk); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b01010 ;
   c_dt_1_i  = 31'd5 ;
   c_dt_2_i  = 31'd10 ;
   c_dt_3_i  = {16'd1, 16'd2} ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = 31'd0 ;
   #100;
         
   // TEST_QNET ();
   // TEST_AXI ();
  #1000;



end


integer DATA_RD;

task WRITE_AXI(integer PORT_AXI, DATA_AXI); begin
   //$display("Write to AXI");
   //$display("PORT %d",  PORT_AXI);
   //$display("DATA %d",  DATA_AXI);
   @ (posedge ps_clk); #0.1;
   axi_mst_0_agent.AXI4LITE_WRITE_BURST(PORT_AXI, prot, DATA_AXI, resp);
   end
endtask

task READ_AXI(integer ADDR_AXI); begin
   @ (posedge ps_clk); #0.1;
   axi_mst_0_agent.AXI4LITE_READ_BURST(ADDR_AXI, 0, DATA_RD, resp);
      $display("READ AXI_DATA %d",  DATA_RD);
   end
endtask

integer cnt ;
integer axi_addr ;
integer num;

task TEST_AXI (); begin
   $display("-----Writting RANDOM AXI Address");
   for ( cnt = 0 ; cnt  < 16; cnt=cnt+1) begin
      axi_addr = cnt*4;
      axi_dt = cnt+1 ;
      //num = ($random %64)+31;
      //num = ($random %32)+31;
      //num = ($random %16)+15;
      //axi_addr = num*4;
      //axi_dt = num+1 ;
      #100
      $display("WRITE AXI_DATA %d",  axi_dt);
      WRITE_AXI( axi_addr, axi_dt); //SET
   end
   /*
   $display("-----Reading ALL AXI Address");
   for ( cnt = 0 ; cnt  <= 64; cnt=cnt+1) begin
      axi_addr = cnt*4;
      $display("READ AXI_ADDR %d",  axi_addr);
      READ_AXI( axi_addr);
      $display("READ AXI_DATA %d",  DATA_RD);
   end
   $display("-----FINISHED ");
   */
end
endtask


endmodule




