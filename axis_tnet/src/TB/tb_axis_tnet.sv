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
`define T_UCLK         15  // Half Clock Period for Simulation


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
reg   t_clk, c_clk, ps_clk, init_clk, user_clock, rst_ni;

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
  user_clock = 1'b0;
  forever # (`T_UCLK) user_clock = ~user_clock;
end

reg       t_ready;

initial begin
   t_ready = 0;
   forever begin
      if (t_ready) # 200;
      @ (posedge user_clock) #1; 
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
parameter NONE          = 6  * 4 ;
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

wire reset_1, reset_2, reset_3    ;
wire init_1, init_2, init_3 ;
wire start_1, start_2, start_3    ;
wire stop_1, stop_2, stop_3     ;
wire time_updt_1, time_updt_2, time_updt_3;
wire [31:0] offset_dt_1, offset_dt_2, offset_dt_3;

always_ff @(posedge t_clk) 
   if (!rst_ni) begin
      t_time_abs1 <= ($random %1024)+1023;;
      t_time_abs2 <= ($random %1024)+1023;;
      t_time_abs3 <= ($random %1024)+1023;;
   end else begin 
      if       (reset_1)      t_time_abs1   <=  0;
      else if  ( init_1)      t_time_abs1   <= offset_dt_1;
      else if  ( time_updt_1) t_time_abs1   <= t_time_abs1 + offset_dt_1;
      else                    t_time_abs1   <= t_time_abs1 + 1'b1;

      if       (reset_2)      t_time_abs2   <= 0;
      else if  ( init_2)      t_time_abs2   <= offset_dt_2;
      else if  ( time_updt_2) t_time_abs2   <= t_time_abs2 + offset_dt_2;
      else                    t_time_abs2   <= t_time_abs2 + 1'b1;

      if       (reset_3)      t_time_abs3   <= 0;
      else if  ( init_3)      t_time_abs3   <= offset_dt_3;
      else if  ( time_updt_3) t_time_abs3   <= t_time_abs3 + offset_dt_3;
      else                    t_time_abs3   <= t_time_abs3 + 1'b1;
   end

wire ready_1;

axis_tnet TNET_1 (
   .t_clk              (  t_clk           ) ,
   .t_aresetn          (  rst_ni          ) ,
   .c_clk              (  c_clk           ) ,
   .c_aresetn          (  rst_ni          ) ,
   .ps_clk             (  ps_clk          ) ,
   .ps_aresetn         (  rst_ni          ) ,
   .init_clk           (  init_clk        ) ,
   .init_aresetn       (  rst_ni          ) ,
   .user_clock         (  user_clock      ) ,
   .user_aresetn       (  rst_ni          ) ,
   .t_time_abs         (  t_time_abs1    )  ,

   .c_cmd_i            ( c_cmd_i        ) ,
   .c_op_i             ( c_op_i        ) ,
   .c_dt1_i             ( c_dt_1_i ) ,
   .c_dt2_i             ( c_dt_2_i ) ,
   .c_dt3_i             ( c_dt_3_i ) ,
   .c_ready_o (ready_1) ,
   .time_rst_o         ( reset_1     ) ,
   .time_init_o        ( init_1      ) ,
   .time_off_dt_o        ( offset_dt_1 ) ,
   .start_o            ( start_1     ) ,
   .stop_o             ( stop_1      ) ,
   .time_updt_o        ( time_updt_1 ) ,
   
   .reset_pb           (  reset_pb            ) ,
   .pma_init           (  pma_init            ) ,
   .channel_up_A       (  channel_up_A        ) ,
   .s_axi_rx_tdata_A   (  m_axi_tx_tdata_B_3    ) ,
   .s_axi_rx_tvalid_A  (  m_axi_tx_tvalid_B_3   ) ,
   .channel_up_B       (  channel_up_B          ) ,
   .m_axi_tx_tdata_B   (  m_axi_tx_tdata_B_1    ) ,
   .m_axi_tx_tvalid_B  (  m_axi_tx_tvalid_B_1   ) ,
   .m_axi_tx_tready_B  (  t_ready   ) ,
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
   
axis_tnet TNET_2 (
   .t_clk              (  t_clk           ) ,
   .t_aresetn          (  rst_ni          ) ,
   .c_clk              (  c_clk           ) ,
   .c_aresetn          (  rst_ni          ) ,
   .ps_clk             (  ps_clk          ) ,
   .ps_aresetn         (  rst_ni          ) ,
   .init_clk           (  init_clk        ) ,
   .init_aresetn       (  rst_ni          ) ,
   .user_clock         (  user_clock      ) ,
   .user_aresetn       (  rst_ni          ) ,
   .t_time_abs         (  t_time_abs2      )  ,
   .c_cmd_i            ( 0        ) ,
   .c_op_i            ( 0        ) ,
   .c_dt1_i             ( 0 ) ,
   .c_dt2_i             ( 0 ) ,
   .c_dt3_i             ( 0 ) ,
   .time_rst_o         ( reset_2     ) ,
   .time_init_o        ( init_2      ) ,
   .time_off_dt_o      ( offset_dt_2 ) ,

   .start_o            ( start_2     ) ,
   .stop_o             ( stop_2      ) ,
   .time_updt_o        ( time_updt_2 ) ,
   .reset_pb           (  reset_pb            ) ,
   .pma_init           (  pma_init            ) ,
   .channel_up_A       (  channel_up_A        ) ,
   .s_axi_rx_tdata_A   (  m_axi_tx_tdata_B_1    ) ,
   .s_axi_rx_tvalid_A  (  m_axi_tx_tvalid_B_1   ) ,
   .channel_up_B       (  channel_up_B          ) ,
   .m_axi_tx_tdata_B   (  m_axi_tx_tdata_B_2    ) ,
   .m_axi_tx_tvalid_B  (  m_axi_tx_tvalid_B_2   ) ,
   .m_axi_tx_tready_B  (  t_ready  ) ,
   .s_axi_awaddr       (  0      ) ,
   .s_axi_awprot       (  0      ) ,
   .s_axi_awvalid      (  0      ) ,
   .s_axi_awready      (        ) ,
   .s_axi_wdata        (  0      ) ,
   .s_axi_wstrb        (  0      ) ,
   .s_axi_wvalid       (  0      ) ,
   .s_axi_wready       (        ) ,
   .s_axi_bresp        (        ) ,
   .s_axi_bvalid       (        ) ,
   .s_axi_bready       (  0      ) ,
   .s_axi_araddr       (  0      ) ,
   .s_axi_arprot       (  0      ) ,
   .s_axi_arvalid      (  0      ) ,
   .s_axi_arready      (        ) ,
   .s_axi_rdata        (        ) ,
   .s_axi_rresp        (        ) ,
   .s_axi_rvalid       (        ) ,
   .s_axi_rready       (        ) );


axis_tnet TNET_3 (
   .t_clk              (  t_clk           ) ,
   .t_aresetn          (  rst_ni          ) ,
   .c_clk              (  c_clk           ) ,
   .c_aresetn          (  rst_ni          ) ,
   .ps_clk             (  ps_clk          ) ,
   .ps_aresetn         (  rst_ni          ) ,
   .init_clk           (  init_clk        ) ,
   .init_aresetn       (  rst_ni          ) ,
   .user_clock         (  user_clock      ) ,
   .user_aresetn       (  rst_ni          ) ,
   .t_time_abs         (  t_time_abs3      )  ,
   .c_cmd_i            ( 0        ) ,
   .c_op_i            ( 0        ) ,
   .c_dt1_i             ( 0 ) ,
   .c_dt2_i             ( 0 ) ,
   .c_dt3_i             ( 0 ) ,
   .time_rst_o         ( reset_3     ) ,
   .time_init_o        ( init_3      ) ,
   .time_off_dt_o      ( offset_dt_3 ) ,

   .start_o            ( start_3     ) ,
   .stop_o             ( stop_3      ) ,
   .time_updt_o        ( time_updt_3 ) ,
   .reset_pb           (  reset_pb            ) ,
   .pma_init           (  pma_init            ) ,
   .channel_up_A       (  channel_up_A        ) ,
   .s_axi_rx_tdata_A   (  m_axi_tx_tdata_B_2    ) ,
   .s_axi_rx_tvalid_A  (  m_axi_tx_tvalid_B_2   ) ,
   .channel_up_B       (  channel_up_B          ) ,
   .m_axi_tx_tdata_B   (  m_axi_tx_tdata_B_3    ) ,
   .m_axi_tx_tvalid_B  (  m_axi_tx_tvalid_B_3   ) ,
   .m_axi_tx_tready_B  (  t_ready  ) ,
   .s_axi_awaddr       (  0      ) ,
   .s_axi_awprot       (  0      ) ,
   .s_axi_awvalid      (  0      ) ,
   .s_axi_awready      (        ) ,
   .s_axi_wdata        (  0      ) ,
   .s_axi_wstrb        (  0      ) ,
   .s_axi_wvalid       (  0      ) ,
   .s_axi_wready       (        ) ,
   .s_axi_bresp        (        ) ,
   .s_axi_bvalid       (        ) ,
   .s_axi_bready       (  0      ) ,
   .s_axi_araddr       (  0      ) ,
   .s_axi_arprot       (  0      ) ,
   .s_axi_arvalid      (  0      ) ,
   .s_axi_arready      (        ) ,
   .s_axi_rdata        (        ) ,
   .s_axi_rresp        (        ) ,
   .s_axi_rvalid       (        ) ,
   .s_axi_rready       (        ) );


reg reset_i, start_i, stop_i, init_i;
reg  [47:0] offset_dt_i ;
reg time_updt_i;



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

channel_up_A       = 1'b0;
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
   offset_dt_i       = 0 ;
   #10 ;

   @ (posedge ps_clk); #0.1;
   rst_ni            = 1'b1;
   #10 ;

   @ (posedge user_clock); #0.1;
   channel_up_A       = 1'b1;
   #10;

/*
   @ (posedge user_clock); #0.1;
   h_type   = 3'b000 ;
   h_cmd    = 6'b000000 ;
   h_flags  = 5'b00000 ;
   h_src    = 9'd1 ;
   h_dst    = 9'd3 ;
   s_axi_rx_tdata_A   = {h_type, h_cmd, h_flags, h_src, h_dst, 96'b0};
   s_axi_rx_tvalid_A  = 1'b1;
*/

   @ (posedge user_clock); #0.1;
   s_axi_rx_tdata_A   = 127'd0;
   s_axi_rx_tvalid_A  = 1'b0;

// GET_NET COMMAND
   wait (ready_1==1'b1);
   #100;
   @ (posedge user_clock); #0.1;
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
   @ (posedge user_clock); #0.1;
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
   @ (posedge user_clock); #0.1;
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
   @ (posedge user_clock); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b01001 ;
   c_dt_1_i  = 31'd2 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = {16'd3, 16'd1} ;
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
   @ (posedge user_clock); #0.1;
   #10;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b1 ;
   c_op_i    = 5'b01010 ;
   c_dt_1_i  = 31'd5 ;
   c_dt_2_i  = 31'd10 ;
   c_dt_3_i  = {16'd3, 16'd2} ;
   @ (posedge c_clk); #0.1;
   c_cmd_i   = 1'b0 ;
   c_op_i    = 5'b00000 ;
   c_dt_1_i  = 31'd0 ;
   c_dt_2_i  = 31'd0 ;
   c_dt_3_i  = 31'd0 ;
   #100;
         
   // TEST_QNET ();
   // TEST_AXI ();
   wait (ready_1==1'b1);
   //GET NET
   WRITE_AXI( TNET_CTRL, 1); // GET_NET (NO PARAMETER)

   //SET NET
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 2); // SET_NET  (NO PARAMETER / Automatic > RTD - CD - NN - ID) 
   // SYNC_NET
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 8); // SYNC_NET (NO PARAMETER / Automatic > Delay, TimeWait)

   //UPDF_OFF
   WRITE_AXI( REG_AXI_DT1 , 4); // DATA
   WRITE_AXI( TNET_CFG    , {16'd3, 16'd0}); // NODE
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 9); // UPDT_OFFSET (NODE - DATA)

//UPDF_OFF
   WRITE_AXI( REG_AXI_DT1 , 2); // DATA
   WRITE_AXI( TNET_CFG    , {16'd2, 16'd0}); // NODE
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 9); // UPDT_OFFSET (NODE - DATA)


   //SET_DT
   WRITE_AXI( REG_AXI_DT1 , 15); // DATA1
   WRITE_AXI( REG_AXI_DT2 , 255); // DATA2
   WRITE_AXI( TNET_CFG    , {16'd3, 16'd0}); // NODE
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 10); // UPDT_OFFSET (NODE - DATA)

   //SET_DT
   WRITE_AXI( REG_AXI_DT1 , 7); // DATA1
   WRITE_AXI( REG_AXI_DT2 , 127); // DATA2
   WRITE_AXI( TNET_CFG    , {16'd2, 16'd0}); // NODE
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CTRL, 10); // UPDT_OFFSET (NODE - DATA)

   //GET_DT
   wait (ready_1==1'b1);
   WRITE_AXI( TNET_CFG    , {16'd2, 16'd0}); // NODE
   WRITE_AXI( TNET_CTRL, 11); // UPDT_OFFSET (NODE - DATA)

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

task TEST_QNET (); begin
   $display("-----Sending AURORA PACKET");
   for ( cnt = 0 ; cnt  < 16; cnt=cnt+1) begin
      channel_up_A       = 1'b1;
      #10;
      @ (posedge user_clock); #0.1;
      h_type   = 3'b000 ;
      h_cmd    = 6'b000000 ;
      h_flags  = 5'b00000 ;
      h_src    = 9'd1 ;
      h_dst    = cnt ;
      s_axi_rx_tdata_A   = {h_type, h_cmd, h_flags, h_src, h_dst, 96'b0};
      s_axi_rx_tvalid_A  = 1'b1;
      @ (posedge user_clock); #0.1;
      s_axi_rx_tdata_A   = ~cnt;
      s_axi_rx_tvalid_A  = 1'b1;
      @ (posedge user_clock); #0.1;
      s_axi_rx_tdata_A   = 127'd0;
      s_axi_rx_tvalid_A  = 1'b0;
   end
      @ (posedge user_clock); #0.1;
      h_dst    = ~0 ;
      s_axi_rx_tdata_A   = {h_type, h_cmd, h_flags, h_src, h_dst, 96'b0};
      s_axi_rx_tvalid_A  = 1'b1;
      @ (posedge user_clock); #0.1;
      s_axi_rx_tdata_A   = 127'd0;
      s_axi_rx_tvalid_A  = 1'b0;
end
endtask


endmodule




