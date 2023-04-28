module axis_tnet (
// Core, Time and AXI CLK & RST.
   input  wire             gt_refclk1_p      ,
   input  wire             gt_refclk1_n      ,
//   output wire             user_clk          ,
   input  wire             t_clk             ,
   input  wire             t_aresetn         ,
   input  wire             c_clk             ,
   input  wire             c_aresetn         ,
   input  wire             ps_clk            , //99.999001
   input  wire             ps_aresetn        ,
//   input  wire             init_clk          ,
//   input  wire             init_aresetn      ,
   input  wire  [47:0]     t_time_abs        ,
// TPROC CONTROL     
   input  wire             c_cmd_i           ,
   input  wire  [4:0]      c_op_i            ,
   input  wire  [31:0]     c_dt1_i           ,
   input  wire  [31:0]     c_dt2_i           ,
   input  wire  [31:0]     c_dt3_i           ,
   output wire             c_ready_o         ,
   output reg              time_rst_o        ,
   output reg              time_init_o       ,
   output reg              time_updt_o       ,
   output reg  [31:0]      time_off_dt_o     ,
   output reg              start_o           ,
   output reg              pause_o           ,
   output reg              stop_o            ,
// AXI-Lite DATA Slave I/F.   
   input  wire [5:0]       s_axi_awaddr      ,
   input  wire [2:0]       s_axi_awprot      ,
   input  wire             s_axi_awvalid     ,
   output wire             s_axi_awready     ,
   input  wire [31:0]      s_axi_wdata       ,
   input  wire [ 3:0]      s_axi_wstrb       ,
   input  wire             s_axi_wvalid      ,
   output wire             s_axi_wready      ,
   output wire [ 1:0]      s_axi_bresp       ,
   output wire             s_axi_bvalid      ,
   input  wire             s_axi_bready      ,
   input  wire [ 5:0]      s_axi_araddr      ,
   input  wire [ 2:0]      s_axi_arprot      ,
   input  wire             s_axi_arvalid     ,
   output wire             s_axi_arready     ,
   output wire [31:0]      s_axi_rdata       ,
   output wire [ 1:0]      s_axi_rresp       ,
   output wire             s_axi_rvalid      ,
   input  wire             s_axi_rready      );

wire             reset_pb            ;
wire             pma_init            ;
wire             channel_up_A        ;
wire  [127:0]    axi_rx_tdata_A    ;
wire             axi_rx_tvalid_A   ;
wire             channel_up_B        ;
wire  [127:0]    axi_tx_tdata_B    ;
wire             axi_tx_tvalid_B   ;
wire             axi_tx_tready_B   ;

wire init_clk;

    
clk_wiz_0 CLK_AURORA (
   .clk_out1      ( init_clk    ), // 14.99985
   .resetn        ( ps_aresetn  ), // input reset
   .locked        ( locked      ), // output locked
   .clk_in1       ( ps_clk      )); // input clk_in1
   
// RECEIVING PORT
aurora_64b66b_SL AURORA_A  (
  .rxn                  ( txnB),
  .rxp                  ( txnB),
  .txn                  ( txnA),
  .txp                  ( txpA),
  .gt_refclk1_p         ( gt_refclk1_p       ), // input wire gt_refclk1_p
  .gt_refclk1_n         ( gt_refclk1_n       ), // input wire gt_refclk1_n
  .init_clk             ( init_clk           ), // input wire init_clk
  .reset_pb             ( reset_pb           ), // input wire reset_pb
  .pma_init             ( pma_init           ), // input wire pma_init
  .s_axi_tx_tdata       ( 0   ), // input wire [0 : 127] s_axi_tx_tdata
//  .s_axi_tx_tkeep       ( 0   ), // input wire [0 : 15] s_axi_tx_tkeep
//  .s_axi_tx_tlast       ( 0   ), // input wire s_axi_tx_tlast
  .s_axi_tx_tvalid      ( 0  ), // input wire s_axi_tx_tvalid
  .s_axi_tx_tready      (   ), // output wire s_axi_tx_tready
  .m_axi_rx_tvalid      ( axi_rx_tvalid_A  ), // output wire m_axi_rx_tvalid
  .m_axi_rx_tdata       ( axi_rx_tdata_A   ), // output wire [0 : 127] m_axi_rx_tdata
//  .m_axi_rx_tkeep       ( axi_rx_tkeep_A   ), // output wire [0 : 15] m_axi_rx_tkeep
//  .m_axi_rx_tlast       ( axi_rx_tlast_A   ), // output wire m_axi_rx_tlast
  .power_down           ( 1'b0               ), // input wire power_down
  .loopback             ( 3'b0               ), // input wire [2 : 0] loopback
  .gt_rxcdrovrden_in    ( 1'b0               ), // input wire gt_rxcdrovrden_in
  .gt_refclk1_out       ( gt_refclk1         ), // output wire gt_refclk1_out
  .user_clk_out         ( user_clk           ), // output wire user_clk_out
  .sync_clk_out         ( sync_clk           ), // output wire sync_clk_out
  .mmcm_not_locked_out  ( mmcm_not_locked    ), // output wire mmcm_not_locked_out
  .channel_up           ( channel_up_A       ), // output wire channel_up
  .lane_up              (                    ), // output wire [0 : 1] lane_up
  .hard_err             (                    ), // output wire hard_err
  .soft_err             (                    ), // output wire soft_err
  .tx_out_clk           (                    ), // output wire tx_out_clk
  .gt_pll_lock          (                    ), // output wire gt_pll_lock
  .gt0_drpaddr          ( 0                   ), // input wire [9 : 0] gt0_drpaddr
  .gt1_drpaddr          ( 0                   ), // input wire [9 : 0] gt1_drpaddr
  .gt0_drpdi            ( 0                   ), // input wire [15 : 0] gt0_drpdi
  .gt1_drpdi            ( 0                   ), // input wire [15 : 0] gt1_drpdi
  .gt0_drprdy           (                    ), // output wire gt0_drprdy
  .gt1_drprdy           (                    ), // output wire gt1_drprdy
  .gt0_drpwe            ( 0                   ), // input wire gt0_drpwe
  .gt1_drpwe            ( 0                   ), // input wire gt1_drpwe
  .gt0_drpen            ( 0                   ), // input wire gt0_drpen
  .gt1_drpen            ( 0                   ), // input wire gt1_drpen
  .gt0_drpdo            (                    ), // output wire [15 : 0] gt0_drpdo
  .gt1_drpdo            (                    ), // output wire [15 : 0] gt1_drpdo
  .link_reset_out       (                    ), // output wire link_reset_out
  .sys_reset_out        ( sys_reset          ), // output wire sys_reset_out
  .gt_reset_out         ( gt_reset           ), // output wire gt_reset_out
  .gt_powergood         (                    )  // output wire [1 : 0] gt_powergood
);

wire [1:0] txnA, txpA;
wire [1:0] txnB, txpB;
wire user_clk;

// SENDING PORT
aurora_64b66b_NSL AURORA_B  (
// FOR SIMULATIONS
  .rxn                  ( txnA),
  .rxp                  ( txpA),
  .txn                  ( txnB),
  .txp                  ( txpB),
  .init_clk             ( init_clk           ), // input wire init_clk

// QPLL CONTROL
  .mmcm_not_locked   (mmcm_not_locked  ), // input wire mmcm_not_locked
  .refclk1_in        ( gt_refclk1       ), // input wire refclk1_in
  .user_clk          ( user_clk         ), // input wire user_clk
  .sync_clk          ( sync_clk         ), // input wire sync_clk
  .reset_pb          ( sys_reset        ), // input wire reset_pb
  .pma_init          ( gt_reset         ), // input wire pma_init


  .s_axi_tx_tvalid      ( axi_tx_tvalid_B  ), // input wire s_axi_tx_tvalid
  .s_axi_tx_tdata       ( axi_tx_tdata_B   ), // input wire [0 : 127] s_axi_tx_tdata
//  .s_axi_tx_tkeep       ( axi_tx_tkeep_B   ), // input wire [0 : 15] s_axi_tx_tkeep
//  .s_axi_tx_tlast       ( axi_tx_tlast_B   ), // input wire s_axi_tx_tlast
  .s_axi_tx_tready      ( axi_tx_tready_B  ), // output wire s_axi_tx_tready
  .m_axi_rx_tdata       (    ), // output wire [0 : 127] m_axi_rx_tdata
//  .m_axi_rx_tkeep       (    ), // output wire [0 : 15] m_axi_rx_tkeep
//  .m_axi_rx_tlast       (    ), // output wire m_axi_rx_tlast
  .m_axi_rx_tvalid      (    ), // output wire m_axi_rx_tvalid
  .channel_up           ( channel_up_B       ), // output wire channel_up
  .lane_up              (                    ), // output wire [0 : 1] lane_up
  .power_down           ( 1'b0               ), // input wire power_down
  .loopback             ( 3'b0               ), // input wire [2 : 0] loopback
  .gt_rxcdrovrden_in    ( 1'b0               ), // input wire gt_rxcdrovrden_in
  .gt0_drpaddr          ( 10'b0 ),// input wire [9 : 0] gt0_drpaddr
  .gt1_drpaddr          ( 10'b0 ),// input wire [9 : 0] gt1_drpaddr
  .gt0_drpdi            ( 16'b0 ),// input wire [15 : 0] gt0_drpdi
  .gt1_drpdi            ( 16'b0 ),// input wire [15 : 0] gt1_drpdi
  .gt0_drpwe            ( 1'b0  ),// input wire gt0_drpwe
  .gt1_drpwe            ( 1'b0  ),// input wire gt1_drpwe
  .gt0_drpen            ( 1'b0  ),// input wire gt0_drpen
  .gt1_drpen            ( 1'b0  )// input wire gt1_drpen

);


qick_net QICK_NET (
   .t_clk             ( t_clk             ) ,
   .t_aresetn         ( t_aresetn         ) ,
   .c_clk             ( c_clk             ) ,
   .c_aresetn         ( c_aresetn         ) ,
   .ps_clk            ( ps_clk            ) ,
   .ps_aresetn        ( ps_aresetn        ) ,
   .init_clk          ( init_clk          ) ,
   .init_aresetn      ( init_aresetn      ) ,
   .user_clk_i        ( user_clk          ) ,
   .user_rst_i        ( mmcm_not_locked  ) ,
   .t_time_abs        ( t_time_abs        ) ,
   .c_cmd_i           ( c_cmd_i           ) ,
   .c_op_i            ( c_op_i            ) ,
   .c_dt1_i           ( c_dt1_i           ) ,
   .c_dt2_i           ( c_dt2_i           ) ,
   .c_dt3_i           ( c_dt3_i           ) ,
   .c_ready_o         ( c_ready_o         ) ,
   .time_rst_o        ( time_rst_o        ) ,
   .time_init_o       ( time_init_o       ) ,
   .time_updt_o       ( time_updt_o       ) ,
   .time_off_dt_o     ( time_off_dt_o     ) ,
   .start_o           ( start_o           ) ,
   .pause_o           ( pause_o           ) ,
   .stop_o            ( stop_o            ) ,
   .reset_pb          ( reset_pb          ) ,
   .pma_init          ( pma_init          ) ,
   .channel_up_A      ( channel_up_A      ) ,
   .s_axi_rx_tdata_A  ( axi_rx_tdata_A  ) ,
   .s_axi_rx_tvalid_A ( axi_rx_tvalid_A ) ,
   .channel_up_B      ( channel_up_B      ) ,
   .m_axi_tx_tdata_B  ( axi_tx_tdata_B  ) ,
   .m_axi_tx_tvalid_B ( axi_tx_tvalid_B ) ,
   .m_axi_tx_tready_B ( axi_tx_tready_B ) ,
   .s_axi_awaddr      ( s_axi_awaddr      ) ,
   .s_axi_awprot      ( s_axi_awprot      ) ,
   .s_axi_awvalid     ( s_axi_awvalid     ) ,
   .s_axi_awready     ( s_axi_awready     ) ,
   .s_axi_wdata       ( s_axi_wdata       ) ,
   .s_axi_wstrb       ( s_axi_wstrb       ) ,
   .s_axi_wvalid      ( s_axi_wvalid      ) ,
   .s_axi_wready      ( s_axi_wready      ) ,
   .s_axi_bresp       ( s_axi_bresp       ) ,
   .s_axi_bvalid      ( s_axi_bvalid      ) ,
   .s_axi_bready      ( s_axi_bready      ) ,
   .s_axi_araddr      ( s_axi_araddr      ) ,
   .s_axi_arprot      ( s_axi_arprot      ) ,
   .s_axi_arvalid     ( s_axi_arvalid     ) ,
   .s_axi_arready     ( s_axi_arready     ) ,
   .s_axi_rdata       ( s_axi_rdata       ) ,
   .s_axi_rresp       ( s_axi_rresp       ) ,
   .s_axi_rvalid      ( s_axi_rvalid      ) ,
   .s_axi_rready      ( s_axi_rready      ) );


endmodule

