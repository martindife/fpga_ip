module axis_tnet_simplex # (
   parameter SIM_LEVEL = 1
)(
// Core, Time and AXI CLK & RST.
   input  wire             gt_refclk1_p      ,
   input  wire             gt_refclk1_n      ,
   input  wire             t_clk             ,
   input  wire             t_aresetn         ,
   input  wire             c_clk             ,
   input  wire             c_aresetn         ,
   input  wire             ps_clk            , //99.999001
   input  wire             ps_aresetn        ,
   input  wire  [47:0]     t_time_abs        ,
   input  wire             net_sync          ,
// TPROC CONTROL
   input  wire             c_cmd_i           ,
   input  wire  [4:0]      c_op_i            ,
   input  wire  [31:0]     c_dt1_i           ,
   input  wire  [31:0]     c_dt2_i           ,
   input  wire  [31:0]     c_dt3_i           ,
   output wire             c_ready_o         ,
   output reg              core_start_o      ,
   output reg              core_stop_o       ,
   output reg              time_rst_o        ,
   output reg              time_init_o       ,
   output reg              time_updt_o       ,
   output reg  [31:0]      time_off_dt_o     ,
   output reg  [31:0]      tnet_dt1_o        ,
   output reg  [31:0]      tnet_dt2_o        ,
///////////////// SIMULATION    
   input  wire             rxn_A_i        ,
   input  wire             rxp_A_i        ,
   output wire             txn_B_o        ,
   output  wire            txp_B_o        ,
////////////////   CHANNEL A LINK
   input  wire             axi_rx_tvalid_A_RX_i  ,
   input  wire  [63:0]     axi_rx_tdata_A_RX_i   ,
   input  wire             axi_rx_tlast_A_RX_i   ,
////////////////   CHANNEL B LINK
   output reg   [63:0]     axi_tx_tdata_B_TX_o   ,
   output reg              axi_tx_tvalid_B_TX_o  ,
   output reg              axi_tx_tlast_B_TX_o   ,
   input  wire             axi_tx_tready_B_TX_i  ,
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

qick_net_simplex # (
   .SIM_LEVEL ( SIM_LEVEL )
) QICK_NET (
   .gt_refclk1_p         ( gt_refclk1_p         ) ,
   .gt_refclk1_n         ( gt_refclk1_n         ) ,
   .t_clk_i              ( t_clk                ) ,
   .t_rst_ni             ( t_aresetn            ) ,
   .c_clk_i              ( c_clk                ) ,
   .c_rst_ni             ( c_aresetn            ) ,
   .ps_clk_i             ( ps_clk               ) ,
   .ps_rst_ni            ( ps_aresetn           ) ,
   .t_time_abs           ( t_time_abs           ) ,
   .net_sync_i           ( net_sync             ) ,
   .c_cmd_i              ( c_cmd_i              ) ,
   .c_op_i               ( c_op_i               ) ,
   .c_dt1_i              ( c_dt1_i              ) ,
   .c_dt2_i              ( c_dt2_i              ) ,
   .c_dt3_i              ( c_dt3_i              ) ,
   .c_ready_o            ( c_ready_o            ) ,
   .core_start_o         ( core_start_o         ) ,
   .core_stop_o          ( core_stop_o          ) ,
   .time_rst_o           ( time_rst_o           ) ,
   .time_init_o          ( time_init_o          ) ,
   .time_updt_o          ( time_updt_o          ) ,
   .time_off_dt_o        ( time_off_dt_o        ) ,
   .tnet_dt1_o           ( tnet_dt1_o           ) ,
   .tnet_dt2_o           ( tnet_dt2_o           ) ,
   .rxn_A_i              ( rxn_A_i              ) ,
   .rxp_A_i              ( rxp_A_i              ) ,
   .txn_B_o              ( txn_B_o              ) ,
   .txp_B_o              ( txp_B_o              ) ,
   .axi_rx_tvalid_A_RX_i ( axi_rx_tvalid_A_RX_i ) ,
   .axi_rx_tdata_A_RX_i  ( axi_rx_tdata_A_RX_i  ) ,
   .axi_rx_tlast_A_RX_i  ( axi_rx_tlast_A_RX_i  ) ,
   .axi_tx_tdata_B_TX_o  ( axi_tx_tdata_B_TX_o  ) ,
   .axi_tx_tvalid_B_TX_o ( axi_tx_tvalid_B_TX_o ) ,
   .axi_tx_tlast_B_TX_o  ( axi_tx_tlast_B_TX_o  ) ,
   .axi_tx_tready_B_TX_i ( axi_tx_tready_B_TX_i ) ,
   .s_axi_awaddr         ( s_axi_awaddr         ) ,
   .s_axi_awprot         ( s_axi_awprot         ) ,
   .s_axi_awvalid        ( s_axi_awvalid        ) ,
   .s_axi_awready        ( s_axi_awready        ) ,
   .s_axi_wdata          ( s_axi_wdata          ) ,
   .s_axi_wstrb          ( s_axi_wstrb          ) ,
   .s_axi_wvalid         ( s_axi_wvalid         ) ,
   .s_axi_wready         ( s_axi_wready         ) ,
   .s_axi_bresp          ( s_axi_bresp          ) ,
   .s_axi_bvalid         ( s_axi_bvalid         ) ,
   .s_axi_bready         ( s_axi_bready         ) ,
   .s_axi_araddr         ( s_axi_araddr         ) ,
   .s_axi_arprot         ( s_axi_arprot         ) ,
   .s_axi_arvalid        ( s_axi_arvalid        ) ,
   .s_axi_arready        ( s_axi_arready        ) ,
   .s_axi_rdata          ( s_axi_rdata          ) ,
   .s_axi_rresp          ( s_axi_rresp          ) ,
   .s_axi_rvalid         ( s_axi_rvalid         ) ,
   .s_axi_rready         ( s_axi_rready         ) );

/*
wire             channel_up_RX        ;
wire             axi_rx_tvalid_RX, axi_rx_tlast_RX   ;
wire  [63:0]     axi_rx_tdata_RX    ;
wire             channel_up_TX        ;
wire             axi_tx_tready_TX, axi_tx_tvalid_TX, axi_tx_tlast_TX ;
wire  [63:0]     axi_tx_tdata_TX    ;
wire  [7 :0]     axi_tx_tkeep_TX , axi_rx_tkeep_RX;
wire [4:0] aurora_dbg;


*/


endmodule

