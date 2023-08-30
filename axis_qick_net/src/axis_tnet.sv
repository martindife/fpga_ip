module axis_qick_net # (
   parameter SIM_LEVEL = 0
)(
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
   input  wire  [47:0]     t_time_abs        ,
// SIMULATION    
   input  wire             rxn_i           ,
   input  wire             rxp_i           ,
   output wire             txn_o           ,
   output  wire            txp_o           ,

////////////////   LINK CHANNEL A
   input  wire             axi_rx_tvalid_RX_i  ,
   input  wire  [63:0]     axi_rx_tdata_RX_i   ,
   input  wire             axi_rx_tlast_RX_i   ,
////////////////   LINK CHANNEL B
   output reg  [63:0]      axi_tx_tdata_TX_o   ,
   output reg              axi_tx_tvalid_TX_o  ,
   output reg              axi_tx_tlast_TX_o   ,
   input  wire             axi_tx_tready_TX_i  ,
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
   output reg  [31:0]      time_dt_o         ,
   output reg  [31:0]      tnet_dt1_o        ,
   output reg  [31:0]      tnet_dt2_o        ,
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
wire             channel_up_RX        ;
wire             axi_rx_tvalid_RX, axi_rx_tlast_RX   ;
wire  [63:0]     axi_rx_tdata_RX    ;
wire             channel_up_TX        ;
wire             axi_tx_tready_TX, axi_tx_tvalid_TX, axi_tx_tlast_TX ;
wire  [63:0]     axi_tx_tdata_TX    ;
wire  [7 :0]     axi_tx_tkeep_TX , axi_rx_tkeep_RX;

wire [4:0] aurora_dbg;
wire user_clk;
wire init_clk;

qick_net QICK_NET (
   .t_clk               ( t_clk             ) ,
   .t_aresetn           ( t_aresetn         ) ,
   .c_clk               ( c_clk             ) ,
   .c_aresetn           ( c_aresetn         ) ,
   .ps_clk              ( ps_clk            ) ,
   .ps_aresetn          ( ps_aresetn        ) ,
   .init_clk            ( init_clk          ) ,
   .init_aresetn        ( locked            ) ,
   .user_clk_i          ( user_clk          ) ,
   .user_rst_i          ( mmcm_not_locked   ) ,
   .t_time_abs          ( t_time_abs        ) ,
   .c_cmd_i             ( c_cmd_i           ) ,
   .c_op_i              ( c_op_i            ) ,
   .c_dt1_i             ( c_dt1_i           ) ,
   .c_dt2_i             ( c_dt2_i           ) ,
   .c_dt3_i             ( c_dt3_i           ) ,
   .c_ready_o           ( c_ready_o         ) ,
   .core_start_o        ( core_start_o        ) ,
   .core_stop_o         ( core_stop_o       ) ,
   .time_rst_o          ( time_rst_o        ) ,
   .time_init_o         ( time_init_o       ) ,
   .time_updt_o         ( time_updt_o       ) ,
   .time_off_dt_o       ( time_dt_o         ) ,
   .tnet_dt1_o          ( tnet_dt1_o        ) ,
   .tnet_dt2_o          ( tnet_dt2_o        ) ,
   .aurora_dbg          ( aurora_dbg        ) ,
   .reset_pb            ( reset_pb          ) ,
   .pma_init            ( pma_init          ) ,
   .channel_up_RX       ( channel_up_RX      ) ,
   .s_axi_rx_tvalid_RX  ( axi_rx_tvalid_RX ), 
   .s_axi_rx_tdata_RX   ( axi_rx_tdata_RX    ),
   .s_axi_rx_tlast_RX   ( axi_rx_tlast_RX   ),
   .channel_up_TX       ( channel_up_TX      ) ,
   .m_axi_tx_tdata_TX   ( axi_tx_tdata_TX  ) ,
   .m_axi_tx_tvalid_TX  ( axi_tx_tvalid_TX ) ,
   .m_axi_tx_tlast_TX   ( axi_tx_tlast_TX   ),
   .m_axi_tx_tready_TX  ( axi_tx_tready_TX ) ,
   .s_axi_awaddr        ( s_axi_awaddr      ) ,
   .s_axi_awprot        ( s_axi_awprot      ) ,
   .s_axi_awvalid       ( s_axi_awvalid     ) ,
   .s_axi_awready       ( s_axi_awready     ) ,
   .s_axi_wdata         ( s_axi_wdata       ) ,
   .s_axi_wstrb         ( s_axi_wstrb       ) ,
   .s_axi_wvalid        ( s_axi_wvalid      ) ,
   .s_axi_wready        ( s_axi_wready      ) ,
   .s_axi_bresp         ( s_axi_bresp       ) ,
   .s_axi_bvalid        ( s_axi_bvalid      ) ,
   .s_axi_bready        ( s_axi_bready      ) ,
   .s_axi_araddr        ( s_axi_araddr      ) ,
   .s_axi_arprot        ( s_axi_arprot      ) ,
   .s_axi_arvalid       ( s_axi_arvalid     ) ,
   .s_axi_arready       ( s_axi_arready     ) ,
   .s_axi_rdata         ( s_axi_rdata       ) ,
   .s_axi_rresp         ( s_axi_rresp       ) ,
   .s_axi_rvalid        ( s_axi_rvalid      ) ,
   .s_axi_rready        ( s_axi_rready      ) );

wire txn, txp;


endmodule

