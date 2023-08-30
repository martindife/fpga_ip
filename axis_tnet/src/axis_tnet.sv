module axis_tnet # (
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
   .core_start_o        ( core_start_o      ) ,
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

generate
/////////////////////////////////////////////////
   if (SIM_LEVEL == 1) begin : SIM_NO_AURORA
      assign user_clk           = ps_clk;
      assign mmcm_not_locked    = ~ps_aresetn;
      assign init_clk    = 1;
      assign channel_up_RX    = 1;
      assign channel_up_TX    = 1;

      assign axi_rx_tvalid_RX   = axi_rx_tvalid_RX_i ;
      assign axi_rx_tdata_RX    = axi_rx_tdata_RX_i  ;
      assign axi_rx_tlast_RX    = axi_rx_tlast_RX_i  ;
      assign axi_tx_tdata_TX_o  = axi_tx_tdata_TX  ;
      assign axi_tx_tvalid_TX_o = axi_tx_tvalid_TX ;
      assign axi_tx_tlast_TX_o  = axi_tx_tlast_TX  ;
      assign axi_tx_tready_TX   = axi_tx_tready_TX_i ;
      
   end else begin 
      if (SIM_LEVEL == 2) begin : SIM_YES_AURORA
         assign txn_o               = txn ;
         assign txp_o               = txp ;
         assign axi_tx_tdata_TX_o   = 0 ;
         assign axi_tx_tvalid_TX_o  = 0 ;
         assign axi_tx_tlast_TX_o   = 0 ;
      end else begin : SYNT_AURORA
         assign txn_o = 0 ;
         assign txp_o = 0 ;
      end

      /////// INIT CLK
      clk_wiz_0 CLK_AURORA (
         .clk_out1      ( init_clk    ), // 14.99985
         .resetn        ( ps_aresetn  ), // input reset
         .locked        ( locked      ), // output locked
         .clk_in1       ( ps_clk      )); // input clk_in1

      ///////// RECEIVE PORT
      aurora_64b66b_SL AURORA_RX  (
        .rxn                  ( rxn_i ),
        .rxp                  ( rxp_i ),
        .gt_refclk1_p         ( gt_refclk1_p       ), // input wire gt_refclk1_p
        .gt_refclk1_n         ( gt_refclk1_n       ), // input wire gt_refclk1_n
        .init_clk             ( init_clk           ), // input wire init_clk
        .reset_pb             ( reset_pb           ), // input wire reset_pb
        .pma_init             ( pma_init           ), // input wire pma_init
        .m_axi_rx_tvalid      ( axi_rx_tvalid_RX   ), // output wire m_axi_rx_tvalid
        .m_axi_rx_tdata       ( axi_rx_tdata_RX    ), // output wire [0 : 63] m_axi_rx_tdata
        .m_axi_rx_tkeep       ( axi_rx_tkeep_RX    ), // output wire [0 : 7] m_axi_rx_tkeep
        .m_axi_rx_tlast       ( axi_rx_tlast_RX    ), // output wire m_axi_rx_tlast
        .power_down           ( 1'b0               ), // input wire power_down
         //  .loopback             ( 3'b000               ), // input wire [2 : 0] loopback
        .gt_rxcdrovrden_in    ( 1'b0               ), // input wire gt_rxcdrovrden_in
        .gt_refclk1_out       ( gt_refclk1         ), // output wire gt_refclk1_out
        .user_clk_out         ( user_clk           ), // output wire user_clk_out
        //.sync_clk_out         ( sync_clk           ), // output wire sync_clk_out
        .mmcm_not_locked_out  ( mmcm_not_locked    ), // output wire mmcm_not_locked_out
        .reset2fc(),                        // output wire reset2fc
        .rx_channel_up        ( channel_up_RX      ), // output wire channel_up
        .rx_lane_up           ( lane_up_RX         ), // output wire lane_up
        .rx_hard_err          (                    ), // output wire hard_err
        .rx_soft_err          (                    ), // output wire soft_err
        .tx_out_clk           (                    ), // output wire tx_out_clk
        .gt_pll_lock          ( gt_pll_lock        ), // output wire gt_pll_lock
        .gt0_drpaddr          ( 10'd0              ), // input wire [9 : 0] gt0_drpaddr
        .gt0_drpdi            ( 16'd0              ), // input wire [15 : 0] gt0_drpdi
        .gt0_drprdy           (                    ), // output wire gt0_drprdy
        .gt0_drpwe            ( 1'b0               ), // input wire gt0_drpwe
        .gt0_drpen            ( 1'b0               ), // input wire gt0_drpen
        .gt0_drpdo            (                    ), // output wire [15 : 0] gt0_drpdo
        .link_reset_out       (                    ), // output wire link_reset_out
        .sys_reset_out        ( sys_reset          ), // output wire sys_reset_out
        .gt_reset_out         ( gt_reset           ), // output wire gt_reset_out
        .gt_powergood         (                    )  // output wire [1 : 0] gt_powergood
      );
      /////// TRANSMIT PORT
      aurora_64b66b_NSL AURORA_TX  (
         .txn                 ( txn ),
         .txp                 ( txp ),
         // QPLL CONTROL
         .init_clk            ( init_clk          ), // input wire init_clk
         .mmcm_not_locked     ( mmcm_not_locked   ), // input wire mmcm_not_locked
         .refclk1_in          ( gt_refclk1        ), // input wire refclk1_in
         .user_clk            ( user_clk          ), // input wire user_clk
         .sync_clk            (                   ), // input wire sync_clk
         .reset_pb            ( sys_reset         ), // input wire reset_pb
         .pma_init            ( gt_reset          ), // input wire pma_init
         .s_axi_tx_tvalid     ( axi_tx_tvalid_TX  ), // input wire s_axi_tx_tvalid
         .s_axi_tx_tdata      ( axi_tx_tdata_TX   ), // input wire [0 : 127] s_axi_tx_tdata
         .s_axi_tx_tkeep      ( axi_tx_tkeep_TX   ), // input wire [0 : 15] s_axi_tx_tkeep
         .s_axi_tx_tlast      ( axi_tx_tlast_TX   ), // input wire s_axi_tx_tlast
         .s_axi_tx_tready     ( axi_tx_tready_TX  ), // output wire s_axi_tx_tready
         .tx_channel_up       ( channel_up_TX     ), // output wire channel_up
         .tx_lane_up          (                   ), // output wire [0 : 1] lane_up
         .power_down          ( 1'b0              ), // input wire power_down
         //   .loopback            ( 3'b000           ), // input wire [2 : 0] loopback
         .gt_rxcdrovrden_in   ( 1'b0             ), // input wire gt_rxcdrovrden_in
         .gt0_drpaddr         ( 10'd0 ),// input wire [9 : 0] gt0_drpaddr
         .gt0_drpdi           ( 16'd0 ),// input wire [15 : 0] gt0_drpdi
         .gt0_drpwe           ( 1'b0  ),// input wire gt0_drpwe
         .gt0_drpen           ( 1'b0  ),// input wire gt0_drpen
         .tx_hard_err         (  ) , 
         .tx_soft_err         (  ) , 
         .tx_out_clk          (  ) , 
         .bufg_gt_clr_out     (  ) , 
         .gt_pll_lock         (  ) , 
         .gt0_drprdy          (  ) , 
         .gt0_drpdo           (  ) , 
         .link_reset_out      (  ) , 
         .reset2fg            (  ) , 
         .sys_reset_out       (  ) , 
         .gt_powergood        (  ) );
      assign aurora_dbg[0] = ~mmcm_not_locked;
      assign aurora_dbg[1] = gt_pll_lock ;
      assign aurora_dbg[2] = lane_up_RX ; 
      assign aurora_dbg[3] = channel_up_RX ;
      assign aurora_dbg[4] = channel_up_TX ;
   end

endgenerate

assign axi_tx_tkeep_TX = {8{axi_tx_tlast_TX}};



endmodule

