module qick_net_link # (
   parameter SIM_LEVEL = 1
)(
// Core, Time and AXI CLK & RST.
   input  wire             gt_refclk1_p      ,
   input  wire             gt_refclk1_n      ,
   input  wire             t_clk             ,
   input  wire             t_aresetn         ,
   input  wire             ps_clk            , //99.999001
   input  wire             ps_aresetn        ,
   input  wire  [47:0]     t_time_abs        ,
// Transmittion 
   input  wire             ID_i         ,
   input  wire             NN_i          ,
// Transmittion 
   input  wire             tx_req_i         ,
   input  wire             tx_ch_i          ,
   input  wire [63 :0]     tx_header_i      ,
   input  wire [63 :0]     tx_data_i        ,
   output reg              tx_ack_o         ,
// Command Processing  
   output reg              cmd_req_o         ,
   output reg              cmd_ch_o         ,
   output reg  [63:0]      cmd_o[2]          ,
   output reg              ready_o           ,

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


// DEBUGGING
   output wire [3:0]       aurora_do         ,
   output wire             channelA_ok_o      ,
   output wire             channelB_ok_o      ,
   output reg  [7:0]       pack_cnt_do    ,
   output reg  [3:0]       last_op_do     ,
   output reg  [2:0]       state_do       );


/////////////////////////////////////////////////
// SIGNALS 

wire axi_rx_tvalid_A_RX;
wire  [63:0]     axi_rx_tdata_A_RX;


//PHY CONNECTION SIGNALS
wire             reset_pb           ;
wire             pma_init           ;
wire             ch_up_A_RX     ;
wire             axi_rx_tvalid_A_RX, axi_rx_tlast_A_RX, axi_rx_tkeep_A_RX ;
wire  [63:0]     axi_rx_tdata_A_RX   ;
wire             ch_up_A_TX, axi_tx_tready_A_TX ;
reg              axi_tx_tvalid_A_TX, axi_tx_tlast_A_TX ;
reg  [63:0]      axi_tx_tdata_A_TX   ;
reg  [7 :0]      axi_tx_tkeep_A_TX   ;

wire             ch_up_B_RX     ;
wire             axi_rx_tvalid_B_RX, axi_rx_tlast_B_RX, axi_rx_tkeep_B_RX ;
wire  [63:0]     axi_rx_tdata_B_RX   ;
wire             ch_up_B_TX, axi_tx_tready_B_TX ;
reg              axi_tx_tvalid_B_TX, axi_tx_tlast_B_TX ;
reg  [63:0]      axi_tx_tdata_B_TX   ;
reg  [7 :0]      axi_tx_tkeep_B_TX   ;

wire txn_A, txp_A, txn_B, txp_B ;

generate
/////////////////////////////////////////////////
   if (SIM_LEVEL == 1) begin : SIM_NO_AURORA
      assign user_clk           = ps_clk;
      assign mmcm_not_locked    = ~ps_aresetn;
      assign init_clk    = 1;
      assign channel_up_ARX    = 1;
      assign channel_up_BRX    = 1;
      assign channel_up_ATX    = 1;
      assign channel_up_BTX    = 1;

// Channel A=0 B=1
      assign axi_rx_tvalid_A_RX   = axi_rx_tvalid_A_RX_i;
      assign axi_rx_tdata_A_RX    = axi_rx_tdata_A_RX_i ;
      assign axi_rx_tlast_A_RX    = axi_rx_tlast_A_RX_i ;

      assign axi_tx_tvalid_A_TX_o = axi_tx_tvalid_A_TX  ;
      assign axi_tx_tdata_A_TX_o  = axi_tx_tdata_A_TX   ;
      assign axi_tx_tlast_A_TX_o  = axi_tx_tlast_A_TX   ;
      assign axi_tx_tready_A_TX   = axi_tx_tready_A_TX_i;

      assign axi_rx_tvalid_B_RX   = axi_rx_tvalid_B_RX_i;
      assign axi_rx_tdata_B_RX    = axi_rx_tdata_B_RX_i ;
      assign axi_rx_tlast_B_RX    = axi_rx_tlast_B_RX_i ;

      assign axi_tx_tvalid_B_TX_o = axi_tx_tvalid_B_TX  ;
      assign axi_tx_tdata_B_TX_o  = axi_tx_tdata_B_TX   ;
      assign axi_tx_tlast_B_TX_o  = axi_tx_tlast_B_TX   ;
      assign axi_tx_tready_B_TX   = axi_tx_tready_B_TX_i;

   end else begin 
      if (SIM_LEVEL == 2) begin : SIM_YES_AURORA
         assign txn_A_o               = txn_A ;
         assign txp_A_o               = txp_A ;
         assign txn_B_o               = txn_B ;
         assign txp_B_o               = txp_B ;
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
      assign aurora_do[0] = ~mmcm_not_locked;
      assign aurora_do[1] = gt_pll_lock ;
      assign aurora_do[2] = ch_up_A_RX ; 
      assign aurora_do[3] = ch_up_A_TX ;
      assign aurora_do[4] = ch_up_B_RX ;
      assign aurora_do[5] = ch_up_B_TX ;
      
   end

endgenerate








reg tx_ack_uo, cmd_req_uo;
reg idle_ing;

wire init_clk;


reg [9:0] h_dst, h_src, h_step, h_step_new;
reg [5:0] h_flags;

reg msg_ok;
reg net_id_ok, net_dst_ones, net_dst_own, net_src_own; 
reg net_sync, net_process, net_propagate ;

/////////////////////////////////////////////////
// RECEIVE
reg         rx_req, rx_ack;
reg         rx_ch;
reg [63:0]  rx_h_buff;
reg [63:0]  rx_dt_buff;
reg [ 7:0]  rx_cnt;

 
// Capture Data From Channels_RX
always_ff @ (posedge user_clk_i, posedge user_rst_i) begin
   if (user_rst_i) begin
      rx_ch       <= 1'b0; 
      rx_cnt      <= 8'd0;
      rx_h_buff   <= 63'd0 ;
      rx_dt_buff  <= 63'd0 ;
   end else begin
// CHANNEL A
      if (axi_rx_tvalid_A_RX)
         if (axi_rx_tlast_A_RX) begin
            rx_ch       <= 0; 
            rx_dt_buff  <= axi_rx_tdata_A_RX ;
            rx_cnt      <= rx_cnt + 1'b1;
         end else
            rx_h_buff   <= axi_rx_tdata_A_RX ;
// CHANNEL B
      if (axi_rx_tvalid_B_RX)
         if (axi_rx_tlast_B_RX) begin
            rx_ch       <= 1; 
            rx_dt_buff  <= axi_rx_tdata_B_RX ;
            rx_cnt        <= rx_cnt + 1'b1;
         end else
            rx_h_buff   <= axi_rx_tdata_B_RX ;
   end
end

// DECODING INCOMING TRANSMISSION
always_comb begin
  h_flags         = rx_h_buff[55:50] ;
  h_dst           = rx_h_buff[49:40] ;
  h_src           = rx_h_buff[39:30] ;
  h_step          = rx_h_buff[29:20] ;
  h_step_new      = h_step + 1'b1;
  net_sync        = h_flags[5];
  net_id_ok       = |ID_i ;
  net_dst_ones    = &h_dst ; //Send to ALL
  net_dst_own     = net_id_ok & (h_dst == ID_i);
  net_src_own     =             (h_src == ID_i);
  net_process     = msg_ok & ( net_dst_own | net_src_own | net_dst_ones) ;
  net_propagate   = msg_ok & ( ~net_process ) ;
end



/////////////////////////////////////////////////
// CHECK TRANSMIT

// PACKET STPES
always_comb begin
   msg_ok = 1'b1;
   if (|ID_i & &h_step) msg_ok = 1'b0;
end

// TIMEOUT
reg [9:0] time_cnt;
reg time_cnt_msb;
always_ff @ (posedge user_clk_i, posedge user_rst_i) begin
   if (user_rst_i) begin
      time_cnt      <= 1 ;
      time_cnt_msb  <= 0 ;
   end else begin
      if (idle_ing | ~ready_o)   time_cnt  <= 0;
      else            time_cnt      <= time_cnt + 1'b1;
      time_cnt_msb  <= time_cnt[9];
   end
end
assign time_out = time_cnt[9] & ~time_cnt_msb;




/////////////////////////////////////////////////
// TRANSMIT
wire          tx_req ;
reg [63:0 ]   tx_h_buff      ;
reg [63:0 ]   tx_dt_buff     ;
sync_reg # (
   .DW ( 1 )
) sync_tx_i (
   .dt_i      ( {tx_req_ti } ) ,
   .clk_i     ( user_clk_i       ) ,
   .rst_ni    ( ~user_rst_i     ) ,
   .dt_o      ( {tx_req}    ) );

// SET TRANSMISSION DATA
always_comb begin
   if (tx_req) begin
      // Data comes from TX_REQ
      tx_h_buff  = tx_header_i ;
      tx_dt_buff = tx_data_i;
   end else begin
      // Data comes from NETWORK (Only change is INCRESE the STEP)
      tx_h_buff  = { rx_h_buff[63:30], h_step_new, rx_h_buff[19:0] } ;
      tx_dt_buff = rx_dt_buff;
   end
end


enum {NOT_READY=0, IDLE=1, RX=2, PROCESS=3, PROPAGATE=4, TX_H=5, TX_D=6, WAIT_nREQ=7 } tnet_st_nxt, tnet_st;

always_ff @(posedge user_clk_i)
   if (user_rst_i)   tnet_st  <= NOT_READY;
   else              tnet_st  <= tnet_st_nxt;



always_comb begin
   tnet_st_nxt          = tnet_st; //Stay current state
   rx_ack               = 1'b0;
   tx_ack_uo            = 1'b0 ;
   axi_tx_tdata_TX      = tx_h_buff;
   axi_tx_tvalid_TX     = 1'b0;
   axi_tx_tlast_TX      = 1'b0;
   axi_tx_channel_TX    = 1'b0;
   idle_ing             = 1'b0;
   cmd_req_uo            = 1'b0;
   ready_o              = 1'b1;
   case (tnet_st)
      NOT_READY: begin
         ready_o = 1'b0;
         if ( channel_ok_i       )    tnet_st_nxt = IDLE;
      end
      IDLE: begin
         idle_ing  = 1;
         if       ( s_axi_rx_tvalid_RX    )  tnet_st_nxt = RX;
         else if  ( tx_req                )  tnet_st_nxt = TX_H;
      end
      RX: begin
         if ( s_axi_rx_tlast_RX )
            if       ( net_process        )  tnet_st_nxt = PROCESS   ;
            else if  ( net_propagate      )  tnet_st_nxt = PROPAGATE ;
      end
      PROCESS: begin
         rx_ack         = 1'b1;
         cmd_req_uo      = 1'b1;
         tnet_st_nxt    = IDLE ;
      end
      PROPAGATE: begin
         if ( !net_sync | (net_sync & !m_axi_tx_tready_TX)) tnet_st_nxt = TX_H;
      end
      TX_H: begin
         tx_ack_uo = 1'b1 ;
         if  ( m_axi_tx_tready_TX ) begin
            m_axi_tx_tvalid_TX   = 1'b1;
            m_axi_tx_tdata_TX    = tx_h_buff;
            tnet_st_nxt            = TX_D;
         end
      end
      TX_D: begin
         tx_ack_uo = 1'b1 ;
         if  ( m_axi_tx_tready_TX ) begin
            m_axi_tx_tvalid_TX   = 1'b1;
            m_axi_tx_tlast_TX    = 1'b1;
            m_axi_tx_tdata_TX    = tx_dt_buff;
            tnet_st_nxt            = WAIT_nREQ;
         end
      end
      WAIT_nREQ: begin
         tx_ack_uo = 1'b1 ;
         if  ( !tx_req )  tnet_st_nxt = IDLE;
      end
   endcase
   // IF TIMEOUT OR CHANNEL NOT READY
   if ( !channel_ok_i | time_out )     tnet_st_nxt = NOT_READY;
end

// OUTPUTS
always_ff @(posedge user_clk_i)
   if (user_rst_i)   begin
      tx_ack_o  <= 1'b0;
      cmd_req_o <= 1'b0;
   end else begin
      tx_ack_o  <= tx_ack_uo;
      cmd_req_o  <= cmd_req_uo;
   end


assign cmd_o       = {rx_h_buff, rx_dt_buff};
assign pack_cnt_do = rx_cnt ;
assign last_op_do  = rx_h_buff[59:56] ;
assign state_do    = tnet_st[2:0] ;
 



assign axi_tx_tkeep_TX = {8{axi_tx_tlast_TX}};

endmodule

