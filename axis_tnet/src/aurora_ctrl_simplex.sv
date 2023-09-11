module aurora_ctrl_simplex # (
   parameter SIM_LEVEL = 1
)( 
// Core, Time and AXI CLK & RST.
   input  wire             gt_refclk1_p     ,
   input  wire             gt_refclk1_n     ,
   input  wire             t_clk_i          ,
   input  wire             t_rst_ni         ,
   input  wire             ps_clk_i         , //99.999001
   input  wire             ps_rst_ni        ,
// Data 
   input  wire  [6 :0]     ID_i             ,
   input  wire  [6 :0]     NN_i             ,
// Transmittion 
   input  wire             tx_req_i         ,
   input  wire [63 :0]     tx_header_i      ,
   input  wire [31 :0]     tx_data_i [2]       ,
   output reg              tx_ack_o        ,
   output wire             sync_tx         ,
   output wire             sync_cc         ,
   output reg [31:0]       qnet_LINK_o     ,
// Command Processing  
   output reg              cmd_net_o       ,
   output reg  [63:0]      cmd_o[2]        ,
   output reg              ready_o         ,
///////////////// SIMULATION    
   input  wire             rxn_A_i        ,
   input  wire             rxp_A_i        ,
   output wire             txn_B_o        ,
   output  wire            txp_B_o        ,
////////////////   LINK CHANNEL A
   input  wire  [63:0]     s_axi_rx_tdata_RX_i  ,
   input  wire             s_axi_rx_tvalid_RX_i ,
   input  wire             s_axi_rx_tlast_RX_i  ,
////////////////   LINK CHANNEL B
   output reg  [63:0]      m_axi_tx_tdata_TX_o  ,
   output reg              m_axi_tx_tvalid_TX_o ,
   output reg              m_axi_tx_tlast_TX_o  ,
   input  wire             m_axi_tx_tready_TX_i ,
// DEBUGGING
   output wire [3:0]       aurora_do        ,
   output wire             channel_TX_up   ,
   output wire             channel_RX_up   ,
   output reg  [7:0]       pack_cnt_do     ,
   output reg  [3:0]       last_op_do      ,
   output reg  [2:0]       state_do        );


////////////////////////////////////////////////
// SIGNALS 

//PHY CONNECTION SIGNALS
reg             reset_pb           ;
reg             pma_init           ;
wire init_clk;

// A RX and B TX CONNECTION
wire             axi_rx_tvalid_RX, axi_rx_tlast_RX   ;
wire  [63:0]     axi_rx_tdata_RX    ;
wire             axi_tx_tready_TX ;
reg              axi_tx_tvalid_TX, axi_tx_tlast_TX ;
reg  [63:0]      axi_tx_tdata_TX    ;
wire  [7 :0]     axi_tx_tkeep_TX ;





// NETWORK LINK MEASURES
///////////////////////////////////////////////////////////////////////////////


// Detect rising and falling Edge on axi_tx_tready_TX
reg axi_ready_TX_r, axi_ready_TX_r2, axi_ready_TX_r3;
always_ff @ (posedge user_clk, posedge user_rst) begin
   if (user_rst) begin
      axi_ready_TX_r    <= 0;
      axi_ready_TX_r2   <= 0;
      axi_ready_TX_r3   <= 0;
   end else begin
      axi_ready_TX_r     <= axi_tx_tready_TX;
      axi_ready_TX_r2    <= axi_ready_TX_r;
      axi_ready_TX_r3    <= axi_ready_TX_r2;
   end
end

reg tx_rdy_r, tx_rdy_r2 ;
(* ASYNC_REG = "TRUE" *) reg tx_rdy_cdc, tx_rdy_000_cdc;
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      tx_rdy_000_cdc   <= 0;
      tx_rdy_r     <= 0;
      tx_rdy_r2    <= 0;
   end else begin
      tx_rdy_000_cdc     <= axi_tx_tready_TX;
      tx_rdy_r       <= tx_rdy_000_cdc;
      tx_rdy_r2      <= tx_rdy_r;
   end
   
wire axi_ready_TX_000;
assign axi_ready_TX_000    = !axi_ready_TX_r3 & !axi_ready_TX_r2 & !axi_ready_TX_r;

wire tx_rdy_01, tx_rdy_10, tx_rdy_cc;
assign tx_rdy_01  = !tx_rdy_r2     &  tx_rdy_r ;
assign tx_rdy_10  =  tx_rdy_r2     & !tx_rdy_r ;
assign tx_rdy_cc  = tx_rdy_01 & axi_ready_TX_000 ;

wire sync_tx, sync_cc;
assign sync_tx  = tx_rdy_01 & !axi_ready_TX_000 ;
assign sync_cc  = tx_rdy_01 &  axi_ready_TX_000 ;

/// Measure LINK Reception Time after a CC
reg cnt_LINK_rst, cnt_LINK_en, update_LINK;
reg [31:0] cnt_LINK ;

enum {MEAS_IDLE, MEAS_LINK, MEAS_TSTART, MEAS_TEND} meas_net_st_nxt, meas_net_st;
always_ff @(posedge t_clk_i)
   if (!t_rst_ni)     meas_net_st  <= MEAS_IDLE;
   else              meas_net_st  <= meas_net_st_nxt;

always_comb begin
   meas_net_st_nxt = meas_net_st;
   cnt_LINK_rst = 1'b0;
   cnt_LINK_en  = 1'b0;
   update_LINK  = 1'b0;
   case (meas_net_st)
      MEAS_IDLE: begin
         if ( sync_cc       )    meas_net_st_nxt = MEAS_LINK;
      end
      MEAS_LINK: begin
         cnt_LINK_rst = 1'b1;
         if ( sync_tx       )    meas_net_st_nxt = MEAS_TSTART;
      end
      MEAS_TSTART: begin
         cnt_LINK_en = 1'b1;
         if      ( sync_tx       )    meas_net_st_nxt = MEAS_TEND;
         else if ( sync_cc       )    meas_net_st_nxt = MEAS_LINK;
      end
      MEAS_TEND: begin
         update_LINK  = 1'b1;
         meas_net_st_nxt   = MEAS_IDLE;
      end
   endcase
end

always_ff @(posedge t_clk_i) begin
   if       ( cnt_LINK_rst )  cnt_LINK <= 32'd0;
   else if  ( cnt_LINK_en  )  cnt_LINK <= cnt_LINK + 1'b1;
   else if  ( update_LINK  )  qnet_LINK_o <= cnt_LINK; 
end





// LINK CONNECTION ///////////////////////////////////////////////////////////////////////////////
wire user_clk, mmcm_not_locked;
wire user_rst;


generate
/////////////////////////////////////////////////
   if (SIM_LEVEL == 1) begin : SIM_NO_AURORA
      assign txn_B_o             = 0 ;
      assign txp_B_o             = 0 ;
      assign init_clk            = 1;
      assign user_clk            = ps_clk_i   ;
      assign mmcm_not_locked     = ~ps_rst_ni ;
      assign channel_RX_up        = 1;
      assign channel_TX_up        = 1;
      assign aurora_do           = 0;

// A RX and B TX CONNECTION
      assign axi_rx_tvalid_RX   = s_axi_rx_tvalid_RX_i;
      assign axi_rx_tdata_RX    = s_axi_rx_tdata_RX_i ;
      assign axi_rx_tlast_RX    = s_axi_rx_tlast_RX_i ;

      assign m_axi_tx_tvalid_TX_o = axi_tx_tvalid_TX  ;
      assign m_axi_tx_tdata_TX_o  = axi_tx_tdata_TX   ;
      assign m_axi_tx_tlast_TX_o  = axi_tx_tlast_TX   ;
      assign axi_tx_tready_TX     = m_axi_tx_tready_TX_i;

 end else begin 
      if (SIM_LEVEL == 2) begin : SIM_YES_AURORA
         assign m_axi_tx_tdata_TX_o   = 0 ;
         assign m_axi_tx_tvalid_TX_o  = 0 ;
         assign m_axi_tx_tlast_TX_o   = 0 ;
      end else begin : SYNT_AURORA
         assign txn_A_o             = 0 ;
         assign txp_A_o             = 0 ;
         assign txn_B_o             = 0 ;
         assign txp_B_o             = 0 ;
      end
      /////// INIT CLK
      clk_wiz_0 CLK_AURORA (
         .clk_out1      ( init_clk    ), // 14.99985
         .resetn        ( ps_rst_ni  ), // input reset
         .locked        ( locked      ), // output locked
         .clk_in1       ( ps_clk_i      )); // input clk_in1

   // RECEIVING PORT
   aurora_64b66b_SL AURORA_RX  (
     .rxn                  ( rxn_A_i ),
     .rxp                  ( rxp_A_i ),
     .gt_refclk1_p         ( gt_refclk1_p       ), // input wire gt_refclk1_p
     .gt_refclk1_n         ( gt_refclk1_n       ), // input wire gt_refclk1_n
     .init_clk             ( init_clk           ), // input wire init_clk
     .reset_pb             ( reset_pb           ), // input wire reset_pb
     .pma_init             ( pma_init           ), // input wire pma_init
     .m_axi_rx_tvalid      ( axi_rx_tvalid_RX  ), // output wire m_axi_rx_tvalid
     .m_axi_rx_tdata       ( axi_rx_tdata_RX   ), // output wire [0 : 63] m_axi_rx_tdata
     .m_axi_rx_tkeep       ( axi_rx_tkeep_RX   ), // output wire [0 : 7] m_axi_rx_tkeep
     .m_axi_rx_tlast       ( axi_rx_tlast_RX   ), // output wire m_axi_rx_tlast
     .power_down           ( 1'b0               ), // input wire power_down
     .gt_rxcdrovrden_in    ( 1'b0               ), // input wire gt_rxcdrovrden_in
     .gt_refclk1_out       ( gt_refclk1         ), // output wire gt_refclk1_out
     .user_clk_out         ( user_clk           ), // output wire user_clk_out
     .mmcm_not_locked_out  ( mmcm_not_locked    ), // output wire mmcm_not_locked_out
     .reset2fc             (                    ), // output wire reset2fc
     .rx_channel_up        ( channel_RX_up      ), // output wire channel_up
     .rx_lane_up           ( lane_RX_up         ), // output wire lane_up
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
///// TX PORT
   aurora_64b66b_NSL AURORA_TX  (
   // FOR SIMULATIONS
      .txn                 ( txn_B_o ),
      .txp                 ( txp_B_o ),
      .init_clk            ( init_clk           ), // input wire init_clk
   // QPLL CONTROL
      .mmcm_not_locked     ( mmcm_not_locked  ), // input wire mmcm_not_locked
      .refclk1_in          ( gt_refclk1       ), // input wire refclk1_in
      .user_clk            ( user_clk         ), // input wire user_clk
      .sync_clk            (          ), // input wire sync_clk
      .reset_pb            ( sys_reset        ), // input wire reset_pb
      .pma_init            ( gt_reset         ), // input wire pma_init
      .s_axi_tx_tvalid     ( axi_tx_tvalid_TX  ), // input wire s_axi_tx_tvalid
      .s_axi_tx_tdata      ( axi_tx_tdata_TX   ), // input wire [0 : 127] s_axi_tx_tdata
      .s_axi_tx_tkeep      ( axi_tx_tkeep_TX   ), // input wire [0 : 15] s_axi_tx_tkeep
      .s_axi_tx_tlast      ( axi_tx_tlast_TX   ), // input wire s_axi_tx_tlast
      .s_axi_tx_tready     ( axi_tx_tready_TX  ), // output wire s_axi_tx_tready
      .tx_channel_up       ( channel_TX_up     ), // output wire channel_up
      .tx_lane_up          ( lane_TX_up        ), // output wire [0 : 1] lane
      .power_down          ( 1'b0             ), // input wire power_down
      .gt_rxcdrovrden_in   ( 1'b0             ), // input wire gt_rxcdrovrden_in
      .gt0_drpaddr         ( 10'd0 ),// input wire [9 : 0] gt0_drpaddr
      .gt0_drpdi           ( 16'd0 ),// input wire [15 : 0] gt0_drpdi
      .gt0_drpwe           ( 1'b0  ),// input wire gt0_drpwe
      .gt0_drpen           ( 1'b0  ),// input wire gt0_drpen
      .tx_hard_err         (  ), 
      .tx_soft_err         (  ), 
      .tx_out_clk          (  ), 
      .bufg_gt_clr_out     (  ), 
      .gt_pll_lock         (  ), 
      .gt0_drprdy          (  ), 
      .gt0_drpdo           (  ), 
      .link_reset_out      (  ), 
      .reset2fg            (  ), 
      .sys_reset_out       (  ), 
      .gt_powergood        (  )
      );
   
      assign aurora_do[0] = ~mmcm_not_locked;
      assign aurora_do[1] = gt_pll_lock ;
      assign aurora_do[2] = channel_RX_up ; 
      assign aurora_do[3] = channel_TX_up ;
      
   end

endgenerate




always_ff @ (posedge init_clk, negedge ps_rst_ni) begin
   if (!ps_rst_ni) begin         
      pma_init   <= 1'b1 ;
      reset_pb   <= 1'b1 ;
   end else begin
      pma_init   <= 1'b0 ;
      reset_pb   <= pma_init;
   end
end


assign axi_tx_tkeep_TX = {8{axi_tx_tlast_TX}};
assign user_rst = mmcm_not_locked;










// RECEIVE
///////////////////////////////////////////////////////////////////////////////
reg [63:0]  RX_h_buff, RX_dt_buff;
reg [ 7:0]  RX_cnt;
reg         RX_req;

// Capture Data From RX
always_ff @ (posedge user_clk, posedge user_rst) begin
   if (user_rst) begin
      RX_cnt      <= 8'd0;
      RX_h_buff   <= 63'd0 ;
      RX_dt_buff  <= '{default:'0};
   end else begin
      if (axi_rx_tvalid_RX)
         if (axi_rx_tlast_RX) begin
            RX_dt_buff  <= axi_rx_tdata_RX ;
            RX_cnt      <= RX_cnt + 1'b1;
         end else
            RX_h_buff   <= axi_rx_tdata_RX ;
   end
end

// DECODING INCOMING TRANSMISSION
wire [ 1:0] cmd_type;
wire [ 4:0] cmd_id;
wire [ 2:0] cmd_flg;
wire [ 9:0] cmd_dst, cmd_src, cmd_step, cmd_step_p1;
wire [23:0] cmd_hdt;
wire last_step;

assign cmd_type = RX_h_buff[63:62];
assign cmd_id   = RX_h_buff[61:57];
assign cmd_flg  = RX_h_buff[56:54];
assign cmd_dst  = RX_h_buff[53:44];
assign cmd_src  = RX_h_buff[43:34];
assign cmd_step = RX_h_buff[33:24];
assign cmd_hdt  = RX_h_buff[23: 0];
assign cmd_step_p1   = cmd_step + 1'b1;
assign last_step     = &cmd_step ;

reg net_id_ok, net_dst_ones, net_dst_own, net_src_own; 
reg net_sync, net_process, net_propagate ;


always_comb begin
  net_sync        = cmd_flg[2];
  net_id_ok       = |ID_i ;
  net_dst_ones    = &cmd_dst ; //Send to ALL
  net_dst_own     = net_id_ok & (cmd_dst == ID_i);
  net_src_own     =             (cmd_src == ID_i);
  net_process     = !last_step & ( net_dst_own | net_src_own | net_dst_ones) ;
  net_propagate   = !last_step & ( ~net_process ) ;
end









// TRANSMIT
///////////////////////////////////////////////////////////////////////////////
wire TX_req_loc ;
reg TX_req_net ;
reg TX_ack, cmd_req_set;

sync_reg # (
   .DW ( 1 )
) sync_tx_i (
   .dt_i      ( {tx_req_i } ) ,
   .clk_i     ( user_clk       ) ,
   .rst_ni    ( ~user_rst       ) ,
   .dt_o      ( {TX_req_loc}    ) );

reg          TX_req ;
reg [63:0 ]   TX_h_buff, TX_dt_buff      ;

always_ff @ (posedge user_clk, posedge user_rst) begin
   if (user_rst) begin
      TX_req     <= 1'b0;
      TX_h_buff  <= 63'd0 ;
      TX_dt_buff <= 63'd0;
   end else begin
      if (TX_req_loc) begin
         TX_req     <= 1'b1;
         TX_h_buff  <= tx_header_i ;
         TX_dt_buff <= {tx_data_i[1],tx_data_i[0]} ;
       end else if (TX_req_net) begin
         TX_req     <= 1'b1;
         TX_h_buff  <= { RX_h_buff[63:30], cmd_step_p1, RX_h_buff[19:0] } ;
         TX_dt_buff <= RX_dt_buff;
      end
      if (TX_req & TX_ack) 
         if (!TX_req_loc & !TX_req_net)
            TX_req    <= 0;
   end
end

// TIMEOUT
reg [9:0] time_cnt;
reg time_cnt_msb;
always_ff @ (posedge user_clk, posedge user_rst) begin
   if (user_rst) begin
      time_cnt      <= 1 ;
      time_cnt_msb  <= 0 ;
   end else begin
      if (idle_ing | ~ready_o)   time_cnt  <= 0;
      else            time_cnt      <= time_cnt + 1'b1;
      time_cnt_msb  <= time_cnt[9];
   end
end
assign time_out = time_cnt[9] & ~time_cnt_msb;

enum {NOT_RDY, IDLE, RX, PROCESS, PROPAGATE, TX_H, TX_D, WAIT_nREQ } tnet_st_nxt, tnet_st;

always_ff @(posedge user_clk)
   if (user_rst)     tnet_st  <= NOT_RDY;
   else              tnet_st  <= tnet_st_nxt;

assign channel_ok = channel_RX_up & channel_TX_up;

reg cmd_req ;
reg idle_ing;

always_comb begin
   tnet_st_nxt          = tnet_st; //Stay current state
   TX_req_net     = 1'b0;
   TX_ack         = 1'b0 ;
   idle_ing             = 1'b0;
   cmd_req_set         = 1'b0;
   ready_o              = 1'b1;
   axi_tx_tvalid_TX   = 1'b0;
   axi_tx_tdata_TX    = 63'd0;
   axi_tx_tlast_TX    = 1'b0;


   case (tnet_st)
      NOT_RDY: begin
         ready_o = 1'b0;
         if ( channel_ok       )    tnet_st_nxt = IDLE;
      end
      IDLE: begin
         idle_ing  = 1;
         if       ( axi_rx_tlast_RX )  tnet_st_nxt = RX;
         else if  ( TX_req          )  tnet_st_nxt = TX_H;
      end
      RX: begin
         if       ( net_process        )  tnet_st_nxt = PROCESS   ;
         else if  ( net_propagate      )  tnet_st_nxt = PROPAGATE ;
      end
      PROCESS: begin
         cmd_req_set  = 1'b1;
         if ( cmd_req ) tnet_st_nxt    = IDLE ;
      end
      PROPAGATE: begin
         TX_req_net = 1'b1 ;
         if ( !net_sync )
            tnet_st_nxt = TX_H ;
         else if ( !axi_tx_tready_TX ) 
           tnet_st_nxt = TX_H;
      end
      TX_H: begin
         TX_ack = 1'b1 ;
         if  ( axi_tx_tready_TX ) begin
            axi_tx_tvalid_TX   = 1'b1;
            axi_tx_tdata_TX    = TX_h_buff;
            tnet_st_nxt        = TX_D;
         end
      end
      TX_D: begin
         TX_ack = 1'b1 ;
         if  ( axi_tx_tready_TX ) begin
            axi_tx_tvalid_TX   = 1'b1;
            axi_tx_tlast_TX    = 1'b1;
            axi_tx_tdata_TX    = TX_dt_buff;
            tnet_st_nxt        = WAIT_nREQ;
         end
      end
      
   
      WAIT_nREQ: begin
         TX_ack = 1'b1 ;
         if  ( !TX_req )  tnet_st_nxt = IDLE;
      end
   endcase
   // IF TIMEOUT OR CHANNEL NOT READY
   if ( !channel_ok | time_out )     tnet_st_nxt = NOT_RDY;
end






always_ff @(posedge user_clk)
   if (user_rst)   begin
      cmd_req <= 1'b0;
   end else begin
      if ( cmd_req_set) begin
         if ( net_sync )  
            if ( tx_rdy_10 ) cmd_req  <= 1'b1;
         else 
            cmd_req  <= 1'b1;
      end else 
            cmd_req  <= 1'b0;
   end


reg cmd_req_r, cmd_req_r2;
(* ASYNC_REG = "TRUE" *) reg cmd_req_cdc;
always_ff @(posedge t_clk_i)
   if (!t_rst_ni) begin
      cmd_req_cdc   <= 0;
      cmd_req_r     <= 0;
      cmd_req_r2    <= 0;
   end else begin
      cmd_req_cdc     <= cmd_req;
      cmd_req_r       <= cmd_req_cdc;
      cmd_req_r2      <= cmd_req_r;
   end


 

     
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// DEBUG
///////////////////////////////////////////////////////////////////////////////
assign pack_cnt_do = RX_cnt ;
assign last_op_do  = RX_h_buff[59:56] ;
assign state_do    = tnet_st[2:0] ;      


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// OUTPUTS
///////////////////////////////////////////////////////////////////////////////
assign tx_ack_o   = TX_ack ;
assign cmd_net_o  = !cmd_req_r2  & cmd_req_r ;
assign cmd_o      = {RX_h_buff, RX_dt_buff} ;


endmodule
