
module aurora_ctrl ( 
   input  wire             user_clk_i     ,
   input  wire             user_rst_i     ,
// Transmittion 
   input  wire             tx_req_ti      ,
   input  wire [63 :0]     tx_header_ti   ,
   input  wire [63 :0]     tx_data_ti     ,
   output reg              tx_ack_uo      ,
// Command Processing  
   input  wire [9 :0]      ID             ,
   input  wire [9 :0]      NN             ,
   output reg              cmd_req_o      ,
   output reg  [63:0]      cmd_o[2]       ,
   input  wire             channel_ok_i       ,
   output reg              ready_o            ,
   output reg  [2:0]       debug_do    ,
   
////////////////   LINK CHANNEL A
   input  wire  [63:0]     s_axi_rx_tdata_RX  ,
   input  wire             s_axi_rx_tvalid_RX ,
   input  wire             s_axi_rx_tlast_RX  ,
////////////////   LINK CHANNEL B
   output reg  [63:0]      m_axi_tx_tdata_TX  ,
   output reg              m_axi_tx_tvalid_TX ,
   output reg              m_axi_tx_tlast_TX  ,
   input  wire             m_axi_tx_tready_TX );


reg idle_ing;
/////////////////////////////////////////////////
// RECEIVE
reg            rx_req, rx_ack;
reg  [63:0]    rx_h_buff;
reg  [63:0]    rx_dt_buff;

reg net_id_ok, net_dst_ones, net_dst_zeros, net_dst_own,net_dst_other; 
reg net_sync, net_process, net_propagate ;
reg [9:0] h_dst, h_src, h_step, h_step_new;
reg [5:0] h_flags;
reg tvalid_r;
// Capture Data From Channel_RX
always_ff @ (posedge user_clk_i) begin
   if (user_rst_i) begin
//      rx_req      <= 1'b0;
      rx_h_buff   <= 63'd0 ;
      rx_dt_buff  <= 63'd0 ;
      tvalid_r    <= 1'b0;
   end else begin
      if (s_axi_rx_tvalid_RX) begin
         if (idle_ing) begin
//            rx_req      <= 1'b1;
            rx_h_buff   <= s_axi_rx_tdata_RX ;
         end else
            rx_dt_buff  <= s_axi_rx_tdata_RX ;
      end
//      if (rx_ack) rx_req   <= 1'b0;
   end
end

// DECODING INCOMING TRANSMISSION
always_comb begin
  h_flags        = rx_h_buff[55:50] ;
  h_dst          = rx_h_buff[49:40] ;
  h_src          = rx_h_buff[39:30] ;
  h_step         = rx_h_buff[29:20] ;
  h_step_new     = h_step + 1'b1;
  net_id_ok          = |ID ;
  net_dst_ones       =  &h_dst ; //Send to ALL
  net_dst_zeros      = ~|h_dst ; //Send to NO_ONE
  net_dst_own        = net_id_ok & (h_dst == ID);
  net_dst_other      =  ~(net_dst_own | net_dst_ones);
  net_sync           = h_flags[5];
  net_process        = net_dst_own   | net_dst_ones  ;
  net_propagate      = msg_ok & (net_dst_other | net_dst_zeros) ;
end
/////////////////////////////////////////////////
// CHECK TRANSMIT
reg msg_ok;
always_comb begin
   msg_ok = 1'b1;
   if (NN != 0 & (h_step > NN) ) msg_ok = 1'b0;
end

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
      tx_h_buff  = tx_header_ti ;
      tx_dt_buff = tx_data_ti;
   end else begin
      // Data comes from NETWORK (Only change is INCRESE the STEP)
      tx_h_buff  = { rx_h_buff[63:30], h_step_new, rx_h_buff[19:0] } ;
      tx_dt_buff = rx_dt_buff;
   end
end



enum {NOT_READY, IDLE, RX, PROCESS, PROPAGATE, TX_H, TX_D, WAIT_nREQ } tnet_st_nxt, tnet_st;


always_ff @(posedge user_clk_i)
   if (user_rst_i)   tnet_st  <= NOT_READY;
   else              tnet_st  <= tnet_st_nxt;

always_comb begin
   tnet_st_nxt        = tnet_st; //Stay current state
   rx_ack             = 1'b0;
   tx_ack_uo          = 1'b0 ;

   m_axi_tx_tdata_TX  = tx_h_buff;
   m_axi_tx_tvalid_TX = 1'b0;
   m_axi_tx_tlast_TX  = 1'b0;

   idle_ing        = 1'b0;
   cmd_req_o          = 1'b0;
   ready_o            = 1'b1;
   case (tnet_st)
      NOT_READY: begin
         ready_o = 1'b0;
         if ( channel_ok_i       )    tnet_st_nxt = IDLE;
      end
      IDLE: begin
         idle_ing  = 1;
         if       ( s_axi_rx_tvalid_RX    )     tnet_st_nxt = RX;
         else if  ( tx_req                )     tnet_st_nxt = TX_H;
      end
      RX: begin
         if ( s_axi_rx_tlast_RX )
            if       ( net_process        )     tnet_st_nxt = PROCESS   ;
            else if  ( net_propagate      )     tnet_st_nxt = PROPAGATE ;
      end
      PROCESS: begin
         rx_ack         = 1'b1;
         cmd_req_o      = 1'b1;
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
            tnet_st_nxt          = TX_D;
         end
      end
      TX_D: begin
         tx_ack_uo = 1'b1 ;
         if  ( m_axi_tx_tready_TX ) begin
            m_axi_tx_tvalid_TX   = 1'b1;
            m_axi_tx_tlast_TX    = 1'b1;
            m_axi_tx_tdata_TX    = tx_dt_buff;
            tnet_st_nxt          = WAIT_nREQ;
         end
      end
      WAIT_nREQ: begin
         tx_ack_uo = 1'b1 ;
         if  ( !tx_req )  tnet_st_nxt = IDLE;
      end
   endcase
end

// Propagate packages to PORT-B
reg [127:0] ZZ_CAPTURED;
always_ff @(posedge user_clk_i)
   if (user_rst_i)   ZZ_CAPTURED  <= 1'b0;
   else if (m_axi_tx_tvalid_TX) ZZ_CAPTURED  <= m_axi_tx_tdata_TX; 

// OUTPUS
assign cmd_o = {rx_h_buff, rx_dt_buff};

assign debug_do = tnet_st[2:0] ;

 
endmodule
