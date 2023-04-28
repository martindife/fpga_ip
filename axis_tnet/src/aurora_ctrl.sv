
module aurora_ctrl ( 
   input  wire             init_clk       ,
   input  wire             init_aresetn   ,
   input  wire             user_clk_i     ,
   input  wire             user_rst_i   ,
// Transmittion 
   input  wire             tx_req_ti       ,
   input  wire [127:0]     tx_dt_ti     ,
   output reg              tx_ack_uo       ,
// Command Processing  
   output reg              cmd_req_o        ,
   output reg  [31:0]      cmd_o[4]       ,
   input  wire [7 :0]      ID       ,

   input  wire             channel_ok_i        ,
   output reg             ready_o            ,
   
////////////////   LINK CHANNEL A
   input  wire  [127:0]    s_axi_rx_tdata_A    ,
   input  wire             s_axi_rx_tvalid_A   ,
   //output wire [127:0]     m_axi_tx_tdata_A    ,
   //output wire             m_axi_tx_tvalid_A   ,
   //input  wire             m_axi_tx_tready_A   ,
////////////////   LINK CHANNEL B
   //input  wire  [127:0]    s_axi_rx_tdata_B   ,
   //input  wire             s_axi_rx_tvalid_B   ,
   output reg  [127:0]     m_axi_tx_tdata_B   ,
   output reg              m_axi_tx_tvalid_B   ,
   input  wire             m_axi_tx_tready_B   ,
////////////////   LINK COMMON
   output reg              reset_pb          ,
   output reg              pma_init          );


wire           tx_req ;
wire [127:0 ]  tx_dt ;

sync_reg # (
   .DW ( 129 )
) sync_tx_i (
   .dt_i      ( {tx_req_ti, tx_dt_ti} ) ,
   .clk_i     ( user_clk_i       ) ,
   .rst_ni    ( ~user_rst_i     ) ,
   .dt_o      ( {tx_req, tx_dt}    ) );


// CAPTURING INCOMING TRANSMISSION
reg [127:0] rx_dt;
reg rx_ack, rx_req;

// Capture Data From Channel_A
always_ff @ (posedge user_clk_i) begin
   if (user_rst_i) begin
      rx_req   <= 1'b0;
      rx_dt    <= 128'd0 ;
   end else begin
      if (s_axi_rx_tvalid_A) begin
         rx_req   <= 1'b1;
         rx_dt    <= s_axi_rx_tdata_A ;
      end
      if (rx_ack) rx_req   <= 1'b0;
   end
end // Always

reg net_dst_ones, net_dst_zeros, net_dst_own,net_dst_other; 

reg [8:0] h_dst;
reg net_sync;

always_comb begin
  h_dst          = s_axi_rx_tdata_A[113:105] ;
  net_dst_ones   =  &h_dst ;
  net_dst_zeros  = ~|h_dst;
  net_dst_own    = (h_dst == ID);
  net_dst_other  =  ~(net_dst_own | net_dst_ones);
  net_sync       = s_axi_rx_tdata_A[119];
end
assign id_ok             = |ID;
assign net_first         = !rx_req  &  s_axi_rx_tvalid_A ;
assign net_process       = net_first & ( ( id_ok & net_dst_own) | net_dst_ones );
assign net_propagate     = net_first & ( net_dst_other | net_dst_zeros );
assign net_propagate_now = net_propagate & !net_sync;



// DECODING INCOMING TRANSMISSION
enum {NOT_READY, IDLE, PROCESS, PROPAGATE_SYNC, PROPAGATE_NOW, SEND, WAIT_nREQ } tnet_st_nxt, tnet_st;

always_ff @(posedge user_clk_i)
   if (user_rst_i)   tnet_st  <= NOT_READY;
   else                 tnet_st  <= tnet_st_nxt;

        
//Always receive ONE PACKET
always_comb begin
   tnet_st_nxt       = tnet_st; //Stay current state
   rx_ack            = 1'b0;
   cmd_req_o         = 1'b0;
   tx_ack_uo         = 1'b0 ;
   m_axi_tx_tdata_B  = rx_dt;
   m_axi_tx_tvalid_B = 1'b0;
   ready_o           = 1'b1;
   case (tnet_st)

      NOT_READY: begin
         ready_o = 1'b0;
         if ( channel_ok_i       )    tnet_st_nxt = IDLE;
      end
      IDLE: begin
            if       ( net_process       )    tnet_st_nxt = PROCESS;
            else if  ( net_propagate     )    tnet_st_nxt = PROPAGATE_SYNC ; // Data Comes from Transpor level
            else if  ( net_propagate_now )    tnet_st_nxt = PROPAGATE_NOW  ; // Data Comes from link level
            if       ( tx_req            )    tnet_st_nxt = SEND;
      end
      PROCESS: begin
         rx_ack         = 1'b1;
         cmd_req_o      = 1'b1;
         tnet_st_nxt    = IDLE ;
      end
      PROPAGATE_SYNC: begin
         if  ( !m_axi_tx_tready_B )    tnet_st_nxt = PROPAGATE_NOW;
      end
      PROPAGATE_NOW: begin
         if  ( m_axi_tx_tready_B ) begin
            rx_ack            = 1'b1;
            m_axi_tx_tdata_B  = rx_dt;
            m_axi_tx_tvalid_B = 1'b1;
            tnet_st_nxt       = IDLE;
         end
      end
      SEND: begin
         if  ( m_axi_tx_tready_B ) begin
            if (net_propagate) 
               m_axi_tx_tdata_B  = rx_dt;
            else
               m_axi_tx_tdata_B  = tx_dt;
            
            tx_ack_uo = 1'b1 ;
            m_axi_tx_tvalid_B = 1'b1;
            tnet_st_nxt = WAIT_nREQ;
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
   else if (m_axi_tx_tvalid_B) ZZ_CAPTURED  <= m_axi_tx_tdata_B; 






//OUTPUTS
always_ff @ (posedge init_clk) begin
   if (!init_aresetn) begin         
      pma_init   <= 1'b1 ;
      reset_pb   <= 1'b1 ;
   end else begin
      pma_init   <= 1'b0 ;
      reset_pb   <= pma_init;
   end
end

assign cmd_o = {rx_dt[127:96], rx_dt[95:64], rx_dt[63:32], rx_dt[31:0]};

assign m_axi_tx_tdata_A   = 128'd0;
assign m_axi_tx_tvalid_A  = 1'd0;
   

 
endmodule

