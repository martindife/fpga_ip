`include "_qnet_defines.svh"

module qnet_cmd_dec (
   input  wire             c_clk_i             ,
   input  wire             c_rst_ni            ,
   input  wire             t_clk_i             ,
   input  wire             t_rst_ni            ,
   input  wire             param_NN            ,
   input  wire             param_ID            ,
   input  wire             c_cmd_i             ,
   input  wire  [4:0]      c_op_i              ,
   input  wire  [31:0]     c_dt_i [3]          ,
   input  wire  [4:0]      p_op_i              ,
   input  wire  [31:0]     p_dt_i [3]          ,
   input  wire             net_cmd_i           ,
   input  wire  [63:0]     net_cmd_h_i         ,
   input  wire  [63:0]     net_cmd_dt_i         ,
   output  wire [63:0]     header_o            ,
   output  wire [31:0]     data_o  [2]            ,
   output  wire            loc_cmd_req_o          ,
   input   wire            loc_cmd_ack_i          ,
   output  wire            net_cmd_req_o          ,
   input   wire            net_cmd_ack_i             
   );


///////////////////////////////////////////////////////////////////////////////
// PYTHON  COMMAND  CONTROL
reg        p_cmd_req;
reg [4:0]  p_cmd_op;
reg [31:0] p_cmd_dt[2];
reg [31:0] p_cmd_hdt;
reg p_cmd_in_r;
wire p_cmd_in;
wire [4:0] control_op;   

// AXI Command Register
sync_reg # (.DW ( 5 ) )       sync_pcmd_op (
   .dt_i      ( p_op_i ) ,
   //.dt_i      ( TNET_CTRL[5:0] ) ,
   .clk_i     ( t_clk_i      ) ,
   .rst_ni    ( t_rst_ni  ) ,
   .dt_o      ( {control_op}  ) );


// AXI Command Register
always_ff @(posedge t_clk_i) 
   if (!t_rst_ni) begin
      p_cmd_req   <= 1'b0;
      p_cmd_op    <= 5'd0;
      p_cmd_dt    <= '{default:'0};
      p_cmd_hdt   <= 32'd0;
      p_cmd_in_r  <= 0;
   end else begin 
      p_cmd_in_r <= p_cmd_in;
      if (p_cmd_in_t01) begin
         p_cmd_req  <= 1'b1;
         p_cmd_op   <= control_op[4:0] ;
         p_cmd_dt   <= {p_dt_i[0], p_dt_i[1] }  ;
         p_cmd_hdt  <= p_dt_i[2] ;
         //p_cmd_dt   <= {REG_AXI_DT1, REG_AXI_DT2 }  ;
         //p_cmd_hdt  <= REG_AXI_DT3 ;
      end
      if ( loc_cmd_ack_i ) p_cmd_req  <= 1'b0;
   end

assign p_cmd_in     =  |control_op ; 
assign p_cmd_in_t01 =  !p_cmd_in_r & p_cmd_in;




///////////////////////////////////////////////////////////////////////////////
// QICK PROCESSOR COMMAND CONTROL
reg         c_cmd_r;
reg  [4:0]  c_cmd_op;
reg  [31:0] c_cmd_dt [2];
reg  [31:0] c_cmd_hdt ;

sync_reg # ( .DW ( 1 ) )      sync_tcmd_ack (
   .dt_i      ( loc_cmd_ack_i ) ,
   .clk_i     ( c_clk_i       ) ,
   .rst_ni    ( c_rst_ni   ) ,
   .dt_o      ( c_cmd_ack   ) );

// Processor Command Register
always_ff @(posedge c_clk_i) 
   if (!c_rst_ni) begin
      c_cmd_r     <= 1'b0;
      c_cmd_op    <= 5'd0;
      c_cmd_dt    <= '{default:'0};
      c_cmd_hdt   <= 32'd0;
   end else begin 
      if (c_cmd_i) begin
         c_cmd_r   <= 1'b1;
         c_cmd_op  <= c_op_i  ;
         c_cmd_dt  <= {c_dt_i[0], c_dt_i[1]} ;
         c_cmd_hdt <= c_dt_i[2]  ;
      end
      if ( c_cmd_ack ) c_cmd_r  <= 1'b0;
   end

reg         c_cmd_req;
sync_reg # (.DW ( 1 ) )       sync_cmd_req (
   .dt_i      ( c_cmd_r ) ,
   .clk_i     ( t_clk_i      ) ,
   .rst_ni    ( t_rst_ni  ) ,
   .dt_o      ( {c_cmd_req}  ) );



// LOCAL COMMAND OPERATON DECODER
reg [4:0]  loc_cmd_op;
reg [63:0] loc_cmd_header;
reg [31:0] loc_cmd_dt[2];
reg [31:0] loc_cmd_hdt;

  
 
always_comb begin
   if (p_cmd_req) begin
      loc_cmd_op  = p_cmd_op  ;
      loc_cmd_dt  = p_cmd_dt  ;
      loc_cmd_hdt = p_cmd_hdt ;
   end else begin
      loc_cmd_op  = c_cmd_op  ;
      loc_cmd_dt  = c_cmd_dt  ;
      loc_cmd_hdt = c_cmd_hdt ;
   end
//////////////////////////////////////////////////////////    CFG_CMD___SXCWOA___DEST_______SOURCE_____STEP_______ID0________ID1
   if      (loc_cmd_op == _get_net    ) loc_cmd_header =  64'b100_00001_100100_1111111111_0000000001_0000000000_0000000000_0000000001;
   else if (loc_cmd_op == _set_net    ) loc_cmd_header = {44'b100_00010_100100_1111111111_0000000001_0000000000, param_NN, 10'b0000000010} ;
   else if (loc_cmd_op == _sync_net   ) loc_cmd_header =  64'b100_01000_100100_1111111111_0000000001_0000000000_0000000000_0000000000;
   else if (loc_cmd_op == _get_off    ) loc_cmd_header = {14'b100_00101_000100, loc_cmd_hdt[9:0] , param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _updt_off   ) loc_cmd_header = {14'b100_01001_000100, loc_cmd_hdt[9:0] , param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _set_dt     ) loc_cmd_header = {14'b100_01010_100100, loc_cmd_hdt[9:0] , param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _get_dt     ) loc_cmd_header = {14'b100_01011_100100, loc_cmd_hdt[9:0] , param_ID, 20'b0000000000_0000000000, loc_cmd_hdt[25:16]};
   else if (loc_cmd_op == _rst_tproc  ) loc_cmd_header = {24'b100_10000_100100_1111111111, param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _start_core ) loc_cmd_header = {24'b100_10001_100100_1111111111, param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _stop_core  ) loc_cmd_header = {24'b100_10010_100100_1111111111, param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _set_cond   ) loc_cmd_header = {24'b100_10011_100100_1111111111, param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _clear_cond ) loc_cmd_header = {24'b100_10100_100100_1111111111, param_ID, 30'b0000000000_0000000000_0000000000};
   else if (loc_cmd_op == _custom     ) loc_cmd_header = {24'b100_10100_100100_1111111111, param_ID, 30'b0000000000_0000000000_0000000000};
   else                                 loc_cmd_header =  64'b000_00000_000000_0000000000_0000000000_0000000000_0000000000_0000000000;
   end

//         tx_cmd_header   = {14'b100_01011_000001, cmd_header_r[39:30] , param_ID, 20'b0000000000_0000000000, loc_cmd_hdt[9:0]};

  
reg [63:0] cmd_header_r;
reg [31:0] cmd_dt_r [2];



sync_reg # (.DW ( 1 ) ) sync_net_cmd (
   .dt_i      ( { net_cmd_i} ) ,
   .clk_i     ( t_clk_i ) ,
   .rst_ni    ( t_rst_ni ) ,
   .dt_o      ( { net_cmd_req_set}    ) );

reg net_cmd_req_set_r;
reg loc_cmd_req, net_cmd_req;

always_ff @(posedge t_clk_i) 
   if (!t_rst_ni) begin
      loc_cmd_req   <= 1'b0;
      net_cmd_req   <= 1'b0;
      net_cmd_req_set_r <= 1'b0;
      cmd_header_r <= 63'd0;
      cmd_dt_r     <= '{default:'0};
   end else begin 
      net_cmd_req_set_r <= net_cmd_req_set;
      if (c_cmd_req | p_cmd_req) begin
         loc_cmd_req    <= 1'b1;
         cmd_header_r   <= loc_cmd_header;
         cmd_dt_r       <= loc_cmd_dt;
      end else if (net_cmd_req_set) begin
         net_cmd_req    <= 1'b1;
         cmd_header_r   <= net_cmd_h_i;
         cmd_dt_r[0]    <= net_cmd_dt_i[31: 0] ;
         cmd_dt_r[1]    <= net_cmd_dt_i[63:32] ;
      end

      if (loc_cmd_ack_i & ~(p_cmd_req | c_cmd_req))  loc_cmd_req <= 1'b0;
      if (net_cmd_ack_i & ~net_cmd_req_set)        net_cmd_req <= 1'b0;
   end

assign loc_cmd_req_o = loc_cmd_req ;
assign net_cmd_req_o = net_cmd_req ;
assign header_o = cmd_header_r;
assign data_o   = cmd_dt_r;
endmodule

      
/*

// TIMEOUT
reg m_axi_ready_TX_r, m_axi_ready_TX_r2, m_axi_ready_TX_r3, m_axi_ready_TX_r4;

always_ff @ (posedge user_clk_i, posedge user_rst_i) begin
   if (user_rst_i) begin
      m_axi_ready_TX_r    <= 0;
      m_axi_ready_TX_r2   <= 0;
      m_axi_ready_TX_r3   <= 0;
      m_axi_ready_TX_r4   <= 0;
      
   end else begin
      m_axi_ready_TX_r     <= m_axi_tx_tready_TX;
      m_axi_ready_TX_r2    <= m_axi_ready_TX_r;
      m_axi_ready_TX_r3    <= m_axi_ready_TX_r2;
   end
end
wire m_axi_ready_TX_cc;
assign m_axi_ready_TX_cc    = !m_axi_ready_TX_r3 & !m_axi_ready_TX_r2 & !m_axi_ready_TX_r & m_axi_tx_tready_TX ;

reg ready_TX_r, ready_TX_r2, cc_TX_r, cc_TX_2r;

wire t_ready_t01, t_ready_t3rd;
reg tready_cnt_rst;
reg [1:0] tready_cnt;
(* ASYNC_REG = "TRUE" *) reg ready_TX_cdc, cc_TX_cdc;

// Detect Rising Edge on TREADY
always_ff @(posedge t_clk)
   if (!t_aresetn) begin
      ready_TX_cdc  <= 0;
      ready_TX_r    <= 0;
      ready_TX_r2    <= 0;

      tready_cnt    <= 0;
   end else begin
      ready_TX_cdc   <= m_axi_tx_tready_TX;
      ready_TX_r     <= ready_TX_cdc;
      ready_TX_r2    <= ready_TX_r;
      cc_TX_cdc      <= m_axi_ready_TX_cc;
      cc_TX_r        <= cc_TX_cdc;
      cc_TX_2r       <= cc_TX_r;

      if (tready_cnt_rst) 
         tready_cnt    <= 0;
      else if (t_ready_t01)
         tready_cnt    <=    tready_cnt + 1'b1;
   end

assign t_ready_t01    = !ready_TX_r2 & ready_TX_r ;
assign cc_TX_t01      = !cc_TX_2r & cc_TX_r;
assign t_ready_t3rd  = (tready_cnt == 2'b11);
*/
