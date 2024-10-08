module qick_net (
// Core, Time and AXI CLK & RST.
   input  wire             t_clk                ,
   input  wire             t_aresetn            ,
   input  wire             c_clk                ,
   input  wire             c_aresetn            ,
   input  wire             ps_clk               ,
   input  wire             ps_aresetn           ,
   input  wire             init_clk             ,
   input  wire             init_aresetn         ,
   input  wire             user_clk_i           ,
   input  wire             user_rst_i           ,
   input  wire  [47:0]     t_time_abs           ,
// TPROC CONTROL
   input  wire             c_cmd_i              ,
   input  wire  [4:0]      c_op_i               ,
   input  wire  [31:0]     c_dt1_i              ,
   input  wire  [31:0]     c_dt2_i              ,
   input  wire  [31:0]     c_dt3_i              ,
   output reg              c_ready_o            ,
   output reg              core_start_o         ,
   output reg              core_stop_o          ,
   output reg              time_rst_o           ,
   output reg              time_init_o          ,
   output reg              time_updt_o          ,
   output wire  [31:0]     time_off_dt_o        ,
   output reg  [31:0]      tnet_dt1_o           ,
   output reg  [31:0]      tnet_dt2_o           ,
// AURORA   ////   LINK COMMON
   input  wire  [4:0]      aurora_dbg           ,
   output reg              reset_pb             ,
   output reg              pma_init             ,
////////////////   LINK CHANNEL A
   input  wire             channel_up_RX        ,
   input  wire             s_axi_rx_tvalid_RX   ,
   input  wire  [63:0]     s_axi_rx_tdata_RX    ,
   input  wire             s_axi_rx_tlast_RX    ,
////////////////   LINK CHANNEL B
   input  wire             channel_up_TX        ,
   output reg  [63:0]      m_axi_tx_tdata_TX    ,
   output reg              m_axi_tx_tvalid_TX   ,
   output reg              m_axi_tx_tlast_TX    ,
   input  wire             m_axi_tx_tready_TX   ,
// AXI-Lite DATA Slave I/F.   
   input  wire [5:0]       s_axi_awaddr         ,
   input  wire [2:0]       s_axi_awprot         ,
   input  wire             s_axi_awvalid        ,
   output wire             s_axi_awready        ,
   input  wire [31:0]      s_axi_wdata          ,
   input  wire [3:0]       s_axi_wstrb          ,
   input  wire             s_axi_wvalid         ,
   output wire             s_axi_wready         ,
   output wire  [1:0]      s_axi_bresp          ,
   output wire             s_axi_bvalid         ,
   input  wire             s_axi_bready         ,
   input  wire [5:0]       s_axi_araddr         ,
   input  wire [2:0]       s_axi_arprot         ,
   input  wire             s_axi_arvalid        ,
   output wire             s_axi_arready        ,
   output wire  [31:0]     s_axi_rdata          ,
   output wire  [1:0]      s_axi_rresp          ,
   output wire             s_axi_rvalid         ,
   input  wire             s_axi_rready         );


///// AXI LITE PORT /////
///////////////////////////////////////////////////////////////////////////////
TYPE_IF_AXI_REG        IF_s_axireg()   ;
assign IF_s_axireg.axi_awaddr  = s_axi_awaddr ;
assign IF_s_axireg.axi_awprot  = s_axi_awprot ;
assign IF_s_axireg.axi_awvalid = s_axi_awvalid;
assign IF_s_axireg.axi_wdata   = s_axi_wdata  ;
assign IF_s_axireg.axi_wstrb   = s_axi_wstrb  ;
assign IF_s_axireg.axi_wvalid  = s_axi_wvalid ;
assign IF_s_axireg.axi_bready  = s_axi_bready ;
assign IF_s_axireg.axi_araddr  = s_axi_araddr ;
assign IF_s_axireg.axi_arprot  = s_axi_arprot ;
assign IF_s_axireg.axi_arvalid = s_axi_arvalid;
assign IF_s_axireg.axi_rready  = s_axi_rready ;
assign s_axi_awready = IF_s_axireg.axi_awready;
assign s_axi_wready  = IF_s_axireg.axi_wready ;
assign s_axi_bresp   = IF_s_axireg.axi_bresp  ;
assign s_axi_bvalid  = IF_s_axireg.axi_bvalid ;
assign s_axi_arready = IF_s_axireg.axi_arready;
assign s_axi_rdata   = IF_s_axireg.axi_rdata  ;
assign s_axi_rresp   = IF_s_axireg.axi_rresp  ;
assign s_axi_rvalid  = IF_s_axireg.axi_rvalid ;


wire [31:0] TNET_CTRL, TNET_CFG, REG_AXI_DT1, REG_AXI_DT2, REG_AXI_DT3;
wire [15:0] TNET_ADDR,TNET_LEN; 
reg ready_TX_cdc, t_ready_TX, t_ready_TX_r;
reg              net_cmd_req;
reg  [63 :0]      net_cmd [2]   ;

reg cmd_end;
wire             net_tx_ack ;

reg wt_loc_inc;
reg [31:0] div_num; 
reg [9:0] div_den;
reg loc_cmd_req ;

reg tx_req;
reg [31:0 ] acc_wt ;
reg loc_cmd_ack_set, loc_cmd_ack_clr  ;
reg net_cmd_ack_set, net_cmd_ack_clr  ; 
reg net_cmd_ack;

reg [31:0] param_OFF, param_RTD, param_CD; 
reg [9 :0] param_NN, param_ID;
reg [31:0] param_DT [2];

reg [4:0] param_LT, param_LT_r, param_LT_2r, param_LT_3r, param_LT_4r, param_LT_5r ;
reg [4:0] param_LT_new; 
reg update_LT;

wire [31:0] div_result_r;
wire div_end;

wire tx_ack;
reg [63:0] cmd_header_r;
reg [31:0] cmd_dt_r [2];

reg  [4 :0] cmd_id;
wire [5 :0] cmd_flg;
reg loc_cmd_ack;



// LOCAL, or EXTERNAL commands and external command
// Core and Python COMMANDS are generated in this NET_NODE. 
// Net can be a external command or an answer of a current command of other NET_NODE.
// If local Command needs answer or ACK, it should wait for an external command for the answer or ACK

// Command Decoding
localparam _nop         = 5'b00000;
localparam _get_net     = 5'b00001;
localparam _set_net     = 5'b00010;
localparam _sync_net    = 5'b01000;
localparam _updt_off    = 5'b01001;
localparam _set_dt      = 5'b01010;
localparam _get_dt      = 5'b01011;
localparam _rst_tproc   = 5'b10000;
localparam _start_core  = 5'b10001;
localparam _stop_core   = 5'b10010;
localparam _set_cond    = 5'b10011;
localparam _clear_cond  = 5'b10100;
localparam _custom      = 5'b11111;

reg get_net, set_net, sync_net, updt_off;
reg set_dt, get_dt, rst_tproc, start_core, stop_core;
reg set_cond, clear_cond, custom;




///////////////////////////////////////////////////////////////////////////////
// C CLOCK DOMAIN
reg         c_cmd_r;
reg  [4:0]  c_cmd_op;
reg  [31:0] c_cmd_dt [2];
reg  [31:0] c_cmd_hdt ;

sync_reg # ( .DW ( 1 ) )      sync_tcmd_ack (
   .dt_i      ( loc_cmd_ack ) ,
   .clk_i     ( c_clk       ) ,
   .rst_ni    ( c_aresetn   ) ,
   .dt_o      ( c_cmd_ack   ) );

// Processor Command Register
always_ff @(posedge c_clk) 
   if (!c_aresetn) begin
      c_cmd_r     <= 1'b0;
      c_cmd_op    <= 5'd0;
      c_cmd_dt    <= '{default:'0};
      c_cmd_hdt   <= 32'd0;
   end else begin 
      if (c_cmd_i) begin
         c_cmd_r   <= 1'b1;
         c_cmd_op  <= c_op_i  ;
         c_cmd_dt  <= {c_dt1_i, c_dt2_i} ;
         c_cmd_hdt <= c_dt3_i  ;
      end
      if ( c_cmd_ack ) c_cmd_r  <= 1'b0;
   end
   
// T_CLK Domain
reg         c_cmd_req;
sync_reg # (.DW ( 1 ) )       sync_cmd_req (
   .dt_i      ( c_cmd_r ) ,
   .clk_i     ( t_clk      ) ,
   .rst_ni    ( t_aresetn  ) ,
   .dt_o      ( {c_cmd_req}  ) );


///////////////////////////////////////////////////////////////////////////////
// PS CLOCK DOMAIN
wire [4:0] control_op;   
wire p_cmd_in;

// AXI Command Register
sync_reg # (.DW ( 6 ) )       sync_pcmd_op (
   .dt_i      ( TNET_CTRL[5:0] ) ,
   .clk_i     ( t_clk      ) ,
   .rst_ni    ( t_aresetn  ) ,
   .dt_o      ( {ctrl_rst, control_op}  ) );

assign p_cmd_in =    |control_op ; 

 
// T_CLK Domain
reg        p_cmd_req;
reg [4:0]  p_cmd_op;
reg [31:0] p_cmd_dt[2];
reg [31:0] p_cmd_hdt;
reg p_cmd_in_r;

// AXI Command Register
always_ff @(posedge t_clk) 
   if (!t_aresetn) begin
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
         p_cmd_dt   <= {REG_AXI_DT1, REG_AXI_DT2 }  ;
         p_cmd_hdt  <= REG_AXI_DT3 ;
      end
      if ( loc_cmd_ack ) p_cmd_req  <= 1'b0;
   end

assign p_cmd_in_t01 = !p_cmd_in_r & p_cmd_in;



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
   else if (loc_cmd_op == _updt_off   ) loc_cmd_header = {14'b100_01001_100100, loc_cmd_hdt[9:0] , param_ID, 30'b0000000000_0000000000_0000000000};
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

sync_reg # (.DW ( 2 ) ) sync_tx_ack (
   .dt_i      ( { net_tx_ack, net_cmd_hit} ) ,
   .clk_i     ( t_clk ) ,
   .rst_ni    ( t_aresetn ) ,
   .dt_o      ( { tx_ack, t_net_cmd_req_set}    ) );
   
always_ff @(posedge t_clk) 
   if (!t_aresetn) begin
      loc_cmd_req   <= 1'b0;
      net_cmd_req   <= 1'b0;
      loc_cmd_ack   <= 1'b0;
      net_cmd_ack   <= 1'b0;
      cmd_header_r <= 63'd0;
      cmd_dt_r     <= '{default:'0};
   end else begin 
      if (c_cmd_req | p_cmd_req) begin
         loc_cmd_req    <= 1'b1;
         cmd_header_r   <= loc_cmd_header;
         cmd_dt_r       <= loc_cmd_dt;
      end else if (t_net_cmd_req_set) begin
         net_cmd_req    <= 1'b1;
         cmd_header_r   <= net_cmd[0];
         cmd_dt_r[0]       <= net_cmd[1][31: 0] ;
         cmd_dt_r[1]       <= net_cmd[1][63:32] ;
      end
      
      if (loc_cmd_ack & ~(p_cmd_req | c_cmd_req))  loc_cmd_req <= 1'b0;
      if (net_cmd_ack & ~t_net_cmd_req_set)        net_cmd_req <= 1'b0;
      if (loc_cmd_ack_set) loc_cmd_ack <= 1'b1;
      if (loc_cmd_ack_clr | time_out | ctrl_rst ) loc_cmd_ack <= 1'b0;
      if (net_cmd_ack_set) net_cmd_ack <= 1'b1;
      if (net_cmd_ack_clr | time_out | ctrl_rst ) net_cmd_ack <= 1'b0;
   end

wire [9:0]  cmd_dst, cmd_src;
assign cmd_id  = cmd_header_r[60:56];
assign cmd_flg = cmd_header_r[55:50];
assign cmd_dst = cmd_header_r[49:40];
assign cmd_src = cmd_header_r[39:30];

//wire [63:0] cmd_header_ans;
//assign cmd_header_ans = { cmd_header_r[63:51],1'b1,cmd_src, cmd_dst, cmd_header_r[29:0] };

assign net_src_hit = (cmd_src == param_ID) ;
assign net_dst_hit = (cmd_dst == param_ID) ;


// Is response if 
// 1) For dst = 111111111 > net_src_hit & cmd_flg[4-WAIT_REQ]
// 2) For dst = DST   > net_dst_hit & & cmd_flg[0-ANS]

assign is_resp  = net_src_hit & cmd_flg[2] ;
assign is_answ  = net_dst_hit & cmd_flg[0];

   
// NET COMMAND OPERATON DECODER
always_comb begin
   get_net       = 1'b0;
   set_net       = 1'b0;
   sync_net      = 1'b0;
   updt_off      = 1'b0;
   set_dt        = 1'b0;
   get_dt        = 1'b0;
   rst_tproc     = 1'b0;
   start_core   = 1'b0;
   stop_core   = 1'b0;
   set_cond      = 1'b0;
   clear_cond    = 1'b0;
   custom        = 1'b0;
   if      (cmd_id == _get_net     ) get_net       = 1'b1 ;
   else if (cmd_id == _set_net     ) set_net       = 1'b1 ;
   else if (cmd_id == _sync_net    ) sync_net      = 1'b1 ;
   else if (cmd_id == _updt_off    ) updt_off      = 1'b1 ;
   else if (cmd_id == _set_dt      ) set_dt        = 1'b1 ;
   else if (cmd_id == _get_dt      ) get_dt        = 1'b1 ;
   else if (cmd_id == _rst_tproc   ) rst_tproc     = 1'b1 ;
   else if (cmd_id == _start_core  ) start_core    = 1'b1 ;
   else if (cmd_id == _stop_core   ) stop_core     = 1'b1 ;
   else if (cmd_id == _set_cond    ) set_cond      = 1'b1 ;
   else if (cmd_id == _clear_cond  ) clear_cond    = 1'b1 ;
end   
   

// Processing-Wait Time
///////////////////////////////////////////////////////////////////////////////
reg wt_rst, wt_init;
reg [31:0] wt_init_dt;
wire wt_inc;
reg wt_ext_inc;
assign wt_inc = wt_loc_inc | wt_ext_inc;
always_ff @(posedge t_clk)
   if (!t_aresetn)    
      acc_wt  <= 31'd0;
   else               
      if      ( wt_rst  ) acc_wt  <= 31'd0;
      else if ( wt_init ) acc_wt  <= wt_init_dt;
      else if ( wt_inc  ) acc_wt  <= acc_wt + 1'b1;

wire t_ready_t01;


// Detect Rising Edge on TREADY
always_ff @(posedge t_clk)
   if (!t_aresetn) begin
      ready_TX_cdc  <= 0;
      t_ready_TX    <= 0;
      t_ready_TX_r  <= 0;
   end else begin
      ready_TX_cdc   <= m_axi_tx_tready_TX;
      t_ready_TX     <= ready_TX_cdc;
      t_ready_TX_r   <= t_ready_TX;
   end

assign t_ready_t01 = !t_ready_TX_r & t_ready_TX ;

// Network Information
///////////////////////////////////////////////////////////////////////////////
reg [31:0] param_OFF_new, param_RTD_new, param_CD_new; 
reg [9:0] param_NN_new, param_ID_new;
reg [31:0] param_DT_new [2];
reg update_OFF, update_RTD, update_CD, update_NN, update_ID, update_DT;
reg time_reset, time_init, time_updt;

// Command Control 
///////////////////////////////////////////////////////////////////////////////
enum {M_NOT_READY=0, M_IDLE=1, LOC_CMD=2, NET_CMD=3, M_WRESP=4, M_WACK=5 , NET_RESP=6, NET_ACK=7, M_ERROR=8 } main_st_nxt, main_st;
always_ff @(posedge t_clk)
   if (!t_aresetn)      main_st  <= M_NOT_READY;
   else if ( ctrl_rst)  main_st  <= M_NOT_READY;
   else                 main_st  <= main_st_nxt;

assign c_ready_o = (main_st == M_IDLE);

///// MAIN STATE
always_comb begin
   main_st_nxt  = main_st; // Default Current
   if (!aurora_ready | time_out)  
      main_st_nxt = M_NOT_READY;
   else
      case (main_st)
         M_NOT_READY :  if (aurora_ready)          main_st_nxt  = M_IDLE;
         M_IDLE      :  if (loc_cmd_req )          main_st_nxt  = LOC_CMD;
         LOC_CMD     :  if (cmd_end     ) begin
                           if      ( cmd_error  )  main_st_nxt = M_ERROR;
                           else if ( cmd_flg[2] )  main_st_nxt = M_WRESP;
                           else if ( cmd_flg[1] )  main_st_nxt = M_WACK ;
                           else                    main_st_nxt = M_IDLE ;
                        end
         NET_CMD     :  if ( cmd_end      )        main_st_nxt = M_IDLE ;
         M_WRESP     :  if ( net_cmd_req  )        main_st_nxt = NET_RESP;
         M_WACK      :  if ( net_cmd_req  )        main_st_nxt = M_IDLE ;
         NET_RESP    :  if ( cmd_end      )        main_st_nxt = M_IDLE ;
         NET_ACK     :  if ( cmd_end      )        main_st_nxt = M_IDLE ;
         M_ERROR     :           main_st_nxt = M_IDLE ;
      endcase
end

enum {T_NOT_READY=0, T_IDLE=1, T_LOC_CMD=2,T_LOC_WSYNC=3,T_LOC_SEND=4,T_LOC_WnREQ=5, T_NET_CMD=6, T_NET_WSYNC=7, T_NET_SEND=8, T_NET_WnREQ=9 } task_st_nxt, task_st;
always_ff @(posedge t_clk)
   if      (!t_aresetn) task_st  <= T_NOT_READY;
   else if ( ctrl_rst)  task_st  <= T_NOT_READY;
   else                 task_st  <= task_st_nxt;

///// TASK STATE
always_comb begin
   task_st_nxt  = task_st; // Default Current
   loc_cmd_ack_set  = 1'b0 ;
   loc_cmd_ack_clr  = 1'b0 ;
   net_cmd_ack_set  = 1'b0 ; 
   net_cmd_ack_clr  = 1'b0 ;
   task_time_ok     = 1'b0 ;
   if (!aurora_ready | time_out)  
      task_st_nxt = T_NOT_READY;

      case (task_st)
         T_NOT_READY : begin
            task_time_ok     = 1'b1 ;
            if (aurora_ready)  task_st_nxt = T_IDLE;
         end
         T_IDLE      : begin
            task_time_ok     = 1'b1 ;
         // LOCAL COMMANDS
            if (loc_cmd_req) begin
               loc_cmd_ack_set = 1'b1 ;
               task_st_nxt      = T_LOC_CMD;
            end
         // NEWORK COMMANDS
            else if (net_cmd_req) begin
               net_cmd_ack_set  = 1'b1 ; 
               task_st_nxt      = T_NET_CMD;
            end
         end
   // LOCAL COMMAND
         T_LOC_CMD : begin
            if (tx_req)             task_st_nxt     = T_LOC_SEND;
            else if (cmd_end)       task_st_nxt     = T_LOC_WnREQ;
         end
         T_LOC_SEND : begin
            if (cmd_end)            task_st_nxt = T_LOC_WnREQ;
         end
         T_LOC_WnREQ : begin
            if (!loc_cmd_req)  begin
               loc_cmd_ack_clr = 1'b1;
               task_st_nxt = T_IDLE;
            end
         end
   // NETWORK COMMAND
         T_NET_CMD : begin
            if (tx_req)             task_st_nxt     = T_NET_SEND;
            else if (cmd_end)       task_st_nxt     = T_NET_WnREQ;
         end
         T_NET_WSYNC : begin
            if (t_ready_t01)        task_st_nxt     = T_NET_SEND;
         end
         T_NET_SEND : begin
            if (!tx_ack & !tx_req)  task_st_nxt = T_NET_WnREQ;
         end
         T_NET_WnREQ : begin
            if (!net_cmd_req) begin
               net_cmd_ack_clr = 1'b1;
               task_st_nxt = T_IDLE;
            end
         end
      endcase
end



// Waiting for Response
assign wfr = (main_st == NET_RESP);

// Command Execution
///////////////////////////////////////////////////////////////////////////////
enum {NOT_READY, IDLE, ST_ERROR, NET_CMD_RT, 
      LOC_GNET      , NET_GNET_P      , NET_GNET_R      ,
      LOC_SNET      , NET_SNET_P      , NET_SNET_R      ,
      LOC_SYNC      , NET_SYNC_P      , NET_SYNC_R      ,
      LOC_UPDT_OFF  , NET_UPDT_OFF_P  , NET_UPDT_OFF_R  ,
      LOC_SET_DT    , NET_SET_DT_P    , NET_SET_DT_R    ,
      LOC_GET_DT    , NET_GET_DT_P    , NET_GET_DT_R    , NET_GET_DT_A      ,
      LOC_RST_PROC  , NET_RST_PROC_P  , NET_RST_PROC_R  , 
      LOC_START_CORE, NET_START_CORE_P, NET_START_CORE_R,
      LOC_STOP_CORE , NET_STOP_CORE_P , NET_STOP_CORE_R ,
      WAIT_TX_ACK, WAIT_TX_nACK, WAIT_CMD_nACK
      } cmd_st_nxt, cmd_st;

always_ff @(posedge t_clk)
   if (!t_aresetn)      cmd_st  <= NOT_READY;
   else if ( ctrl_rst)  cmd_st  <= NOT_READY;
   else                 cmd_st  <= cmd_st_nxt;

reg [3:0] cmd_error_cnt;
reg [7:0] cmd_error_code;

always_ff @(posedge t_clk)
   if (!t_aresetn) begin   
      cmd_error_code  <= 0;
      cmd_error_cnt   <= 0;
   end else if (cmd_error) begin             
      cmd_error_code  <= {net_ctrl_id, cmd_id} ;
      cmd_error_cnt   <= cmd_error_cnt+1'b1;
   end else if (time_out) begin             
      cmd_error_code  <= 8'b11111111 ;
      cmd_error_cnt   <= cmd_error_cnt+1'b1;
   end


reg cmd_error;

reg tx_req_set ;

reg [63:0] tx_cmd_header;
reg[31:0] tx_cmd_dt [2];


// COMMAND STATE CHANGE
always_comb begin
   cmd_st_nxt      = cmd_st; // Default Current
   if (!aurora_ready )
      cmd_st_nxt    = NOT_READY;
   else if ( time_out)
      cmd_st_nxt    = ST_ERROR;
   else
      case (cmd_st)
         NOT_READY: if (aurora_ready)  cmd_st_nxt = IDLE;
         IDLE: begin
            // GET_NET
            if      (get_net & loc_cmd_ack                  )  cmd_st_nxt  = LOC_GNET;
            else if (get_net & net_cmd_ack & !wfr           )  cmd_st_nxt  = NET_GNET_P;
            else if (get_net & net_cmd_ack & wfr & is_resp  )  cmd_st_nxt  = NET_GNET_R;
            else if (get_net & net_cmd_ack & wfr & !is_resp )  cmd_st_nxt  = ST_ERROR;
            // SET_NET
            else if (set_net & loc_cmd_ack                  )  cmd_st_nxt = LOC_SNET;
            else if (set_net & net_cmd_ack & !wfr           )  cmd_st_nxt = NET_SNET_P;
            else if (set_net & net_cmd_ack & wfr & is_resp  )  cmd_st_nxt = NET_SNET_R;
            else if (set_net & net_cmd_ack & wfr & !is_resp )  cmd_st_nxt = ST_ERROR;
            // SYNC_NET 
            else if (sync_net & loc_cmd_ack                  ) cmd_st_nxt = LOC_SYNC;
            else if (sync_net & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_SYNC_P;
            else if (sync_net & net_cmd_ack & wfr & is_resp  ) cmd_st_nxt = NET_SYNC_R;
            else if (sync_net & net_cmd_ack & wfr & !is_resp ) cmd_st_nxt = ST_ERROR;
            // UPDATE_OFFSET  
            else if (updt_off & loc_cmd_ack                  ) cmd_st_nxt = LOC_UPDT_OFF;
            else if (updt_off & net_cmd_ack & !wfr           ) cmd_st_nxt = NET_UPDT_OFF_P;
            else if (updt_off & net_cmd_ack & wfr & is_resp  ) cmd_st_nxt = NET_UPDT_OFF_R;
            else if (updt_off & net_cmd_ack & wfr & !is_resp ) cmd_st_nxt = ST_ERROR;
            // SET_DT   
            else if (set_dt & loc_cmd_ack                    ) cmd_st_nxt = LOC_SET_DT;
            else if (set_dt & net_cmd_ack & !wfr             ) cmd_st_nxt = NET_SET_DT_P;
            else if (set_dt & net_cmd_ack & wfr & is_resp    ) cmd_st_nxt = NET_SET_DT_R;
            else if (set_dt & net_cmd_ack & wfr & !is_resp   ) cmd_st_nxt = ST_ERROR;
            // GET_DT   
            else if (get_dt & loc_cmd_ack                    )   cmd_st_nxt = LOC_GET_DT;
            else if (get_dt & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_GET_DT_P;
            else if (get_dt & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_GET_DT_R;
            else if (get_dt & net_cmd_ack & wfr & is_answ    )   cmd_st_nxt = NET_GET_DT_A;
            else if (get_dt & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // RESET_TPROC   
            else if (rst_tproc & loc_cmd_ack & ~net_ctrl_ack  )   cmd_st_nxt = LOC_RST_PROC;
            else if (rst_tproc & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_RST_PROC_P;
            else if (rst_tproc & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_RST_PROC_R;
            else if (rst_tproc & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // START_CORE
            else if (start_core & loc_cmd_ack & ~net_ctrl_ack  )   cmd_st_nxt = LOC_START_CORE;
            else if (start_core & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_START_CORE_P;
            else if (start_core & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_START_CORE_R;
            else if (start_core & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // STOP_CORE   
            else if (stop_core & loc_cmd_ack & ~net_ctrl_ack  )   cmd_st_nxt = LOC_STOP_CORE;
            else if (stop_core & net_cmd_ack & !wfr             )   cmd_st_nxt = NET_STOP_CORE_P;
            else if (stop_core & net_cmd_ack & wfr & is_resp    )   cmd_st_nxt = NET_STOP_CORE_R;
            else if (stop_core & net_cmd_ack & wfr & !is_resp   )   cmd_st_nxt = ST_ERROR;
            // OTHER 
            else if (loc_cmd_ack | net_cmd_ack) cmd_st_nxt  = ST_ERROR;
         end
         ST_ERROR: cmd_st_nxt    = WAIT_TX_nACK;

   // LOCAL COMMAND
         LOC_GNET       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SNET       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SYNC       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_UPDT_OFF   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_SET_DT     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_GET_DT     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         LOC_RST_PROC   : if (t_ready_t01) begin if (net_ctrl_ack) cmd_st_nxt = WAIT_TX_ACK; else cmd_st_nxt =  ST_ERROR; end
         LOC_START_CORE : if (t_ready_t01) begin if (net_ctrl_ack) cmd_st_nxt = WAIT_TX_ACK; else cmd_st_nxt =  ST_ERROR; end
         LOC_STOP_CORE  : if (t_ready_t01) begin if (net_ctrl_ack) cmd_st_nxt = WAIT_TX_ACK; else cmd_st_nxt =  ST_ERROR; end
   // NET Command Process
         NET_GNET_P       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SNET_P       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SYNC_P       : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_UPDT_OFF_P   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_SET_DT_P     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_GET_DT_P     : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_RST_PROC_P   : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_START_CORE_P : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
         NET_STOP_CORE_P  : if (t_ready_t01) cmd_st_nxt = WAIT_TX_ACK;
   // NET Response Process
         NET_GNET_R       : if (div_end)     cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SNET_R       :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SYNC_R       :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_UPDT_OFF_R   :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_SET_DT_R     :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_GET_DT_R     :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_RST_PROC_R   :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_START_CORE_R :                  cmd_st_nxt =  WAIT_CMD_nACK;
         NET_STOP_CORE_R  :                  cmd_st_nxt =  WAIT_CMD_nACK;
   // NET ANSWER Process
         NET_GET_DT_A   :                  cmd_st_nxt =  WAIT_CMD_nACK;
   // WAIT FOR SYNC
         WAIT_TX_ACK   : if ( tx_ack )    cmd_st_nxt = WAIT_TX_nACK;
         WAIT_TX_nACK  : if (!tx_ack )    cmd_st_nxt = WAIT_CMD_nACK;
         WAIT_CMD_nACK : if (!cmd_ack)    cmd_st_nxt = IDLE;
      endcase
end

reg [47:0] net_ctrl_time;
reg [2:0]  net_ctrl_id;

reg div_start;
//OUTPUTS
always_comb begin
   tx_req_set = 1'b0 ;
   net_ctrl_id = 0 ;
   net_ctrl_time = 0;
   cmd_end        = 1'b0;
   wt_rst         = 1'b0;
   wt_loc_inc     = 1'b0;
   wt_ext_inc     = 1'b0;
   wt_init        = 1'b0 ;
   wt_init_dt     = 0;
   cmd_error      = 1'b0;
   time_reset     = 1'b0;
   time_init      = 1'b0;
   time_updt       = 1'b0;
   tx_cmd_dt      = '{default:'0};
   tx_cmd_header  = 0;
   div_start      = 1'b0;
   update_NN  = 1'b0 ;
   update_ID  = 1'b0 ;
   update_CD  = 1'b0 ;
   update_RTD = 1'b0 ;
   update_OFF = 1'b0 ;    
   update_DT  = 1'b0 ;
   update_LT  = 1'b0 ;
   param_NN_new   = 0;
   param_ID_new   = 0;
   param_CD_new   = 0;
   param_RTD_new  = 0;
   param_OFF_new  = 0;
   param_LT_new   = 0;
   param_DT_new   = '{default:'0};;
   div_num        = t_time_abs[31:0] - cmd_dt_r[1]; // Total Round Time minus Accumulated Wait Time
   div_den        = cmd_header_r[9:0];
   cmd_time_ok = 1'b0;
   if (!aurora_ready | time_out) begin
      update_LT     = 1'b1;
      param_LT_new  = 5'd25;
      if (time_out) param_LT_new  = 5'd26;
   end
   case (cmd_st)
      NOT_READY:    cmd_time_ok = 1'b1;
      IDLE: begin
         cmd_time_ok = 1'b1;
         if (get_net & net_cmd_ack & !wfr )  begin
         //if (cmd_st_nxt  == NET_GNET_P) begin 
            update_LT     = 1'b1; 
            param_LT_new  = 5'd1;
            wt_init     = 1'b1 ;
            time_reset  = 1'b1 ;
            wt_init_dt  = cmd_dt_r[1] ;
         end
         if (sync_net & net_cmd_ack & !wfr ) begin
         // if (cmd_st_nxt  == NET_SYNC_P) begin 
            update_LT     = 1'b1;
            param_LT_new  = 5'd2;
            wt_init        = 1'b1 ;
            wt_init_dt     = cmd_dt_r[1] ;
            time_init      = 1'b1 ;
            update_OFF     = 1'b1 ;
            param_OFF_new  = cmd_dt_r[0] + cmd_dt_r[1] ;
         end
         if (get_net & net_cmd_ack & wfr & is_resp  ) begin
         //if (cmd_st_nxt  == NET_GNET_R) begin 
            update_LT      = 1'b1;
            param_LT_new   = 5'd3;
            update_RTD     = 1'b1  ;
            param_RTD_new  = t_time_abs[31:0];
            update_NN      = 1'b1  ;
            param_NN_new   = cmd_header_r[9:0];
            div_start      = 1'b1;
         end
      end
      ST_ERROR: begin
         cmd_error     = 1'b1;
         cmd_end = 1'b1 ;
         update_LT     = 1'b1;
         param_LT_new  = 5'd29;
      end
// COMMAND DATA      
      LOC_GNET: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = 0;
         update_ID     = 1'b1;
         param_ID_new  = 9'd1;
         // Command wait for answer
         if (t_ready_t01) begin
            update_LT     = 1'b1;
            param_LT_new  = 5'd4;
            time_reset  = 1'b1 ;
            tx_req_set  = 1'b1 ;
         end
      end
      LOC_SNET: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = param_RTD;
         tx_cmd_dt[0]  = param_CD;
         // Command wait for answer
         if (t_ready_t01) begin
            update_LT     = 1'b1;
            param_LT_new  = 5'd5;
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_SYNC: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = 0;
         tx_cmd_dt[0]  = param_CD;
         // Command wait for answer
         if (t_ready_t01) begin
            update_LT     = 1'b1;
            param_LT_new  = 5'd6;
            time_reset    = 1'b1 ;
            tx_req_set    = 1'b1 ;
         end
      end
      LOC_UPDT_OFF: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) begin
            tx_req_set    = 1'b1 ;
            update_LT     = 1'b1;
            param_LT_new  = 5'd7;
         end
      end
      LOC_SET_DT: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
            update_LT     = 1'b1;
            param_LT_new  = 5'd8;
         end
      end
      LOC_GET_DT: begin
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = '{default:'0};
         if (t_ready_t01) begin
            tx_req_set    = 1'b1 ;
            update_LT     = 1'b1;
            param_LT_new  = 5'd9;
         end
      end
      LOC_RST_PROC: begin
         net_ctrl_id    = 3'b001;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end
      LOC_START_CORE: begin
         net_ctrl_id    = 3'b010;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end
      LOC_STOP_CORE: begin
         net_ctrl_id    = 3'b100;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r; //Time
         if (t_ready_t01 & net_ctrl_ack) tx_req_set    = 1'b1 ;
      end

// PROCESS AND PROPAGATE COMMAND
      NET_GNET_P: begin 
         wt_loc_inc    = 1'b1;
         tx_cmd_header = cmd_header_r + 1'b1;
         tx_cmd_dt[1]  = acc_wt;
         tx_cmd_dt[0]  = 0;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
            update_LT     = 1'b1;
            param_LT_new  = 5'd10;
         end
      end
      NET_SNET_P: begin   // PROCESS AND PROPAGATE COMMAND
         update_RTD     = 1'b1  ;
         param_RTD_new  = cmd_dt_r[1];
         update_CD      = 1'b1  ;
         param_CD_new   = cmd_dt_r[0];
         update_NN      = 1'b1  ;
         param_NN_new   = cmd_header_r[19:10];
         update_ID      = 1'b1  ;
         param_ID_new   = cmd_header_r[9:0];
         tx_cmd_header  = cmd_header_r + 1'b1;
         tx_cmd_dt      =  cmd_dt_r ;
         if (t_ready_t01) begin 
            tx_req_set = 1'b1 ;
            update_LT     = 1'b1;
            param_LT_new  = 5'd11;
         end
      end
      NET_SYNC_P: begin
         wt_loc_inc    = 1'b1;
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt[1]  = acc_wt ;
         tx_cmd_dt[0]  = cmd_dt_r[0]+param_CD;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
            update_LT      = 1'b1;
            param_LT_new   = 5'd12;
         end            
      end
      NET_UPDT_OFF_P: begin
         update_OFF     = 1'b1 ;
         param_OFF_new  = cmd_dt_r[0];
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r ;
         if (t_ready_t01) begin
            time_updt      = 1'b1 ;         
            tx_req_set     = 1'b1 ;
            update_LT      = 1'b1;
            param_LT_new   = 5'd13;
         end
      end
      NET_SET_DT_P: begin
         cmd_end        = 1'b1;
         update_DT      = 1'b1 ;
         param_DT_new   = cmd_dt_r;
         tx_cmd_header  = cmd_header_r;
         tx_cmd_dt      = cmd_dt_r ;
         if (t_ready_t01) begin
            tx_req_set = 1'b1 ;
            update_LT      = 1'b1;
            param_LT_new   = 5'd14;
         end
      end
      NET_GET_DT_P: begin
         tx_cmd_header   = {14'b100_01011_000001, cmd_header_r[39:30] , param_ID, 20'b0000000000_0000000000, loc_cmd_hdt[9:0]};
         tx_cmd_dt        = param_DT;
         if (t_ready_t01) begin
            tx_req_set     = 1'b1 ;
            update_LT      = 1'b1;
            param_LT_new   = 5'd15;
         end
      end

      NET_RST_PROC_P: begin
         net_ctrl_id    = 3'b001;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end
      NET_START_CORE_P: begin
         net_ctrl_id    = 3'b010;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end
      NET_STOP_CORE_P: begin
         net_ctrl_id    = 3'b100;
         net_ctrl_time  = {cmd_dt_r[0][15:0] , cmd_dt_r[1] };
         tx_cmd_header = cmd_header_r;
         tx_cmd_dt     = cmd_dt_r;
         if (t_ready_t01) tx_req_set     = 1'b1 ;
      end


// RESPONSES 
      NET_GNET_R: begin
         if (div_end) begin
            cmd_end        = 1'b1;
            update_CD      = 1'b1  ;
            param_CD_new   = div_result_r;
            update_LT      = 1'b1;
            param_LT_new   = 5'd16;
         end
      end
      NET_SNET_R     : begin
         cmd_end        = 1'b1;
         update_LT      = 1'b1;
         param_LT_new   = 5'd17;
         end
      NET_SYNC_R     : begin 
         cmd_end        = 1'b1;
         update_LT      = 1'b1;
         param_LT_new   = 5'd18;
         end
      NET_UPDT_OFF_R : begin 
         cmd_end        = 1'b1;
         update_LT      = 1'b1;
         param_LT_new   = 5'd19;
         end
      NET_SET_DT_R   : begin 
         cmd_end        = 1'b1;
         update_LT      = 1'b1;
         param_LT_new   = 5'd20;
         end
      NET_GET_DT_R: begin
         cmd_end        = 1'b1;
         update_LT      = 1'b1;
         param_LT_new   = 5'd21;
      end
      NET_RST_PROC_R: begin
         cmd_end        = 1'b1;
      end
      NET_START_CORE_R: begin
         cmd_end        = 1'b1;
      end
      NET_STOP_CORE_R: begin
         cmd_end        = 1'b1;
      end

// ANSWERS
      NET_GET_DT_A: begin
         cmd_end        = 1'b1;
         update_LT      = 1'b1;
         param_LT_new   = 5'd24;
         update_DT      = 1'b1 ;
         param_DT_new   = cmd_dt_r;
         
      end
      WAIT_TX_nACK: begin
         if (!tx_ack) begin 
            cmd_end = 1'b1;
            update_LT      = 1'b1;
            param_LT_new   = 5'd30;
         end
      end
//DEBUG      
      WAIT_TX_ACK   : begin
         if ( tx_ack ) begin
            update_LT      = 1'b1;
            param_LT_new   = 5'd27;
         end
      end
      WAIT_CMD_nACK : begin
         if (!cmd_ack) begin
            update_LT      = 1'b1;
            param_LT_new   = 5'd28;
         end
      end
      
   endcase
end
assign cmd_ack = loc_cmd_ack | net_cmd_ack;


reg [63:0] tx_cmd_header_r, tx_cmd_dt_r;
//TX Communication Control

always_ff @(posedge t_clk)
   if (!t_aresetn) begin
      tx_req            <= 1'b0;
      tx_cmd_header_r   <= 64'd0;
      tx_cmd_dt_r       <= 64'd0;
   end else begin 
      if (tx_req_set ) begin   
         tx_req          <= 1'b1;
         tx_cmd_header_r <= tx_cmd_header;
         tx_cmd_dt_r     <= { tx_cmd_dt[1], tx_cmd_dt[0] };
      end 
      if (tx_req & tx_ack) begin
         tx_req <= 1'b0;
      end
   end

// Parameters
always_ff @(posedge t_clk)
   if (!t_aresetn) begin
      param_OFF   <= 32'd0;
      param_RTD   <= 32'd0;
      param_CD    <= 32'd0;
      param_NN    <=  9'd0;
      param_ID    <=  9'd0;
      param_LT     <=  5'd0;
      param_LT_r   <=  5'd22;
      param_LT_2r  <=  5'd23;
      param_LT_3r  <=  5'd23;
      param_LT_4r  <=  5'd23;
      param_LT_5r  <=  5'd23;
      
      param_DT    <=  '{default:'0};
   end else begin
      if (update_OFF)  param_OFF  <= param_OFF_new;
      if (update_RTD)  param_RTD  <= param_RTD_new; 
      if (update_CD )  param_CD   <= param_CD_new; 
      if (update_NN )  param_NN   <= param_NN_new; 
      if (update_ID )  param_ID   <= param_ID_new; 
      if (update_DT )  param_DT   <= param_DT_new; 
      if (update_LT ) begin
         param_LT     <= param_LT_new; 
         param_LT_r   <= param_LT; 
         param_LT_2r  <= param_LT_r; 
         param_LT_3r  <= param_LT_2r ;
         param_LT_4r  <= param_LT_3r;
         param_LT_5r  <= param_LT_4r;
      end
   end
   
// Pipelined Divider
net_div_r #(
   .DW (32) ,
   .N_PIPE (32)
) div_r_inst (
   .clk_i           ( t_clk      ) ,
   .rst_ni          ( t_aresetn  ) ,
   .start_i         ( div_start  ) ,
   .end_o           ( div_end  ) ,
   .A_i             ( div_num    ) ,
   .B_i             ( {22'd0, div_den } ) ,
   .ready_o         ( div_rdy    ) ,
   .div_remainder_o (  ) ,
   .div_quotient_o  ( div_result_r ) );

wire channel_ok ;
assign channel_ok = channel_up_RX & channel_up_TX;

wire [6:0]  aurora_cnt;
wire [3:0]  aurora_op;
wire [2:0]  aurora_st;

// TIMEOUT
reg cmd_time_ok, task_time_ok;
reg [9:0] time_cnt;
always_ff @ (posedge user_clk_i, posedge user_rst_i) begin
   if (user_rst_i) begin
      time_cnt      <= 1 ;
   end else begin
      if (cmd_time_ok & task_time_ok)   time_cnt  <= 0;
      else           time_cnt      <= time_cnt + 1'b1;
   end
end
assign time_out = time_cnt[9];

aurora_ctrl QNET_LINK (
   .user_clk_i          ( user_clk_i            ) , 
   .user_rst_i          ( user_rst_i            ) , 
   .tx_req_ti           ( tx_req                ) , 
   .tx_header_ti        ( tx_cmd_header_r       ) , 
   .tx_data_ti          ( tx_cmd_dt_r           ) , 
   .tx_ack_o            ( net_tx_ack            ) , 
   .cmd_req_o           ( net_cmd_hit           ) , 
   .cmd_o               ( net_cmd               ) , 
   .ID                  ( param_ID              ) , 
   .channel_ok_i        ( channel_ok            ) , 
   .ready_o             ( aurora_ready          ) , 
   .s_axi_rx_tdata_RX   ( s_axi_rx_tdata_RX     ) , 
   .s_axi_rx_tvalid_RX  ( s_axi_rx_tvalid_RX    ) , 
   .s_axi_rx_tlast_RX   ( s_axi_rx_tlast_RX     ) , 
   .m_axi_tx_tdata_TX   ( m_axi_tx_tdata_TX   ) , 
   .m_axi_tx_tvalid_TX  ( m_axi_tx_tvalid_TX  ) , 
   .m_axi_tx_tlast_TX   ( m_axi_tx_tlast_TX   ) , 
   .m_axi_tx_tready_TX  ( m_axi_tx_tready_TX    ) ,
   .pack_cnt_do         ( aurora_cnt ) ,
   .last_op_do          ( aurora_op    ) ,
   .state_do            ( aurora_st) );


wire [31:0] TNET_DEBUG, TNET_STATUS;  
// AXI Slave.
tnet_axi_reg TNET_xREG (
   .ps_aclk        ( ps_clk         ) , 
   .ps_aresetn     ( ps_aresetn     ) , 
   .IF_s_axireg    ( IF_s_axireg    ) ,
   .TNET_CTRL      ( TNET_CTRL      ) ,
   .TNET_CFG       ( TNET_CFG       ) ,
   .TNET_ADDR      ( TNET_ADDR      ) ,
   .TNET_LEN       ( TNET_LEN       ) ,
   .REG_AXI_DT1    ( REG_AXI_DT1    ) ,
   .REG_AXI_DT2    ( REG_AXI_DT2    ) ,
   .REG_AXI_DT3    ( REG_AXI_DT3    ) ,
   .NN             ( param_NN       ) ,
   .ID             ( param_ID       ) ,
   .CDELAY         ( param_CD       ) ,
   .RTD            ( param_RTD      ) ,
   .VERSION        ( TNET_VER  ) ,
   .TNET_W_DT1     ( param_DT[0]    ) ,
   .TNET_W_DT2     ( param_DT[1]    ) ,
   .TNET_STATUS    ( TNET_STATUS    ) ,
   .TNET_DEBUG     ( TNET_DEBUG     ) );
   
wire  [31:0]TNET_VER;
assign TNET_VER[31 : 27]    = param_LT_5r ;
assign TNET_VER[26 : 22]    = param_LT_4r ;
assign TNET_VER[21 : 17]    = param_LT_3r ;
assign TNET_VER[16 : 12]    = param_LT_2r ;
assign TNET_VER[11 :  7]    = param_LT_r ;
assign TNET_VER[6  :  2]    = param_LT ;


assign TNET_DEBUG[31 : 29]    = cmd_src[2:0] ;
assign TNET_DEBUG[28 : 26]    = cmd_dst[2:0] ;
assign TNET_DEBUG[25 : 18]    = main_st[7:0] ;
assign TNET_DEBUG[17 : 14]    = task_st[3:0] ;
assign TNET_DEBUG[13 : 11]    = aurora_st[2:0] ;
assign TNET_DEBUG[10 :  7]    = aurora_op[3:0] ;
assign TNET_DEBUG[ 6 :  0]    = aurora_cnt[6:0] ;


assign TNET_STATUS[31 : 30]    = { clear_cond, set_cond } ;
assign TNET_STATUS[29 : 27]    = { stop_core, start_core, rst_tproc} ;
assign TNET_STATUS[26 : 25]    = { get_dt, set_dt } ;
assign TNET_STATUS[24 : 21]    = { updt_off, sync_net, set_net, get_net } ;
assign TNET_STATUS[20 : 18]    = net_ctrl_id_r;
assign TNET_STATUS[17 : 10]    = cmd_error_code ;
assign TNET_STATUS[ 9 :  6]    = cmd_error_cnt;
assign TNET_STATUS[ 5 ]        = aurora_ready;
assign TNET_STATUS[ 4 :  0]    = aurora_dbg ;

//OUTPUTS
always_ff @ (posedge init_clk, negedge init_aresetn) begin
   if (!init_aresetn) begin         
      pma_init   <= 1'b1 ;
      reset_pb   <= 1'b1 ;
   end else begin
      pma_init   <= 1'b0 ;
      reset_pb   <= pma_init;
   end
end
   
  
always_ff @(posedge t_clk)
   if (!t_aresetn) begin
      time_rst_o  <= 1'b0;
      time_init_o <= 1'b0;
      time_updt_o <= 1'b0;
   end else begin
      time_rst_o  <=  time_reset | net_ctrl_exec_r[0];
      time_init_o <=  time_init;
      time_updt_o <=  time_updt;
   end

reg [47:0] net_ctrl_time_r;
reg [2:0]  net_ctrl_id_r, net_ctrl_exec_r;
reg net_ctrl_ack;
reg ctrl_sign_time_r;

assign net_ctrl_req_set = |net_ctrl_id  ;
assign net_ctrl_ack_set = |net_ctrl_id_r & ctrl_time[47]; // ACK if RTD time is OK 

reg net_ctrl_req;

always_ff @(posedge t_clk)
   if (!t_aresetn) begin
      net_ctrl_req   <= 1'b0;
      net_ctrl_ack   <= 1'b0;
      net_ctrl_exec_r  <= 1'b0;
      net_ctrl_id_r    <= 3'b0;
      net_ctrl_time_r  <= 47'b0111111111111111_1111111111111111_111111111111111;
   end else begin
      ctrl_sign_time_r <= ctrl_left_time[47];

      if ( net_ctrl_req_set )   net_ctrl_req   <= 1'b1;
      else if (cmd_error)       net_ctrl_req   <= 1'b0;

      if ( net_ctrl_ack_set ) begin
         net_ctrl_ack   <= 1'b1;
      end

      if ( net_ctrl_req & ~net_ctrl_ack) begin
         net_ctrl_id_r    <= net_ctrl_id ;
         net_ctrl_time_r  <= net_ctrl_time;
      end

      if (ctrl_execute )   net_ctrl_exec_r  <= net_ctrl_id_r;
      else                 net_ctrl_exec_r  <= 1'b0;

      if (net_ctrl_ack & ctrl_left_time[47] ) begin
         net_ctrl_id_r    <= 0;
         if ( !net_ctrl_ack_set ) begin
            net_ctrl_ack   <= 1'b0;
            net_ctrl_req   <= 1'b0;
         end
      end
      
   end

wire ctrl_execute;
assign ctrl_execute = ~ctrl_sign_time_r & ctrl_left_time[47] ;

wire [47:0] ctrl_left_time;

ADDSUB_MACRO #(
      .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
      .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
      .WIDTH      ( 48  )             // Input / output bus width, 1-48
   ) TIME_CMP_inst (
      .CARRYOUT   (                     ),  // 1-bit carry-out output signal
      .RESULT     ( ctrl_left_time    ),  // A-B
      .B          ( t_time_abs   ),  // Input A bus, width defined by WIDTH parameter
      .ADD_SUB    ( 1'b0              ),  // 1-bit add/sub input, high selects add, low selects subtract
      .A          ( net_ctrl_time_r   ),  // Input B bus, width defined by WIDTH parameter
      .CARRYIN    ( 1'b0              ),    // 1-bit carry-in input
      .CE         ( 1'b1              ),    // 1-bit clock enable input
      .CLK        ( t_clk             ),    // 1-bit clock input
      .RST        ( ~t_aresetn        )     // 1-bit active high synchronous reset
   );

reg [47:0] ctrl_time;

ADDSUB_MACRO #(
      .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
      .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
      .WIDTH      ( 48  )             // Input / output bus width, 1-48
   ) CMD_TIME (
      .CARRYOUT   (               ),    // 1-bit carry-out output signal
      .RESULT     ( ctrl_time     ), // Add/sub result output, width defined by WIDTH parameter
      .B          ( ctrl_left_time     ),    // Input A bus, width defined by WIDTH parameter
      .ADD_SUB    ( 1'b0          ),    // 1-bit add/sub input, high selects add, low selects subtract
      .A          ( {16'd0, param_RTD}     ),   // Input B bus, width defined by WIDTH parameter
      .CARRYIN    ( 1'b0          ),    // 1-bit carry-in input
      .CE         ( net_ctrl_req  ),    // 1-bit clock enable input
      .CLK        ( t_clk         ),    // 1-bit clock input
      .RST        ( ~t_aresetn    )     // 1-bit active high synchronous reset
   );



assign core_start_o  = net_ctrl_exec_r[1];
assign core_stop_o   = net_ctrl_exec_r[2];
      
assign time_off_dt_o = param_OFF;

assign tnet_dt1_o = param_DT[0];
assign tnet_dt2_o = param_DT[1];

endmodule

