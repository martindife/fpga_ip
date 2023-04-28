///////////////////////////////////////////////////////////////////////////////
//  FERMI RESEARCH LAB
///////////////////////////////////////////////////////////////////////////////
//  Author         : Martin Di Federico
//  Date           : 4-2023
//  Version        : 2
///////////////////////////////////////////////////////////////////////////////
//Description:  QICK PROCESSOR
//////////////////////////////////////////////////////////////////////////////

`include "_qick_defines.svh"

module qick_processor # (
   parameter DEBUG          =  0 ,
   parameter DUAL_CORE      =  0 ,
   parameter LFSR           =  1 ,
   parameter DIVIDER        =  1 ,
   parameter ARITH          =  1 ,
   parameter TIME_READ      =  1 ,
   parameter PMEM_AW        =  8 ,
   parameter DMEM_AW        =  8 ,
   parameter WMEM_AW        =  8 ,
   parameter REG_AW         =  4 ,
   parameter IN_PORT_QTY    =  2 ,
   parameter OUT_DPORT_QTY  =  4 ,
   parameter OUT_WPORT_QTY  =  4 
)(
// Time, Core and AXI CLK & RST.
   input   wire               t_clk_i        ,
   input   wire               t_rst_ni       ,
   input   wire               c_clk_i        ,
   input   wire               c_rst_ni       ,
   input   wire               ps_clk_i       ,
   input   wire               ps_rst_ni      ,
// External Control
   input  wire                proc_rst_i            ,
   input  wire                proc_run_i            ,
   input  wire                proc_stop_i              ,
   input  wire                proc_pause_i             ,
   input  wire                time_rst_i              ,
   input  wire                time_init_i             ,
   input  wire                time_updt_i       ,
   input  wire  [31:0]        offset_dt_i       ,
   output wire  [47:0]        t_time_abs_o         ,
// External PERIPHERALS 
   output wire  [31:0]        periph_a_dt_o        ,
   output wire  [31:0]        periph_b_dt_o        ,
   output wire  [31:0]        periph_c_dt_o        ,
   output wire  [31:0]        periph_d_dt_o        ,
   output wire  [4 :0]        periph_addr_o        ,
   output wire  [4 :0]        periph_op_o          ,
// CUSTOM  
   input  wire                periph_rdy_i         , 
   input  wire  [31 :0]       periph_dt_i [2]      , 
//TNET_DT
   input wire                 tnet_rdy_i           , 
   input wire  [31 :0]        tnet_dt_i [2]        , 
// DMA AXIS FOR READ AND WRITE MEMORY             
   input  wire  [255 :0]      s_dma_axis_tdata_i   ,
   input  wire                s_dma_axis_tlast_i   ,
   input  wire                s_dma_axis_tvalid_i  ,
   output wire                s_dma_axis_tready_o  ,
   output wire  [255 :0]      m_dma_axis_tdata_o   ,
   output wire                m_dma_axis_tlast_o   ,
   output wire                m_dma_axis_tvalid_o  ,
   input  wire                m_dma_axis_tready_i  ,
// AXI-Lite DATA Slave I/F.
   TYPE_AXI_LITE_IF_IN        IF_s_axireg          ,
// DATA INPUT INTERFACE
   input   wire               port_tvalid_i  [ IN_PORT_QTY ]  ,
   input   wire [63:0]        port_tdata_i   [ IN_PORT_QTY ] ,
// DATA OUTPUT INTERFACE
   output  wire               port_tvalid_o  [ OUT_DPORT_QTY]  ,
   output  wire [31:0]        port_tdata_o   [ OUT_DPORT_QTY] ,
// AXI Stream Master I/F.
   output  wire [167:0]       m_axis_tdata   [OUT_WPORT_QTY] ,
   output  wire               m_axis_tvalid  [OUT_WPORT_QTY] ,
   input   wire               m_axis_tready  [OUT_WPORT_QTY] ,
// DEBUG INTERFACE
   output  wire [31:0]        t_time_usr_do        ,
   output  wire [31:0]        t_debug_do     ,
   output  wire [31:0]        t_fifo_do      ,
   output  wire [31:0]        c_debug_do      ,
   output  wire [31:0]        c_time_ref_do  ,
   output  wire [31:0]        c_port_do      ,
   output  wire [31:0]        c_core_do      );

localparam DECO_OUT = DUAL_CORE + 2;

// SIGNALS
///////////////////////////////////////////////////////////////////////////////

// PORTS
wire                 port_we        ;
PORT_DT              out_port_data  ;
wire [47:0] time_abs_r, init_off;
wire [31:0] core_usr_a_dt, core_usr_b_dt, core_usr_c_dt, core_usr_d_dt ;
// REGISTERS
wire [ 31 : 0 ] xreg_TPROC_CTRL  , xreg_TPROC_CFG       ;
wire [ 15 : 0 ] xreg_MEM_ADDR    , xreg_MEM_LEN         ;
wire [ 31 : 0 ] xreg_MEM_DT_I    , xreg_MEM_DT_O        ;
wire [ 31 : 0 ] xreg_TPROC_STATUS, xreg_TPROC_DEBUG     ;



reg [63:0]           in_port_dt_r [ IN_PORT_QTY ];


reg [31:0] tproc_w_dt [2];

wire [4:0] core_usr_addr, core_usr_operation;


wire        t_time_en ;
reg         c_time_en ;
wire [31:0] t_time_usr, c_time_usr ;

wire [31:0] reg_axi_dt [2] ;




// IN PORT DATA REGISTER
///////////////////////////////////////////////////////////////////////////////
localparam ZFP = 16 - IN_PORT_QTY; // Zero Fill for Input Port



qproc_inport_reg # (
   .PORT_QTY    (IN_PORT_QTY) 
) IN_PORT_REG (
   .c_clk_i       ( c_clk_i       ) ,
   .c_rst_ni      ( c_rst_ni      ) ,
   .c_clear       ( port_clr      ) ,
   .port_tvalid_i ( port_tvalid_i ) ,
   .port_tdata_i  ( port_tdata_i  ) ,
   .port_tnew_o   ( port_new      ) ,
   .port_tdata_o  ( in_port_dt_r  ) );

wire [15:0] port_dt_new ;
assign port_dt_new = { {ZFP{1'b0}} , port_new };

// T-PROCESSOR CONTROL
///////////////////////////////////////////////////////////////////////////////



wire ctrl_t_rst, ctrl_t_freeze, ctrl_t_step, ctrl_t_updt;
wire ctrl_p_rst, ctrl_p_stop, ctrl_p_run, ctrl_p_pause, ctrl_p_step ;
wire ctrl_c_step ;
wire time_updt_cpu;

// In case of Inputs are in U...
assign time_rst_s   = time_rst_i   & ~xreg_TPROC_CFG[10] ;
assign time_updt_s  = time_updt_i  & ~xreg_TPROC_CFG[10] ;
assign proc_rst_s   = proc_rst_i   & ~xreg_TPROC_CFG[10] ;
assign proc_stop_s  = proc_stop_i  & ~xreg_TPROC_CFG[10] ;
assign proc_run_s   = proc_run_i   & ~xreg_TPROC_CFG[10] ;
assign proc_pause_s = proc_pause_i & ~xreg_TPROC_CFG[10] ;
assign time_init_s  = time_init_i  & ~xreg_TPROC_CFG[10] ;

// Processor Control [ Python | External Input | CPU ]
assign ctrl_t_rst     = xreg_TPROC_CTRL[0]  | time_rst_s   | time_rst_cpu;
assign ctrl_t_updt    = xreg_TPROC_CTRL[1]  | time_updt_s  | time_updt_cpu;
assign ctrl_p_rst     = xreg_TPROC_CTRL[2]  | proc_rst_s   ;
assign ctrl_p_stop    = xreg_TPROC_CTRL[3]  | proc_stop_s  ;
assign ctrl_p_run     = xreg_TPROC_CTRL[4]  | proc_run_s   ;
assign ctrl_p_pause   = xreg_TPROC_CTRL[5]  | proc_pause_s ;
assign ctrl_p_step    = xreg_TPROC_CTRL[6]  ;
assign ctrl_t_freeze  = xreg_TPROC_CTRL[7]  ;
assign ctrl_t_step    = xreg_TPROC_CTRL[8]  ;
assign ctrl_c_step    = xreg_TPROC_CTRL[9]  ;

// TIME CONTROL
///////////////////////////////////////////////////////////////////////////////
reg ctrl_t_updt_r, ctrl_t_init_r;
reg [31:0] offset_dt_r;

assign time_rst_cpu   = ( core_usr_addr[0] & core_usr_operation[0]) ;
assign time_updt_cpu  = ( core_usr_addr[0] & core_usr_operation[1]) ;
assign time_ref_set   = ( core_usr_addr[0] & core_usr_operation[2]) ;
assign time_ref_inc   = ( core_usr_addr[0] & core_usr_operation[3]) ;

// Selec Source of Time Update Data, depending on the Source of ctrl_t_updt
always_ff @(posedge c_clk_i)
   if (!c_rst_ni) begin
      ctrl_t_updt_r   <= 0;
      ctrl_t_init_r   <= 0;
      offset_dt_r     <= 0;
   end else begin
      ctrl_t_updt_r     <= ctrl_t_updt;
      ctrl_t_init_r     <= time_init_s;
      if      ( time_updt_cpu   )            offset_dt_r  <= core_usr_a_dt ;
      else if ( time_updt_s | time_init_s  ) offset_dt_r  <= offset_dt_i ;
      else if ( xreg_TPROC_CTRL[1] )         offset_dt_r  <= tproc_w_dt[0] ;
      
   end


reg c_core_en, proc_rst, c_time_rst, c_time_init;
   
// Time REF
///////////////////////////////////////////////////////////////////////////////
reg [47:0] c_time_ref_dt;

always_ff @ (posedge c_clk_i, negedge c_rst_ni) begin
   if (!c_rst_ni)              c_time_ref_dt    <= '{default:'0} ;
   else if (c_time_rst)      c_time_ref_dt    <= '{default:'0} ;
   else if (c_time_en)
      if       (time_ref_set )  c_time_ref_dt    <=  {16'd0, core_usr_b_dt} ;
      else if  (time_ref_inc )  c_time_ref_dt    <=  c_time_ref_dt + {16'd0, core_usr_b_dt} ;
end
   

// DEBUG - STEP CONTROL
///////////////////////////////////////////////////////////////////////////////
wire step;
reg step_r;
assign step = ctrl_p_step | ctrl_c_step | ctrl_t_step;
always_ff @(posedge c_clk_i) if (!c_rst_ni) step_r <= 0; else step_r = step;




enum {T_RST, P_RST,RST_WAIT,  T_INIT, STOP, PLAY, PAUSE, UPDATE, FREEZE, END_STEP} proc_st_nxt, proc_st;

always_ff @(posedge c_clk_i)
   if (!c_rst_ni)   proc_st  <= T_RST;
   else             proc_st  <= proc_st_nxt;
      
always_comb begin
   c_core_en      = 0;
   proc_rst       = 0;
   c_time_en      = 0;
   c_time_rst     = 0;
   c_time_init    = 0;
   proc_st_nxt = proc_st;
   //COMMON TRANSITIONS
   if       ( ctrl_p_rst    )  proc_st_nxt = P_RST;
   else if  ( ctrl_t_rst    )  proc_st_nxt = T_RST;
   else if  ( ctrl_t_init_r )  proc_st_nxt = T_INIT;
   else if  ( ctrl_p_stop   )  proc_st_nxt = STOP;
   else if  ( ctrl_p_run    )  proc_st_nxt = PLAY;
   else if  ( ctrl_t_updt_r )  proc_st_nxt = UPDATE;

   
// Just for Debugging
   else if  ( ctrl_t_freeze )  proc_st_nxt = FREEZE;
   else if  ( ctrl_p_pause  )  proc_st_nxt = PAUSE;
   else if  ( ctrl_p_step   )  proc_st_nxt = PLAY;
   else if  ( ctrl_t_step   )  proc_st_nxt = PAUSE;
   else if  ( ctrl_c_step   )  proc_st_nxt = FREEZE;
   
   case (proc_st)
      T_RST : begin
         proc_st_nxt = P_RST ;
         c_time_rst = 1;
      end
      P_RST : begin
         if (all_fifo_full) proc_st_nxt = RST_WAIT;
         proc_rst = 1;            
      end
      RST_WAIT : begin
         if (!all_fifo_full) proc_st_nxt = STOP;
      end
      T_INIT: begin
         proc_st_nxt = PLAY;
         proc_rst = 1;            
      end
      PLAY: begin
         if    ( step_r )      proc_st_nxt = END_STEP;
         c_time_en = 1;
         c_core_en = 1;
      end
      PAUSE: begin
         if    ( step_r )      proc_st_nxt = END_STEP;
         c_time_en = 1;
      end
      UPDATE: begin
         proc_st_nxt = PLAY;
         c_core_en = 1;
         c_time_en = 1;
      end
      FREEZE: begin
         if    ( step_r )      proc_st_nxt = END_STEP;
         c_core_en = 1;
      end
      END_STEP: begin
         if  (!step_r)       proc_st_nxt = STOP;
         else                proc_st_nxt = END_STEP;
      end

   endcase
end

///////////////////////////////////////////////////////////////////////////////
// Timing Control    // CHECK CLOCK BOUNDARY CROSSING //
///////////////////////////////////////////////////////////////////////////////



reg [OUT_DPORT_QTY-1:0]  c_fifo_data_empty, t_fifo_data_full ;
wire [OUT_DPORT_QTY-1:0]  t_fifo_data_empty, c_fifo_data_full ;

reg [OUT_WPORT_QTY-1:0]  c_fifo_wave_empty, t_fifo_wave_full;
wire [OUT_WPORT_QTY-1:0]  t_fifo_wave_empty , c_fifo_wave_full   ;


wire div_rdy, arith_rdy;
wire dfifo_full, wfifo_full, fifo_ok;


// STATUS 
///////////////////////////////////////////////////////////////////////////////
reg arith_rdy_r, div_rdy_r, tnet_rdy_r, periph_rdy_r;
reg arith_dt_new, div_dt_new, tnet_dt_new, periph_dt_new ;
wire status_clr, arith_clr, div_clr, tnet_clr, periph_clr ;
wire port_clr;
wire [2:0]core0_src_dt, core1_src_dt;

wire [31:0] core0_cfg, core1_cfg;
assign core0_src_dt = core0_cfg[2:0];
assign core1_src_dt = core1_cfg[2:0];

assign status_clr   = core0_cfg[3] | core1_cfg[3] ; 
assign arith_clr    = core0_cfg[4] | core1_cfg[4] ;
assign div_clr      = core0_cfg[5] | core1_cfg[5] ;
assign tnet_clr     = core0_cfg[6] | core1_cfg[6] ;
assign periph_clr   = core0_cfg[7] | core1_cfg[7] ;
assign port_clr     = core0_cfg[8] | core1_cfg[8] ;



reg  [31 :0]        tnet_dt_r [2] ;
reg  [31 :0]        periph_dt_r [2] ;

// With rising edge of RDY detect new values
always_ff @(posedge c_clk_i) begin
   if (proc_rst) begin
      arith_rdy_r  <= 1'b0 ;
      div_rdy_r    <= 1'b0 ;
      tnet_rdy_r   <= 1'b0 ;
      periph_rdy_r <= 1'b0 ;
      arith_dt_new  <= 1'b0 ;
      div_dt_new    <= 1'b0 ;
      tnet_dt_new   <= 1'b0 ;
      periph_dt_new <= 1'b0 ;
   end else begin 
      arith_rdy_r  <= arith_rdy   ;
      div_rdy_r    <= div_rdy     ;
      tnet_rdy_r   <= tnet_rdy_i  ;
      periph_rdy_r <= periph_rdy_i;     
      // Arith Control
      if       ( arith_rdy    & ~arith_rdy_r  ) 
         arith_dt_new  <= 1 ;
      else if  (~arith_rdy    &  arith_rdy_r  ) arith_dt_new  <= 0 ;
      else if  (arith_clr) arith_dt_new  <= 0 ;
      // DIV Control
      if       ( div_rdy      & ~div_rdy_r    )
         div_dt_new    <= 1 ;
      else if  (~div_rdy      &  div_rdy_r    ) div_dt_new    <= 0 ;
      else if  (div_clr) div_dt_new  <= 0 ;
      // TNET Control
      if       ( tnet_rdy_i   & ~tnet_rdy_r   ) begin 
         tnet_dt_new   <= 1 ;
         tnet_dt_r     <= tnet_dt_i ;
      end else if  (~tnet_rdy_i   &  tnet_rdy_r   ) tnet_dt_new   <= 0 ;
      else if  (tnet_clr) tnet_dt_new  <= 0 ;
      // PERIPH Control
      if       ( periph_rdy_i & ~periph_rdy_r ) begin
         periph_dt_new <= 1 ;
         periph_dt_r   <= periph_dt_i ;
      end else if  (~periph_rdy_i &  periph_rdy_r ) periph_dt_new <= 0 ;
      else if  (periph_clr) periph_dt_new  <= 0 ;
     
   end
end

wire [31:0] sreg_status;
assign sreg_status[0] =  arith_dt_new ;
assign sreg_status[1] =  div_dt_new   ;
assign sreg_status[2] =  tnet_dt_new  ;
assign sreg_status[3] =  periph_dt_new;
assign sreg_status[4] =  arith_rdy_r  ;
assign sreg_status[5] =  div_rdy_r    ;
assign sreg_status[6] =  tnet_rdy_r   ;
assign sreg_status[7] =  periph_rdy_r ;

assign sreg_status[8]  =  c_fifo_data_full ;
assign sreg_status[9]  =  c_fifo_data_empty ;
assign sreg_status[10] =  c_fifo_wave_full ;
assign sreg_status[11] =  c_fifo_wave_empty ;


assign sreg_status[31:16] = port_dt_new ;



// EXTERNAL CONDITION FLAG GENERATION
///////////////////////////////////////////////////////////////////////////////
reg ext_cond_r ;

wire cond_ext_set, cond_ext_clear;
assign cond_ext_set   = xreg_TPROC_CTRL[10] | (core_usr_addr[1] & core_usr_operation[0]);
assign cond_ext_clear = xreg_TPROC_CTRL[11] | (core_usr_addr[1] & core_usr_operation[1]);



always_ff @(posedge c_clk_i) begin
   if (proc_rst) begin
      ext_cond_r        <= 0;
   end else begin 
      // Condition 
      if       ( cond_ext_set   )     ext_cond_r     <= 1 ; // SET COND
      else if  ( cond_ext_clear )     ext_cond_r     <= 0 ; // CLEAR COND

   end
end

TYPE_IF_AXI_REG            s_axireg_00 ();
TYPE_IF_AXI_REG            s_axireg_01 ();
TYPE_IF_AXI_REG            s_axireg_10 ();
TYPE_IF_AXI_REG            s_axireg_11 ();
wire axi_ready, axi_bvalid;



// Co Processors
///////////////////////////////////////////////////////////////////////////////

wire [63:0] arith_result;
wire [31:0] div_remainder, div_quotient;

generate
   if (DIVIDER == 1) begin : DIVIDER_YES
      div_r #(
         .DW     ( 32 ) ,
         .N_PIPE ( 32 )
      ) div_r_inst (
         .clk_i           ( c_clk_i ) ,
         .rst_ni          ( c_rst_ni ) ,
         .start_i         ( core_usr_addr[3] ) ,
         .A_i             ( core_usr_a_dt ) ,
         .B_i             ( core_usr_b_dt ) ,
         .ready_o         ( div_rdy  ) ,
         .div_remainder_o ( div_remainder ) ,
         .div_quotient_o  ( div_quotient ) );
   end else begin : DIVIDER_NO
      assign div_rdy          = 0;
      assign div_remainder    = 0;
      assign div_quotient     = 0;
   end
endgenerate

generate
   if (ARITH == 1) begin : ARITH_YES
      arith arith_inst (
         .clk_i          ( c_clk_i ) ,
         .rst_ni         ( c_rst_ni ) ,
         .start_i        ( core_usr_addr[2] ) ,
         .A_i            ( core_usr_a_dt ) ,
         .B_i            ( core_usr_b_dt ) ,
         .C_i            ( core_usr_c_dt ) ,
         .D_i            ( core_usr_d_dt ) ,
         .alu_op_i       ( core_usr_operation ) ,
         .ready_o        ( arith_rdy ) ,
         .arith_result_o ( arith_result ) );
   end else begin : ARITH_NO
      assign arith_result     = 0;
   end
endgenerate

generate
   if ( TIME_READ == 1) begin : TIME_READ_YES
      assign t_time_usr = (time_abs_r - c_time_ref_dt);
      sync_reg sync_time_usr_c (
         .dt_i       ( t_time_usr   ) ,
         .clk_i      ( c_clk_i      ) ,
         .rst_ni     ( c_rst_ni     ) ,
         .dt_o       ( c_time_usr   ) );
   end else begin : TIME_READ_NO
      assign t_time_usr       = 0;
      assign c_time_usr       = 0;
   end
endgenerate


///////////////////////////////////////////////////////////////////////////////
// INSTANCES
///////////////////////////////////////////////////////////////////////////////
wire [1:0]       ext_core_sel     ;
wire [1:0]       ext_mem_sel  ;
wire             ext_mem_we  ;
wire [15:0]      ext_mem_addr;
wire [167:0]     ext_mem_w_dt     ;
wire [167:0]     ext_mem_r_dt, ext_mem_r_0_dt, ext_mem_r_1_dt     ;


assign ext_mem_r_dt = ext_core_sel[0] ?  ext_mem_r_1_dt : ext_mem_r_0_dt ;


// T PROCESSOR CORE
///////////////////////////////////////////////////////////////////////////////
wire [31:0] core_do;




// CLOCK DOMAIN CHANGE
reg [OUT_WPORT_QTY-1:0] fifo_data_empty_rcd, fifo_wave_empty_rcd;
always_ff @(posedge c_clk_i) begin
   fifo_data_empty_rcd      <= t_fifo_data_empty;
   fifo_wave_empty_rcd      <= t_fifo_data_empty;
   c_fifo_data_empty        <= fifo_data_empty_rcd;
   c_fifo_wave_empty        <= fifo_wave_empty_rcd;
end

reg [OUT_WPORT_QTY-1:0] fifo_data_full_rcd, fifo_wave_full_rcd ;
reg all_fifo_empty_r;
always_ff @(posedge t_clk_i) begin
   fifo_data_full_rcd      <= c_fifo_data_full;
   fifo_wave_full_rcd      <= c_fifo_wave_full;
   t_fifo_data_full        <= fifo_data_full_rcd;
   t_fifo_wave_full        <= fifo_wave_full_rcd;
end




assign all_dfifo_empty = &c_fifo_data_empty ;
assign all_wfifo_empty = &c_fifo_wave_empty ;
assign all_fifo_empty = all_dfifo_empty & all_wfifo_empty ;   

assign all_dfifo_full = &c_fifo_data_full ;
assign all_wfifo_full = &c_fifo_wave_full ;
assign all_fifo_full = all_dfifo_full & all_wfifo_full ;   

assign dfifo_full = |c_fifo_data_full & xreg_TPROC_CFG[10]; // With 1 BLOCK
assign wfifo_full = |c_fifo_wave_full & xreg_TPROC_CFG[11]; // With 1 BLOCK
assign fifo_ok    = ~wfifo_full & ~dfifo_full ;

wire core_en;
assign core_en = c_core_en  & fifo_ok; 



reg [31:0] core0_r_dt [2], core0_w_dt [2];

always_comb begin
   case (core0_src_dt)
      3'b000 : core0_r_dt = tproc_w_dt ;
      3'b001 : core0_r_dt = core0_w_dt;
      3'b010 : core0_r_dt = '{default:'0} ;
      3'b011 : core0_r_dt = '{default:'0} ;
      3'b100 : core0_r_dt = periph_dt_i;
      3'b101 : core0_r_dt = reg_axi_dt;
      3'b110 : core0_r_dt = '{default:'0} ;
      3'b111 : core0_r_dt = tnet_dt_i ;
      default: core0_r_dt = '{default:'0} ;
   endcase
end


wire [1:0]core0_src_flg;
assign core0_src_flg = core0_cfg[10:9];
reg flag_c0;
always_comb begin
   case (core0_src_flg)
      2'b00 : flag_c0 = ext_cond_r ;
      2'b01 : flag_c0 = |port_dt_new ;
      2'b10 : flag_c0 = 0;
      2'b11 : flag_c0 = 0;
      default : flag_c0 = 0;
   endcase
end


qproc_core # (
   .LFSR        (  LFSR  ),
   .IN_PORT_QTY (  IN_PORT_QTY  ),
   .PMEM_AW     (   PMEM_AW  ),
   .DMEM_AW     (   DMEM_AW  ),
   .WMEM_AW     (   WMEM_AW  ),
   .REG_AW      (   REG_AW  )
) CORE_0 (
   .c_clk_i          ( c_clk_i           ) ,
   .c_rst_ni         ( c_rst_ni          ) ,
   .ps_clk_i         ( ps_clk_i          ) ,
   .ps_rst_ni        ( ps_rst_ni         ) ,
   .en_i             ( core_en           ) ,    
   .restart_i        ( proc_rst          ) ,    
   .IF_s_axireg      ( s_axireg_01       ) ,
   .port_dt_i        ( in_port_dt_r      ) , //ALL The port Values
   .flag_i           ( flag_c0          ) ,
   .sreg_cfg_o       ( core0_cfg          ) ,
   .sreg_arith_i     ( arith_result[31:0] ) ,
   .sreg_div_i       ( {div_quotient  ,div_remainder }  ) ,
   .sreg_status_i    ( sreg_status       ) ,
   .sreg_core_r_dt_i ( core0_r_dt        ) ,
   .sreg_time_dt_i   ( c_time_usr        ) , 
   .sreg_core_w_dt_o ( core0_w_dt        ) ,
   .usr_dt_a_o       ( core_usr_a_dt     ) ,
   .usr_dt_b_o       ( core_usr_b_dt     ) ,
   .usr_dt_c_o       ( core_usr_c_dt     ) ,
   .usr_dt_d_o       ( core_usr_d_dt     ) ,
   .usr_ctrl_o       ( {core_usr_operation, core_usr_addr}  ) ,
   .ps_mem_sel_i     ( ext_mem_sel       ) ,
   .ps_mem_we_i      ( ext_mem_we        ) ,
   .ps_mem_addr_i    ( ext_mem_addr      ) ,
   .ps_mem_w_dt_i    ( ext_mem_w_dt      ) ,
   .ps_mem_r_dt_o    ( ext_mem_r_0_dt      ) ,
   .port_we_o        ( port_we           ) ,
   .port_o           ( out_port_data     ) ,
   .core_do          ( core0_do           ) );

reg [31:0] core1_w_dt [2];

generate
   if ( DUAL_CORE == 1) begin : DUAL_CORE_YES
      
      reg  [31:0] core1_r_dt [2];

      always_comb begin
      case (core1_src_dt)
            3'b000 : core1_r_dt = tproc_w_dt ;
            3'b001 : core1_r_dt = core0_w_dt;
            3'b010 : core1_r_dt = core1_w_dt;
            3'b011 : core1_r_dt = '{default:'0} ;
            3'b100 : core1_r_dt = periph_dt_i;
            3'b101 : core1_r_dt = reg_axi_dt;
            3'b110 : core1_r_dt = '{default:'0} ;
            3'b111 : core1_r_dt = tnet_dt_i ;
            default: core1_r_dt = '{default:'0} ;
         endcase
         end
   
      qproc_core # (
         .LFSR        (  LFSR  ),
         .IN_PORT_QTY (  IN_PORT_QTY  ),
         .PMEM_AW     (  PMEM_AW  ),
         .DMEM_AW     (  DMEM_AW  ),
         .WMEM_AW     (  WMEM_AW  ),
         .REG_AW      (  REG_AW   )
      ) CORE_1 (
         .c_clk_i          ( c_clk_i           ) ,
         .c_rst_ni         ( c_rst_ni          ) ,
         .ps_clk_i         ( ps_clk_i          ) ,
         .ps_rst_ni        ( ps_rst_ni         ) ,
         .en_i             ( core_en           ) ,    
         .restart_i        ( proc_rst          ) ,    
         .IF_s_axireg      ( s_axireg_10       ) , // AXI REG 10
         .port_dt_i        ( in_port_dt_r      ) , 
         .sreg_arith_i     ( {arith_result[63:32],arith_result[31:0]}  ) ,
         .sreg_div_i       ( {div_quotient  ,div_remainder }  ) ,
         .sreg_status_i    ( core1_status       ) ,
         .sreg_core_r_dt_i ( core1_r_dt        ) ,
         .sreg_core_w_dt_o ( core1_w_dt        ) ,
         .sreg_time_dt_i   ( c_time_usr        ) , 
         .sreg_cfg_o       ( core1_cfg          ) ,
         .usr_dt_a_o       ( ) ,
         .usr_dt_b_o       ( ) ,
         .usr_dt_c_o       ( ) ,
         .usr_dt_d_o       ( ) ,
         .usr_ctrl_o       ( ) ,
         .ps_mem_sel_i     ( ext_mem_sel       ) ,
         .ps_mem_we_i      ( ext_mem_we        ) ,
         .ps_mem_addr_i    ( ext_mem_addr      ) ,
         .ps_mem_w_dt_i    ( ext_mem_w_dt      ) ,
         .ps_mem_r_dt_o    ( ext_mem_r_1_dt      ) ,
         .port_we_o        (            ) ,
         .port_o           (      ) ,
         .core_do          (      ) );
   end else begin : DUAL_CORE_NO
      assign core1_w_dt     = '{default:'0} ;
      assign core1_cfg      = '{default:'0} ;
      assign ext_mem_r_1_dt = '{default:'0} ;
   end
endgenerate


qproc_time_ctrl QTIME_CTRL ( 
   .t_clk_i        ( t_clk_i        ) ,
   .t_rst_ni       ( t_rst_ni       ) ,
   .c_time_en_i    ( c_time_en      ) ,
   .c_time_rst_i   ( c_time_rst     ) ,
   .c_time_init_i  ( ctrl_t_init_r    ) ,
   .c_time_updt_i  ( ctrl_t_updt_r  ) ,
   .c_offset_dt_i  ( offset_dt_r    ) ,
   .t_time_en_o    ( t_time_en      ) ,
   .t_time_abs_o   ( time_abs_r     ) ,
   .t_init_off_o   ( init_off       ) );


reg  [31:0] tproc_r_dt [2];
wire [ 2:0] tproc_src_dt;

assign tproc_src_dt = xreg_TPROC_CFG[9:7];

always_comb begin
   case (tproc_src_dt)
      3'b000 : tproc_r_dt = tproc_w_dt ;
      3'b001 : tproc_r_dt = core0_w_dt;
      3'b010 : tproc_r_dt = core1_w_dt;
      3'b011 : tproc_r_dt = tnet_dt_r ;
      3'b100 : tproc_r_dt = periph_dt_r;
      3'b101 : tproc_r_dt = '{default:'0} ;
      3'b110 : tproc_r_dt = '{default:'0} ;
      3'b111 : tproc_r_dt = '{default:'0} ;
      default: tproc_r_dt = '{default:'0} ;
   endcase
end

assign xreg_TPROC_STATUS[19 : 16]  = { 1'b0 , fifo_ok, wfifo_full, dfifo_full };
assign xreg_TPROC_STATUS[15 : 12]  = { all_wfifo_full, all_dfifo_full, all_wfifo_empty, all_dfifo_empty };
assign xreg_TPROC_STATUS[11 :  8]  = { 1'b0  , flag_c0, port_dt_new , ext_cond_r};
assign xreg_TPROC_STATUS[7  :  4]  = { 1'b0  , proc_rst, c_time_en, core_en};
assign xreg_TPROC_STATUS[3  :  0]  = { 1'b0  , proc_st};

assign xreg_TPROC_DEBUG[15: 8]  = { c_time_ref_dt[7:0]};
assign xreg_TPROC_DEBUG[ 7: 4]  = { t_fifo_data_dt[0][3:0]};
assign xreg_TPROC_DEBUG[ 3: 0]  = { t_fifo_data_time[0][3:0]};


// AXI Slave.
qproc_axi_reg QPROC_xREG (
   .ps_aclk          ( ps_clk_i         ) , 
   .ps_aresetn       ( ps_rst_ni        ) , 
   .c_clk_i          ( c_clk_i         ) , 
   .c_rst_ni         ( c_rst_ni        ) , 
   .IF_s_axireg      ( s_axireg_00      ) ,
   .TPROC_CTRL       ( xreg_TPROC_CTRL       ) ,
   .TPROC_CFG        ( xreg_TPROC_CFG        ) ,
   .MEM_ADDR         ( xreg_MEM_ADDR         ) ,
   .MEM_LEN          ( xreg_MEM_LEN          ) ,
   .INIT_TIME        ( offset_dt_r[31:0]     ) ,
   .TPROC_R_DT1      ( 0        ) ,
   .TPROC_R_DT2      ( 0        ) ,
   .MEM_DT_I         ( xreg_MEM_DT_I         ) ,
   .MEM_DT_O         ( xreg_MEM_DT_O         ) ,
   .TIME_USR         ( c_time_usr      ) ,
   .TPROC_W_DT1      ( tproc_w_dt [0]        ) ,
   .TPROC_W_DT2      ( tproc_w_dt [1]        ) ,
   .TPROC_STATUS     ( xreg_TPROC_STATUS     ) ,
   .TPROC_DEBUG      ( xreg_TPROC_DEBUG      ) );
   
qproc_mem_ctrl # (
   .PMEM_AW ( PMEM_AW ),
   .DMEM_AW ( DMEM_AW ),
   .WMEM_AW ( WMEM_AW )
) Q_MEM_CTRL (
   .ps_clk_i         ( ps_clk_i              ) ,
   .ps_rst_ni        ( ps_rst_ni             ) ,
   .ext_core_sel_o   ( ext_core_sel      ) ,
   .ext_mem_sel_o    ( ext_mem_sel   ) ,
   .ext_mem_we_o     ( ext_mem_we   ) ,
   .ext_mem_addr_o   ( ext_mem_addr ) ,
   .ext_mem_w_dt_o   ( ext_mem_w_dt      ) ,
   .ext_mem_r_dt_i   ( ext_mem_r_dt      ) ,
   .s_axis_tdata_i   ( s_dma_axis_tdata_i    ) ,
   .s_axis_tlast_i   ( s_dma_axis_tlast_i    ) ,
   .s_axis_tvalid_i  ( s_dma_axis_tvalid_i   ) ,
   .s_axis_tready_o  ( s_dma_axis_tready_o   ) ,
   .m_axis_tdata_o   ( m_dma_axis_tdata_o    ) ,
   .m_axis_tlast_o   ( m_dma_axis_tlast_o    ) ,
   .m_axis_tvalid_o  ( m_dma_axis_tvalid_o   ) ,
   .m_axis_tready_i  ( m_dma_axis_tready_i   ) ,
   .MEM_CTRL         ( xreg_TPROC_CFG[7:0]        ) ,
   .MEM_ADDR         ( xreg_MEM_ADDR              ) ,
   .MEM_LEN          ( xreg_MEM_LEN               ) ,
   .MEM_DT_I         ( xreg_MEM_DT_I              ) ,
   .MEM_DT_O         ( xreg_MEM_DT_O              ) ,
   .STATUS_O         ( xreg_TPROC_STATUS[31:24]   ) ,
   .DEBUG_O          ( xreg_TPROC_DEBUG[31:16]    ) );
   

///////////////////////////////////////////////////////////////////////////////
/// FIFO 
///////////////////////////////////////////////////////////////////////////////
 
wire [167:0]        t_fifo_wave_dt      [OUT_WPORT_QTY-1:0];
wire [47:0]         t_fifo_wave_time    [OUT_WPORT_QTY-1:0];
wire [47 :0]        W_RESULT          [OUT_WPORT_QTY-1:0] ;

reg  data_pop   [OUT_DPORT_QTY] ; 
reg  data_pop_r [OUT_DPORT_QTY]; 
reg  data_pop_r2   [OUT_DPORT_QTY]; 
reg  data_pop_r3   [OUT_DPORT_QTY]; 
reg  data_pop_r4   [OUT_DPORT_QTY]; 


wire [31 :0]               t_fifo_data_dt      [OUT_DPORT_QTY-1:0] ;
wire [47: 0]               t_fifo_data_time    [OUT_DPORT_QTY-1:0];
wire [47 :0]               D_RESULT          [OUT_DPORT_QTY-1:0] ;


reg  [OUT_WPORT_QTY-1:0]  c_fifo_wave_push, c_fifo_wave_push_r ;
reg  [OUT_DPORT_QTY-1:0]  c_fifo_data_push, c_fifo_data_push_r ; 
reg [47 :0]                c_fifo_time_in_r ;
reg [167:0]         c_fifo_data_in_r ;
reg data_pop_prev [OUT_DPORT_QTY-1:0] ;

/// FIFO CTRL-REG
///////////////////////////////////////////////////////////////////////////////
wire fifo_we;
assign fifo_we = port_we & core_en ;

always_ff @ (posedge c_clk_i, negedge c_rst_ni) begin
   if (!c_rst_ni) begin
      c_fifo_data_in_r       <= '{default:'0} ;
      c_fifo_time_in_r       <= '{default:'0} ;
      c_fifo_wave_push_r     <= '{default:'0} ;
      c_fifo_data_push_r     <= '{default:'0} ;
   end else begin
      c_fifo_data_in_r       <= out_port_data.p_data ;
      c_fifo_time_in_r       <= {16'd0, out_port_data.p_time} + c_time_ref_dt;
      c_fifo_data_push_r     <= c_fifo_data_push ;
      c_fifo_wave_push_r     <= c_fifo_wave_push ;
   end
end

always_comb begin
   c_fifo_wave_push    = 0;
   c_fifo_data_push    = 0;
   if (fifo_we)
      if (out_port_data.p_type)
         c_fifo_data_push [out_port_data.p_addr] = 1'b1 ;
      else
         c_fifo_wave_push [out_port_data.p_addr] = 1'b1 ;
end   

 ///////////////////////////////////////////////////////////////////////////////
/// WAVE PORT
///////////////////////////////////////////////////////////////////////////////
reg [OUT_WPORT_QTY-1:0] wave_t_eq, wave_t_gr;
reg                     wave_pop_prev      [OUT_WPORT_QTY-1:0] ;
reg [OUT_WPORT_QTY-1:0]  wave_pop ;
reg [OUT_WPORT_QTY-1:0]  wave_pop_r, wave_pop_r2, wave_pop_r3, wave_pop_r4;

genvar ind_wfifo;
generate
   for (ind_wfifo=0; ind_wfifo < OUT_WPORT_QTY; ind_wfifo=ind_wfifo+1) begin: WAVE_FIFO
      // WaveForm FIFO
      BRAM_FIFO_DC_2 # (
         .FIFO_DW (168+48) , 
         .FIFO_AW (8) 
      ) wave_fifo_inst ( 
         .wr_clk_i   ( c_clk_i   ) ,
         .wr_rst_ni  ( c_rst_ni  ) ,
         .wr_en_i    ( core_en   ) ,
         .push_i     ( c_fifo_wave_push_r   [ind_wfifo] ) ,
         .data_i     ( {c_fifo_data_in_r,c_fifo_time_in_r}     ) ,
         .rd_clk_i   ( t_clk_i   ) ,
         .rd_rst_ni  ( t_rst_ni  ) ,
         .rd_en_i    ( t_time_en ) ,
         .pop_i      ( wave_pop         [ind_wfifo] ) ,
         .data_o     ( {t_fifo_wave_dt[ind_wfifo],t_fifo_wave_time[ind_wfifo]} ) ,
         .flush_i    ( proc_rst ),
         .async_empty_o ( t_fifo_wave_empty [ind_wfifo] ) , // SYNC with RD_CLK
         .async_full_o  ( c_fifo_wave_full  [ind_wfifo] ) ); // SYNC with WR_CLK
      // Time Comparator
         ADDSUB_MACRO #(
            .DEVICE     ( "7SERIES" ),                   // Target Device: "7SERIES" 
            .LATENCY    ( 1         ),                   // Desired clock cycle latency, 0-2
            .WIDTH      ( 48        )                    // Input / output bus width, 1-48
         ) ADDSUB_MACRO_inst (
            .CARRYOUT   (                             ), // 1-bit carry-out output signal
            .RESULT     ( W_RESULT[ind_wfifo]         ), // Add/sub result output, width defined by WIDTH parameter
            .B          ( time_abs_r[47:0]              ), // Input A bus, width defined by WIDTH parameter
            .A          ( t_fifo_wave_time[ind_wfifo]   ), // Input B bus, width defined by WIDTH parameter
            .ADD_SUB    ( 1'b0                        ), // 1-bit add/sub input, high selects add, low selects subtract
            .CARRYIN    ( 1'b0                        ), // 1-bit carry-in input
            .CE         ( 1'b1                        ), // 1-bit clock enable input
            .CLK        ( t_clk_i                       ), // 1-bit clock input
            .RST        ( ~t_rst_ni                     )  // 1-bit active high synchronous reset
         );
      // POP Generator
      always_comb begin : WAVE_DISPATCHER
//         wave_t_eq[ind_wfifo]  = (time_abs_r == t_fifo_wave_time[ind_wfifo]) ;
//         wave_t_gr[ind_wfifo]  = W_RESULT[ind_wfifo][47];
//         wave_pop[ind_wfifo]   = 0;
//         wave_pop_prev[ind_wfifo] = wave_pop_r[ind_wfifo] | wave_pop_r2[ind_wfifo] | wave_pop_r3[ind_wfifo];
//         if (~t_fifo_wave_empty[ind_wfifo])
//            if ( ( wave_t_eq[ind_wfifo] | wave_t_gr[ind_wfifo] ) & ~wave_pop_prev[ind_wfifo] ) 
//               wave_pop      [ind_wfifo] = 1'b1 ;
         wave_t_gr[ind_wfifo]  = W_RESULT[ind_wfifo][47];
         wave_pop[ind_wfifo]   = 0;
         wave_pop_prev[ind_wfifo] = wave_pop_r[ind_wfifo] | wave_pop_r2[ind_wfifo] | wave_pop_r3[ind_wfifo]| wave_pop_r4[ind_wfifo];
         if (t_time_en & ~t_fifo_wave_empty[ind_wfifo])
            if ( wave_t_gr[ind_wfifo] & ~wave_pop_prev[ind_wfifo] ) 
               wave_pop      [ind_wfifo] = 1'b1 ;
      end //ALWAYS
   end // FOR
endgenerate
               
///////////////////////////////////////////////////////////////////////////////
/// DATA PORT
///////////////////////////////////////////////////////////////////////////////
reg [OUT_DPORT_QTY-1:0] data_t_eq, data_t_gr;

genvar ind_dfifo;
generate
   for (ind_dfifo=0; ind_dfifo < OUT_DPORT_QTY; ind_dfifo=ind_dfifo+1) begin: DATA_FIFO
      // DATA FIFO
      BRAM_FIFO_DC_2 # (
         .FIFO_DW (32+48) , 
         .FIFO_AW (8) 
      ) data_fifo_inst ( 
         .wr_clk_i   ( c_clk_i      ) ,
         .wr_rst_ni  ( c_rst_ni     ) ,
         .wr_en_i    ( core_en      ) ,
         .push_i     ( c_fifo_data_push_r[ind_dfifo] ) ,
         .data_i     ( {c_fifo_data_in_r[31:0],c_fifo_time_in_r}  ) ,
         .rd_clk_i   ( t_clk_i      ) ,
         .rd_rst_ni  ( t_rst_ni     ) ,
         .rd_en_i    ( t_time_en    ) ,
         .pop_i      ( data_pop        [ind_dfifo] ) ,
         .data_o     ( {t_fifo_data_dt[ind_dfifo], t_fifo_data_time[ind_dfifo]} ) ,
         .flush_i    ( proc_rst     ),
         .async_empty_o ( t_fifo_data_empty [ind_dfifo] ) , // SYNC with RD_CLK
         .async_full_o  ( c_fifo_data_full  [ind_dfifo] ) ); // SYNC with WR_CLK
      // Time Comparator
      ADDSUB_MACRO #(
            .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
            .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
            .WIDTH      ( 48  )             // Input / output bus width, 1-48
         ) ADDSUB_MACRO_inst (
            .CARRYOUT   (                             ),    // 1-bit carry-out output signal
            .RESULT     ( D_RESULT[ind_dfifo]         ), // Add/sub result output, width defined by WIDTH parameter
            .B          ( time_abs_r[47:0]              ),    // Input A bus, width defined by WIDTH parameter
            .ADD_SUB    ( 1'b0                        ),    // 1-bit add/sub input, high selects add, low selects subtract
            .A          ( t_fifo_data_time[ind_dfifo]   ),   // Input B bus, width defined by WIDTH parameter
            .CARRYIN    ( 1'b0                        ),    // 1-bit carry-in input
            .CE         ( 1'b1                        ),    // 1-bit clock enable input
            .CLK        ( t_clk_i                     ),    // 1-bit clock input
            .RST        ( ~t_rst_ni                   )     // 1-bit active high synchronous reset
         );

      // POP Generator
      always_comb begin : DATA_DISPATCHER
//         data_t_eq[ind_dfifo]  = (time_abs_r == t_fifo_data_time[ind_dfifo]) ;
//         data_t_gr[ind_dfifo]  = D_RESULT[ind_dfifo][47];
//         data_pop[ind_dfifo] = 0;
//         data_pop_prev[ind_dfifo] = data_pop_r[ind_dfifo] | data_pop_r2[ind_dfifo] | data_pop_r3[ind_dfifo];
//         if (~t_fifo_data_empty[ind_dfifo] )
//            if ( ( data_t_eq[ind_dfifo] | data_t_gr[ind_dfifo] ) & ~data_pop_prev[ind_dfifo] ) 
//               data_pop      [ind_dfifo] = 1'b1 ;
         data_t_gr[ind_dfifo]  = D_RESULT[ind_dfifo][47];
         data_pop[ind_dfifo] = 0;
         data_pop_prev[ind_dfifo] = data_pop_r[ind_dfifo] | data_pop_r2[ind_dfifo] | data_pop_r3[ind_dfifo] | data_pop_r4[ind_dfifo];
         if (t_time_en & ~t_fifo_data_empty[ind_dfifo] )
            if ( data_t_gr[ind_dfifo] & ~data_pop_prev[ind_dfifo] ) 
               data_pop      [ind_dfifo] = 1'b1 ;


      end //ALWAYS
   end //FOR      
endgenerate      




axi_lite_deco_4 #(
   .OUTS ( DECO_OUT ) 
) AXI_DECO (
// AXI-Lite DATA Slave I/F.   
   .ps_aclk       ( ps_clk_i           ) , 
   .ps_aresetn    ( ps_rst_ni         ) , 
   .s_axi_lite    ( IF_s_axireg ) ,
   .m00_axi_lite  ( s_axireg_00 ) ,
   .m01_axi_lite  ( s_axireg_01 ) ,
   .m10_axi_lite  ( s_axireg_10 ) ,
   .m11_axi_lite  ( s_axireg_11 ) );


   
// OUT DATA
///////////////////////////////////////////////////////////////////////////////
always_ff @ (posedge t_clk_i, negedge t_rst_ni) begin
   if (!t_rst_ni) begin
      data_pop_r     <= '{default:'0} ;
      data_pop_r2    <= '{default:'0} ;
      data_pop_r3    <= '{default:'0} ;
      data_pop_r4    <= '{default:'0} ;
      wave_pop_r     <= '{default:'0} ;
      wave_pop_r2    <= '{default:'0} ;
      wave_pop_r3    <= '{default:'0} ;
      wave_pop_r4    <= '{default:'0} ;
   end else begin
      data_pop_r     <= data_pop;
      data_pop_r2    <= data_pop_r;
      data_pop_r3    <= data_pop_r2;
      data_pop_r4    <= data_pop_r3;
      wave_pop_r      <= wave_pop;
      wave_pop_r2     <= wave_pop_r;
      wave_pop_r3     <= wave_pop_r2;
      wave_pop_r4     <= wave_pop_r3;

   end
end

reg [31:0]  port_dt_r [OUT_DPORT_QTY];
integer ind_dport;

// OUTPUTS
///////////////////////////////////////////////////////////////////////////////


// OUT DATA
///////////////////////////////////////////////////////////////////////////////
always_ff @ (posedge t_clk_i, negedge t_rst_ni) begin
   for (ind_dport=0; ind_dport < OUT_DPORT_QTY; ind_dport=ind_dport+1) begin: OUT_DATA_PORT
      if (!t_rst_ni) 
         port_dt_r[ind_dport]   <= '{default:'0} ;
      else 
        if (data_pop_r[ind_dport]) port_dt_r[ind_dport] <= t_fifo_data_dt[ind_dport] ;
   end
end
  
assign port_tvalid_o = data_pop_r;
assign port_tdata_o  = port_dt_r;

assign periph_a_dt_o = core_usr_a_dt;
assign periph_b_dt_o = core_usr_b_dt;
assign periph_c_dt_o = core_usr_c_dt;
assign periph_d_dt_o = core_usr_d_dt;
assign periph_addr_o = core_usr_addr;
assign periph_op_o   = core_usr_operation;

///// External Control
assign t_time_abs_o = time_abs_r ;


// OUT WAVES
///////////////////////////////////////////////////////////////////////////////
genvar ind_wport ;
generate
   for (ind_wport=0; ind_wport < OUT_WPORT_QTY; ind_wport=ind_wport+1) begin: OUT_WAVE_PORT
      assign m_axis_tvalid[ind_wport]  = wave_pop_r      [ind_wport] ;
      assign m_axis_tdata [ind_wport]  = t_fifo_wave_dt [ind_wport] ;
   end
endgenerate

// DEBUG
///////////////////////////////////////////////////////////////////////////////
generate
   if (DEBUG == 1) begin : DEBUG_YES
      ///// T_CLOCK Debug Signals   
      assign t_time_usr_do          = t_time_usr ;
      assign t_fifo_do              = {t_fifo_data_time[0][15:0], t_fifo_data_dt[0][15:0]} ;
      assign t_debug_do[31 : 28]     = {data_pop_r2[1], data_pop_r2[0], wave_pop_r2[1], wave_pop_r2[0] } ;
      assign t_debug_do[27 : 24]     = { 4'b0000 };
      assign t_debug_do[23 : 20]     = { c_fifo_wave_empty[1], c_fifo_wave_empty[0], c_fifo_wave_full[1], c_fifo_wave_full[0] } ;
      assign t_debug_do[19 : 16]     = { c_fifo_data_empty[1], c_fifo_data_empty[0], c_fifo_data_full[1], c_fifo_data_full[0] }  ;
      assign t_debug_do[15 : 12]     = {  2'b00, fifo_ok, wfifo_full } ;
      assign t_debug_do[11 :  8]     = {  dfifo_full, all_fifo_full, all_wfifo_full, all_dfifo_full } ;
      assign t_debug_do[ 7 :  4]     = { 4'b0000 };
      assign t_debug_do[ 3 :  0]     = { 1'b0  ,all_fifo_empty , all_wfifo_empty, all_dfifo_empty};
      ///// C_CLOCK Debug Signals   
      assign c_time_usr_do          = t_time_usr ;
      assign c_debug_do[31 : 28]     = {  flag_c0, port_dt_new, 1'b0 , ctrl_p_step } ;
      assign c_debug_do[27 : 24]     = {  ctrl_p_pause, ctrl_p_run, ctrl_p_stop, ctrl_p_rst } ;
      assign c_debug_do[23 : 20]     = {  2'b00, ctrl_t_init_r, ctrl_t_updt_r } ;
      assign c_debug_do[19 : 16]     = {  ctrl_c_step, ctrl_t_step, ctrl_t_freeze, ctrl_t_rst } ;
      assign c_debug_do[15 : 12]     = {  2'b00, core_en, c_time_init } ;
      assign c_debug_do[11 : 8]     = {  c_time_rst, c_time_en, proc_rst, c_core_en } ;
      assign c_debug_do[7 : 4]      = { 1'b0  , ext_cond_r, cond_ext_clear , cond_ext_set};
      assign c_debug_do[3 : 0]      = { 1'b0  , proc_st};
      assign c_time_ref_do = c_time_ref_dt[31:0];
      assign c_port_do     = {c_fifo_data_in_r[15:0] , c_fifo_time_in_r[15:0] } ;
      assign c_core_do     = core0_do ;
   end else begin : DEBUG_NO
      assign t_time_usr_do  = 0 ;
      assign t_fifo_do      = 0 ;
      assign t_debug_do     = 0 ;
      ///// C_CLOCK Debug Signals   
      assign c_time_usr_do  = 0 ;
      assign c_debug_do     = 0 ;
      assign c_time_ref_do  = 0 ;
      assign c_port_do      = 0 ;
      assign c_core_do      = 0 ;
   end
endgenerate

endmodule


