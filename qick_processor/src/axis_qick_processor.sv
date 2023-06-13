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
 
module axis_qick_proccessor # (
   parameter DUAL_CORE      =  0  ,
   parameter IO_CTRL        =  1  ,
   parameter DEBUG          =  1  ,
   parameter TNET           =  1  ,
   parameter CUSTOM_PERIPH  =  1  ,
   parameter LFSR           =  1  ,
   parameter DIVIDER        =  1  ,
   parameter ARITH          =  1  ,
   parameter TIME_READ      =  1  ,
   parameter PMEM_AW        =  10 ,
   parameter DMEM_AW        =  10 ,
   parameter WMEM_AW        =  8  ,
   parameter REG_AW         =  4  ,
   parameter IN_PORT_QTY    =  2  ,
   parameter OUT_TRIG_QTY   =  2  ,
   parameter OUT_DPORT_QTY  =  1  ,
   parameter OUT_DPORT_DW   =  8 ,
   parameter OUT_WPORT_QTY  =  4 
)(
// Core, Time and AXI CLK & RST.
   input  wire                t_clk_i         ,
   input  wire                t_resetn       ,
   input  wire                c_clk_i         ,
   input  wire                c_resetn       ,
   input  wire                ps_clk_i         ,
   input  wire                ps_resetn      ,
// External Control
   input  wire                proc_start_i      ,
   input  wire                proc_stop_i     ,
   input  wire                core_start_i      ,
   input  wire                core_stop_i     ,
   input  wire                time_rst_i      ,
   input  wire                time_init_i     ,
   input  wire                time_updt_i     ,
   input  wire  [31:0]        time_dt_i       ,
   output wire  [47:0]        t_time_abs_o    ,
//TNET_DT
   output wire                tnet_en_o       ,
   output wire  [4 :0]        tnet_op_o       ,
   output wire  [31:0]        tnet_a_dt_o     ,
   output wire  [31:0]        tnet_b_dt_o     ,
   output wire  [31:0]        tnet_c_dt_o     ,
   output wire  [31:0]        tnet_d_dt_o     ,
   input  wire                tnet_rdy_i      , 
   input  wire  [31 :0]       tnet_dt1_i      , 
   input  wire  [31 :0]       tnet_dt2_i      , 
// CUSTOM PERIPHERALS 
   output wire                periph_en_o     ,
   output wire  [4 :0]        periph_op_o     ,
   output wire  [31:0]        periph_a_dt_o   ,
   output wire  [31:0]        periph_b_dt_o   ,
   output wire  [31:0]        periph_c_dt_o   ,
   output wire  [31:0]        periph_d_dt_o   ,
   input  wire                periph_rdy_i    , 
   input  wire  [31 :0]       periph_dt1_i    , 
   input  wire  [31 :0]       periph_dt2_i    , 


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
   input  wire  [7:0]         s_axi_awaddr         ,
   input  wire  [2:0]         s_axi_awprot         ,
   input  wire                s_axi_awvalid        ,
   output wire                s_axi_awready        ,
   input  wire  [31:0]        s_axi_wdata          ,
   input  wire  [3:0]         s_axi_wstrb          ,
   input  wire                s_axi_wvalid         ,
   output wire                s_axi_wready         ,
   output wire  [1:0]         s_axi_bresp          ,
   output wire                s_axi_bvalid         ,
   input  wire                s_axi_bready         ,
   input  wire  [7:0]         s_axi_araddr         ,
   input  wire  [2:0]         s_axi_arprot         ,
   input  wire                s_axi_arvalid        ,
   output wire                s_axi_arready        ,
   output wire  [31:0]        s_axi_rdata          ,
   output wire  [1:0]         s_axi_rresp          ,
   output wire                s_axi_rvalid         ,
   input  wire                s_axi_rready         ,

/// DATA PORT INPUT
   input wire   [63:0]       s0_axis_tdata         ,
   input wire                s0_axis_tvalid        ,
   input wire   [63:0]       s1_axis_tdata         ,
   input wire                s1_axis_tvalid        ,
   input wire   [63:0]       s2_axis_tdata         ,
   input wire                s2_axis_tvalid        ,
   input wire   [63:0]       s3_axis_tdata         ,
   input wire                s3_axis_tvalid        ,
   input wire   [63:0]       s4_axis_tdata         ,
   input wire                s4_axis_tvalid        ,
   input wire   [63:0]       s5_axis_tdata         ,
   input wire                s5_axis_tvalid        ,
   input wire   [63:0]       s6_axis_tdata         ,
   input wire                s6_axis_tvalid        ,
   input wire   [63:0]       s7_axis_tdata         ,
   input wire                s7_axis_tvalid        ,
   input wire   [63:0]       s8_axis_tdata         ,
   input wire                s8_axis_tvalid        ,
   input wire   [63:0]       s9_axis_tdata         ,
   input wire                s9_axis_tvalid        ,
   input wire   [63:0]       s10_axis_tdata        ,
   input wire                s10_axis_tvalid       ,
   input wire   [63:0]       s11_axis_tdata        ,
   input wire                s11_axis_tvalid       ,
   input wire   [63:0]       s12_axis_tdata        ,
   input wire                s12_axis_tvalid       ,
   input wire   [63:0]       s13_axis_tdata        ,
   input wire                s13_axis_tvalid       ,
   input wire   [63:0]       s14_axis_tdata        ,
   input wire                s14_axis_tvalid       ,
   input wire   [63:0]       s15_axis_tdata        ,
   input wire                s15_axis_tvalid       ,
// OUT WAVE PORTS
   // AXI Stream Master  0 ///
   output wire  [167:0]       m0_axis_tdata        ,
   output wire                m0_axis_tvalid       ,
   input  wire                m0_axis_tready       ,
   output wire  [167:0]       m1_axis_tdata        ,
   output wire                m1_axis_tvalid       ,
   input  wire                m1_axis_tready       ,
   output wire  [167:0]       m2_axis_tdata        ,
   output wire                m2_axis_tvalid       ,
   input  wire                m2_axis_tready       ,
   output wire  [167:0]       m3_axis_tdata        ,
   output wire                m3_axis_tvalid       ,
   input  wire                m3_axis_tready       ,
   output wire  [167:0]       m4_axis_tdata        ,
   output wire                m4_axis_tvalid       ,
   input  wire                m4_axis_tready       ,
   output wire  [167:0]       m5_axis_tdata        ,
   output wire                m5_axis_tvalid       ,
   input  wire                m5_axis_tready       ,
   output wire  [167:0]       m6_axis_tdata        ,
   output wire                m6_axis_tvalid       ,
   input  wire                m6_axis_tready       ,
   output wire  [167:0]       m7_axis_tdata        ,
   output wire                m7_axis_tvalid       ,
   input  wire                m7_axis_tready       ,
   output wire  [167:0]       m8_axis_tdata        ,
   output wire                m8_axis_tvalid       ,
   input  wire                m8_axis_tready       ,
   output wire  [167:0]       m9_axis_tdata        ,
   output wire                m9_axis_tvalid       ,
   input  wire                m9_axis_tready       ,
   output wire  [167:0]       m10_axis_tdata       ,
   output wire                m10_axis_tvalid      ,
   input  wire                m10_axis_tready      ,
   output wire  [167:0]       m11_axis_tdata       ,
   output wire                m11_axis_tvalid      ,
   input  wire                m11_axis_tready      ,
   output wire  [167:0]       m12_axis_tdata       ,
   output wire                m12_axis_tvalid      ,
   input  wire                m12_axis_tready      ,
   output wire  [167:0]       m13_axis_tdata       ,
   output wire                m13_axis_tvalid      ,
   input  wire                m13_axis_tready      ,
   output wire  [167:0]       m14_axis_tdata       ,
   output wire                m14_axis_tvalid      ,
   input  wire                m14_axis_tready      ,
   output wire  [167:0]       m15_axis_tdata       ,
   output wire                m15_axis_tvalid      ,
   input  wire                m15_axis_tready      ,
// OUT DATA PORTS
   output reg                 trig_0_o          ,
   output reg                 trig_1_o          ,
   output reg                 trig_2_o          ,
   output reg                 trig_3_o          ,
   output reg                 trig_4_o          ,
   output reg                 trig_5_o          ,
   output reg                 trig_6_o          ,
   output reg                 trig_7_o          ,
   output reg   [OUT_DPORT_DW-1:0] port_0_dt_o          ,
   output reg   [OUT_DPORT_DW-1:0] port_1_dt_o          ,
   output reg   [OUT_DPORT_DW-1:0] port_2_dt_o          ,
   output reg   [OUT_DPORT_DW-1:0] port_3_dt_o          ,


// Debug Signals
   output  wire [31:0]        ps_debug_do        ,
   output wire  [31:0]        t_time_usr_do         ,
   output wire  [31:0]        t_debug_do            ,
   output wire  [31:0]        t_fifo_do             ,
   output wire  [31:0]        c_debug_do            ,
   output wire  [31:0]        c_time_ref_do         ,
   output wire  [31:0]        c_port_do             ,
   output wire  [31:0]        c_core_do             
   );

// DATA IN INTERFACE
reg                  port_tvalid_si  [16]  ;
reg  [63:0]          port_tdata_si   [16] ;


// TRIGGER INTERFACE
wire   port_trig_so     [8] ;

// DATA OUT INTERFACE
wire [OUT_DPORT_DW-1:0]  port_tdata_so     [4] ;
wire                     port_tvalid_so    [4] ;

wire [31:0]          ext_op_r, ext_dt_r, ext_dt_i   ;

wire [167:0]         m_axis_tdata_s [16] ;
wire                 m_axis_tvalid_s[16] ; 
wire                 m_axis_tready_s[16] ;

wire [31:0] t_time_usr_ds , t_debug_ds, t_fifo_ds ;
wire [31:0] c_debug_ds, c_time_ref_ds, c_port_ds, c_core_ds ;

wire [31:0] periph_a_dt, periph_b_dt, periph_c_dt, periph_d_dt ;
wire [4:0] periph_op, periph_addr ;

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


///// DATA IN PORTS /////
///////////////////////////////////////////////////////////////////////////////
always_comb begin
   port_tdata_si[0]   <= s0_axis_tdata ;
   port_tdata_si[1]   <= s1_axis_tdata ;
   port_tdata_si[2]   <= s2_axis_tdata ;
   port_tdata_si[3]   <= s3_axis_tdata ;
   port_tdata_si[4]   <= s4_axis_tdata ;
   port_tdata_si[5]   <= s5_axis_tdata ;
   port_tdata_si[6]   <= s6_axis_tdata ;
   port_tdata_si[7]   <= s7_axis_tdata ;
   port_tdata_si[8]   <= s8_axis_tdata ;
   port_tdata_si[9]   <= s9_axis_tdata ;
   port_tdata_si[10]  <= s10_axis_tdata ;
   port_tdata_si[11]  <= s11_axis_tdata ;
   port_tdata_si[12]  <= s12_axis_tdata ;
   port_tdata_si[13]  <= s13_axis_tdata ;
   port_tdata_si[14]  <= s14_axis_tdata ;
   port_tdata_si[15]  <= s15_axis_tdata ;
   port_tvalid_si[0]   <= s0_axis_tvalid ;
   port_tvalid_si[1]   <= s1_axis_tvalid ;
   port_tvalid_si[2]   <= s2_axis_tvalid ;
   port_tvalid_si[3]   <= s3_axis_tvalid ;
   port_tvalid_si[4]   <= s4_axis_tvalid ;
   port_tvalid_si[5]   <= s5_axis_tvalid ;
   port_tvalid_si[6]   <= s6_axis_tvalid ;
   port_tvalid_si[7]   <= s7_axis_tvalid ;
   port_tvalid_si[8]   <= s8_axis_tvalid ;
   port_tvalid_si[9]   <= s9_axis_tvalid ;
   port_tvalid_si[10]  <= s10_axis_tvalid ;
   port_tvalid_si[11]  <= s11_axis_tvalid ;
   port_tvalid_si[12]  <= s12_axis_tvalid ;
   port_tvalid_si[13]  <= s13_axis_tvalid ;
   port_tvalid_si[14]  <= s14_axis_tvalid ;
   port_tvalid_si[15]  <= s15_axis_tvalid ;
end

qick_processor # (
   .DEBUG          ( DEBUG      ),
   .DUAL_CORE      ( DUAL_CORE      ),
   .LFSR           ( LFSR           ),
   .DIVIDER        ( DIVIDER        ),
   .ARITH          ( ARITH          ),
   .TIME_READ      ( TIME_READ      ),
   .PMEM_AW        ( PMEM_AW        ),
   .DMEM_AW        ( DMEM_AW        ),
   .WMEM_AW        ( WMEM_AW        ),
   .REG_AW         ( REG_AW         ),
   .IN_PORT_QTY    ( IN_PORT_QTY    ),
   .OUT_TRIG_QTY   ( OUT_TRIG_QTY  ),
   .OUT_DPORT_QTY  ( OUT_DPORT_QTY  ),
   .OUT_DPORT_DW   ( OUT_DPORT_DW  ),
   .OUT_WPORT_QTY  ( OUT_WPORT_QTY  )
) QPROC (
   .t_clk_i             ( t_clk_i               ) ,
   .t_rst_ni            ( t_resetn             ) ,
   .c_clk_i             ( c_clk_i               ) ,
   .c_rst_ni            ( c_resetn             ) ,
   .ps_clk_i            ( ps_clk_i              ) ,
   .ps_rst_ni           ( ps_resetn            ) ,
   .proc_start_i        ( proc_start_i            ) ,
   .proc_stop_i         ( proc_stop_i           ) ,
   .core_start_i        ( core_start_i            ) ,
   .core_stop_i         ( core_stop_i           ) ,
   .time_rst_i          ( time_rst_i            ) ,
   .time_init_i         ( time_init_i           ) ,
   .time_updt_i         ( time_updt_i           ) ,
   .time_updt_dt_i      ( time_dt_i           ) ,
   .time_abs_o          ( t_time_abs_o          ) ,
   .periph_a_dt_o       ( periph_a_dt           ) ,
   .periph_b_dt_o       ( periph_b_dt           ) ,
   .periph_c_dt_o       ( periph_c_dt           ) ,
   .periph_d_dt_o       ( periph_d_dt           ) ,
   .periph_addr_o       ( periph_addr           ) ,
   .periph_op_o         ( periph_op             ) ,
   .periph_rdy_i        ( periph_rdy_i           ) ,
   .periph_dt_i         ( {periph_dt2_i, periph_dt1_i} ) ,
   .tnet_rdy_i          ( tnet_rdy_i           ) ,
   .tnet_dt_i           ( {tnet_dt2_i, tnet_dt1_i} ) ,

   .s_dma_axis_tdata_i  ( s_dma_axis_tdata_i    ) ,
   .s_dma_axis_tlast_i  ( s_dma_axis_tlast_i    ) ,
   .s_dma_axis_tvalid_i ( s_dma_axis_tvalid_i   ) ,
   .s_dma_axis_tready_o ( s_dma_axis_tready_o   ) ,
   .m_dma_axis_tdata_o  ( m_dma_axis_tdata_o    ) ,
   .m_dma_axis_tlast_o  ( m_dma_axis_tlast_o    ) ,
   .m_dma_axis_tvalid_o ( m_dma_axis_tvalid_o   ) ,
   .m_dma_axis_tready_i ( m_dma_axis_tready_i   ) ,
   .IF_s_axireg         ( IF_s_axireg           ) ,
   .port_trig_o         ( port_trig_so    [0:OUT_TRIG_QTY-1]      ) ,
   .port_tvalid_i       ( port_tvalid_si  [0:IN_PORT_QTY-1]    ) ,
   .port_tdata_i        ( port_tdata_si   [0:IN_PORT_QTY-1]    ) ,
   .port_tvalid_o       ( port_tvalid_so  [0:OUT_DPORT_QTY-1]  ) ,
   .port_tdata_o        ( port_tdata_so   [0:OUT_DPORT_QTY-1]  ) ,
   .m_axis_tdata        ( m_axis_tdata_s  [0:OUT_WPORT_QTY-1]  ) ,
   .m_axis_tvalid       ( m_axis_tvalid_s [0:OUT_WPORT_QTY-1]  ) ,
   .m_axis_tready       ( m_axis_tready_s [0:OUT_WPORT_QTY-1]  ) ,
   .ps_debug_do          ( ps_debug_do            ) ,
   .t_time_usr_do       ( t_time_usr_ds            ) ,
   .t_debug_do          ( t_debug_ds            ) ,
   .t_fifo_do           ( t_fifo_ds             ) ,
   .c_debug_do          ( c_debug_ds            ) ,
   .c_time_ref_do       ( c_time_ref_ds         ) ,
   .c_port_do           ( c_port_ds             ) ,
   .c_core_do           ( c_core_ds             ) );
       
   




// OUTPUT ASSIGNMENT
///////////////////////////////////////////////////////////////////////////////

///// TNET_DT
assign tnet_en_o    = (periph_addr == 5'b10001);
assign tnet_op_o    = periph_op ; 
assign tnet_a_dt_o  = periph_a_dt ;  
assign tnet_b_dt_o  = periph_b_dt ;  
assign tnet_c_dt_o  = periph_c_dt ;  
assign tnet_d_dt_o  = periph_d_dt ;  

///// PERIPHERALS
assign periph_en_o   = (periph_addr == 5'b10010);
assign periph_op_o   = periph_op ; 
assign periph_a_dt_o = periph_a_dt ;  
assign periph_b_dt_o = periph_b_dt ;  
assign periph_c_dt_o = periph_c_dt ;  
assign periph_d_dt_o = periph_d_dt ;  
   
///// TRIGGER PORTS
genvar ind_t;
generate
   if (OUT_TRIG_QTY < 7)
      for (ind_t=7; ind_t >= OUT_TRIG_QTY; ind_t=ind_t-1) begin: TRIGGER_PORT_NOT_PRESENT
         assign port_trig_so [ind_t] = 0;
      end
endgenerate
   
///// TRIGGERS
assign trig_0_o = port_trig_so[0] ;
assign trig_1_o = port_trig_so[1] ;
assign trig_2_o = port_trig_so[2] ;
assign trig_3_o = port_trig_so[3] ;
assign trig_4_o = port_trig_so[4] ;
assign trig_5_o = port_trig_so[5] ;
assign trig_6_o = port_trig_so[6] ;
assign trig_7_o = port_trig_so[7] ;

   
///// DATA OUT PORTS
genvar ind;
generate
   if (OUT_DPORT_QTY < 3)
      for (ind=3; ind >= OUT_DPORT_QTY; ind=ind-1) begin: DATA_PORT_NOT_PRESENT
         assign port_tdata_so [ind] = '{default:'0} ;
         assign port_tvalid_so[ind] = 0;
      end
endgenerate

assign port_0_dt_o = port_tdata_so[0] ;
assign port_1_dt_o = port_tdata_so[1] ;
assign port_2_dt_o = port_tdata_so[2] ;
assign port_3_dt_o = port_tdata_so[3] ;

///// WAVE OUT PORTS
generate
   if (OUT_WPORT_QTY < 16)
      for (ind=15; ind >= OUT_WPORT_QTY; ind=ind-1) begin: WAVE_PORT_NOT_PRESENT
         assign m_axis_tdata_s[ind]  = '{default:'0} ;
         assign m_axis_tvalid_s[ind] = 0 ;
      end
endgenerate

assign m_axis_tready_s[0]  = m0_axis_tready      ;
assign m_axis_tready_s[1]  = m1_axis_tready      ;
assign m_axis_tready_s[2]  = m2_axis_tready      ;
assign m_axis_tready_s[3]  = m3_axis_tready      ;
assign m_axis_tready_s[4]  = m4_axis_tready      ;
assign m_axis_tready_s[5]  = m5_axis_tready      ;
assign m_axis_tready_s[6]  = m6_axis_tready      ;
assign m_axis_tready_s[7]  = m7_axis_tready      ;
assign m_axis_tready_s[8]  = m8_axis_tready      ;
assign m_axis_tready_s[9]  = m9_axis_tready      ;
assign m_axis_tready_s[10] = m10_axis_tready      ;
assign m_axis_tready_s[11] = m11_axis_tready      ;
assign m_axis_tready_s[12] = m12_axis_tready      ;
assign m_axis_tready_s[13] = m13_axis_tready      ;
assign m_axis_tready_s[14] = m14_axis_tready      ;
assign m_axis_tready_s[15] = m15_axis_tready      ;

assign m0_axis_tdata      = m_axis_tdata_s [0]  ;
assign m0_axis_tvalid     = m_axis_tvalid_s[0]  ;
assign m1_axis_tdata      = m_axis_tdata_s [1]  ;
assign m1_axis_tvalid     = m_axis_tvalid_s[1]  ;
assign m2_axis_tdata      = m_axis_tdata_s [2]  ;
assign m2_axis_tvalid     = m_axis_tvalid_s[2]  ;
assign m3_axis_tdata      = m_axis_tdata_s [3]  ;
assign m3_axis_tvalid     = m_axis_tvalid_s[3]  ;
assign m4_axis_tdata      = m_axis_tdata_s [4]  ;
assign m4_axis_tvalid     = m_axis_tvalid_s[4]  ;
assign m5_axis_tdata      = m_axis_tdata_s [5]  ;
assign m5_axis_tvalid     = m_axis_tvalid_s[5]  ;
assign m6_axis_tdata      = m_axis_tdata_s [6]  ;
assign m6_axis_tvalid     = m_axis_tvalid_s[6]  ;
assign m7_axis_tdata      = m_axis_tdata_s [7]  ;
assign m7_axis_tvalid     = m_axis_tvalid_s[7]  ;
assign m8_axis_tdata      = m_axis_tdata_s [8]  ;
assign m8_axis_tvalid     = m_axis_tvalid_s[8]  ;
assign m9_axis_tdata      = m_axis_tdata_s [9]  ;
assign m9_axis_tvalid     = m_axis_tvalid_s[9]  ;
assign m10_axis_tdata     = m_axis_tdata_s [10]  ;
assign m10_axis_tvalid    = m_axis_tvalid_s[10]  ;
assign m11_axis_tdata     = m_axis_tdata_s [11]  ;
assign m11_axis_tvalid    = m_axis_tvalid_s[11]  ;
assign m12_axis_tdata     = m_axis_tdata_s [12]  ;
assign m12_axis_tvalid    = m_axis_tvalid_s[12]  ;
assign m13_axis_tdata     = m_axis_tdata_s [13]  ;
assign m13_axis_tvalid    = m_axis_tvalid_s[13]  ;
assign m14_axis_tdata     = m_axis_tdata_s [14]  ;
assign m14_axis_tvalid    = m_axis_tvalid_s[14]  ;
assign m15_axis_tdata     = m_axis_tdata_s [15]  ;

///// DEBUG 
generate
   if (DEBUG) begin
      assign t_time_usr_do = t_time_usr_ds ;
      assign t_debug_do    = t_debug_ds    ;
      assign t_fifo_do     = t_fifo_ds     ;
      assign c_debug_do    = c_debug_ds    ;
      assign c_time_ref_do = c_time_ref_ds ;
      assign c_port_do     = c_port_ds     ;
      assign c_core_do     = c_core_ds     ;
   end else begin
      assign t_time_usr_o  = 0;
      assign t_debug_do    = 0;
      assign t_fifo_do     = 0;
      assign c_debug_do    = 0;
      assign c_time_ref_do = 0;
      assign c_port_do     = 0;
      assign c_core_do     = 0;
   end
endgenerate

endmodule
