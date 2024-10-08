`include "_qick_defines.svh"

module qcore_reg_bank # (
   parameter LFSR           =  1 ,
   parameter PMEM_AW        =  8 ,
   parameter REG_AW         =  4 
)(
   input   wire                  clk_i          ,
   input   wire                  halt_i          ,
   input   wire                  rst_ni         ,
   input   wire                  clear_i        ,
   input   wire [1:0]            lfsr_cfg_i     ,
   input   wire [31:0]           reg_arith_i    ,
   input   wire [31:0]           reg_div_i  [2] ,
   input   wire [31:0]           reg_port_i [2]    ,
   input   wire [31:0]           tproc_ext_i[2] ,
   input   wire [31:0]           time_dt_i      ,
   input   wire [167:0]          wave_dt_i      ,
   input   wire [31:0]           status_i       ,
   output  wire [10:0]           reg_cfg_o          ,
   input   wire                  wave_we_i      ,
   input   wire                  we_i           ,
   input   wire [ 6 : 0 ]        w_addr_i       ,
   input   wire [31:0]           w_dt_i         ,
   input   wire [ 5 : 0 ]        rs_A_addr_i[2] ,
   input   wire [ 6 : 0 ]        rs_D_addr_i[2] ,
   output  reg  [31:0]           w_dt_o         ,
   output  wire [31:0]           rs_D_dt_o [2]  ,
   output  wire [31:0]           rs_A_dt_o [2]  ,
   output  wire [31:0]           sreg_dt_o [2]  ,       
   output  wire [PMEM_AW-1:0]    out_addr_o     ,
   output  wire [31:0]           out_time_o     ,       
   output  wire [167:0]          out_wreg_o     ,       
   output  wire [31:0]           lfsr_o         );

/*
The memory is a group of REGs (ZERO, RAND, DIV, MULT, REGBANK), and connected with a generate
Input has 8 Bits>
XX-000000 2 Page and 64 Address
00-Special Registers (32 Bits)
01-User Register (32 Bits)
10-User Register (16 Bits)
11-RFU
*/

// PARAMETERS
///////////////////////////////////////////////////////////////////////////////
localparam REG_QTY    = (2 ** (REG_AW) ) ;

// SIGNALS
///////////////////////////////////////////////////////////////////////////////


//LFSR
wire [31:0] lfsr_reg;
reg lfsr_en;
wire lfsr_sel, lfsr_we, lfsr_step;

// Data Registers 
///////////////////////////////////////////////////////////////////////////////
reg  [31:0]          dreg_32_dt [REG_QTY];
wire [REG_AW-1 : 0]  dreg_32_addr   ;
wire                 dreg_32_en, dreg_32_we;
assign dreg_32_addr  = w_addr_i[5 : 0] ;
assign dreg_32_en   = ~w_addr_i[6] & ~w_addr_i[5] ;
assign dreg_32_we    = we_i & dreg_32_en ;

// Wave Registers
///////////////////////////////////////////////////////////////////////////////
reg  [31:0]          wreg_32_dt [6] ;
wire [ 2:0]          wreg_32_addr   ;
wire                 wreg_32_en, wreg_32_we;
assign wreg_32_addr  = w_addr_i[2:0] ;
assign wreg_32_en    = ~w_addr_i[6] & w_addr_i[5] ;
assign wreg_32_we    = we_i & wreg_32_en;

// SPECIAL REGISTER BANK
///////////////////////////////////////////////////////////////////////////////
reg  [31:0]  sreg_32_dt [4]; // Four SFR
wire         sreg_32_en, sreg_32_we;
wire         sreg_cfg_en, sreg_cfg_we;
reg  [10:0]  sreg_cfg_dt ; // Configuration Register

assign sreg_32_en   = w_addr_i[6:2] == 5'b10011 ; //Register 12 to 15 selected 
assign sreg_32_we   = we_i & sreg_32_en ;
assign sreg_cfg_en  = w_addr_i      == 7'b1000010 ; //Register 2 Selected 
assign sreg_cfg_we  = we_i & sreg_cfg_en;

   
// DATA, WAVE and SFR REGISTER BANK
///////////////////////////////////////////////////////////////////////////////
always_ff @ (posedge clk_i, negedge rst_ni) begin
   if (!rst_ni) begin 
      dreg_32_dt                 = '{default:'0};
      wreg_32_dt                 = '{default:'0};
      sreg_32_dt                 = '{default:'0};
      sreg_cfg_dt                = 0;
   end else if (clear_i) begin
      dreg_32_dt                 = '{default:'0};
      wreg_32_dt                 = '{default:'0};
      sreg_32_dt                 = '{default:'0};
      sreg_cfg_dt                = 0;
   end else begin
      if (~halt_i) begin
         if (dreg_32_we)   
            dreg_32_dt [dreg_32_addr]  = w_dt_i;
         if (wave_we_i) begin
            wreg_32_dt [5]  = wave_dt_i[167:152];
            wreg_32_dt [4]  = wave_dt_i[151:120];
            wreg_32_dt [3]  = wave_dt_i[119: 88];
            wreg_32_dt [2]  = wave_dt_i[ 87: 64];
            wreg_32_dt [1]  = wave_dt_i[ 63: 32];
            wreg_32_dt [0]  = wave_dt_i[ 31:  0];
         end
         if (wreg_32_we)
            wreg_32_dt [wreg_32_addr]  = w_dt_i;
         if (sreg_32_we)   
           sreg_32_dt [w_addr_i[1:0]]  = w_dt_i;
         if (sreg_cfg_we)   
           sreg_cfg_dt                 = w_dt_i;
       // Not Used Register to GND
           sreg_32_dt [3][31:16] = '{default:'0};
      end
   end
end
// LFSR 
///////////////////////////////////////////////////////////////////////////////
// cfg_i 00_FreeRunning 10_Change WHen Read 11_Change when writes to 0
generate
   if (LFSR == 1) begin : LFSR_YES
      always_comb
         unique case (lfsr_cfg_i)
            2'b00 : lfsr_en = 1'b0     ;
            2'b01 : lfsr_en = 1'b1     ;
            2'b10 : lfsr_en = lfsr_sel  ;
            2'b11 : lfsr_en = lfsr_step  ;
         endcase
      
      assign lfsr_sel   = (rs_D_addr_i[0]   == 7'b1000001) ;
      assign lfsr_we    = we_i & (w_addr_i == 7'b1000001) ;
      assign lfsr_step  = we_i & (w_addr_i == 7'b1000000) ;
      
      LFSR lfsr (
         .clk_i       ( clk_i ) ,
         .rst_ni      ( rst_ni ) ,
         .en_i        ( lfsr_en ) ,
         .load_we_i   ( lfsr_we ) ,
         .load_dt_i   ( w_dt_i ) ,
         .lfsr_dt_o   ( lfsr_reg ) );
   end else begin : LFSR_NO
      assign lfsr_sel   = 0;
      assign lfsr_we    = 0;
      assign lfsr_step  = 0;
      assign lfsr_reg   = 0;
   end
endgenerate





// SFR ASSEMBLY
///////////////////////////////////////////////////////////////////////////////
wire [31:0] sreg_dt [16] ;

assign sreg_dt[0]  = 0                  ;
assign sreg_dt[1]  = lfsr_reg           ;
assign sreg_dt[2]  = sreg_cfg_dt  ;
assign sreg_dt[3]  = status_i ;
assign sreg_dt[4]  = reg_div_i[0]       ;
assign sreg_dt[5]  = reg_div_i[1]       ;
assign sreg_dt[6]  = reg_arith_i ;
assign sreg_dt[7]  = tproc_ext_i [0]    ; 
assign sreg_dt[8]  = tproc_ext_i [1]    ;
assign sreg_dt[9]  = reg_port_i [0]  ;
assign sreg_dt[10] = reg_port_i [1] ;
assign sreg_dt[11] = time_dt_i          ;
assign sreg_dt[12] = sreg_32_dt [0]     ; // CORE_W_DT1
assign sreg_dt[13] = sreg_32_dt [1]     ; // CORE_W_DT2
assign sreg_dt[14] = sreg_32_dt [2]     ; // OUT TIME
assign sreg_dt[15] = sreg_32_dt [3]     ; // PC_NXT_ADDR_REG


wire  [31:0] data_A   [2];

genvar ind_A;
generate
   for (ind_A=0; ind_A <2 ; ind_A=ind_A+1) begin
      assign data_A[ind_A]   = dreg_32_dt[ rs_A_addr_i[ind_A][REG_AW-1:0] ];
   end
endgenerate

reg [31:0] data_D [2] ;

genvar ind_D;
generate
   for (ind_D=0; ind_D <2 ; ind_D=ind_D+1) begin
      always_comb begin
         case (rs_D_addr_i[ind_D][6:5])
            2'b00 : data_D[ind_D] = dreg_32_dt[ rs_D_addr_i[ind_D][REG_AW-1:0] ];
            2'b01 : data_D[ind_D] = wreg_32_dt[ rs_D_addr_i[ind_D][2:0] ]; // 6 Registers
            2'b10 : data_D[ind_D] = sreg_dt   [ rs_D_addr_i[ind_D][3:0] ]; // 16 Registers
            2'b11 : data_D[ind_D] = 0 ;
         endcase
      end
   end //for
endgenerate

// Value Just Written - Forwarding
always_ff @ (posedge clk_i, negedge rst_ni) 
   if (!rst_ni) 
      w_dt_o               <= 0;
   else if (~halt_i)
      w_dt_o               <= w_dt_i;


///////////////////////////////////////////////////////////////////////////////
// OUTPUT ASSIGNMENT
assign rs_A_dt_o     =  data_A ;
assign rs_D_dt_o     =  data_D ;

assign lfsr_o        =  lfsr_reg ;
assign reg_cfg_o     = sreg_cfg_dt ;
assign sreg_dt_o[0]  =  sreg_dt[12] ;
assign sreg_dt_o[1]  =  sreg_dt[13] ;
assign out_time_o    =  sreg_dt[14] ;
assign out_addr_o    =  sreg_dt[15] [PMEM_AW-1:0];

assign out_wreg_o[167:152] = wreg_32_dt[5][15:0];
assign out_wreg_o[151:120] = wreg_32_dt[4]  ;
assign out_wreg_o[119: 88] = wreg_32_dt[3]  ;
assign out_wreg_o[ 87: 64] = wreg_32_dt[2][23:0] ;
assign out_wreg_o[ 63: 32] = wreg_32_dt[1]  ;
assign out_wreg_o[ 31:  0] = wreg_32_dt[0]  ;
   
endmodule






module LFSR (
   input   wire             clk_i         ,
   input   wire             rst_ni        ,
   input   wire             en_i          ,
   input   wire             load_we_i     ,
   input   wire [31:0]      load_dt_i     ,
   output  wire [31:0]      lfsr_dt_o     );

// LFSR
///////////////////////////////////////////////////////////////////////////////

reg [31:0] reg_lfsr ;

always_ff @(posedge clk_i, negedge rst_ni)
   if (!rst_ni)
      reg_lfsr <= 0;//32'h00000000;
   else begin
      if (load_we_i)
         reg_lfsr <= load_dt_i ;
      else if (en_i) begin
         //reg_lfsr[0] <= ~^{reg_lfsr[31], reg_lfsr[21], reg_lfsr[1:0]};
         reg_lfsr[31:1] <= reg_lfsr[30:0];
         reg_lfsr[0] <= ~^{reg_lfsr[31], reg_lfsr[21], reg_lfsr[1:0]};
      end
   end
///////////////////////////////////////////////////////////////////////////////
//wire [15:0] lfsr_up, lfsr_dn ;
// Mixe a little bit the optpus
//assign lfsr_up = {<<{reg_lfsr[31 : 16]}} ;
//assign lfsr_dn =     reg_lfsr[15 : 0 ] ;
//assign lfsr_dt_o = {<<2{lfsr_dn, lfsr_up}} ;
assign lfsr_dt_o = reg_lfsr ;

endmodule
