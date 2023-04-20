module qproc_time_ctrl ( 
   input  wire            t_clk_i         ,
   input  wire            t_rst_ni        ,
   input  wire            c_time_rst_i    , // Set Time to 0
   input  wire            c_time_init_i   , // Set Time to current OFFSET
   input  wire            c_time_en_i     , // Time RUNS
   input  wire            c_time_updt_i   , // Increment Time c_offset_dt_i
   input  wire  [31:0]    c_offset_dt_i   , 
   output wire            t_time_en_o     , // Used to Enable the FIFO
   output wire  [47:0]    t_time_abs_o    , 
   output wire  [47:0]    t_init_off_o    );

// Timing Control
///////////////////////////////////////////////////////////////////////////////

reg   time_rst_rcd, t_time_rst_r;
reg   time_init_rcd, t_time_init_r;
reg   time_en_rcd, t_time_en_r ;
reg   time_updt_rcd, t_time_updt_r ;
reg   t_time_updt_2r ;
reg   offset_updt_r ;
wire  t_time_updt_t01;
reg   time_cnt_en, offset_updt;

// CROSS DOMAIN SIGNALS
always_ff @(posedge t_clk_i) begin
   if (!t_rst_ni) begin
      time_rst_rcd     <= 0;
      time_init_rcd    <= 0;
      time_en_rcd      <= 0;
      time_updt_rcd    <= 0;
      t_time_rst_r     <= 1;
      t_time_init_r    <= 0;
      t_time_en_r      <= 0;
      t_time_updt_r    <= 0;
   end else begin
      time_rst_rcd     <= c_time_rst_i;
      time_init_rcd    <= c_time_init_i;
      time_en_rcd      <= c_time_en_i;
      time_updt_rcd    <= c_time_updt_i;
      t_time_rst_r     <= time_rst_rcd;
      t_time_init_r    <= time_init_rcd;
      t_time_en_r      <= time_en_rcd;
      t_time_updt_r    <= time_updt_rcd;
      t_time_updt_2r   <= t_time_updt_r;
      offset_updt_r    <= offset_updt;
   end
end

assign t_time_updt_t01  = t_time_updt_r & ~t_time_updt_2r;

// Time ABS
///////////////////////////////////////////////////////////////////////////////
reg [47:0] time_abs;

enum {ST_IDLE, ST_RESET, ST_INIT, ST_LOAD_OFFSET, ST_INCREMENT, ST_UPDATE } ctrl_time_st, ctrl_time_st_nxt;

////////// Sequential Logic
always @ (posedge t_clk_i or negedge t_rst_ni) begin : CTRL_SYNC_PROC
   if (!t_rst_ni)    ctrl_time_st <=  ST_IDLE;
   else              ctrl_time_st <=  ctrl_time_st_nxt;
end

////////// Comb Logic - Outputs and State
reg  [47:0] time_inc;
reg  [47:0] initial_offset;
wire [47:0] updated_offset ;
reg time_c_in;

wire time_cnt_rst;
assign time_cnt_rst = t_time_init_r | t_time_rst_r ;


always_comb begin : CTRL_ST_AND_OUTPUT_DECODE
   //time_cnt_rst  = 1'b0;
   time_cnt_en   = 1'b0;
   time_inc      = 0 ;
   time_c_in    = 1'b0;
   offset_updt  = 1'b0;
   ctrl_time_st_nxt  = ctrl_time_st; // Default Current State
   case (ctrl_time_st)
      ST_IDLE : begin
         if      ( t_time_rst_r   )  ctrl_time_st_nxt = ST_RESET;
         else if ( t_time_init_r  )  ctrl_time_st_nxt = ST_INIT;
         else if ( t_time_en_r    )  ctrl_time_st_nxt = ST_INCREMENT;
         else if ( t_time_updt_t01)  ctrl_time_st_nxt = ST_UPDATE;
      end
      ST_RESET : begin
         // time_cnt_en   = 1'b1;
         if ( !t_time_rst_r )  ctrl_time_st_nxt = ST_IDLE;
      end
 
      
      ST_INIT : begin
         time_cnt_en   = 1'b1;
         if ( !t_time_init_r )  ctrl_time_st_nxt  = ST_LOAD_OFFSET;
      end
      ST_LOAD_OFFSET : begin
         time_cnt_en      = 1'b1;
         time_inc         = initial_offset ;
         ctrl_time_st_nxt = ST_INCREMENT;
      end
      ST_INCREMENT : begin
         time_cnt_en  = 1'b1;
         time_inc     = 48'd1 ;
         if       ( t_time_rst_r    )  ctrl_time_st_nxt = ST_RESET;
         else if  ( t_time_init_r   )  ctrl_time_st_nxt = ST_INIT;
         else if  ( t_time_updt_t01 )  ctrl_time_st_nxt = ST_UPDATE;
         else if  ( !t_time_en_r    )  ctrl_time_st_nxt = ST_IDLE;
      end
      ST_UPDATE : begin
         offset_updt  = 1'b1;
         time_cnt_en  = 1'b1;
         time_inc     = c_offset_dt_i ;
         time_c_in    = 1'b1;
         ctrl_time_st_nxt = ST_INCREMENT;
      end
   endcase 
end

// Initial OFFSET
///////////////////////////////////////////////////////////////////////////////
always_ff @ (posedge t_clk_i, negedge t_rst_ni) begin
   if (!t_rst_ni)          initial_offset       <= '{default:'0} ;

   else if (t_time_init_r) initial_offset       <= {16'd0,c_offset_dt_i};
   else if (offset_updt_r) initial_offset       <= updated_offset;
end

// Time Operation
   ADDSUB_MACRO #(
         .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
         .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
         .WIDTH      ( 48  )             // Input / output bus width, 1-48
      ) TIME_ADDER (
         .CARRYOUT   (                   ), // 1-bit carry-out output signal
         .RESULT     ( time_abs          ), // Add/sub result output, width defined by WIDTH parameter
         .B          ( time_abs          ), // Input A bus, width defined by WIDTH parameter
         .ADD_SUB    ( 1'b1              ), // 1-bit add/sub input, high selects add, low selects subtract
         .A          ( time_inc          ), // Input B bus, width defined by WIDTH parameter
         .CARRYIN    ( time_c_in         ), // 1-bit carry-in input
         .CE         ( time_cnt_en | time_cnt_rst        ), // 1-bit clock enable input
         .CLK        ( t_clk_i           ), // 1-bit clock input
         .RST        ( time_cnt_rst      )  // 1-bit active high synchronous reset
      );
// Offset Operation
   ADDSUB_MACRO #(
         .DEVICE     ("7SERIES"),        // Target Device: "7SERIES" 
         .LATENCY    ( 1   ),            // Desired clock cycle latency, 0-2
         .WIDTH      ( 48  )             // Input / output bus width, 1-48
      ) OFFSET_ADDER (
         .CARRYOUT   (                    ), // 1-bit carry-out output signal
         .RESULT     ( updated_offset     ), // Add/sub result output, width defined by WIDTH parameter
         .B          ( initial_offset     ), // Input A bus, width defined by WIDTH parameter
         .ADD_SUB    ( 1'b1               ), // 1-bit add/sub input, high selects add, low selects subtract
         .A          ( {16'd0,c_offset_dt_i}         ), // Input B bus, width defined by WIDTH parameter
         .CARRYIN    ( 1'b0               ), // 1-bit carry-in input
         .CE         ( offset_updt |t_time_updt_t01       ), // 1-bit clock enable input
         .CLK        ( t_clk_i            ), // 1-bit clock input
         .RST        ( ~t_rst_ni          )  // 1-bit active high synchronous reset
      );
      
assign t_time_en_o  = t_time_en_r ;
assign t_time_abs_o = time_abs;
assign t_init_off_o = initial_offset;

endmodule

