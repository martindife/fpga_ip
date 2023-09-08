///////////////////////////////////////////////////////////////////////////////
//  FERMI RESEARCH LAB
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// GENERAL
`ifndef NET_DEFINES
   `define NET_DEFINES
   // Command Decoding
   parameter _nop         = 5'b00000;
   parameter _get_net     = 5'b00001;
   parameter _set_net     = 5'b00010;
   parameter _sync_net    = 5'b01000;
   parameter _get_off     = 5'b00101;
   parameter _updt_off    = 5'b01001;
   parameter _set_dt      = 5'b01010;
   parameter _get_dt      = 5'b01011;
   parameter _rst_tproc   = 5'b10000;
   parameter _start_core  = 5'b10001;
   parameter _stop_core   = 5'b10010;
   parameter _set_cond    = 5'b10011;
   parameter _clear_cond  = 5'b10100;
   parameter _custom      = 5'b11111;

typedef enum {NOT_READY, IDLE, ST_ERROR, NET_CMD_RT, 
      LOC_GNET       , NET_GNET_P       , NET_GNET_R      ,
      LOC_SNET       , NET_SNET_P       , NET_SNET_R      ,
      LOC_SYNC1      , NET_SYNC1_P      , NET_SYNC1_R      ,
      LOC_SYNC2      , NET_SYNC2_P      , NET_SYNC2_R      ,
      LOC_SYNC3      , NET_SYNC3_P      , NET_SYNC3_R      ,
      LOC_SYNC4      , NET_SYNC4_P      , NET_SYNC4_R      ,
      LOC_GET_OFF    , NET_GET_OFF_P    , NET_GET_OFF_A    ,
      LOC_UPDT_OFF   , NET_UPDT_OFF_P   , NET_UPDT_OFF_R   ,
      LOC_SET_DT     , NET_SET_DT_P     , NET_SET_DT_R     ,
      LOC_GET_DT     , NET_GET_DT_P     , NET_GET_DT_R     , NET_GET_DT_A      ,
      LOC_RST_PROC   , NET_RST_PROC_P   , NET_RST_PROC_R   , 
      LOC_START_CORE , NET_START_CORE_P , NET_START_CORE_R ,
      LOC_STOP_CORE  , NET_STOP_CORE_P  , NET_STOP_CORE_R  ,
      WAIT_TX_ACK, WAIT_TX_nACK, WAIT_CMD_nACK
      } TYPE_QNET_CMD ;
      
   typedef struct packed {
      bit    RTD    ;
      bit    OFF    ;
      bit    NN     ;
      bit    ID     ;
      bit    DT    ;
      } TYPE_PARAM_WE ;

   typedef struct packed {
      bit [31:0]   T_NCR  ;
      bit [31:0]   T_LTR  ;
      bit [31:0]   T_LCS  ;
      bit [31:0]   T_LCC  ;
      bit [31:0]   RTD    ;
      bit [31:0]   OFF    ;
      bit [ 9:0]   NN     ;
      bit [ 9:0]   ID     ;
      } TYPE_QPARAM ;
      
`endif

