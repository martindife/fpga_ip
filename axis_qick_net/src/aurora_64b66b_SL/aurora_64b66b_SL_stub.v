// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2022.1 (lin64) Build 3526262 Mon Apr 18 15:47:01 MDT 2022
// Date        : Wed May 17 12:10:19 2023
// Host        : teddy01.dhcp.fnal.gov running 64-bit Scientific Linux release 7.9 (Nitrogen)
// Command     : write_verilog -force -mode synth_stub -rename_top aurora_64b66b_SL -prefix
//               aurora_64b66b_SL_ aurora_64b66b_SL_stub.v
// Design      : aurora_64b66b_SL
// Purpose     : Stub declaration of top-level module interface
// Device      : xczu49dr-ffvf1760-2-e
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "aurora_64b66b_v12_0_9, Coregen v14.3_ip3, Number of lanes = 1, Line rate is double1.25Gbps, Reference Clock is double156.25MHz, Interface is Framing, Flow Control is None and is operating in DUPLEX configuration" *)
module aurora_64b66b_SL(m_axi_rx_tdata, m_axi_rx_tlast, 
  m_axi_rx_tkeep, m_axi_rx_tvalid, rxp, rxn, gt_refclk1_p, gt_refclk1_n, gt_refclk1_out, 
  rx_hard_err, rx_soft_err, rx_channel_up, rx_lane_up, user_clk_out, mmcm_not_locked_out, 
  reset2fc, reset_pb, gt_rxcdrovrden_in, power_down, pma_init, gt_pll_lock, gt0_drpaddr, 
  gt0_drpdi, gt0_drpdo, gt0_drprdy, gt0_drpen, gt0_drpwe, init_clk, link_reset_out, gt_powergood, 
  sys_reset_out, gt_reset_out, tx_out_clk)
/* synthesis syn_black_box black_box_pad_pin="m_axi_rx_tdata[0:63],m_axi_rx_tlast,m_axi_rx_tkeep[0:7],m_axi_rx_tvalid,rxp[0:0],rxn[0:0],gt_refclk1_p,gt_refclk1_n,gt_refclk1_out,rx_hard_err,rx_soft_err,rx_channel_up,rx_lane_up[0:0],user_clk_out,mmcm_not_locked_out,reset2fc,reset_pb,gt_rxcdrovrden_in,power_down,pma_init,gt_pll_lock,gt0_drpaddr[9:0],gt0_drpdi[15:0],gt0_drpdo[15:0],gt0_drprdy,gt0_drpen,gt0_drpwe,init_clk,link_reset_out,gt_powergood[0:0],sys_reset_out,gt_reset_out,tx_out_clk" */;
  output [0:63]m_axi_rx_tdata;
  output m_axi_rx_tlast;
  output [0:7]m_axi_rx_tkeep;
  output m_axi_rx_tvalid;
  input [0:0]rxp;
  input [0:0]rxn;
  input gt_refclk1_p;
  input gt_refclk1_n;
  output gt_refclk1_out;
  output rx_hard_err;
  output rx_soft_err;
  output rx_channel_up;
  output [0:0]rx_lane_up;
  output user_clk_out;
  output mmcm_not_locked_out;
  output reset2fc;
  input reset_pb;
  input gt_rxcdrovrden_in;
  input power_down;
  input pma_init;
  output gt_pll_lock;
  input [9:0]gt0_drpaddr;
  input [15:0]gt0_drpdi;
  output [15:0]gt0_drpdo;
  output gt0_drprdy;
  input gt0_drpen;
  input gt0_drpwe;
  input init_clk;
  output link_reset_out;
  output [0:0]gt_powergood;
  output sys_reset_out;
  output gt_reset_out;
  output tx_out_clk;
endmodule
