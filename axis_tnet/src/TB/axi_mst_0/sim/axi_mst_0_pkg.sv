///////////////////////////////////////////////////////////////////////////
//NOTE: This file has been automatically generated by Vivado.
///////////////////////////////////////////////////////////////////////////
`timescale 1ps/1ps
package axi_mst_0_pkg;
import axi_vip_pkg::*;
///////////////////////////////////////////////////////////////////////////
// These parameters are named after the component for use in your verification 
// environment.
///////////////////////////////////////////////////////////////////////////
      parameter axi_mst_0_VIP_PROTOCOL           = 2;
      parameter axi_mst_0_VIP_READ_WRITE_MODE    = "READ_WRITE";
      parameter axi_mst_0_VIP_INTERFACE_MODE     = 0;
      parameter axi_mst_0_VIP_ADDR_WIDTH         = 32;
      parameter axi_mst_0_VIP_DATA_WIDTH         = 32;
      parameter axi_mst_0_VIP_ID_WIDTH           = 0;
      parameter axi_mst_0_VIP_AWUSER_WIDTH       = 0;
      parameter axi_mst_0_VIP_ARUSER_WIDTH       = 0;
      parameter axi_mst_0_VIP_RUSER_WIDTH        = 0;
      parameter axi_mst_0_VIP_WUSER_WIDTH        = 0;
      parameter axi_mst_0_VIP_BUSER_WIDTH        = 0;
      parameter axi_mst_0_VIP_SUPPORTS_NARROW    = 0;
      parameter axi_mst_0_VIP_HAS_BURST          = 0;
      parameter axi_mst_0_VIP_HAS_LOCK           = 0;
      parameter axi_mst_0_VIP_HAS_CACHE          = 0;
      parameter axi_mst_0_VIP_HAS_REGION         = 0;
      parameter axi_mst_0_VIP_HAS_QOS            = 0;
      parameter axi_mst_0_VIP_HAS_PROT           = 1;
      parameter axi_mst_0_VIP_HAS_WSTRB          = 1;
      parameter axi_mst_0_VIP_HAS_BRESP          = 1;
      parameter axi_mst_0_VIP_HAS_RRESP          = 1;
      parameter axi_mst_0_VIP_HAS_ACLKEN         = 0;
      parameter axi_mst_0_VIP_HAS_ARESETN        = 1;
///////////////////////////////////////////////////////////////////////////
typedef axi_mst_agent #(axi_mst_0_VIP_PROTOCOL, 
                        axi_mst_0_VIP_ADDR_WIDTH,
                        axi_mst_0_VIP_DATA_WIDTH,
                        axi_mst_0_VIP_DATA_WIDTH,
                        axi_mst_0_VIP_ID_WIDTH,
                        axi_mst_0_VIP_ID_WIDTH,
                        axi_mst_0_VIP_AWUSER_WIDTH, 
                        axi_mst_0_VIP_WUSER_WIDTH, 
                        axi_mst_0_VIP_BUSER_WIDTH, 
                        axi_mst_0_VIP_ARUSER_WIDTH,
                        axi_mst_0_VIP_RUSER_WIDTH, 
                        axi_mst_0_VIP_SUPPORTS_NARROW, 
                        axi_mst_0_VIP_HAS_BURST,
                        axi_mst_0_VIP_HAS_LOCK,
                        axi_mst_0_VIP_HAS_CACHE,
                        axi_mst_0_VIP_HAS_REGION,
                        axi_mst_0_VIP_HAS_PROT,
                        axi_mst_0_VIP_HAS_QOS,
                        axi_mst_0_VIP_HAS_WSTRB,
                        axi_mst_0_VIP_HAS_BRESP,
                        axi_mst_0_VIP_HAS_RRESP,
                        axi_mst_0_VIP_HAS_ARESETN) axi_mst_0_mst_t;
      
///////////////////////////////////////////////////////////////////////////
// How to start the verification component
///////////////////////////////////////////////////////////////////////////
//      axi_mst_0_mst_t  axi_mst_0_mst;
//      initial begin : START_axi_mst_0_MASTER
//        axi_mst_0_mst = new("axi_mst_0_mst", `axi_mst_0_PATH_TO_INTERFACE);
//        axi_mst_0_mst.start_master();
//      end



endpackage : axi_mst_0_pkg