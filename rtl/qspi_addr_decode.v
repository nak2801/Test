//------------------------------------------------------------------------------
// QSPI Address Decoder
// 
// Simple address decoder for QSPI controller
// Used for separating XIP access and Register access
//
// Default Memory Map:
// - QSPI XIP:       0x1000_0000 - 0x10FF_FFFF (16MB)
// - QSPI Registers: 0x4002_0000 - 0x4002_0FFF (4KB)
//------------------------------------------------------------------------------

`timescale 1ns/1ps

module qspi_addr_decode #(
    parameter XIP_BASE_ADDR  = 32'h1000_0000,
    parameter XIP_SIZE_MASK  = 32'h00FF_FFFF,  // 16MB
    parameter REG_BASE_ADDR  = 32'h4002_0000,
    parameter REG_SIZE_MASK  = 32'h0000_0FFF   // 4KB
)(
    input  wire [31:0]  haddr,
    
    // QSPI Selection signals
    output wire         qspi_xip_hsel,    // XIP read access
    output wire         qspi_reg_hsel     // Register access
);

    // =========================================================================
    // QSPI XIP Decode
    // Address: 0x1000_0000 - 0x10FF_FFFF (16MB)
    // Check upper 8 bits = 0x10
    // =========================================================================
    assign qspi_xip_hsel = (haddr[31:24] == XIP_BASE_ADDR[31:24]);

    // =========================================================================
    // QSPI Register Decode  
    // Address: 0x4002_0000 - 0x4002_0FFF (4KB)
    // Check upper 20 bits = 0x4002_0
    // =========================================================================
    assign qspi_reg_hsel = (haddr[31:12] == REG_BASE_ADDR[31:12]);

endmodule
