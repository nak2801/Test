/*
    Combined QSPI XIP + Program Controller with Unified AHB Interface
    
    This is the TOP-LEVEL module for standalone QSPI controller
    
    Features:
    - XIP Read via AHB-Lite (memory-mapped flash access)
    - Program/Erase operations via control registers
    - Direct-mapped cache for XIP
    - Unified AHB interface with address decode
    
    Memory Map (default REG_BASE = 0x010000):
    - 0x00000000 - 0x00FFFFFF: Flash XIP Read (24-bit address)
    - REG_BASE + 0x00: CTRL register
    - REG_BASE + 0x04: STATUS register
    - REG_BASE + 0x08: ADDR register
    - REG_BASE + 0x0C: LEN register
    - REG_BASE + 0x10: DATA register
    - REG_BASE + 0x14: ID register
    
    Author: Nguyen Kiet
    Date: 2026-01-23
*/

`timescale 1ns/1ps

module qspi_xip_prog_ctrl #(
    parameter NUM_LINES     = 32,
    parameter LINE_SIZE     = 16,
    parameter RESET_CYCLES  = 999,
    parameter FIFO_DEPTH    = 256,
    parameter REG_BASE      = 24'h010000  // Register base address
)(
    // Clock and Reset
    input  wire         HCLK,
    input  wire         HRESETn,
    
    // AHB-Lite Slave Interface (Unified: XIP + Control Registers)
    input  wire         HSEL,
    input  wire [31:0]  HADDR,
    input  wire [1:0]   HTRANS,
    input  wire         HWRITE,
    input  wire [31:0]  HWDATA,
    input  wire         HREADY,
    output wire         HREADYOUT,
    output wire [31:0]  HRDATA,
    output wire         HRESP,
    
    // Interrupt
    output wire         irq_done,
    output wire         irq_error,
    
    // External Interface to Quad I/O Flash
    output wire         sck,
    output wire         ce_n,
    input  wire [3:0]   din,
    output wire [3:0]   dout,
    output wire [3:0]   douten
);

    // Address decode
    wire is_reg_access = (HADDR[23:8] == REG_BASE[23:8]);  // Register access
    wire is_xip_access = ~is_reg_access;                   // XIP access

    // Internal signals
    wire        prog_start;
    wire [2:0]  prog_op;
    wire [23:0] prog_addr;
    wire [7:0]  prog_data_to_flash;
    wire        prog_data_valid;
    wire        prog_data_req;
    wire [7:0]  prog_data_from_flash;
    wire        prog_data_from_flash_valid;
    wire [8:0]  prog_byte_count;
    wire        prog_busy;
    wire        prog_done;
    wire        prog_error;

    // XIP Controller signals
    wire        xip_hreadyout;
    wire [31:0] xip_hrdata;

    // Register Controller signals
    wire        reg_hreadyout;
    wire [31:0] reg_hrdata;
    wire        reg_hresp;

    // Mux outputs based on address
    assign HREADYOUT = is_reg_access ? reg_hreadyout : xip_hreadyout;
    assign HRDATA    = is_reg_access ? reg_hrdata    : xip_hrdata;
    assign HRESP     = is_reg_access ? reg_hresp     : 1'b0;

    // XIP Controller
    qspi_xip_prog_ctrl_ahbl #(
        .NUM_LINES(NUM_LINES),
        .LINE_SIZE(LINE_SIZE),
        .RESET_CYCLES(RESET_CYCLES)
    ) xip_ctrl (
        .HCLK(HCLK),
        .HRESETn(HRESETn),
        .HSEL(HSEL & is_xip_access),
        .HADDR(HADDR),
        .HTRANS(HTRANS),
        .HWRITE(HWRITE),
        .HWDATA(HWDATA),
        .HREADY(HREADY),
        .HREADYOUT(xip_hreadyout),
        .HRDATA(xip_hrdata),
        
        .prog_start(prog_start),
        .prog_op(prog_op),
        .prog_addr(prog_addr),
        .prog_data_in(prog_data_to_flash),
        .prog_data_valid(prog_data_valid),
        .prog_data_req(prog_data_req),
        .prog_data_out(prog_data_from_flash),
        .prog_data_out_valid(prog_data_from_flash_valid),
        .prog_byte_count(prog_byte_count),
        .prog_busy(prog_busy),
        .prog_done(prog_done),
        .prog_error(prog_error),
        
        .sck(sck),
        .ce_n(ce_n),
        .din(din),
        .dout(dout),
        .douten(douten)
    );

    // Register Controller (AHB)
    qspi_prog_ahb #(
        .FIFO_DEPTH(FIFO_DEPTH)
    ) reg_ctrl (
        .HCLK(HCLK),
        .HRESETn(HRESETn),
        .HSEL(HSEL & is_reg_access),
        .HADDR(HADDR),
        .HTRANS(HTRANS),
        .HWRITE(HWRITE),
        .HWDATA(HWDATA),
        .HREADY(HREADY),
        .HRDATA(reg_hrdata),
        .HREADYOUT(reg_hreadyout),
        .HRESP(reg_hresp),
        
        .prog_start(prog_start),
        .prog_op(prog_op),
        .prog_addr(prog_addr),
        .prog_data_out(prog_data_to_flash),
        .prog_data_valid(prog_data_valid),
        .prog_data_req(prog_data_req),
        .prog_data_in(prog_data_from_flash),
        .prog_data_in_valid(prog_data_from_flash_valid),
        .prog_byte_count(prog_byte_count),
        .prog_busy(prog_busy),
        .prog_done(prog_done),
        .prog_error(prog_error)
    );

    // Interrupts
    assign irq_done  = prog_done;
    assign irq_error = prog_error;

endmodule
