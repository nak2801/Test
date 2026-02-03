/*
    AHB-Lite Interface for QSPI Flash Programmer Control
    
    Memory Map:
    +------------+--------+------------------------------------------+
    | Address    | Name   | Description                              |
    +------------+--------+------------------------------------------+
    | 0x00       | CTRL   | Control Register                         |
    |            |        | [2:0]  OP_TYPE - Operation type          |
    |            |        |        000: Write Enable                 |
    |            |        |        001: Write Disable                |
    |            |        |        010: Read Status                  |
    |            |        |        011: Page Program                 |
    |            |        |        100: Sector Erase (4KB)           |
    |            |        |        101: Block Erase (32KB)           |
    |            |        |        110: Block Erase (64KB)           |
    |            |        |        111: Read JEDEC ID                |
    |            |        | [7]    START - Write 1 to start          |
    +------------+--------+------------------------------------------+
    | 0x04       | STATUS | Status Register (Read Only)              |
    |            |        | [0]    BUSY                              |
    |            |        | [1]    DONE (cleared on read)            |
    |            |        | [2]    ERROR                             |
    |            |        | [15:8] FLASH_STATUS (after Read Status)  |
    +------------+--------+------------------------------------------+
    | 0x08       | ADDR   | Flash Address [23:0]                     |
    +------------+--------+------------------------------------------+
    | 0x0C       | LEN    | Byte Count [8:0] for Page Program        |
    +------------+--------+------------------------------------------+
    | 0x10       | DATA   | Data FIFO (Write for program, Read for   |
    |            |        | status/ID)                               |
    +------------+--------+------------------------------------------+
    | 0x14       | ID     | JEDEC ID [23:0] (Read Only)              |
    +------------+--------+------------------------------------------+
    
    Author: Nguyen Kiet
    Date: 2026-01-23
*/

`timescale 1ns/1ps

module qspi_prog_ahb #(
    parameter FIFO_DEPTH = 256
)(
    // AHB-Lite Interface
    input  wire         HCLK,
    input  wire         HRESETn,
    input  wire         HSEL,
    input  wire [31:0]  HADDR,
    input  wire [1:0]   HTRANS,
    input  wire         HWRITE,
    input  wire [31:0]  HWDATA,
    input  wire         HREADY,
    output reg  [31:0]  HRDATA,
    output wire         HREADYOUT,
    output wire         HRESP,
    
    // Programmer Interface
    output reg          prog_start,
    output reg  [2:0]   prog_op,
    output reg  [23:0]  prog_addr,
    output reg  [7:0]   prog_data_out,
    output reg          prog_data_valid,
    input  wire         prog_data_req,
    input  wire [7:0]   prog_data_in,
    input  wire         prog_data_in_valid,
    output reg  [8:0]   prog_byte_count,
    input  wire         prog_busy,
    input  wire         prog_done,
    input  wire         prog_error
);

    // Register addresses (offset from base)
    localparam  REG_CTRL    = 5'h00,
                REG_STATUS  = 5'h04,
                REG_ADDR    = 5'h08,
                REG_LEN     = 5'h0C,
                REG_DATA    = 5'h10,
                REG_ID      = 5'h14;

    // AHB always ready, no errors
    assign HREADYOUT = 1'b1;
    assign HRESP     = 1'b0;

    // AHB Address Phase registers
    reg         last_HSEL;
    reg         last_HWRITE;
    reg [31:0]  last_HADDR;
    reg [1:0]   last_HTRANS;
    
    wire        ahb_access = last_HSEL & last_HTRANS[1];
    wire        ahb_write  = ahb_access & last_HWRITE;
    wire        ahb_read   = ahb_access & ~last_HWRITE;
    wire [4:0]  reg_addr   = last_HADDR[4:0];

    // Capture address phase
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            last_HSEL   <= 1'b0;
            last_HWRITE <= 1'b0;
            last_HADDR  <= 32'b0;
            last_HTRANS <= 2'b0;
        end
        else if (HREADY) begin
            last_HSEL   <= HSEL;
            last_HWRITE <= HWRITE;
            last_HADDR  <= HADDR;
            last_HTRANS <= HTRANS;
        end
    end

    // FIFO for write data
    reg [7:0]   fifo_mem [0:FIFO_DEPTH-1];
    reg [8:0]   fifo_wr_ptr;
    reg [8:0]   fifo_rd_ptr;
    wire [8:0]  fifo_count = fifo_wr_ptr - fifo_rd_ptr;
    wire        fifo_empty = (fifo_count == 0);
    wire        fifo_full  = (fifo_count >= FIFO_DEPTH);

    // Status registers
    reg         done_flag;
    reg         error_flag;
    reg [7:0]   flash_status;
    reg [23:0]  jedec_id;
    reg [1:0]   id_byte_cnt;

    // Register write
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            prog_start <= 1'b0;
            prog_op <= 3'b0;
            prog_addr <= 24'b0;
            prog_byte_count <= 9'b0;
            fifo_wr_ptr <= 0;
        end
        else begin
            prog_start <= 1'b0;  // Self-clearing
            
            if (ahb_write) begin
                case (reg_addr)
                    REG_CTRL: begin
                        prog_op <= HWDATA[2:0];
                        if (HWDATA[7] && !prog_busy) begin
                            prog_start <= 1'b1;
                            fifo_rd_ptr <= 0;
                        end
                    end
                    
                    REG_ADDR: begin
                        prog_addr <= HWDATA[23:0];
                    end
                    
                    REG_LEN: begin
                        prog_byte_count <= HWDATA[8:0];
                    end
                    
                    REG_DATA: begin
                        if (!fifo_full) begin
                            fifo_mem[fifo_wr_ptr[7:0]] <= HWDATA[7:0];
                            fifo_wr_ptr <= fifo_wr_ptr + 1;
                        end
                    end
                endcase
            end
        end
    end

    // FIFO read for programmer
    reg data_req_d;  // Delayed data_req for edge detection
    
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            fifo_rd_ptr <= 0;
            prog_data_out <= 8'h00;
            prog_data_valid <= 1'b0;
            data_req_d <= 1'b0;
        end
        else begin
            prog_data_valid <= 1'b0;
            data_req_d <= prog_data_req;
            
            // Reset read pointer when new operation starts
            if (prog_start) begin
                fifo_rd_ptr <= 0;
            end
            // Read FIFO on rising edge of data_req only
            else if (prog_data_req && !data_req_d && !fifo_empty) begin
                prog_data_out <= fifo_mem[fifo_rd_ptr[7:0]];
                prog_data_valid <= 1'b1;
                fifo_rd_ptr <= fifo_rd_ptr + 1;
            end
        end
    end

    // Status capture
    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            done_flag <= 1'b0;
            error_flag <= 1'b0;
            flash_status <= 8'h00;
            jedec_id <= 24'h000000;
            id_byte_cnt <= 0;
        end
        else begin
            // Capture done
            if (prog_done)
                done_flag <= 1'b1;
            
            // Capture error
            if (prog_error)
                error_flag <= 1'b1;
            
            // Capture data from programmer (status or ID)
            if (prog_data_in_valid) begin
                if (prog_op == 3'd2) begin  // Read Status
                    flash_status <= prog_data_in;
                end
                else if (prog_op == 3'd7) begin  // Read ID
                    case (id_byte_cnt)
                        0: jedec_id[23:16] <= prog_data_in;
                        1: jedec_id[15:8]  <= prog_data_in;
                        2: jedec_id[7:0]   <= prog_data_in;
                    endcase
                    id_byte_cnt <= id_byte_cnt + 1;
                end
            end
            
            // Reset ID counter on new operation
            if (prog_start && prog_op == 3'd7)
                id_byte_cnt <= 0;
            
            // Clear done/error flags on status read
            if (ahb_read && reg_addr == REG_STATUS) begin
                done_flag <= 1'b0;
                error_flag <= 1'b0;
            end
        end
    end

    // Register read
    // Note: OR done_flag with prog_done to avoid 1-cycle delay issue
    wire done_status = done_flag | prog_done;
    wire error_status = error_flag | prog_error;
    
    always @* begin
        HRDATA = 32'h00000000;
        if (ahb_read) begin
            case (reg_addr)
                REG_CTRL:   HRDATA = {24'b0, 4'b0, 1'b0, prog_op};
                REG_STATUS: HRDATA = {16'b0, flash_status, 5'b0, error_status, done_status, prog_busy};
                REG_ADDR:   HRDATA = {8'b0, prog_addr};
                REG_LEN:    HRDATA = {23'b0, prog_byte_count};
                REG_DATA:   HRDATA = {24'b0, prog_data_in};
                REG_ID:     HRDATA = {8'b0, jedec_id};
                default:    HRDATA = 32'h00000000;
            endcase
        end
    end

endmodule
