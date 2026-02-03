/*
    QSPI Flash Controller with XIP Read and Program capabilities
    Simplified version - Fixed timing issues
    
    Author: Nguyen Kiet
    Date: 2026-01-23
*/

`timescale 1ns/1ps

// =============================================================================
// QSPI Flash Commands
// =============================================================================
`define CMD_READ_DATA       8'h03   // Read Data (SPI)
`define CMD_FAST_READ       8'h0B   // Fast Read (SPI)
`define CMD_FAST_READ_QUAD  8'hEB   // Fast Read Quad I/O
`define CMD_PAGE_PROG       8'h02   // Page Program (SPI)
`define CMD_WRITE_ENABLE    8'h06   // Write Enable
`define CMD_WRITE_DISABLE   8'h04   // Write Disable
`define CMD_READ_STATUS1    8'h05   // Read Status Register 1
`define CMD_SECTOR_ERASE    8'h20   // Sector Erase (4KB)
`define CMD_BLOCK_ERASE_32  8'h52   // Block Erase (32KB)
`define CMD_BLOCK_ERASE_64  8'hD8   // Block Erase (64KB)
`define CMD_READ_ID         8'h9F   // Read JEDEC ID

// =============================================================================
// Simple SPI Master for Flash Programming
// =============================================================================
module spi_master (
    input  wire         clk,
    input  wire         rst_n,
    
    // Control
    input  wire         start,
    input  wire [7:0]   tx_data,
    output reg  [7:0]   rx_data,
    output reg          busy,
    output reg          done,
    
    // SPI signals
    output reg          sck,
    output reg          mosi,
    input  wire         miso
);

    // SPI Mode 0: CPOL=0, CPHA=0
    // - Clock idles low
    // - Data sampled on rising edge
    // - MOSI must be stable BEFORE rising edge
    
    reg [3:0] bit_cnt;
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    
    localparam IDLE    = 3'd0;
    localparam SETUP   = 3'd1;  // MOSI setup time before rising edge
    localparam RISE    = 3'd2;  // Rising edge - sample MISO
    localparam HOLD    = 3'd3;  // Hold time
    localparam FALL    = 3'd4;  // Falling edge - shift out next bit
    
    reg [2:0] state;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_cnt <= 0;
            sck <= 0;
            mosi <= 1;
            busy <= 0;
            done <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            rx_data <= 0;
        end
        else begin
            done <= 0;
            
            case (state)
                IDLE: begin
                    sck <= 0;
                    busy <= 0;
                    if (start) begin
                        tx_shift <= tx_data;
                        mosi <= tx_data[7];  // MSB first - setup before first rising edge
                        bit_cnt <= 0;
                        busy <= 1;
                        state <= SETUP;
                    end
                end
                
                SETUP: begin
                    // MOSI already setup, wait one cycle for stability
                    sck <= 0;
                    state <= RISE;
                end
                
                RISE: begin
                    // Rising edge - sample MISO
                    sck <= 1;
                    rx_shift <= {rx_shift[6:0], miso};
                    state <= HOLD;
                end
                
                HOLD: begin
                    // Hold SCK high for one more cycle
                    sck <= 1;
                    state <= FALL;
                end
                
                FALL: begin
                    // Falling edge - shift to next bit
                    sck <= 0;
                    bit_cnt <= bit_cnt + 1;
                    
                    if (bit_cnt == 7) begin
                        // All 8 bits done
                        rx_data <= rx_shift;
                        done <= 1;
                        state <= IDLE;
                    end
                    else begin
                        // Shift to next bit
                        tx_shift <= {tx_shift[6:0], 1'b1};
                        mosi <= tx_shift[6];  // Next bit
                        state <= SETUP;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule

// =============================================================================
// QSPI Flash Programmer - Simple SPI-only version
// =============================================================================
module qspi_flash_programmer_simple (
    input  wire         clk,
    input  wire         rst_n,
    
    // Control interface
    input  wire         start,
    input  wire [2:0]   op_type,
    input  wire [23:0]  addr,
    input  wire [7:0]   data_in,
    input  wire         data_valid,
    output reg          data_req,
    output reg  [7:0]   data_out,
    output reg          data_out_valid,
    input  wire [8:0]   byte_count,
    output reg          busy,
    output reg          done,
    output reg          error,
    
    // SPI interface
    output wire         sck,
    output reg          ce_n,
    input  wire [3:0]   din,
    output wire [3:0]   dout,
    output wire [3:0]   douten  // 4-bit: only io[0] is driven in SPI mode
);

    // Operation types
    localparam  OP_WRITE_ENABLE     = 3'd0,
                OP_WRITE_DISABLE    = 3'd1,
                OP_READ_STATUS      = 3'd2,
                OP_PAGE_PROGRAM     = 3'd3,
                OP_SECTOR_ERASE     = 3'd4,
                OP_BLOCK_ERASE_32   = 3'd5,
                OP_BLOCK_ERASE_64   = 3'd6,
                OP_READ_ID          = 3'd7;

    // State machine
    localparam  ST_IDLE         = 4'd0,
                ST_WEN_START    = 4'd1,
                ST_WEN_WAIT     = 4'd2,
                ST_CS_GAP1      = 4'd3,
                ST_CMD_START    = 4'd4,
                ST_CMD_WAIT     = 4'd5,
                ST_ADDR_START   = 4'd6,
                ST_ADDR_WAIT    = 4'd7,
                ST_DATA_START   = 4'd8,
                ST_DATA_WAIT    = 4'd9,
                ST_CS_GAP2      = 4'd10,
                ST_POLL_CMD     = 4'd11,
                ST_POLL_WAIT_CMD= 4'd12,
                ST_POLL_READ    = 4'd13,
                ST_DONE         = 4'd14;

    reg [3:0]   state;
    reg [2:0]   op_reg;
    reg [23:0]  addr_reg;
    reg [8:0]   data_cnt;
    reg [1:0]   addr_byte;
    reg [7:0]   gap_cnt;
    reg [15:0]  poll_timeout;
    
    // SPI Master signals
    reg         spi_start;
    reg  [7:0]  spi_tx;
    wire [7:0]  spi_rx;
    wire        spi_busy;
    wire        spi_done;
    wire        spi_sck;
    wire        spi_mosi;
    wire        spi_miso;
    
    assign spi_miso = din[1];
    assign sck = spi_sck & ~ce_n;
    assign dout = {3'b111, spi_mosi};
    // In SPI mode: only enable io[0] (MOSI), io[1] (MISO) is input
    assign douten = {3'b000, ~ce_n};  // 4-bit: Only drive io[0]
    
    // SPI Master instance
    spi_master spi (
        .clk(clk),
        .rst_n(rst_n),
        .start(spi_start),
        .tx_data(spi_tx),
        .rx_data(spi_rx),
        .busy(spi_busy),
        .done(spi_done),
        .sck(spi_sck),
        .mosi(spi_mosi),
        .miso(spi_miso)
    );

    // Command lookup
    function [7:0] get_cmd;
        input [2:0] op;
        begin
            case (op)
                OP_WRITE_ENABLE:    get_cmd = `CMD_WRITE_ENABLE;
                OP_WRITE_DISABLE:   get_cmd = `CMD_WRITE_DISABLE;
                OP_READ_STATUS:     get_cmd = `CMD_READ_STATUS1;
                OP_PAGE_PROGRAM:    get_cmd = `CMD_PAGE_PROG;
                OP_SECTOR_ERASE:    get_cmd = `CMD_SECTOR_ERASE;
                OP_BLOCK_ERASE_32:  get_cmd = `CMD_BLOCK_ERASE_32;
                OP_BLOCK_ERASE_64:  get_cmd = `CMD_BLOCK_ERASE_64;
                OP_READ_ID:         get_cmd = `CMD_READ_ID;
                default:            get_cmd = 8'h00;
            endcase
        end
    endfunction
    
    // Needs write enable?
    function needs_wen;
        input [2:0] op;
        begin
            needs_wen = (op == OP_PAGE_PROGRAM) || 
                        (op == OP_SECTOR_ERASE) ||
                        (op == OP_BLOCK_ERASE_32) ||
                        (op == OP_BLOCK_ERASE_64);
        end
    endfunction
    
    // Needs address?
    function needs_addr;
        input [2:0] op;
        begin
            needs_addr = (op == OP_PAGE_PROGRAM) || 
                         (op == OP_SECTOR_ERASE) ||
                         (op == OP_BLOCK_ERASE_32) ||
                         (op == OP_BLOCK_ERASE_64);
        end
    endfunction
    
    // Needs data?
    function needs_data;
        input [2:0] op;
        begin
            needs_data = (op == OP_PAGE_PROGRAM) ||
                         (op == OP_READ_STATUS) ||
                         (op == OP_READ_ID);
        end
    endfunction
    
    // Needs polling?
    function needs_poll;
        input [2:0] op;
        begin
            needs_poll = (op == OP_PAGE_PROGRAM) || 
                         (op == OP_SECTOR_ERASE) ||
                         (op == OP_BLOCK_ERASE_32) ||
                         (op == OP_BLOCK_ERASE_64);
        end
    endfunction

    // Main state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            ce_n <= 1;
            spi_start <= 0;
            spi_tx <= 0;
            busy <= 0;
            done <= 0;
            error <= 0;
            data_req <= 0;
            data_out <= 0;
            data_out_valid <= 0;
            op_reg <= 0;
            addr_reg <= 0;
            data_cnt <= 0;
            addr_byte <= 0;
            gap_cnt <= 0;
            poll_timeout <= 0;
        end
        else begin
            spi_start <= 0;
            done <= 0;
            // data_req is handled per state - don't auto-clear here
            data_out_valid <= 0;
            
            case (state)
                ST_IDLE: begin
                    if (start && !busy) begin
                        busy <= 1;
                        op_reg <= op_type;
                        addr_reg <= addr;
                        data_cnt <= 0;
                        poll_timeout <= 16'hFFFF;
                        
                        if (needs_wen(op_type))
                            state <= ST_WEN_START;
                        else
                            state <= ST_CMD_START;
                    end
                end
                
                // Write Enable
                ST_WEN_START: begin
                    ce_n <= 0;
                    spi_tx <= `CMD_WRITE_ENABLE;
                    spi_start <= 1;
                    state <= ST_WEN_WAIT;
                end
                
                ST_WEN_WAIT: begin
                    if (spi_done) begin
                        ce_n <= 1;
                        gap_cnt <= 0;
                        state <= ST_CS_GAP1;
                    end
                end
                
                ST_CS_GAP1: begin
                    gap_cnt <= gap_cnt + 1;
                    if (gap_cnt >= 10) begin
                        state <= ST_CMD_START;
                    end
                end
                
                // Main Command
                ST_CMD_START: begin
                    ce_n <= 0;
                    spi_tx <= get_cmd(op_reg);
                    spi_start <= 1;
                    state <= ST_CMD_WAIT;
                end
                
                ST_CMD_WAIT: begin
                    if (spi_done) begin
                        if (op_reg == OP_WRITE_ENABLE || op_reg == OP_WRITE_DISABLE) begin
                            ce_n <= 1;
                            state <= ST_DONE;
                        end
                        else if (needs_addr(op_reg)) begin
                            addr_byte <= 0;
                            state <= ST_ADDR_START;
                        end
                        else if (needs_data(op_reg)) begin
                            state <= ST_DATA_START;
                        end
                        else begin
                            ce_n <= 1;
                            state <= ST_DONE;
                        end
                    end
                end
                
                // Address (3 bytes)
                ST_ADDR_START: begin
                    case (addr_byte)
                        0: spi_tx <= addr_reg[23:16];
                        1: spi_tx <= addr_reg[15:8];
                        2: spi_tx <= addr_reg[7:0];
                        default: spi_tx <= 0;
                    endcase
                    spi_start <= 1;
                    state <= ST_ADDR_WAIT;
                end
                
                ST_ADDR_WAIT: begin
                    if (spi_done) begin
                        if (addr_byte == 2) begin
                            if (op_reg == OP_PAGE_PROGRAM) begin
                                data_req <= 1;  // Request first byte
                                state <= ST_DATA_START;
                            end
                            else begin
                                // Erase - just deassert CS
                                ce_n <= 1;
                                gap_cnt <= 0;
                                state <= ST_CS_GAP2;
                            end
                        end
                        else begin
                            addr_byte <= addr_byte + 1;
                            state <= ST_ADDR_START;
                        end
                    end
                end
                
                // Data phase
                ST_DATA_START: begin
                    if (op_reg == OP_PAGE_PROGRAM) begin
                        // Write data - wait for data_valid from FIFO
                        if (data_valid) begin
                            spi_tx <= data_in;
                            spi_start <= 1;
                            data_req <= 0;  // Clear request
                            state <= ST_DATA_WAIT;
                        end
                        // else: keep data_req high and wait
                    end
                    else begin
                        // Read data (status or ID)
                        spi_tx <= 8'hFF;
                        spi_start <= 1;
                        state <= ST_DATA_WAIT;
                    end
                end
                
                ST_DATA_WAIT: begin
                    if (spi_done) begin
                        if (op_reg == OP_PAGE_PROGRAM) begin
                            data_cnt <= data_cnt + 1;
                            if (data_cnt + 1 >= byte_count) begin
                                ce_n <= 1;
                                gap_cnt <= 0;
                                state <= ST_CS_GAP2;
                            end
                            else begin
                                data_req <= 1;
                                state <= ST_DATA_START;
                            end
                        end
                        else if (op_reg == OP_READ_STATUS) begin
                            data_out <= spi_rx;
                            data_out_valid <= 1;
                            ce_n <= 1;
                            state <= ST_DONE;
                        end
                        else if (op_reg == OP_READ_ID) begin
                            data_out <= spi_rx;
                            data_out_valid <= 1;
                            data_cnt <= data_cnt + 1;
                            if (data_cnt >= 2) begin
                                ce_n <= 1;
                                state <= ST_DONE;
                            end
                            else begin
                                state <= ST_DATA_START;
                            end
                        end
                        else begin
                            ce_n <= 1;
                            state <= ST_DONE;
                        end
                    end
                end
                
                ST_CS_GAP2: begin
                    gap_cnt <= gap_cnt + 1;
                    if (gap_cnt >= 10) begin
                        if (needs_poll(op_reg))
                            state <= ST_POLL_CMD;
                        else
                            state <= ST_DONE;
                    end
                end
                
                // Poll WIP bit - Phase 1: Send command
                ST_POLL_CMD: begin
                    ce_n <= 0;
                    spi_tx <= `CMD_READ_STATUS1;
                    spi_start <= 1;
                    state <= ST_POLL_WAIT_CMD;
                end
                
                // Wait for command to complete
                ST_POLL_WAIT_CMD: begin
                    if (spi_done) begin
                        // Command sent, now read status byte
                        spi_tx <= 8'hFF;
                        spi_start <= 1;
                        state <= ST_POLL_READ;
                    end
                end
                
                // Read status byte and check WIP
                ST_POLL_READ: begin
                    if (spi_done) begin
                        ce_n <= 1;
                        
                        if (spi_rx[0] == 0) begin
                            // WIP = 0, done
                            state <= ST_DONE;
                        end
                        else begin
                            poll_timeout <= poll_timeout - 1;
                            if (poll_timeout == 0) begin
                                error <= 1;
                                state <= ST_DONE;
                            end
                            else begin
                                gap_cnt <= 0;
                                state <= ST_CS_GAP2;
                            end
                        end
                    end
                end
                
                ST_DONE: begin
                    ce_n <= 1;
                    busy <= 0;
                    done <= 1;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

// =============================================================================
// XIP Reader for Fast Read Quad I/O (0xEB)
// =============================================================================
module qspi_xip_reader #(
    parameter LINE_SIZE = 16
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire [23:0]                  addr,
    input  wire                         rd,
    output wire                         done,
    output wire [(LINE_SIZE*8)-1:0]     line,
    
    output reg                          sck,
    output reg                          ce_n,
    input  wire [3:0]                   din,
    output wire [3:0]                   dout,
    output wire                         douten
);

    localparam LINE_BYTES = LINE_SIZE;

    localparam  IDLE = 1'b0,
                READ = 1'b1;

    reg         state, nstate;
    reg [7:0]   counter;
    reg [23:0]  saddr;
    reg [7:0]   data [LINE_BYTES-1:0];
    reg         first;

    wire [7:0]  EBH = 8'hEB;

    always @* begin
        case (state)
            IDLE:   nstate = rd ? READ : IDLE;
            READ:   nstate = done ? IDLE : READ;
            default: nstate = IDLE;
        endcase
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) first <= 1'b1;
        else if (first & done) first <= 0;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) state <= IDLE;
        else state <= nstate;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) sck <= 1'b0;
        else if (~ce_n) sck <= ~sck;
        else if (state == IDLE) sck <= 1'b0;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) ce_n <= 1'b1;
        else if (state == READ) ce_n <= 1'b0;
        else ce_n <= 1'b1;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) counter <= 8'b0;
        else if (sck & ~done) counter <= counter + 1'b1;
        else if (state == IDLE)
            if (first) counter <= 8'b0;
            else counter <= 8'd8;

    always @(posedge clk or negedge rst_n)
        if (!rst_n) saddr <= 24'b0;
        else if ((state == IDLE) && rd) saddr <= addr;

    integer idx;
    always @(posedge clk)
        if (counter >= 20 && counter <= 19 + LINE_BYTES*2)
            if (sck) begin
                idx = counter/2 - 10;
                data[idx] <= {data[idx][3:0], din};
            end

    assign dout =   (counter < 8)   ? {3'b0, EBH[7-counter]}    :
                    (counter == 8)  ? saddr[23:20]              :
                    (counter == 9)  ? saddr[19:16]              :
                    (counter == 10) ? saddr[15:12]              :
                    (counter == 11) ? saddr[11:8]               :
                    (counter == 12) ? saddr[7:4]                :
                    (counter == 13) ? saddr[3:0]                :
                    (counter == 14) ? 4'hA                      :
                    (counter == 15) ? 4'h5                      :
                                      4'h0;

    assign douten = (counter < 20);
    assign done   = (counter == 19 + LINE_BYTES*2);

    generate
        genvar i;
        for (i = 0; i < LINE_BYTES; i = i + 1)
            assign line[i*8+7:i*8] = data[i];
    endgenerate

endmodule

// =============================================================================
// Direct Mapped Cache
// =============================================================================
module qspi_dmc #(
    parameter NUM_LINES = 32,
    parameter LINE_SIZE = 16
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire [23:0]                  A,
    output wire [31:0]                  Do,
    output wire                         hit,
    input  wire [(LINE_SIZE*8)-1:0]     line,
    input  wire                         wr
);

    localparam  OFFSET_WIDTH    = $clog2(LINE_SIZE);
    localparam  INDEX_WIDTH     = $clog2(NUM_LINES);
    localparam  TAG_WIDTH       = 24 - OFFSET_WIDTH - INDEX_WIDTH;

    wire [OFFSET_WIDTH-1:0]     offset  = A[OFFSET_WIDTH-1:0];
    wire [INDEX_WIDTH-1:0]      index   = A[OFFSET_WIDTH+INDEX_WIDTH-1:OFFSET_WIDTH];
    wire [TAG_WIDTH-1:0]        tag     = A[23:OFFSET_WIDTH+INDEX_WIDTH];

    reg [TAG_WIDTH-1:0]         TAG     [NUM_LINES-1:0];
    reg [(LINE_SIZE*8)-1:0]     DATA    [NUM_LINES-1:0];
    reg                         VALID   [NUM_LINES-1:0];

    wire [TAG_WIDTH-1:0]        stored_tag = TAG[index];

    assign hit = VALID[index] && (stored_tag == tag);

    wire [(LINE_SIZE*8)-1:0]    stored_line = DATA[index];
    wire [31:0]                 word_sel = stored_line >> ({offset[OFFSET_WIDTH-1:2], 5'b0});
    assign Do = word_sel[31:0];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_LINES; i = i + 1)
                VALID[i] <= 1'b0;
        end
        else if (wr) begin
            TAG[index]   <= tag;
            DATA[index]  <= line;
            VALID[index] <= 1'b1;
        end
    end

endmodule

// =============================================================================
// Top-Level QSPI XIP + Program Controller with AHB-Lite Interface
// =============================================================================
module qspi_xip_prog_ctrl_ahbl #(
    parameter NUM_LINES     = 32,
    parameter LINE_SIZE     = 16,
    parameter RESET_CYCLES  = 999
)(
    // AHB-Lite Slave Interface (XIP Read)
    input  wire         HCLK,
    input  wire         HRESETn,
    input  wire         HSEL,
    input  wire [31:0]  HADDR,
    input  wire [1:0]   HTRANS,
    input  wire         HWRITE,
    input  wire [31:0]  HWDATA,
    input  wire         HREADY,
    output reg          HREADYOUT,
    output wire [31:0]  HRDATA,
    
    // Program/Erase Control Interface
    input  wire         prog_start,
    input  wire [2:0]   prog_op,
    input  wire [23:0]  prog_addr,
    input  wire [7:0]   prog_data_in,
    input  wire         prog_data_valid,
    output wire         prog_data_req,
    output wire [7:0]   prog_data_out,
    output wire         prog_data_out_valid,
    input  wire [8:0]   prog_byte_count,
    output wire         prog_busy,
    output wire         prog_done,
    output wire         prog_error,
    
    // External Interface to Quad I/O Flash
    output wire         sck,
    output wire         ce_n,
    input  wire [3:0]   din,
    output wire [3:0]   dout,
    output wire [3:0]   douten
);

    localparam OFF_WIDTH = $clog2(LINE_SIZE);

    // Cache signals
    wire [31:0]                 c_datao;
    wire [(LINE_SIZE*8)-1:0]    c_line;
    wire                        c_hit;
    reg  [2:0]                  c_wr;
    wire [23:0]                 c_A;

    // XIP Reader signals
    wire        xip_rd;
    wire        xip_done;
    wire        xip_sck;
    wire        xip_ce_n;
    wire [3:0]  xip_dout;
    wire        xip_douten;

    // Programmer signals
    wire        prg_sck;
    wire        prg_ce_n;
    wire [3:0]  prg_dout;
    wire [3:0]  prg_douten;  // 4-bit douten for SPI programmer

    // State machine
    localparam [1:0]    IDLE    = 2'b00,
                        WAIT    = 2'b01,
                        RW      = 2'b10;

    reg [1:0]   state, nstate;

    // AHB-Lite Address Phase Regs
    reg             last_HSEL;
    reg [31:0]      last_HADDR;
    reg             last_HWRITE;
    reg [1:0]       last_HTRANS;
    reg             last_valid;

    wire valid = HSEL & HTRANS[1] & HREADY & ~HWRITE;

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn) begin
            last_HSEL   <= 1'b0;
            last_HADDR  <= 32'b0;
            last_HWRITE <= 1'b0;
            last_HTRANS <= 2'b0;
            last_valid  <= 1'b0;
        end
        else if (HREADY) begin
            last_HSEL   <= HSEL;
            last_HADDR  <= HADDR;
            last_HWRITE <= HWRITE;
            last_HTRANS <= HTRANS;
            last_valid  <= valid;
        end
    end

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            state <= IDLE;
        else
            state <= nstate;
    end

    always @* begin
        nstate = IDLE;
        case (state)
            IDLE:   if (valid & c_hit)
                        nstate = IDLE;
                    else if (valid & ~c_hit & ~prog_busy)
                        nstate = WAIT;
                    else
                        nstate = IDLE;

            WAIT:   if (c_wr[2])
                        nstate = RW;
                    else
                        nstate = WAIT;

            RW:     nstate = IDLE;
        endcase
    end

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            HREADYOUT <= 1'b1;
        else
            case (state)
                IDLE:   if (valid & c_hit)
                            HREADYOUT <= 1'b1;
                        else if (valid & ~c_hit & ~prog_busy)
                            HREADYOUT <= 1'b0;
                        else
                            HREADYOUT <= 1'b1;

                WAIT:   HREADYOUT <= 1'b0;

                RW:     HREADYOUT <= 1'b1;
            endcase
    end

    always @(posedge HCLK or negedge HRESETn) begin
        if (!HRESETn)
            c_wr <= 3'b0;
        else
            c_wr <= {c_wr[1:0], xip_done};
    end

    assign xip_rd = (HTRANS[1] & HSEL & HREADY & ~HWRITE & ~c_hit & (state == IDLE) & ~prog_busy) |
                    (HTRANS[1] & HSEL & HREADY & ~HWRITE & ~c_hit & (state == RW) & ~prog_busy);

    assign c_A = (state != IDLE) ? last_HADDR[23:0] : HADDR[23:0];

    // Cache instance
    qspi_dmc #(
        .NUM_LINES(NUM_LINES),
        .LINE_SIZE(LINE_SIZE)
    ) CACHE (
        .clk(HCLK),
        .rst_n(HRESETn),
        .A(c_A),
        .Do(c_datao),
        .hit(c_hit),
        .line(c_line),
        .wr(c_wr[1])
    );

    // XIP Reader instance
    qspi_xip_reader #(
        .LINE_SIZE(LINE_SIZE)
    ) XIP_READER (
        .clk(HCLK),
        .rst_n(HRESETn),
        .addr({c_A[23:OFF_WIDTH], {OFF_WIDTH{1'b0}}}),
        .rd(xip_rd),
        .done(xip_done),
        .line(c_line),
        .sck(xip_sck),
        .ce_n(xip_ce_n),
        .din(din),
        .dout(xip_dout),
        .douten(xip_douten)
    );

    // Programmer instance
    qspi_flash_programmer_simple PROGRAMMER (
        .clk(HCLK),
        .rst_n(HRESETn),
        .start(prog_start),
        .op_type(prog_op),
        .addr(prog_addr),
        .data_in(prog_data_in),
        .data_valid(prog_data_valid),
        .data_req(prog_data_req),
        .data_out(prog_data_out),
        .data_out_valid(prog_data_out_valid),
        .byte_count(prog_byte_count),
        .busy(prog_busy),
        .done(prog_done),
        .error(prog_error),
        .sck(prg_sck),
        .ce_n(prg_ce_n),
        .din(din),
        .dout(prg_dout),
        .douten(prg_douten)
    );

    // MUX between XIP and Programmer
    assign sck    = prog_busy ? prg_sck    : xip_sck;
    assign ce_n   = prog_busy ? prg_ce_n   : xip_ce_n;
    assign dout   = prog_busy ? prg_dout   : xip_dout;
    assign douten = prog_busy ? prg_douten : (xip_douten ? 4'b1111 : 4'b0000);

    // Read data output
    reg [31:0] cdata;
    assign HRDATA = cdata;

    always @(posedge HCLK) begin
        case (state)
            IDLE: if (valid & c_hit) cdata <= c_datao;
            WAIT: if (c_wr[1]) cdata <= c_datao;
        endcase
    end

endmodule
