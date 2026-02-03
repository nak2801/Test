/*
    Testbench for QSPI XIP + Program Controller (AHB-Lite only)
    
    Memory Map:
    - 0x00000000 - 0x00FFFFFF: Flash XIP Read
    - 0x00010000 - 0x0001001F: Control Registers
    
    Author: AI Generated
    Date: 2026-01-23
*/

`timescale 1ns/1ps

module tb_qspi_xip_prog;

    // Parameters
    parameter CLK_PERIOD = 20;  // 50MHz
    parameter MEM_SIZE   = 16*1024;  // 16KB for simulation

    // Register base address
    parameter REG_BASE = 32'h00010000;

    // Signals
    reg         clk;
    reg         rst_n;
    
    // AHB-Lite signals
    reg         HSEL;
    reg  [31:0] HADDR;
    reg  [1:0]  HTRANS;
    reg         HWRITE;
    reg  [31:0] HWDATA;
    reg         HREADY;
    wire        HREADYOUT;
    wire [31:0] HRDATA;
    wire        HRESP;
    
    // Interrupts
    wire        irq_done;
    wire        irq_error;
    
    // QSPI signals
    wire        sck;
    wire        ce_n;
    wire [3:0]  din;
    wire [3:0]  dout;
    wire [3:0]  douten;
    
    // Bidirectional QSPI data
    wire [3:0]  qspi_io;
    
    // Generate tristate for QSPI
    assign qspi_io[0] = douten[0] ? dout[0] : 1'bz;
    assign qspi_io[1] = douten[1] ? dout[1] : 1'bz;
    assign qspi_io[2] = douten[2] ? dout[2] : 1'bz;
    assign qspi_io[3] = douten[3] ? dout[3] : 1'bz;
    assign din = qspi_io;

    // DUT
    qspi_xip_prog_ctrl #(
        .NUM_LINES(16),
        .LINE_SIZE(16),
        .RESET_CYCLES(10),
        .FIFO_DEPTH(256),
        .REG_BASE(24'h010000)
    ) dut (
        .HCLK(clk),
        .HRESETn(rst_n),
        
        .HSEL(HSEL),
        .HADDR(HADDR),
        .HTRANS(HTRANS),
        .HWRITE(HWRITE),
        .HWDATA(HWDATA),
        .HREADY(HREADY),
        .HREADYOUT(HREADYOUT),
        .HRDATA(HRDATA),
        .HRESP(HRESP),
        
        .irq_done(irq_done),
        .irq_error(irq_error),
        
        .sck(sck),
        .ce_n(ce_n),
        .din(din),
        .dout(dout),
        .douten(douten)
    );

    // Flash model
    flash_model #(
        .MEM_SIZE(MEM_SIZE)
    ) flash (
        .sck(sck),
        .ce_n(ce_n),
        .io(qspi_io)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // AHB write task
    task ahb_write;
        input [31:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            HSEL   <= 1'b1;
            HADDR  <= addr;
            HTRANS <= 2'b10;  // NONSEQ
            HWRITE <= 1'b1;
            HREADY <= 1'b1;
            
            @(posedge clk);
            while (!HREADYOUT) @(posedge clk);
            
            HWDATA <= data;
            HTRANS <= 2'b00;  // IDLE
            HSEL   <= 1'b0;
            HWRITE <= 1'b0;
            
            @(posedge clk);
            while (!HREADYOUT) @(posedge clk);
        end
    endtask

    // AHB read task
    task ahb_read;
        input  [31:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            HSEL   <= 1'b1;
            HADDR  <= addr;
            HTRANS <= 2'b10;  // NONSEQ
            HWRITE <= 1'b0;
            HREADY <= 1'b1;
            
            @(posedge clk);
            while (!HREADYOUT) @(posedge clk);
            
            HTRANS <= 2'b00;  // IDLE
            HSEL   <= 1'b0;
            
            @(posedge clk);
            while (!HREADYOUT) @(posedge clk);
            data = HRDATA;
        end
    endtask

    // Wait for operation done
    task wait_done;
        reg [31:0] status;
        integer timeout;
        begin
            status = 32'h1;  // BUSY
            timeout = 10000;
            while (status[0] && timeout > 0) begin
                ahb_read(REG_BASE + 32'h04, status);
                timeout = timeout - 1;
                #100;
            end
            if (timeout == 0)
                $display("[TB] WARNING: Timeout waiting for done!");
            else
                $display("[TB] Operation complete, STATUS=0x%08X", status);
        end
    endtask

    // Register offsets
    localparam  REG_CTRL    = 32'h00,
                REG_STATUS  = 32'h04,
                REG_ADDR    = 32'h08,
                REG_LEN     = 32'h0C,
                REG_DATA    = 32'h10,
                REG_ID      = 32'h14;

    // Operation types
    localparam  OP_WRITE_ENABLE     = 3'd0,
                OP_WRITE_DISABLE    = 3'd1,
                OP_READ_STATUS      = 3'd2,
                OP_PAGE_PROGRAM     = 3'd3,
                OP_SECTOR_ERASE     = 3'd4,
                OP_BLOCK_ERASE_32   = 3'd5,
                OP_BLOCK_ERASE_64   = 3'd6,
                OP_READ_ID          = 3'd7;

    // Test variables
    reg [31:0] read_data;
    reg [31:0] ahb_data;
    integer i;
    integer errors;

    // Main test
    initial begin
        $display("\n========================================");
        $display("  QSPI XIP + Program Controller Test");
        $display("  (Unified AHB-Lite Interface)");
        $display("========================================\n");

        // Initialize
        rst_n   = 0;
        HSEL    = 0;
        HADDR   = 0;
        HTRANS  = 0;
        HWRITE  = 0;
        HWDATA  = 0;
        HREADY  = 1;
        errors  = 0;

        // Reset
        #100;
        rst_n = 1;
        #100;

        // ============================================
        // Test 1: Read JEDEC ID
        // ============================================
        $display("[TEST 1] Read JEDEC ID via AHB");
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_READ_ID});
        wait_done();
        ahb_read(REG_BASE + REG_ID, read_data);
        $display("  JEDEC ID = 0x%06X", read_data[23:0]);
        
        if (read_data[23:0] == 24'hEF4018) begin
            $display("  [PASS] JEDEC ID correct (Winbond W25Q128)");
        end else begin
            $display("  [FAIL] JEDEC ID incorrect, expected 0xEF4018");
            errors = errors + 1;
        end

        #1000;

        // ============================================
        // Test 2: Read Status Register
        // ============================================
        $display("\n[TEST 2] Read Status Register via AHB");
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_READ_STATUS});
        wait_done();
        ahb_read(REG_BASE + REG_STATUS, read_data);
        $display("  Flash Status = 0x%02X", read_data[15:8]);
        $display("  [INFO] WIP=%b, WEL=%b", read_data[8], read_data[9]);

        #1000;

        // ============================================
        // Test 3: Sector Erase
        // ============================================
        $display("\n[TEST 3] Sector Erase at 0x001000 via AHB");
        ahb_write(REG_BASE + REG_ADDR, 24'h001000);
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_SECTOR_ERASE});
        wait_done();
        $display("  [INFO] Erase complete");

        #1000;

        // ============================================
        // Test 4: Page Program
        // ============================================
        $display("\n[TEST 4] Page Program at 0x001000 via AHB");
        
        // Set address and length
        ahb_write(REG_BASE + REG_ADDR, 24'h001000);
        ahb_write(REG_BASE + REG_LEN, 9'd16);
        
        // Write data to FIFO
        for (i = 0; i < 16; i = i + 1) begin
            ahb_write(REG_BASE + REG_DATA, i + 8'hA0);
        end
        
        // Start programming
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_PAGE_PROGRAM});
        wait_done();
        
        $display("  [INFO] Programming complete");

        #5000;

        // ============================================
        // Test 5: XIP Read (verify programmed data)
        // ============================================
        $display("\n[TEST 5] XIP Read at 0x001000 via AHB");
        
        for (i = 0; i < 4; i = i + 1) begin
            ahb_read(32'h00001000 + i*4, ahb_data);
            $display("  XIP Read [0x%06X] = 0x%08X", 32'h001000 + i*4, ahb_data);
        end

        // Verify data
        ahb_read(32'h00001000, ahb_data);
        if (ahb_data == 32'hA3A2A1A0) begin
            $display("  [PASS] XIP read data correct");
        end else begin
            $display("  [FAIL] XIP read data incorrect, expected 0xA3A2A1A0, got 0x%08X", ahb_data);
            errors = errors + 1;
        end

        #1000;

        // ============================================
        // Summary
        // ============================================
        $display("\n========================================");
        if (errors == 0) begin
            $display("  All tests PASSED!");
        end else begin
            $display("  %0d tests FAILED!", errors);
        end
        $display("========================================\n");

        #1000;
        $finish;
    end

    // Timeout
    initial begin
        #5000000;
        $display("\n[ERROR] Simulation timeout!");
        $finish;
    end

    // Waveform dump
    initial begin
        $dumpfile("tb_qspi_xip_prog.vcd");
        $dumpvars(0, tb_qspi_xip_prog);
    end

endmodule

// =============================================================================
// Simple Flash Model for Testing
// =============================================================================
module flash_model #(
    parameter MEM_SIZE = 16*1024
)(
    input  wire         sck,
    input  wire         ce_n,
    inout  wire [3:0]   io
);

    // Memory
    reg [7:0] mem [0:MEM_SIZE-1];

    // JEDEC ID (Winbond W25Q128)
    localparam [23:0] JEDEC_ID = 24'hEF4018;

    // Status registers
    reg [7:0] status1;
    reg [7:0] status2;

    // State machine
    localparam  ST_IDLE     = 4'd0,
                ST_CMD      = 4'd1,
                ST_ADDR     = 4'd2,
                ST_DUMMY    = 4'd3,
                ST_READ     = 4'd4,
                ST_WRITE    = 4'd5,
                ST_STATUS   = 4'd6,
                ST_ID       = 4'd7;

    reg [3:0]   state;
    reg [7:0]   cmd;
    reg [23:0]  addr;
    reg [7:0]   bit_cnt;
    reg [7:0]   byte_cnt;
    reg [7:0]   shift_reg;
    reg [3:0]   out_reg;
    reg         out_en;
    reg [7:0]   dummy_cnt;
    reg         quad_mode;

    // Output driver
    assign io[0] = out_en ? out_reg[0] : 1'bz;
    assign io[1] = out_en ? out_reg[1] : 1'bz;
    assign io[2] = out_en ? out_reg[2] : 1'bz;
    assign io[3] = out_en ? out_reg[3] : 1'bz;

    // Initialize memory
    integer i;
    initial begin
        for (i = 0; i < MEM_SIZE; i = i + 1)
            mem[i] = 8'hFF;
        
        status1 = 8'h00;  // WIP=0, WEL=0
        status2 = 8'h02;  // QE=1
        state = ST_IDLE;
        out_en = 0;
        out_reg = 4'hF;
        quad_mode = 0;
        cmd = 0;
        addr = 0;
        bit_cnt = 0;
        byte_cnt = 0;
        shift_reg = 0;
        dummy_cnt = 0;
    end

    // Reset on CS deassert
    always @(posedge ce_n) begin
        state <= ST_IDLE;
        bit_cnt <= 0;
        byte_cnt <= 0;
        out_en <= 0;
        out_reg <= 4'hF;
        quad_mode <= 0;
        
        // Clear WEL after program/erase complete
        if (cmd == 8'h02 || cmd == 8'h20 || cmd == 8'hD8 || cmd == 8'h52) begin
            status1[1] <= 0;  // Clear WEL
            status1[0] <= 0;  // Clear WIP
        end
    end

    // Main state machine - process on rising edge of SCK
    always @(posedge sck) begin
        if (!ce_n) begin
            case (state)
                ST_IDLE: begin
                    // First rising edge - capture first bit of command
                    state <= ST_CMD;
                    cmd <= {7'b0, io[0]};  // Capture bit 7 (MSB)
                    bit_cnt <= 1;
                    out_en <= 0;
                end

                ST_CMD: begin
                    // Shift in command bit
                    cmd <= {cmd[6:0], io[0]};
                    bit_cnt <= bit_cnt + 1;
                    
                    if (bit_cnt == 7) begin
                        // Command complete, decode
                        case ({cmd[6:0], io[0]})
                            8'h06: begin  // Write Enable
                                status1[1] <= 1;  // Set WEL
                                state <= ST_IDLE;
                                $display("[FLASH] Write Enable - WEL set");
                            end
                            8'h04: begin  // Write Disable
                                status1[1] <= 0;
                                state <= ST_IDLE;
                            end
                            8'h05: begin  // Read Status 1
                                state <= ST_STATUS;
                                shift_reg <= status1;
                                bit_cnt <= 0;
                                out_en <= 1;
                                out_reg <= {2'b11, status1[7], 1'b1};  // MISO on io[1]
                                $display("[FLASH] Read Status 1 = 0x%02X", status1);
                            end
                            8'h35: begin  // Read Status 2
                                state <= ST_STATUS;
                                shift_reg <= status2;
                                bit_cnt <= 0;
                                out_en <= 1;
                                out_reg <= {2'b11, status2[7], 1'b1};  // MISO on io[1]
                            end
                            8'h9F: begin  // Read JEDEC ID
                                state <= ST_ID;
                                bit_cnt <= 0;
                                byte_cnt <= 0;
                                shift_reg <= JEDEC_ID[23:16];
                                out_en <= 1;
                                out_reg <= {2'b11, JEDEC_ID[23], 1'b1};  // MISO on io[1]
                                $display("[FLASH] Read JEDEC ID");
                            end
                            8'h03, 8'h0B: begin  // Read Data, Fast Read
                                state <= ST_ADDR;
                                bit_cnt <= 0;
                                quad_mode <= 0;
                                addr <= 0;
                            end
                            8'hEB: begin  // Fast Read Quad I/O
                                state <= ST_ADDR;
                                bit_cnt <= 0;
                                quad_mode <= 1;
                                addr <= 0;
                                $display("[FLASH] Fast Read Quad I/O (0xEB)");
                            end
                            8'h02: begin  // Page Program
                                state <= ST_ADDR;
                                bit_cnt <= 0;
                                quad_mode <= 0;
                                addr <= 0;
                                $display("[FLASH] Page Program");
                            end
                            8'h20, 8'h52, 8'hD8: begin  // Erase
                                state <= ST_ADDR;
                                bit_cnt <= 0;
                                quad_mode <= 0;
                                addr <= 0;
                                $display("[FLASH] Erase command 0x%02X", {cmd[6:0], io[0]});
                            end
                            default: begin
                                state <= ST_IDLE;
                                $display("[FLASH] Unknown command: 0x%02X", {cmd[6:0], io[0]});
                            end
                        endcase
                    end
                end

                ST_ADDR: begin
                    if (quad_mode && cmd == 8'hEB) begin
                        // Quad address input (4 bits per clock)
                        addr <= {addr[19:0], io};
                        bit_cnt <= bit_cnt + 1;
                        
                        if (bit_cnt == 5) begin  // 6 nibbles = 24 bits
                            bit_cnt <= 0;
                            dummy_cnt <= 0;
                            state <= ST_DUMMY;
                            $display("[FLASH] Quad address: 0x%06X", {addr[19:0], io});
                        end
                    end
                    else begin
                        // SPI address input (1 bit per clock)
                        addr <= {addr[22:0], io[0]};
                        bit_cnt <= bit_cnt + 1;
                        
                        if (bit_cnt == 23) begin
                            bit_cnt <= 0;
                            
                            if (cmd == 8'h03) begin
                                state <= ST_READ;
                                out_en <= 1;
                                shift_reg <= mem[{addr[22:0], io[0]} % MEM_SIZE];
                                out_reg <= {2'b11, mem[{addr[22:0], io[0]} % MEM_SIZE][7], 1'b1};  // MISO on io[1]
                                $display("[FLASH] Read from 0x%06X", {addr[22:0], io[0]});
                            end
                            else if (cmd == 8'h0B) begin
                                dummy_cnt <= 0;
                                state <= ST_DUMMY;
                            end
                            else if (cmd == 8'h02) begin
                                state <= ST_WRITE;
                                $display("[FLASH] Program at 0x%06X", {addr[22:0], io[0]});
                            end
                            else if (cmd == 8'h20 || cmd == 8'h52 || cmd == 8'hD8) begin
                                // Erase sector/block
                                do_erase(cmd, {addr[22:0], io[0]});
                                state <= ST_IDLE;
                            end
                        end
                    end
                end

                ST_DUMMY: begin
                    dummy_cnt <= dummy_cnt + 1;
                    
                    // For 0xEB: 2 mode clocks + 4 dummy = 6 clocks
                    // For 0x0B: 8 dummy clocks
                    if ((cmd == 8'hEB && dummy_cnt >= 5) ||
                        (cmd == 8'h0B && dummy_cnt >= 7)) begin
                        state <= ST_READ;
                        out_en <= 1;
                        bit_cnt <= 0;
                        shift_reg <= mem[addr % MEM_SIZE];
                        if (quad_mode)
                            out_reg <= mem[addr % MEM_SIZE][7:4];
                        else
                            out_reg <= {2'b11, mem[addr % MEM_SIZE][7], 1'b1};  // MISO on io[1]
                        $display("[FLASH] Start reading from 0x%06X, data=0x%02X", addr, mem[addr % MEM_SIZE]);
                    end
                end

                ST_READ: begin
                    if (quad_mode) begin
                        // Quad output (4 bits per clock)
                        if (bit_cnt[0] == 0) begin
                            out_reg <= shift_reg[7:4];
                        end
                        else begin
                            out_reg <= shift_reg[3:0];
                            addr <= addr + 1;
                            shift_reg <= mem[(addr + 1) % MEM_SIZE];
                        end
                        bit_cnt <= bit_cnt + 1;
                    end
                    else begin
                        // SPI output - just count bits on rising edge
                        // Data shift handled on falling edge
                        bit_cnt <= bit_cnt + 1;
                    end
                end

                ST_WRITE: begin
                    // SPI write (1 bit per clock)
                    shift_reg <= {shift_reg[6:0], io[0]};
                    bit_cnt <= bit_cnt + 1;
                    
                    if (bit_cnt[2:0] == 7) begin
                        // Program byte (AND with existing data)
                        mem[addr % MEM_SIZE] <= mem[addr % MEM_SIZE] & {shift_reg[6:0], io[0]};
                        $display("[FLASH] Program byte 0x%02X at 0x%06X", {shift_reg[6:0], io[0]}, addr);
                        addr <= addr + 1;
                    end
                end

                ST_STATUS: begin
                    // On rising edge: just count bits
                    bit_cnt <= bit_cnt + 1;
                end

                ST_ID: begin
                    // On rising edge: count bits and load next byte when done with current
                    bit_cnt <= bit_cnt + 1;
                    
                    if (bit_cnt[2:0] == 7) begin
                        byte_cnt <= byte_cnt + 1;
                        // Load next byte for next 8 clocks
                        case (byte_cnt)
                            0: shift_reg <= JEDEC_ID[15:8];
                            1: shift_reg <= JEDEC_ID[7:0];
                            default: shift_reg <= 8'hFF;
                        endcase
                    end
                end
            endcase
        end
    end
    
    // Output data on falling edge of SCK (flash drives data after rising edge)
    always @(negedge sck) begin
        if (!ce_n) begin
            case (state)
                ST_READ: begin
                    if (!quad_mode) begin
                        // SPI output (1 bit per clock, MISO = io[1])
                        out_reg <= {2'b11, shift_reg[7], 1'b1};
                        shift_reg <= {shift_reg[6:0], 1'b1};
                        
                        if (bit_cnt[2:0] == 0 && bit_cnt != 0) begin
                            addr <= addr + 1;
                            shift_reg <= mem[addr % MEM_SIZE];
                        end
                    end
                end
                
                ST_STATUS: begin
                    // SPI output for status (MISO = io[1])
                    out_reg <= {2'b11, shift_reg[7], 1'b1};
                    shift_reg <= {shift_reg[6:0], 1'b1};
                end

                ST_ID: begin
                    // SPI output for JEDEC ID (MISO = io[1])
                    // Shift out current bit
                    out_reg <= {2'b11, shift_reg[7], 1'b1};
                    shift_reg <= {shift_reg[6:0], 1'b1};
                end
            endcase
        end
    end

    // Erase task
    task do_erase;
        input [7:0]  ecmd;
        input [23:0] eaddr;
        integer start_addr, end_addr, j;
        begin
            case (ecmd)
                8'h20: begin  // Sector Erase (4KB)
                    start_addr = (eaddr / 4096) * 4096;
                    end_addr = start_addr + 4095;
                end
                8'h52: begin  // Block Erase (32KB)
                    start_addr = (eaddr / 32768) * 32768;
                    end_addr = start_addr + 32767;
                end
                8'hD8: begin  // Block Erase (64KB)
                    start_addr = (eaddr / 65536) * 65536;
                    end_addr = start_addr + 65535;
                end
                default: begin
                    start_addr = 0;
                    end_addr = 0;
                end
            endcase
            
            $display("[FLASH] Erasing from 0x%06X to 0x%06X", start_addr, end_addr);
            
            for (j = start_addr; j <= end_addr && j < MEM_SIZE; j = j + 1)
                mem[j] = 8'hFF;
        end
    endtask

endmodule
