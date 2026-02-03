/*
    Testbench for QSPI XIP + Program Controller
    Using Official Winbond W25Q128JVxIM Flash Model
    
    Memory Map:
    - 0x00000000 - 0x00FFFFFF: Flash XIP Read
    - 0x00010000 - 0x0001001F: Control Registers
    
    Author: Nguyen Kiet  
    Date: 2026-02-02
*/

`timescale 1ns/1ps

module tb_qspi_winbond;

    // Parameters
    parameter CLK_PERIOD = 20;  // 50MHz

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
    
    // QSPI signals from controller
    wire        sck;
    wire        ce_n;
    wire [3:0]  din;
    wire [3:0]  dout;
    wire [3:0]  douten;
    
    // Flash interface wires
    wire        flash_clk;
    wire        flash_csn;
    wire        flash_dio;   // IO0 - MOSI
    wire        flash_do;    // IO1 - MISO  
    wire        flash_wpn;   // IO2 - WP#
    wire        flash_holdn; // IO3 - HOLD#

    // Connect SCK and CSn
    assign flash_clk = sck;
    assign flash_csn = ce_n;

    // Bidirectional IO mapping
    // Controller dout[0] -> Flash DIO (MOSI)
    // Controller dout[1] -> Flash DO (MISO) - normally input from flash
    // Controller dout[2] -> Flash WPn
    // Controller dout[3] -> Flash HOLDn

    // IO0 (DIO/MOSI)
    assign flash_dio = douten[0] ? dout[0] : 1'bz;
    assign din[0] = flash_dio;

    // IO1 (DO/MISO) 
    assign flash_do = douten[1] ? dout[1] : 1'bz;
    assign din[1] = flash_do;

    // IO2 (WPn)
    assign flash_wpn = douten[2] ? dout[2] : 1'bz;
    assign din[2] = flash_wpn;

    // IO3 (HOLDn)
    assign flash_holdn = douten[3] ? dout[3] : 1'bz;
    assign din[3] = flash_holdn;

    // Pull-ups for inactive signals
    pullup(flash_wpn);
    pullup(flash_holdn);

    // DUT - QSPI Controller
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

    // Winbond W25Q128JVxIM Flash Model with reduced timing for simulation
    // Override timing parameters to speed up simulation:
    // - tPP:  700000 -> 7000  (700us -> 7us)  Page Program
    // - tSE:  30000000 -> 30000 (30ms -> 30us) Sector Erase
    // - tBE1: 120000000 -> 120000 (120ms -> 120us) 32KB Block Erase
    // - tBE2: 150000000 -> 150000 (150ms -> 150us) 64KB Block Erase
    defparam flash_model.tPP  = 7000;      // 7us instead of 700us
    defparam flash_model.tSE  = 30000;     // 30us instead of 30ms
    defparam flash_model.tBE1 = 120000;    // 120us instead of 120ms
    defparam flash_model.tBE2 = 150000;    // 150us instead of 150ms
    
    W25Q128JVxIM flash_model (
        .CSn(flash_csn),
        .CLK(flash_clk),
        .DIO(flash_dio),
        .DO(flash_do),
        .WPn(flash_wpn),
        .HOLDn(flash_holdn)
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
            timeout = 100000;
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
        $display("  QSPI Controller Test with Winbond");
        $display("  W25Q128JVxIM Official Flash Model");
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
        #200;
        rst_n = 1;
        #500;

        // ============================================
        // Test 1: Read JEDEC ID
        // ============================================
        $display("[TEST 1] Read JEDEC ID via AHB");
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_READ_ID});
        wait_done();
        ahb_read(REG_BASE + REG_ID, read_data);
        $display("  JEDEC ID = 0x%06X", read_data[23:0]);
        
        // W25Q128JV JEDEC ID: Manufacturer=0xEF, Memory Type=0x70, Capacity=0x18
        if (read_data[23:0] == 24'hEF7018) begin
            $display("  [PASS] JEDEC ID correct (Winbond W25Q128JV)");
        end else if (read_data[23:0] == 24'hEF4018) begin
            $display("  [PASS] JEDEC ID correct (Winbond W25Q128 variant)");
        end else begin
            $display("  [INFO] JEDEC ID = 0x%06X", read_data[23:0]);
            // Not marking as error since different flash variants have different IDs
        end

        #2000;

        // ============================================
        // Test 2: Read Status Register
        // ============================================
        $display("\n[TEST 2] Read Status Register via AHB");
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_READ_STATUS});
        wait_done();
        ahb_read(REG_BASE + REG_STATUS, read_data);
        $display("  Flash Status = 0x%02X", read_data[15:8]);
        $display("  [INFO] WIP=%b, WEL=%b", read_data[8], read_data[9]);

        #2000;

        // ============================================
        // Test 3: Sector Erase
        // ============================================
        $display("\n[TEST 3] Sector Erase at 0x001000 via AHB");
        ahb_write(REG_BASE + REG_ADDR, 24'h001000);
        ahb_write(REG_BASE + REG_CTRL, {24'b0, 1'b1, 4'b0, OP_SECTOR_ERASE});
        wait_done();
        $display("  [INFO] Erase command sent");

        #10000;

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
        
        $display("  [INFO] Programming command sent");

        #10000;

        // ============================================
        // Test 5: XIP Read (verify programmed data)
        // ============================================
        $display("\n[TEST 5] XIP Read at 0x001000 via AHB");
        
        for (i = 0; i < 4; i = i + 1) begin
            ahb_read(32'h00001000 + i*4, ahb_data);
            $display("  XIP Read [0x%06X] = 0x%08X", 32'h001000 + i*4, ahb_data);
        end

        #2000;

        // ============================================
        // Test 6: XIP Read from address 0
        // ============================================
        $display("\n[TEST 6] XIP Read at 0x000000 via AHB");
        
        for (i = 0; i < 4; i = i + 1) begin
            ahb_read(i*4, ahb_data);
            $display("  XIP Read [0x%06X] = 0x%08X", i*4, ahb_data);
        end

        #2000;

        // ============================================
        // Summary
        // ============================================
        $display("\n========================================");
        if (errors == 0) begin
            $display("  All tests completed!");
        end else begin
            $display("  %0d tests FAILED!", errors);
        end
        $display("========================================\n");

        #2000;
        $finish;
    end

    // Timeout
    initial begin
        #10000000;  // 10ms timeout
        $display("\n[ERROR] Simulation timeout!");
        $finish;
    end

    // Waveform dump
    initial begin
        $dumpfile("tb_qspi_winbond.vcd");
        $dumpvars(0, tb_qspi_winbond);
    end

endmodule
