# QSPI Flash Controller Module

## Overview
QSPI Flash Controller với hỗ trợ XIP (Execute-in-Place) và Program/Erase operations qua AHB-Lite interface.

## Directory Structure

```
QSPI_module/
├── rtl/                          # RTL Source Files
│   ├── qspi_flash_ctrl.v         # Core modules (SPI master, XIP reader, Cache, etc.)
│   ├── qspi_prog_ahb.v           # AHB register interface for programmer
│   ├── qspi_xip_prog_ctrl.v      # TOP-LEVEL: Unified controller
│   └── qspi_addr_decode.v        # Address decoder
│
├── tb/                           # Testbench Files  
│   └── tb_qspi_xip_prog.v        # Main testbench + Flash model
│
├── Makefile                      # Simulation Makefile
└── README.md                     # This file
```

## Module Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│              qspi_xip_prog_ctrl (TOP)                           │
│              [qspi_xip_prog_ctrl.v]                             │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                                                          │  │
│  │  ┌────────────────────────┐  ┌─────────────────────────┐│  │
│  │  │ qspi_xip_prog_ctrl_ahbl│  │    qspi_prog_ahb       ││  │
│  │  │ [qspi_flash_ctrl.v]    │  │  [qspi_prog_ahb.v]     ││  │
│  │  │                        │  │                         ││  │
│  │  │  ┌──────────────────┐  │  │  - AHB Register IF     ││  │
│  │  │  │   qspi_dmc       │  │  │  - FIFO for data       ││  │
│  │  │  │   (Cache)        │  │  │  - Status capture      ││  │
│  │  │  └──────────────────┘  │  │                         ││  │
│  │  │                        │  └─────────────────────────┘│  │
│  │  │  ┌──────────────────┐  │                             │  │
│  │  │  │ qspi_xip_reader  │  │                             │  │
│  │  │  │ (XIP Read FSM)   │  │                             │  │
│  │  │  └──────────────────┘  │                             │  │
│  │  │                        │                             │  │
│  │  │  ┌──────────────────┐  │                             │  │
│  │  │  │qspi_flash_prog.. │  │                             │  │
│  │  │  │ (Program FSM)    │  │                             │  │
│  │  │  │                  │  │                             │  │
│  │  │  │  ┌────────────┐  │  │                             │  │
│  │  │  │  │ spi_master │  │  │                             │  │
│  │  │  │  └────────────┘  │  │                             │  │
│  │  │  └──────────────────┘  │                             │  │
│  │  └────────────────────────┘                             │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## RTL Modules Description

| Module | File | Description |
|--------|------|-------------|
| `qspi_xip_prog_ctrl` | qspi_xip_prog_ctrl.v | **TOP-LEVEL**: Unified AHB interface với address decode |
| `qspi_xip_prog_ctrl_ahbl` | qspi_flash_ctrl.v | XIP controller với cache và programmer |
| `qspi_xip_reader` | qspi_flash_ctrl.v | XIP Read FSM (Fast Read Quad I/O - 0xEB) |
| `qspi_dmc` | qspi_flash_ctrl.v | Direct-Mapped Cache |
| `qspi_flash_programmer_simple` | qspi_flash_ctrl.v | Program/Erase FSM |
| `spi_master` | qspi_flash_ctrl.v | SPI Master (Mode 0) |
| `qspi_prog_ahb` | qspi_prog_ahb.v | AHB Register interface cho programmer |
| `qspi_addr_decode` | qspi_addr_decode.v | Address decoder (XIP vs Register) |

## Memory Map (Default)

| Address Range | Description |
|---------------|-------------|
| 0x00000000 - 0x00FFFFFF | Flash XIP Read (24-bit address) |
| 0x00010000 + 0x00 | CTRL Register |
| 0x00010000 + 0x04 | STATUS Register |
| 0x00010000 + 0x08 | ADDR Register |
| 0x00010000 + 0x0C | LEN Register |
| 0x00010000 + 0x10 | DATA Register (FIFO) |
| 0x00010000 + 0x14 | ID Register |

## Register Description

### CTRL (0x00) - Control Register
| Bit | Name | Description |
|-----|------|-------------|
| [2:0] | OP_TYPE | Operation: 0=WEN, 1=WDIS, 2=RDSR, 3=PP, 4=SE, 5=BE32, 6=BE64, 7=RDID |
| [7] | START | Write 1 to start operation |

### STATUS (0x04) - Status Register (Read Only)
| Bit | Name | Description |
|-----|------|-------------|
| [0] | BUSY | Operation in progress |
| [1] | DONE | Operation complete (cleared on read) |
| [2] | ERROR | Error occurred |
| [15:8] | FLASH_STATUS | Flash status register value |

## Simulation

```bash
# Run simulation
make sim

# Run with GUI
make gui

# Clean
make clean
```

## Integration

Để tích hợp vào hệ thống:

```verilog
qspi_xip_prog_ctrl #(
    .NUM_LINES(32),      // Cache lines
    .LINE_SIZE(16),      // Bytes per line
    .FIFO_DEPTH(256),    // Program FIFO depth
    .REG_BASE(24'h010000) // Register base address
) u_qspi (
    .HCLK(clk),
    .HRESETn(rst_n),
    // AHB-Lite interface
    .HSEL(qspi_hsel),
    .HADDR(haddr),
    .HTRANS(htrans),
    .HWRITE(hwrite),
    .HWDATA(hwdata),
    .HREADY(hready),
    .HREADYOUT(qspi_hreadyout),
    .HRDATA(qspi_hrdata),
    .HRESP(qspi_hresp),
    // Interrupts
    .irq_done(qspi_irq_done),
    .irq_error(qspi_irq_error),
    // QSPI pins
    .sck(QSPI_SCK),
    .ce_n(QSPI_CSn),
    .din(qspi_din),
    .dout(qspi_dout),
    .douten(qspi_douten)
);
```

## Features

- ✅ XIP (Execute-in-Place) với Fast Read Quad I/O (0xEB)
- ✅ Direct-mapped cache
- ✅ Page Program
- ✅ Sector Erase (4KB)
- ✅ Block Erase (32KB, 64KB)
- ✅ Read JEDEC ID
- ✅ Read Status Register
- ✅ AHB-Lite slave interface
- ✅ Interrupt support

