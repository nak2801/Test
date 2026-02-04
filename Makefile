# QSPI Flash Controller Module
# Makefile for simulation with Simple or Winbond W25Q128JVxIM Flash Model

# Simulator
VSIM = vsim
VLOG = vlog

# Directories  
RTL_DIR = rtl
TB_DIR  = tb
WORK    = work

# RTL Files (in dependency order)
RTL_FILES = \
	$(RTL_DIR)/qspi_flash_ctrl.v \
	$(RTL_DIR)/qspi_prog_ahb.v \
	$(RTL_DIR)/qspi_xip_prog_ctrl.v \
	$(RTL_DIR)/qspi_addr_decode.v

# Flash Model - Winbond W25Q128JVxIM Official Model
FLASH_MODEL = W25Q128JVxIM.v

# Testbench Files
TB_FILES = $(TB_DIR)/tb_qspi_xip_prog.v
TB_WINBOND = $(TB_DIR)/tb_qspi_winbond.v

# Top module
TB_TOP = tb_qspi_xip_prog
TB_TOP_WINBOND = tb_qspi_winbond

# Simulation options
VSIM_OPTS = -c -do "run -all; quit"
VLOG_OPTS = +acc

.PHONY: all compile sim sim_winbond compile_vcs sim_vcs compile_vcs_winbond sim_vcs_winbond clean help

all: sim

#----------------------------------------------------------------------
# ModelSim/Questa - Simple Flash Model
#----------------------------------------------------------------------
$(WORK):
	vlib $(WORK)

compile: $(WORK)
	$(VLOG) $(VLOG_OPTS) -work $(WORK) $(RTL_FILES) $(TB_FILES)

sim: compile
	$(VSIM) $(VSIM_OPTS) -work $(WORK) $(TB_TOP)

gui: compile
	$(VSIM) -work $(WORK) $(TB_TOP)

#----------------------------------------------------------------------
# ModelSim/Questa - Winbond W25Q128JVxIM Flash Model  
#----------------------------------------------------------------------
compile_winbond: $(WORK)
	@echo "=== Compiling with Winbond W25Q128JVxIM Model ==="
	$(VLOG) $(VLOG_OPTS) -work $(WORK) $(RTL_FILES) $(FLASH_MODEL) $(TB_WINBOND)

sim_winbond: compile_winbond
	@echo "=== Running with Winbond W25Q128JVxIM Model ==="
	$(VSIM) $(VSIM_OPTS) -work $(WORK) $(TB_TOP_WINBOND)

gui_winbond: compile_winbond
	@echo "=== Running GUI with Winbond W25Q128JVxIM Model ==="
	$(VSIM) -work $(WORK) $(TB_TOP_WINBOND)

#----------------------------------------------------------------------
# VCS (Verilog Compiler Simulator) - Simple Flash Model
#----------------------------------------------------------------------
compile_vcs:
	@echo "=== Compiling with VCS ==="
	vcs -full64 -sverilog $(RTL_FILES) $(TB_FILES) -o simv

sim_vcs: compile_vcs
	@echo "=== Running Simulation ==="
	./simv

gui_vcs: compile_vcs
	@echo "=== Running GUI with VCS ==="
	./simv -gui

#----------------------------------------------------------------------
# VCS (Verilog Compiler Simulator) - Winbond W25Q128JVxIM Flash Model
#----------------------------------------------------------------------
compile_vcs_winbond:
	@echo "=== Compiling with VCS (Winbond Model) ==="
	vcs -full64 -sverilog $(RTL_FILES) $(FLASH_MODEL) $(TB_WINBOND) -o simv_winbond

sim_vcs_winbond: compile_vcs_winbond
	@echo "=== Running Simulation ==="
	./simv_winbond

gui_vcs_winbond: compile_vcs_winbond
	@echo "=== Running GUI with VCS (Winbond Model) ==="
	./simv_winbond -gui

#----------------------------------------------------------------------
# Icarus Verilog
#----------------------------------------------------------------------
iverilog:
	iverilog -g2012 -o sim.vvp -s $(TB_TOP) $(RTL_FILES) $(TB_FILES)
	vvp sim.vvp

iverilog_winbond:
	@echo "=== Compiling with Icarus Verilog (Winbond Model) ==="
	iverilog -g2012 -o sim_winbond.vvp -s $(TB_TOP_WINBOND) $(RTL_FILES) $(FLASH_MODEL) $(TB_WINBOND)
	@echo "=== Running Simulation ==="
	vvp sim_winbond.vvp

#----------------------------------------------------------------------
# Utilities
#----------------------------------------------------------------------
wave:
	gtkwave tb_qspi_winbond.vcd &

clean:
	rm -rf $(WORK)
	rm -f *.vcd *.vvp *.wlf *.log *.vpd
	rm -f simv simv_winbond
	rm -rf simv.daidir simv_winbond.daidir
	rm -f transcript

help:
	@echo ""
	@echo "QSPI Flash Controller Makefile"
	@echo "=============================="
	@echo ""
	@echo "ModelSim/Questa (Simple Flash Model):"
	@echo "  make sim       - Run simulation (CLI)"
	@echo "  make gui       - Run simulation (GUI)"
	@echo ""
	@echo "ModelSim/Questa (Winbond W25Q128JVxIM):"
	@echo "  make sim_winbond  - Run with Winbond model (CLI)"
	@echo "  make gui_winbond  - Run with Winbond model (GUI)"
	@echo ""
	@echo "VCS (Simple Flash Model):"
	@echo "  make compile_vcs  - Compile only"
	@echo "  make sim_vcs      - Compile and run simulation (CLI)"
	@echo "  make gui_vcs      - Compile and run with GUI"
	@echo ""
	@echo "VCS (Winbond W25Q128JVxIM):"
	@echo "  make compile_vcs_winbond  - Compile only"
	@echo "  make sim_vcs_winbond      - Compile and run with Winbond model (CLI)"
	@echo "  make gui_vcs_winbond      - Compile and run with Winbond model (GUI)"
	@echo ""
	@echo "Icarus Verilog:"
	@echo "  make iverilog         - Simple flash model"
	@echo "  make iverilog_winbond - Winbond W25Q128JVxIM model"
	@echo ""
	@echo "Utilities:"
	@echo "  make wave    - Open waveform in GTKWave"
	@echo "  make clean   - Remove generated files"
	@echo ""
