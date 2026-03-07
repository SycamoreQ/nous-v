VERILATOR     = verilator
SBT           = sbt
CHISEL_TOP    = fetch.iFetch
GEN_DIR       = gen
SIM_DIR       = sim_build
TB_CPP        = testbench.cpp
TB_SV         = sim_mem.sv
VERILOG_TOP   = $(GEN_DIR)/iFetch.sv

# ----------------------------------------------------------------
#  Step 1 — Generate Verilog from Chisel
# ----------------------------------------------------------------
$(VERILOG_TOP):
	mkdir -p $(GEN_DIR)
	$(SBT) "runMain circt.stage.ChiselMain \
		--target-dir $(GEN_DIR) \
		--top-name iFetch \
		fetch.iFetch"

gen: $(VERILOG_TOP)

# ----------------------------------------------------------------
#  Step 2 — Verilate
# ----------------------------------------------------------------
$(SIM_DIR)/ViFetch.mk: $(VERILOG_TOP) $(TB_SV)
	mkdir -p $(SIM_DIR)
	$(VERILATOR) \
		--cc \
		--exe $(TB_CPP) \
		--sv \
		--top-module iFetch \
		--Mdir $(SIM_DIR) \
		--trace \
		--assert \
		+1800-2017ext+sv \
		-I$(GEN_DIR) \
		$(TB_SV) \
		$(VERILOG_TOP)

verilate: $(SIM_DIR)/ViFetch.mk

# ----------------------------------------------------------------
#  Step 3 — Build simulation binary
# ----------------------------------------------------------------
$(SIM_DIR)/ViFetch: $(SIM_DIR)/ViFetch.mk $(TB_CPP)
	$(MAKE) -C $(SIM_DIR) -f ViFetch.mk

build: $(SIM_DIR)/ViFetch

# ----------------------------------------------------------------
#  Step 4 — Run
# ----------------------------------------------------------------
run: $(SIM_DIR)/ViFetch
	cd $(SIM_DIR) && ./ViFetch

# ----------------------------------------------------------------
#  Waveform viewer (GTKWave)
# ----------------------------------------------------------------
waves:
	gtkwave $(SIM_DIR)/ifetch_sim.vcd &

# ----------------------------------------------------------------
#  All-in-one
# ----------------------------------------------------------------
all: gen verilate build run

clean:
	rm -rf $(GEN_DIR) $(SIM_DIR) ifetch_sim.vcd

.PHONY: gen verilate build run waves all clean
