# =============================================================================
# fpga/Makefile
#
# Simulate the AD5940 FPGA RTL using Icarus Verilog.
#
# Targets:
#   sim_polling  – run tb_ad5940_adc_polling (ADC polling core only)
#   sim_top      – run tb_ad5940_top (full top with UART output)
#   all          – run both testbenches
#   clean        – remove generated artefacts
# =============================================================================

IVERILOG  = iverilog
VVP       = vvp
IFLAGS    = -g2012

SIM_DIR   = /tmp

.PHONY: all sim_polling sim_top clean

all: sim_polling sim_top

# ---------------------------------------------------------------------------
# tb_ad5940_adc_polling: verify ADC polling FSM, SPI master, and adc_valid
# ---------------------------------------------------------------------------
sim_polling: $(SIM_DIR)/sim_polling.vvp
	$(VVP) $<

$(SIM_DIR)/sim_polling.vvp: ad5940_spi_master.v ad5940_adc_polling.v tb_ad5940_adc_polling.v
	$(IVERILOG) $(IFLAGS) -o $@ $^

# ---------------------------------------------------------------------------
# tb_ad5940_top: verify full design including UART serialisation of ADC data
# ---------------------------------------------------------------------------
sim_top: $(SIM_DIR)/sim_top.vvp
	$(VVP) $<

$(SIM_DIR)/sim_top.vvp: ad5940_spi_master.v ad5940_adc_polling.v uart_tx.v ad5940_top.v tb_ad5940_top.v
	$(IVERILOG) $(IFLAGS) -o $@ $^

# ---------------------------------------------------------------------------
clean:
	rm -f $(SIM_DIR)/sim_polling.vvp $(SIM_DIR)/sim_top.vvp \
	      tb_ad5940_adc_polling.vcd tb_ad5940_top.vcd
