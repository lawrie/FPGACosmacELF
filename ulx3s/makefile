VERILOG = ../TopLevel.v pll.v

FPGA_SIZE ?= 12

FPGA_KS ?= $(FPGA_SIZE)k

ifeq ($(FPGA_SIZE), 12)
	CHIP_ID=0x21111043
	FPGA_KS = 25k
endif
ifeq ($(FPGA_SIZE), 25)
	CHIP_ID=0x41111043
endif
ifeq ($(FPGA_SIZE), 45)
	CHIP_ID=0x41112043
endif
ifeq ($(FPGA_SIZE), 85)
	CHIP_ID=0x41113043
endif

IDCODE ?= $(CHIP_ID)

compile : bin/toplevel.bit

prog: bin/toplevel.bit
	ujprog $<

generate:
	(cd ..; sbt "runMain Spinal1802.TopLevelGen")

sim:
	(cd ..; sbt "runMain Spinal1802.CPD1802_OPCode_Verification")

bin/toplevel.json: ${VERILOG}
	mkdir -p bin
	yosys \
		-p "synth_ecp5 -json $@" \
		${VERILOG}

bin/toplevel.config: bin/toplevel.json
	nextpnr-ecp5 \
		--json $< \
		--textcfg $@ \
		--lpf ulx3s.lpf \
		--$(FPGA_KS) \
		--freq 40 \
		--package CABGA381 \
		--lpf-allow-unconstrained

bin/toplevel.bit: bin/toplevel.config
	ecppack --compress --idcode $(IDCODE) $< $@

pll:
	ecppll \
                -i 25 \
                -o 8 \
                -n  PLL\
                -f pll.v
clean:
	$(RM) -rf bin *.bin
