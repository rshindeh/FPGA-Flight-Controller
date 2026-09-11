# ==============================================================================
# FPGA Quadcopter Flight Controller - Simulation Makefile
# Supports AMD Vivado (xsim, default) and Icarus Verilog (iverilog)
# Cross-platform: Linux, macOS, Windows (PowerShell / MSYS / Git Bash)
# ==============================================================================

SIM ?= vivado

# Auto-detect Windows and configure paths
ifeq ($(OS),Windows_NT)
    # Default Vivado install location if not already in system PATH
    VIVADO_BIN ?= C:/AMDDesignTools/2025.2.1/Vivado/bin
    export PATH := $(VIVADO_BIN);$(PATH)
    RM_CMD = powershell -NoProfile -ExecutionPolicy Bypass -Command "Remove-Item -Force -Recurse -ErrorAction SilentlyContinue *.log, *.jou, *.pb, *.wdb, *.vcd, *.out, xsim.dir, .Xil; exit 0"
    CHECK_SIM = powershell -NoProfile -Command "if (Select-String -Path xsim.log -Pattern 'Fatal:|\[FAIL\]|VIOLATIONS' -Quiet) { Write-Error 'Testbench Failure Detected in xsim.log'; exit 1 }"
else
    RM_CMD = rm -rf *.log *.jou *.pb *.wdb *.vcd *.out xsim.dir .Xil
    CHECK_SIM = ! grep -E "Fatal:|\[FAIL\]|VIOLATIONS" xsim.log
endif

# RTL Design Sources
RTL_SRCS = \
	rtl/pwm_generator.sv \
	rtl/pid_calculator.sv \
	rtl/motor_mixer.sv \
	rtl/rc_receiver.sv \
	rtl/rc_mapper.sv \
	rtl/safety_mgr.sv \
	rtl/attitude_estimator.sv \
	rtl/spi_master.sv \
	rtl/flight_core.sv

# Testbench Directory & Targets
TB_DIR = tb
TESTBENCHES = \
	safety_mgr_tb \
	rc_mapper_tb \
	pwm_generator_tb \
	motor_mixer_tb \
	pid_calculator_tb \
	rc_receiver_tb \
	attitude_estimator_tb \
	spi_master_tb \
	flight_core_tb

.PHONY: all sim test clean help synth $(TESTBENCHES)

all: test
sim: test

synth:
	vivado -mode batch -source scripts/synth.tcl -nojournal -nolog

help:
	@echo FPGA Flight Controller Verification Suite
	@echo Usage:
	@echo   make test              Run all testbenches (default: SIM=vivado)
	@echo   make sim               Alias for make test
	@echo   make synth             Run Vivado batch synthesis for Artix-7
	@echo   make clean             Remove simulation artifacts and logs
	@echo   make <tb_name>         Run specific testbench (e.g., make flight_core_tb)
	@echo.
	@echo Available testbenches:
	@echo   flight_core_tb         Full closed-loop system simulation
	@echo   attitude_estimator_tb  6-DOF complementary filter test
	@echo   pid_calculator_tb      Q8.8 fixed-point PID controller test
	@echo   motor_mixer_tb         Quad-X mixer and saturation clamping test
	@echo   rc_receiver_tb         4-channel pulse decoder and watchdog test
	@echo   spi_master_tb          MPU-6500 6 MHz SPI master test
	@echo   i2c_master_tb          MPU-6050 400 kHz I2C master test (legacy)
	@echo   pwm_generator_tb       400 Hz double-buffered PWM generator test
	@echo.
	@echo Supported simulators (override via SIM=^<simulator^>):
	@echo   SIM=vivado             AMD Vivado xvlog/xelab/xsim (default)
	@echo   SIM=iverilog           Icarus Verilog iverilog/vvp (IEEE 1800-2012)

test: $(TESTBENCHES)

# ------------------------------------------------------------------------------
# Vivado Simulator Rules (default)
# ------------------------------------------------------------------------------
ifeq ($(SIM),vivado)

safety_mgr_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/safety_mgr.sv $(TB_DIR)/safety_mgr_tb.sv
	xelab -debug typical safety_mgr_tb -s safety_sim
	xsim safety_sim -R
	@$(CHECK_SIM)

rc_mapper_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/rc_mapper.sv $(TB_DIR)/rc_mapper_tb.sv
	xelab -debug typical rc_mapper_tb -s mapper_sim
	xsim mapper_sim -R
	@$(CHECK_SIM)

flight_core_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv $(RTL_SRCS) $(TB_DIR)/flight_core_tb.sv
	xelab -debug typical flight_core_tb -s flight_sim
	xsim flight_sim -R
	@$(CHECK_SIM)

attitude_estimator_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/attitude_estimator.sv $(TB_DIR)/attitude_estimator_tb.sv
	xelab -debug typical attitude_estimator_tb -s att_sim
	xsim att_sim -R
	@$(CHECK_SIM)

pid_calculator_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/pid_calculator.sv $(TB_DIR)/pid_calculator_tb.sv
	xelab -debug typical pid_calculator_tb -s pid_sim
	xsim pid_sim -R
	@$(CHECK_SIM)

motor_mixer_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/motor_mixer.sv $(TB_DIR)/motor_mixer_tb.sv
	xelab -debug typical motor_mixer_tb -s mixer_sim
	xsim mixer_sim -R
	@$(CHECK_SIM)

rc_receiver_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/rc_receiver.sv $(TB_DIR)/rc_receiver_tb.sv
	xelab -debug typical rc_receiver_tb -s rc_sim
	xsim rc_sim -R
	@$(CHECK_SIM)

spi_master_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/spi_master.sv $(TB_DIR)/spi_master_tb.sv
	xelab -debug typical spi_master_tb -s spi_sim
	xsim spi_sim -R
	@$(CHECK_SIM)

i2c_master_tb:
	xvlog -sv rtl/i2c_master.sv $(TB_DIR)/i2c_master_tb.sv
	xelab -debug typical i2c_master_tb -s i2c_sim
	xsim i2c_sim -R

pwm_generator_tb:
	xvlog -sv $(TB_DIR)/fc_tb_pkg.sv rtl/pwm_generator.sv $(TB_DIR)/pwm_generator_tb.sv
	xelab -debug typical pwm_generator_tb -s pwm_sim
	xsim pwm_sim -R
	@$(CHECK_SIM)

# ------------------------------------------------------------------------------
# Icarus Verilog Rules
# ------------------------------------------------------------------------------
else ifeq ($(SIM),iverilog)

safety_mgr_tb:
	iverilog -g2012 -o safety_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/safety_mgr.sv $(TB_DIR)/safety_mgr_tb.sv
	vvp safety_sim.out

rc_mapper_tb:
	iverilog -g2012 -o mapper_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/rc_mapper.sv $(TB_DIR)/rc_mapper_tb.sv
	vvp mapper_sim.out

flight_core_tb:
	iverilog -g2012 -o flight_sim.out $(TB_DIR)/fc_tb_pkg.sv $(RTL_SRCS) $(TB_DIR)/flight_core_tb.sv
	vvp flight_sim.out

attitude_estimator_tb:
	iverilog -g2012 -o att_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/attitude_estimator.sv $(TB_DIR)/attitude_estimator_tb.sv
	vvp att_sim.out

pid_calculator_tb:
	iverilog -g2012 -o pid_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/pid_calculator.sv $(TB_DIR)/pid_calculator_tb.sv
	vvp pid_sim.out

motor_mixer_tb:
	iverilog -g2012 -o mixer_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/motor_mixer.sv $(TB_DIR)/motor_mixer_tb.sv
	vvp mixer_sim.out

rc_receiver_tb:
	iverilog -g2012 -o rc_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/rc_receiver.sv $(TB_DIR)/rc_receiver_tb.sv
	vvp rc_sim.out

spi_master_tb:
	iverilog -g2012 -o spi_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/spi_master.sv $(TB_DIR)/spi_master_tb.sv
	vvp spi_sim.out

i2c_master_tb:
	iverilog -g2012 -o i2c_sim.out rtl/i2c_master.sv $(TB_DIR)/i2c_master_tb.sv
	vvp i2c_sim.out

pwm_generator_tb:
	iverilog -g2012 -o pwm_sim.out $(TB_DIR)/fc_tb_pkg.sv rtl/pwm_generator.sv $(TB_DIR)/pwm_generator_tb.sv
	vvp pwm_sim.out

endif

# ------------------------------------------------------------------------------
# Clean Artifacts
# ------------------------------------------------------------------------------
clean:
	$(RM_CMD)
