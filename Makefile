# ==============================================================================
# FPGA Quadcopter Flight Controller - Simulation Makefile
# Supports AMD Vivado (xsim, default) and Icarus Verilog (iverilog)
# ==============================================================================

SIM ?= vivado

# RTL Design Sources
RTL_SRCS = \
	rtl/pwm_generator.sv \
	rtl/pid_calculator.sv \
	rtl/motor_mixer.sv \
	rtl/rc_receiver.sv \
	rtl/rc_mapper.sv \
	rtl/safety_mgr.sv \
	rtl/attitude_estimator.sv \
	rtl/i2c_master.sv \
	rtl/flight_core.sv

# Testbench Directory & Names
TB_DIR = tb
TESTBENCHES = \
	flight_core_tb \
	attitude_estimator_tb \
	pid_calculator_tb \
	motor_mixer_tb \
	rc_receiver_tb \
	i2c_master_tb \
	pwm_generator_tb

.PHONY: all sim test clean help $(TESTBENCHES)

all: test
sim: test

help:
	@echo "FPGA Flight Controller Verification Suite"
	@echo "Usage:"
	@echo "  make test              Run all testbenches (default: SIM=vivado)"
	@echo "  make sim               Alias for make test"
	@echo "  make <tb_name>         Run specific testbench (e.g., make flight_core_tb)"
	@echo "  make clean             Remove simulation artifacts and logs"
	@echo ""
	@echo "Available testbenches:"
	@echo "  flight_core_tb         Full closed-loop system simulation"
	@echo "  attitude_estimator_tb  6-DOF complementary filter test"
	@echo "  pid_calculator_tb      Q8.8 fixed-point PID controller test"
	@echo "  motor_mixer_tb         Quad-X mixer and saturation clamping test"
	@echo "  rc_receiver_tb         4-channel pulse decoder and watchdog test"
	@echo "  i2c_master_tb          MPU-6050 400 kHz I2C master test"
	@echo "  pwm_generator_tb       400 Hz double-buffered PWM generator test"
	@echo ""
	@echo "Supported simulators (override via SIM=<simulator>):"
	@echo "  SIM=vivado             AMD Vivado xvlog/xelab/xsim (default)"
	@echo "  SIM=iverilog           Icarus Verilog iverilog/vvp (IEEE 1800-2012)"

test: $(TESTBENCHES)

# ------------------------------------------------------------------------------
# Vivado Simulator Rules (default)
# ------------------------------------------------------------------------------
ifeq ($(SIM),vivado)

flight_core_tb:
	xvlog -sv $(RTL_SRCS) $(TB_DIR)/flight_core_tb.sv
	xelab -debug typical flight_core_tb -s flight_sim
	xsim flight_sim -R

attitude_estimator_tb:
	xvlog -sv rtl/attitude_estimator.sv $(TB_DIR)/attitude_estimator_tb.sv
	xelab -debug typical attitude_estimator_tb -s att_sim
	xsim att_sim -R

pid_calculator_tb:
	xvlog -sv rtl/pid_calculator.sv $(TB_DIR)/pid_calculator_tb.sv
	xelab -debug typical pid_calculator_tb -s pid_sim
	xsim pid_sim -R

motor_mixer_tb:
	xvlog -sv rtl/motor_mixer.sv $(TB_DIR)/motor_mixer_tb.sv
	xelab -debug typical motor_mixer_tb -s mixer_sim
	xsim mixer_sim -R

rc_receiver_tb:
	xvlog -sv rtl/rc_receiver.sv $(TB_DIR)/rc_receiver_tb.sv
	xelab -debug typical rc_receiver_tb -s rc_sim
	xsim rc_sim -R

i2c_master_tb:
	xvlog -sv rtl/i2c_master.sv $(TB_DIR)/i2c_master_tb.sv
	xelab -debug typical i2c_master_tb -s i2c_sim
	xsim i2c_sim -R

pwm_generator_tb:
	xvlog -sv rtl/pwm_generator.sv $(TB_DIR)/pwm_generator_tb.sv
	xelab -debug typical pwm_generator_tb -s pwm_sim
	xsim pwm_sim -R

# ------------------------------------------------------------------------------
# Icarus Verilog Rules
# ------------------------------------------------------------------------------
else ifeq ($(SIM),iverilog)

flight_core_tb:
	iverilog -g2012 -o flight_sim.out $(RTL_SRCS) $(TB_DIR)/flight_core_tb.sv
	vvp flight_sim.out

attitude_estimator_tb:
	iverilog -g2012 -o att_sim.out rtl/attitude_estimator.sv $(TB_DIR)/attitude_estimator_tb.sv
	vvp att_sim.out

pid_calculator_tb:
	iverilog -g2012 -o pid_sim.out rtl/pid_calculator.sv $(TB_DIR)/pid_calculator_tb.sv
	vvp pid_sim.out

motor_mixer_tb:
	iverilog -g2012 -o mixer_sim.out rtl/motor_mixer.sv $(TB_DIR)/motor_mixer_tb.sv
	vvp mixer_sim.out

rc_receiver_tb:
	iverilog -g2012 -o rc_sim.out rtl/rc_receiver.sv $(TB_DIR)/rc_receiver_tb.sv
	vvp rc_sim.out

i2c_master_tb:
	iverilog -g2012 -o i2c_sim.out rtl/i2c_master.sv $(TB_DIR)/i2c_master_tb.sv
	vvp i2c_sim.out

pwm_generator_tb:
	iverilog -g2012 -o pwm_sim.out rtl/pwm_generator.sv $(TB_DIR)/pwm_generator_tb.sv
	vvp pwm_sim.out

endif

# ------------------------------------------------------------------------------
# Clean Artifacts
# ------------------------------------------------------------------------------
clean:
	rm -rf *.log *.jou *.pb *.wdb *.vcd *.out xsim.dir .Xil
