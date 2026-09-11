set_param general.maxThreads 8
read_verilog -sv rtl/pwm_generator.sv
read_verilog -sv rtl/pid_calculator.sv
read_verilog -sv rtl/motor_mixer.sv
read_verilog -sv rtl/rc_receiver.sv
read_verilog -sv rtl/rc_mapper.sv
read_verilog -sv rtl/safety_mgr.sv
read_verilog -sv rtl/attitude_estimator.sv
read_verilog -sv rtl/spi_master.sv
read_verilog -sv rtl/flight_core.sv
read_xdc constraints/cmod_a7_pins.xdc
synth_design -top flight_core -part xc7a35tcpg236-1
report_utilization -file scripts/utilization.rpt
report_timing_summary -file scripts/timing.rpt
exit
