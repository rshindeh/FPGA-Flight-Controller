## ============================================================================
## Physical Constraints (XDC) for Digilent CMOD A7-35T
## Target Part: xc7a35tcpg236-1
## Target Design: FPGA Quadcopter Flight Controller (Self-Leveling / Cascaded PID)
## ============================================================================

## 1. Primary 12 MHz Master Clock Input (On-board Oscillator)
set_property -dict { PACKAGE_PIN L17   IOSTANDARD LVCMOS33 } [get_ports { clk }];
create_clock -add -name sys_clk_pin -period 83.333 -waveform {0 41.667} [get_ports { clk }];

## 2. Active-Low Synchronized Reset (Mapped to On-Board Button 0 with PULLUP)
set_property -dict { PACKAGE_PIN A18   IOSTANDARD LVCMOS33 } [get_ports { rst_n }];

## 3. 4-Wire SPI Interface to MPU-6500 6-DOF IMU (Mode 0, 6 MHz Streaming)
set_property -dict { PACKAGE_PIN M3    IOSTANDARD LVCMOS33 } [get_ports { spi_sclk }]; # PIO1: Serial Clock
set_property -dict { PACKAGE_PIN L3    IOSTANDARD LVCMOS33 } [get_ports { spi_mosi }]; # PIO2: Master Out Slave In
set_property -dict { PACKAGE_PIN J1    IOSTANDARD LVCMOS33 PULLUP true } [get_ports { spi_miso }]; # PIO11: Master In Slave Out
set_property -dict { PACKAGE_PIN K2    IOSTANDARD LVCMOS33 } [get_ports { spi_cs_n }]; # PIO12: Active-Low Chip Select

## 4. RC Receiver PWM Inputs (50 Hz nominal pulse inputs, 1.0 - 2.0 ms)
set_property -dict { PACKAGE_PIN A16   IOSTANDARD LVCMOS33 } [get_ports { rc_inputs[0] }]; # PIO3: Roll Input
set_property -dict { PACKAGE_PIN K3    IOSTANDARD LVCMOS33 } [get_ports { rc_inputs[1] }]; # PIO4: Pitch Input
set_property -dict { PACKAGE_PIN C15   IOSTANDARD LVCMOS33 } [get_ports { rc_inputs[2] }]; # PIO5: Yaw Input
set_property -dict { PACKAGE_PIN H1    IOSTANDARD LVCMOS33 } [get_ports { rc_inputs[3] }]; # PIO6: Throttle Input

## 5. ESC Motor PWM Outputs (400 Hz Frame Rate, 1.0 - 2.0 ms Duty Cycle)
set_property -dict { PACKAGE_PIN A15   IOSTANDARD LVCMOS33 } [get_ports { esc_pwm_outputs[0] }]; # PIO7: Motor 1 (Front Right CCW)
set_property -dict { PACKAGE_PIN B15   IOSTANDARD LVCMOS33 } [get_ports { esc_pwm_outputs[1] }]; # PIO8: Motor 2 (Rear Right CW)
set_property -dict { PACKAGE_PIN A14   IOSTANDARD LVCMOS33 } [get_ports { esc_pwm_outputs[2] }]; # PIO9: Motor 3 (Rear Left CCW)
set_property -dict { PACKAGE_PIN J3    IOSTANDARD LVCMOS33 } [get_ports { esc_pwm_outputs[3] }]; # PIO10: Motor 4 (Front Left CW)

## 6. Timing & CDC Constraints
set_false_path -from [get_ports { rc_inputs[*] }] -to [all_registers]
