# FPGA Quadcopter Flight Controller

A synthesizable quadcopter flight controller implemented in SystemVerilog for the Digilent Cmod A7-35T (Xilinx Artix-7 XC7A35T FPGA).

## Overview

This project implements a digital flight controller entirely in hardware. It reads IMU sensor data, decodes RC pilot commands, estimates drone attitude, runs cascaded PID control loops, and generates PWM motor signals for ESCs.

## Key Features

- **IMU Interface:** 400 kHz I2C master communicating with an MPU-6050 6-axis accelerometer and gyroscope.
- **Attitude Estimation:** Complementary filter combining accelerometer tilt and integrated gyroscope rate data.
- **Cascaded PID Control:**
  - Outer loop: Angle controller converting tilt error to target angular rate.
  - Inner loop: Rate PID controller calculating motor corrections for roll, pitch, and yaw.
- **RC Input:** 4-channel PWM pulse width decoder (Roll, Pitch, Yaw, Throttle) with synchronization and signal-loss watchdog.
- **Motor Mixer:** Quad-X mixing matrix that maps throttle and attitude corrections to 4 motor outputs.
- **PWM Motor Outputs:** 400 Hz PWM generation with double-buffered duty cycle registers to prevent output glitches.
- **Safety and Arming:** Stick-based arming sequence and failsafe shutdown on signal loss.

## Project Structure

- `rtl/`: SystemVerilog RTL design files
  - `flight_core.sv`: Top-level module integrating the full flight controller
  - `attitude_estimator.sv`: Complementary filter for attitude estimation
  - `i2c_master.sv`: I2C master interface for the MPU-6050
  - `motor_mixer.sv`: Quad-X motor mixing logic
  - `pid_calculator.sv`: Fixed-point PID controller
  - `pwm_generator.sv`: 400 Hz PWM generator for ESCs
  - `rc_mapper.sv`: Stick scaling and deadband processing
  - `rc_receiver.sv`: PWM pulse width measurement for RC receiver
  - `safety_mgr.sv`: Arming/disarming state machine and safety interlocks
- `tb/`: Simulation testbenches for individual modules and the full system
- `constraints/`: Timing and pin constraints (`cmod_a7_pins.xdc`) for the Digilent Cmod A7-35T

## Target Hardware

- **FPGA Board:** Digilent Cmod A7-35T (Xilinx Artix-7 XC7A35T)
- **IMU:** MPU-6050 (I2C)
- **RC Receiver:** Standard 4-channel PWM receiver (1.0 ms to 2.0 ms pulse width, 50 Hz)
- **ESCs / Motors:** Standard 400 Hz PWM ESCs in Quad-X configuration

## Running Simulations

Simulations can be compiled and run using AMD Vivado command-line tools:

### Full System Simulation
```powershell
xvlog -sv rtl/*.sv tb/flight_core_tb.sv
xelab -debug typical flight_core_tb -s flight_sim
xsim flight_sim -R
```

### Module-Level Simulation Example (Attitude Estimator)
```powershell
xvlog -sv rtl/attitude_estimator.sv tb/attitude_estimator_tb.sv
xelab -debug typical attitude_estimator_tb -s att_sim
xsim att_sim -R
```
