# Custom FPGA Flight Controller System Specification

## 1. System Overview
This document serves as the absolute reference design configuration for the autonomous flight controller implementation. All RTL modules, testbenches, and hardware integration steps must strictly adhere to the constraints, mathematical limits, and timing architectures defined herein.

### Hardware Platform
- **Target FPGA:** Digilent CMOD A7-35T (Xilinx Artix-7 XC7A35T-1CPG236C)
- **Inertial Measurement Unit (IMU):** MPU-6050 Gyroscope/Accelerometer breakout board

---

## 2. Clocking Architecture & Timing Constraints
The master clocking framework must drive all internal logic and sample rates precisely based on a single input clock distribution network:

- **System Master Clock (clk):** 12 MHz oscillator input from the CMOD A7 board.
- **I2C Serial Clock (SCL):** Fast-Mode 400 kHz SCL line generated via a hardwired division counter from the 12 MHz clock.
- **Output PWM Frame Rate:** 400 Hz pulse-frequency square wave driving Electronic Speed Controllers (ESCs).
- **RC Receiver Target Input:** Decoding nominal 50 Hz PWM control pulses (1 ms to 2 ms durations) from a standard RC receiver.

---

## 3. High-Level Dataflow Architecture
1. **Sensor Ingestion:** The `i2c_master` module queries the MPU-6050 gyroscope registers at standard sampling loops to acquire raw angular velocity inputs.
2. **Pilot Ingestion:** The `rc_receiver` module measures incoming pilot commands from the physical controller, synchronizes signals against metastability, and computes the desired setpoint target values.
3. **Error Evaluation:** The `pid_calculator` compares pilot target values against current sensor feedback data to compute active correction vectors.
4. **Mixing & Saturation:** The `motor_mixer` blends master throttle data with the PID correction factors, runs anti-wrap truncation checks, and routes the individual channels out.
5. **Signal Drive:** The `pwm_generator` channels map raw binary parameters out to the ESC motor pins.

---

## 4. Specific Module Requirements

### A. PWM Generator (`rtl/pwm_generator.sv`)
- **Resolution:** 15-bit unsigned counter operating up to a terminal frame limit of `30,000` ticks (yielding exactly 400 Hz from a 12 MHz clock source).
- **Safety Protocol:** Implements double-buffering via an internal shadow register (`duty_cycle_buf`). Input `duty_cycle` values are only loaded into the active comparator at the exact boundary where `counter == 0` to completely eliminate mid-frame frequency glitching.

### B. I2C Controller State Machine (`rtl/i2c_master.sv`)
- **Protocol Loop:** A hardwired finite state machine handling START, Device Address broadcast, Register Pointer configuration, Repeated START, Data Read (with ACK generation), and STOP protocols.
- **Target Ingestion:** Autonomously streams 16-bit signed internal gyroscope output registers (Roll, Pitch, Yaw velocities) from the MPU-6050.

### C. PID Calculator (`rtl/pid_calculator.sv`)
- **Arithmetic Protocol:** 16-bit fixed-point signed arithmetic using a **Q8.8 representation** (8 bits for the integer part, 8 bits for the fractional component).
- **Control Constraints:** Computes proportional, integral, and derivative corrections based on `Error = Target - Actual`. 
- **Internal Sizing:** Intermediate multiplier steps must utilize full signed 32-bit arithmetic to prevent intermediate overflow before truncating back down to standard fixed-point constraints.

### D. Motor Mixer (`rtl/motor_mixer.sv`)
- **Functional Isolation:** This block is functionally separate from signal decoding modules.
- **Logic Matrix:** Blends core throttle thresholds with the Roll, Pitch, and Yaw error corrections calculated by the PID logic.
- **Saturation Controls:** Enforces rigid lower-bound and upper-bound truncation limits. If mixed channel math exceeds `30,000` or drops below `0`, the value must saturate cleanly at the absolute limits to prevent catastrophic overflow bit wrapping.

### E. RC Receiver Decoder (`rtl/rc_receiver.sv`)
- **Synchronization:** Passes all raw incoming 50 Hz asynchronous pulse lines through a dedicated 2-stage flip-flop chain to completely eliminate metastability issues.
- **Decoding:** Uses synchronous edge detection to log active pulse lengths, translating time widths (1 ms to 2 ms) into standardized 15-bit target tracking values.