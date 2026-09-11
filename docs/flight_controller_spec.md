# Custom FPGA Flight Controller System Specification

## 1. System Overview
This document serves as the absolute reference design configuration for the autonomous flight controller implementation. All RTL modules, testbenches, and hardware integration steps must strictly adhere to the constraints, mathematical limits, and timing architectures defined herein.

### Hardware Platform
- **Target FPGA:** Digilent CMOD A7-35T (Xilinx Artix-7 XC7A35T-1CPG236C)
- **Inertial Measurement Unit (IMU):** InvenSense MPU-6500 6-DOF Gyroscope/Accelerometer (4-Wire SPI)
- **Electronic Speed Controllers (ESCs):** Standard 400 Hz PWM (1.0 ms to 2.0 ms active pulse width)

---

## 2. Clocking Architecture & Timing Constraints
The master clocking framework drives all internal logic and sample rates based on a single input clock distribution network:

- **System Master Clock (`clk`):** 12 MHz oscillator input from the CMOD A7 board.
- **SPI Serial Clock (`spi_sclk`):** High-speed 6.0 MHz SCLK line generated synchronously via clock enable strobes (1.0 MHz during power-on register initialization).
- **Sensor Loop Rate:** 1 kHz periodic burst read loop with $\approx 20\ \mu\text{s}$ transaction duration (98% bus idle headroom).
- **Output PWM Frame Rate:** 400 Hz pulse-frequency square wave driving ESCs (2.5 ms frame period = 30,000 ticks).
- **RC Receiver Target Input:** Decoding nominal 50 Hz PWM control pulses (1.0 ms to 2.0 ms durations = 12,000 to 24,000 ticks) from a standard RC receiver.

---

## 3. High-Level Dataflow & Cascaded Control Architecture
1. **Sensor Ingestion (`spi_master.sv`):** Streams the MPU-6500 in 15-byte continuous SPI bursts (1 command byte `0xBB` + 14 data bytes) starting at `0x3B` (`ACCEL_XOUT_H`) to acquire Accelerometer X/Y/Z, Temperature, and Gyroscope X/Y/Z readings at 1 kHz with sub-$25\ \mu\text{s}$ transport delay.
2. **Pilot Ingestion & Synchronization (`rc_receiver.sv`):** Synchronizes raw incoming RC pulses through a 2-stage flip-flop chain for metastability immunity, decodes pulse widths with synchronous edge detection, and implements a 100 ms lost-signal watchdog.
3. **Setpoint Mapping (`rc_mapper.sv`):** Centers sticks around 1.5 ms with a $\pm 100\text{ tick}$ deadband, generating target roll/pitch tilt angles ($\pm 30.0^\circ$) and target yaw rate ($\pm 100.0^\circ/\text{s}$).
4. **Safety & Arming Management (`safety_mgr.sv`):** Evaluates stick sequences for arming (Throttle Min + Yaw Full Right for 1.0s) and disarming (Throttle Min + Yaw Full Left). Provides ground-idle PID integral clearing.
5. **Attitude Estimation (`attitude_estimator.sv`):** 6-DOF Complementary Filter executing in 32-bit Q16.16 fixed-point math, fusing gyro integration with accelerometer gravity vectors to produce zero-drift `roll_angle` and `pitch_angle` outputs in Q8.8 format.
6. **Cascaded Dual-Loop PID Control (`flight_core.sv`):**
   - **Outer Angle P-Loop:** Compares target angle against estimated angle, producing demanded angular rates ($^\circ/\text{s}$).
   - **Inner Rate PID-Loop:** Compares demanded rate against raw gyroscope rate, computing dynamic differential thrust corrections with anti-windup clamping.
7. **Mixing & Saturation (`motor_mixer.sv`):** Blends collective throttle with Roll, Pitch, and Yaw PID corrections according to the Quad-X matrix, strictly clamping outputs to standard ESC bounds $[12000, 24000]\text{ ticks}$.
8. **Signal Drive (`pwm_generator.sv`):** Generates 400 Hz ESC pulses with synchronous double-buffering at `counter == 0` to prevent mid-frame glitching.

---

## 4. Specific Module Requirements

### A. PWM Generator (`rtl/pwm_generator.sv`)
- **Resolution:** 15-bit unsigned counter operating up to a terminal frame limit of `30,000` ticks (yielding exactly 400 Hz from a 12 MHz clock source).
- **Safety Protocol:** Implements double-buffering via an internal shadow register (`duty_cycle_buf`). Input `duty_cycle` values are only loaded into the active comparator at the exact boundary where `counter == 0` to completely eliminate mid-frame frequency glitching.

### B. SPI Master (`rtl/spi_master.sv`)
- **Protocol Loop:** Synchronous SPI Mode 0 master executing automated startup power management (disabling I2C via `USER_CTRL` bit 4, waking up PLL clock source in `PWR_MGMT_1`, configuring full-scale ranges, and validating WHO_AM_I).
- **Target Ingestion:** Continuously streams 16-bit signed Accelerometer X/Y/Z, Temperature, and Gyroscope X/Y/Z registers starting at `0x3B` (`0x80 | 0x3B = 0xBB`) at 6 MHz SCLK.

### C. Attitude Estimator (`rtl/attitude_estimator.sv`)
- **Arithmetic Precision:** 32-bit Q16.16 signed fixed-point accumulators to eliminate truncation drift.
- **Output Format:** 16-bit signed Q8.8 fixed-point ($1.0^\circ = 256\text{ units}$).

### D. PID Calculator (`rtl/pid_calculator.sv`)
- **Arithmetic Protocol:** 16-bit fixed-point signed arithmetic using a **Q8.8 representation** (8 bits for integer, 8 bits for fractional component).
- **Anti-Windup Protection:** Integral accumulator is clamped to 16-bit signed limits ($\pm 32767$), and zeroed upon landing/idle (`clear_i = 1`) to eliminate on-ground motor spool-up.

### E. Motor Mixer (`rtl/motor_mixer.sv`)
- **Logic Matrix:** Quad-X mixing matrix blending collective throttle with Roll, Pitch, and Yaw error corrections.
- **Saturation Controls:** Enforces rigid lower-bound ($12,000\text{ ticks} = 1.0\text{ ms}$) and upper-bound ($24,000\text{ ticks} = 2.0\text{ ms}$) limits. Disarmed state rigidly locks all motor outputs to $12,000\text{ ticks}$.

### F. RC Receiver Decoder (`rtl/rc_receiver.sv`)
- **Synchronization:** Passes all raw incoming 50 Hz asynchronous pulse lines through a dedicated 2-stage flip-flop chain to completely eliminate metastability issues.
- **Decoding:** Uses synchronous edge detection to log active pulse lengths, translating time widths (1.0 ms to 2.0 ms) into standardized 15-bit target tracking values.
