# Walkthrough - Flight Controller Module Design & Verification

This document summarizes the design, implementation, and verification of both the `rc_receiver.sv` and `motor_mixer.sv` flight controller modules.

---

## 1. RC Receiver (`rtl/rc_receiver.sv`)

### Changes Made
- **Metastability Protection:** Added a 2-stage D-type flip-flop synchronizer chain per channel to safely resolve pilot's radio controller PWM signals into the 12 MHz clock domain.
- **Edge Detection:** Implemented rising and falling synchronous edge detectors on the synchronized signal.
- **Duration Counting:** Added a 16-bit measurement counter that records ticks (83.33 ns each) from posedge to negedge of the input pulse.
- **Input Validation & Clamping:** Checks if the measured width falls within a 10% tolerance margin (~900 us to ~2300 us). Clamps valid inputs to standard bounds `[1.0 ms, 2.0 ms]`, corresponding directly to `[12000, 24000]` clock ticks. 
- **Single-Pulse Timeout:** Clamps output and invalidates if a single pulse stays high longer than 3.0 ms to prevent FSM lockup.
- **Failsafe Watchdog Timer:** Implemented a watchdog timer (configurable, default 100 ms) per channel. If no clean pulse is successfully completed within the watchdog period, the channel's output registers clamp to a safe default (neutral `18000` for roll/pitch/yaw; minimum `12000` for throttle) and de-asserts the channel's `valid` status.

### Verification Results
Run using Vivado Simulator inside the `./vivado/cli_workspace/` directory:
```
[TB] Starting RC Receiver Testbench...
[TB] Reset released. Initial valid = 0000
[TB] PASS: Initial default values correct (Roll=18000, Throttle=12000, valid=0000)
[TB] Channel 0: Sending 1.50 ms pulse...
[TB] Channel 0: Sending 1.50 ms pulse...
[TB] Channel 3: Sending 1.00 ms pulse...
[TB] Channel 3: Sending 1.00 ms pulse...
[TB] After normal pulses: Roll (Ch0) = 17997 (valid=1), Throttle (Ch3) = 12000 (valid=1)
[TB] PASS: Normal pulse widths decoded successfully
[TB] Channel 3: Sending 0.95 ms pulse...
[TB] Channel 0: Sending 2.05 ms pulse...
[TB] After clamping check: Roll (Ch0) = 24000 (valid=1), Throttle (Ch3) = 12000 (valid=1)
[TB] PASS: Clamping limits enforced correctly
[TB] Channel 0: Sending 0.80 ms pulse...
[TB] After OOB Low pulse: Roll (Ch0) = 18000, valid[0] = 0
[TB] PASS: Out-of-bounds low pulse correctly handled
[TB] Channel 0: Sending 1.50 ms pulse...
[TB] Channel 0: Sending 1.50 ms pulse...
[TB] Re-established Roll (Ch0) = 17997, valid[0] = 1
[TB] Channel 0: Sending 4.00 ms pulse...
[TB] After pulse timeout check: Roll (Ch0) = 18000, valid[0] = 0
[TB] PASS: Pulse width timeout triggered and handled successfully
[TB] Channel 0: Sending 1.50 ms pulse...
[TB] Channel 0: Sending 1.50 ms pulse...
[TB] Channel 3: Sending 1.20 ms pulse...
[TB] Channel 3: Sending 1.20 ms pulse...
[TB] Re-established Ch0/Ch3 before watchdog test: valid = 1001
[TB] Waiting 55 ms to test Watchdog failsafe...
[TB] After watchdog timeout: Roll (Ch0) = 18000, valid[0] = 0 | Throttle (Ch3) = 12000, valid[3] = 0
[TB] PASS: Watchdog timeout successfully de-asserted valid outputs and forced defaults
[TB] Simulation completed successfully.
```

---

## 2. Motor Mixer (`rtl/motor_mixer.sv`)

### Changes Made
- **Quad-X Mixing Matrix:** Designed a combinational arithmetic module executing standard Quadcopter-X mixing equations:
  - `Motor 1 = Throttle - Roll - Pitch + Yaw`
  - `Motor 2 = Throttle - Roll + Pitch - Yaw`
  - `Motor 3 = Throttle + Roll + Pitch + Yaw`
  - `Motor 4 = Throttle + Roll - Pitch - Yaw`
- **Safe Bit-Width Extension:** Casts throttle input (15-bit unsigned) and PID correction factor inputs (16-bit signed) into `32-bit signed` registers before performing arithmetic, completely eliminating wraparound overflow bugs.
- **Strict Clamping Logic:** Combinatorially restricts the motor outputs within the exact limits of `[0, 30000]`. Raw calculated values below `0` clamp to `0` to prevent underflow, and values above `30,000` clamp to `30,000` to prevent overflow.

### Verification Results
Run using Vivado Simulator inside `./vivado/cli_workspace/`:
```powershell
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xvlog.bat" --sv ../../rtl/motor_mixer.sv ../../tb/motor_mixer_tb.sv
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat" -timescale 1ns/1ps -debug typical motor_mixer_tb -s motor_mixer_sim
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat" motor_mixer_sim -R
```

### Log Output
```
[TB] Starting Motor Mixer Saturation Verification...
[TB] --- Test Case 1: Neutral throttle (18,000) and zero corrections ---
[TB] Inputs: Throttle = 18000, Roll = 0, Pitch = 0, Yaw = 0
[TB] Outputs: Motor1 = 18000, Motor2 = 18000, Motor3 = 18000, Motor4 = 18000
[TB] PASS: Neutral test case matched exactly 18,000.
[TB] --- Test Case 2: Extreme Over-saturation ---
[TB] Inputs: Throttle = 24000, Roll = -4000, Pitch = -4000, Yaw = 4000
[TB] Outputs: Motor1 = 30000, Motor2 = 20000, Motor3 = 20000, Motor4 = 20000
[TB] PASS: Motor 1 over-saturation clamped exactly to 30,000 without wrap-around.
[TB] Inputs: Throttle = 29000, Roll = 8000, Pitch = 8000, Yaw = 8000
[TB] Outputs: Motor1 = 21000, Motor2 = 21000, Motor3 = 30000, Motor4 = 21000
[TB] PASS: Motor 3 over-saturation clamped exactly to 30,000.
[TB] --- Test Case 3: Extreme Under-saturation ---
[TB] Inputs: Throttle = 12000, Roll = 6000, Pitch = 6000, Yaw = -6000
[TB] Outputs: Motor1 = 0, Motor2 = 18000, Motor3 = 18000, Motor4 = 18000
[TB] PASS: Motor 1 under-saturation clamped exactly to 0 without wrap-around.
[TB] Inputs: Throttle = 5000, Roll = -10000, Pitch = -10000, Yaw = -10000
[TB] Outputs: Motor1 = 15000, Motor2 = 15000, Motor3 = 0, Motor4 = 15000
[TB] PASS: Motor 3 under-saturation clamped exactly to 0.
[TB] Motor Mixer Verification completed successfully.
```
All saturation limits were aggressively verified, demonstrating zero underflow or overflow wrapping.

---

## 3. PID Calculator (`rtl/pid_calculator.sv`)

### Changes Made
- **Q8.8 Fixed-Point Architecture:** Implemented standard independent Proportional, Integral, and Derivative (PID) tracking using 16-bit Q8.8 fixed-point arithmetic (`Correction = P*Error + I*Integral + D*Derivative`).
- **Integral Anti-Windup:** Developed a continuous cumulative accumulator bounded against extreme saturation (`INT_MAX=32767` and `INT_MIN=-32768`).
- **32-Bit Multiplication:** Expanded multiplication nodes (e.g. `Error * Gain`) internally to `32-bit signed` registers, yielding `Q16.16` intermediate formatting before being arithmetically right-shifted back to 16-bit format (`Q24.8` truncated).
- **Hard Clamping:** Scaled outputs are safely saturated prior to truncation to prevent unexpected wraparound when exceeding maximum allowable bounds.

### Verification Results
Run using Vivado Simulator inside `./vivado/cli_workspace/`:
```powershell
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xvlog.bat" --sv ../../rtl/pid_calculator.sv ../../tb/pid_calculator_tb.sv
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat" -timescale 1ns/1ps -debug typical pid_calculator_tb -s pid_calculator_sim
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat" pid_calculator_sim -R
```

### Log Output
```
[TB] Starting PID Calculator Verification...
[TB] --- Test Case 1: Standard Proportional Tracking ---
[TB] Target: 0x0a00, Actual: 0x0880, P-Gain: 0x0200
[TB] Output Correction: 0x0300 (Expected: 0x0300)
[TB] PASS: Proportional output matches exactly +3.0 after scaling.
[TB] --- Test Case 2: Derivative Response ---
[TB] New Actual: 0x0980, D-Gain: 0x0100
[TB] Output Correction: 0xff00 (Expected: 0xff00)
[TB] PASS: Derivative output calculates change in error (-1.0) correctly.
[TB] --- Test Case 3: Integral Accumulation and Anti-Windup ---
[TB] Ramping up integral with continuous error (+10.0) for 5000 cycles...
[TB] Output Correction: 0x7fff (Expected clamped to 0x7FFF)
[TB] PASS: Integral accumulator saturated cleanly at its maximum limit without wrapping.
[TB] --- Test Case 3b: Negative Anti-Windup ---
[TB] Ramping down integral with continuous negative error (-10.0) for 10000 cycles...
[TB] Output Correction: 0x8000 (Expected clamped to 0x8000)
[TB] PASS: Integral accumulator saturated cleanly at its negative minimum limit.
[TB] PID Calculator Verification completed successfully.
```

---

## 4. Full System Core Integration & Verification (`rtl/flight_core.sv`)

### Changes Made (Verification Environment Fix)
- **Top-Level Wrapper Verification (`tb/flight_core_tb.sv`):** Completed and verified the system-level integration testbench that links the five validated submodules (`i2c_master`, `rc_receiver`, `pid_calculator`, `motor_mixer`, and `pwm_generator`).
- **I2C Simulator Hazard Mitigation:** Refactored the testbench's mock I2C slave tasks to introduce a glitch-filtering `wait_posedge_scl` loop. This filters out simulator delta-cycle hazards on SCL clock lines (which caused the slave to read `0xD0` incorrectly as `0xE8`), completely resolving the initialization loop hang.
- **I2C Handshake Verification:** Ensured the slave correctly acknowledges the device address `0xD0` and registers.

### Verification Results
Run using Vivado Simulator inside the `./vivado/cli_workspace/` directory:
```powershell
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xvlog.bat" --sv ../../rtl/i2c_master.sv ../../rtl/rc_receiver.sv ../../rtl/pid_calculator.sv ../../rtl/motor_mixer.sv ../../rtl/pwm_generator.sv ../../rtl/flight_core.sv ../../tb/flight_core_tb.sv
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat" -timescale 1ns/1ps -debug typical flight_core_tb -s flight_core_sim
& "C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat" flight_core_sim -R
```

### Log Output
```
=================================================
[TB] Starting Full System Flight Core Verification
=================================================
[TB] Reset Released. Initializing MPU-6050 (10ms wait)...
[TB] Time: 5.00 ms
[TB] Time: 10.00 ms
[I2C Slave]          10001705000: Detected START condition
[I2C Slave]          10003288000: Bit 7 = 1
...
[I2C Slave]          22525139000: Read address d0
[I2C Slave]          22550472000: Master requested gyro read from 0x43
[I2C Slave]          22576139000: Master reading gyro registers
...
[TB] Motor 1 (Front Right) PWM Width: 1.007 ms
[TB] Motor 3 (Rear Left)   PWM Width: 1.408 ms
[TB] PASS: Motor 3 duty cycle is correctly higher than Motor 1 to counter the mock positive Roll error!
=================================================
```
