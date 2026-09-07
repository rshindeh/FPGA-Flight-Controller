# FPGA Quadcopter Flight Controller

A synthesizable, hardware-accelerated quadcopter flight controller developed in SystemVerilog for the **Digilent Cmod A7-35T** development board (featuring the **AMD/Xilinx Artix-7 XC7A35T FPGA**).

---

## Executive Summary

Most modern commercial and hobbyist flight controllers run on general-purpose microcontrollers (e.g., ARM Cortex-M4/M7), where control loop frequencies, sensor sampling, and pulse generation are constrained by software interrupt latency, task scheduling, and bus contention.

This project implements a dedicated digital flight controller entirely in hardware RTL:
- **Zero Operating System Overhead & Deterministic Latency:** Control loops, sensor acquisition, and PWM generation run concurrently on dedicated hardware pipelines with cycle-accurate timing.
- **Cascaded Dual-Loop Architecture:** Features an outer angle P-loop for self-leveling attitude control coupled to an inner rate PID loop for agile disturbance rejection.
- **Metastability-Hardened Peripherals:** Features two-stage flip-flop synchronizers for asynchronous RC pulses, a robust 400 kHz I2C master for MPU-6050 6-DOF IMU streaming, and double-buffered glitch-free PWM outputs at 400 Hz.
- **Hardware Safety Failsafes:** Hardware state machine enforcing stick-based arming/disarming, automatic throttle cutoffs, and RC signal-loss watchdog interlocks.

For complete mathematical derivations, register maps, and architectural specifications, see [`docs/flight_controller_spec.md`](docs/flight_controller_spec.md).

---

## Directory Structure

| Directory / File | Description |
| :--- | :--- |
| [`rtl/`](rtl/) | Synthesizable SystemVerilog source code (controllers, estimation, peripherals, mixing). |
| [`tb/`](tb/) | Comprehensive simulation testbenches with self-checking assertions. |
| [`constraints/`](constraints/) | Physical pinout and clock constraints for Digilent Cmod A7 (`cmod_a7_pins.xdc`). |
| [`docs/`](docs/) | Engineering specifications, design requirements, and verification summaries. |
| [`Makefile`](Makefile) | Simulation and verification automation runner for Vivado (`xsim`) and Icarus Verilog. |
| [`docs/flight_controller_spec.md`](docs/flight_controller_spec.md) | Comprehensive engineering specification and control loop mathematics. |
| [`docs/verification_summary.md`](docs/verification_summary.md) | Testbench results, bug resolution log, and validation metrics. |

---

## Architecture & Hardware Pipeline

The top-level hardware entity ([`rtl/flight_core.sv`](rtl/flight_core.sv)) coordinates sensor acquisition, pilot command mapping, safety state machines, dual-loop PID control, and motor output generation:

```text
   EXTERNAL INPUTS                         FPGA FABRIC (flight_core)                           EXTERNAL OUTPUTS
+-------------------+       +---------------------------------------------------+       +--------------------+
|                   |       |  +-------------+                 +-------------+  |       |                    |
|    RC Receiver    |------>|  | rc_receiver |--(Sticks)------>|  rc_mapper  |  |       |                    |
| 4-Ch PWM (50 Hz)  |       |  +-------------+                 +-------------+  |       |                    |
|                   |       |         |                         |            |  |       |                    |
+-------------------+       |         v (Sticks)   (Target Ang) v            |  |       |                    |
                            |  +------------+  +------------+ +-----------+  |  |       |                    |
                            |  | safety_mgr |  |attitude_est|-> Angle PID |  |  |       |                    |
                            |  +------------+  +------------+ |  (Outer)  |  |  |       |                    |
                            |        |               ^        +-----------+  |  |       |                    |
+-------------------+       |        |               |              | (Rate) |  |       |    4x Motor ESCs   |
|   MPU-6050 IMU    |<=====>|  +------------+ (IMU)  |              v        |  |       | 400 Hz PWM Signals |
|  I2C Bus (400kHz) |       |  | i2c_master |--------+        +-----------+  |  |       | (1.0 ms - 2.0 ms)  |
|                   |       |  +------------+                 | Rate PID  |  |  |       |                    |
+-------------------+       |    |         |                  |  (Inner)  |  |  |       |                    |
                            |    | (Armed) +--(Gyro Rates)--->+-----------+  |  |       |                    |
                            |    |                                  | (PID)  |  |       |                    |
                            |    |   +------------------------------+        |  |       |                    |
                            |    |   |     (Throttle Bypass) <---------------+  |       |                    |
                            |    v   v                       v                  |       |                    |
                            |  +----------------------------------------------+ |       |                    |
                            |  |                 motor_mixer                  | |       |                    |
                            |  +----------------------------------------------+ |       |                    |
                            |                         |                         |       |                    |
                            |                         v                         |       |                    |
                            |  +----------------------------------------------+ |       |                    |
                            |  |                pwm_generator                 |-------->|                    |
                            |  +----------------------------------------------+ |       |                    |
                            +---------------------------------------------------+       +--------------------+
```

*(Detailed register interfaces, pin mappings, and mathematical models are documented in [`docs/flight_controller_spec.md`](docs/flight_controller_spec.md).)*

---

## Hardware Platform & Target Peripherals

The design is targeted for physical deployment on standard quadcopter avionics hardware:

- **Target FPGA:** AMD/Xilinx Artix-7 XC7A35T-1CPG236C (Digilent Cmod A7-35T breadboard-friendly form factor).
- **Master Clock:** 12.0 MHz on-board oscillator constraint (`cmod_a7_pins.xdc`), providing an 83.33 ns base clock tick.
- **Inertial Measurement Unit (IMU):** InvenSense MPU-6050 6-DOF sensor connected via open-drain I2C (`M3` / `L3`). The custom master streams 14-byte sensor bursts at 400 kHz Fast-Mode with a 1 kHz update rate.
- **Pilot RC Interface:** 4 input channels (Roll, Pitch, Yaw, Throttle) accepting standard 50 Hz PWM signals (1.0 ms to 2.0 ms pulse width) with two-stage flip-flop synchronizers and a 100 ms loss-of-signal watchdog timer.
- **ESC / Motor Drive:** 4 output channels configured for Quad-X motor layouts driving ESCs at 400 Hz frame rate with double-buffered shadow registers to ensure glitch-free pulse width transitions.
- **Arithmetic Processing:** Synthesizable fixed-point arithmetic units:
  - 32-bit Q16.16 complementary filter for real-time attitude estimation (gyro integration + accelerometer gravity compensation).
  - 16-bit Q8.8 cascaded PID pipelines with anti-windup clamping and dynamic integral reset.

---

## RTL Module Summary

| Module | File | Function |
| :--- | :--- | :--- |
| **flight_core** | [`rtl/flight_core.sv`](rtl/flight_core.sv) | System top-level integrating sensors, safety logic, PID control, and motor drivers. |
| **i2c_master** | [`rtl/i2c_master.sv`](rtl/i2c_master.sv) | Synthesizable 400 kHz I2C master streaming 14-byte bursts (`0x3B`..`0x48`) at 1 kHz. |
| **attitude_estimator** | [`rtl/attitude_estimator.sv`](rtl/attitude_estimator.sv) | 6-DOF complementary filter computing roll and pitch angles using Q16.16 math. |
| **rc_receiver** | [`rtl/rc_receiver.sv`](rtl/rc_receiver.sv) | 4-channel pulse decoder with 2-stage synchronization and 100 ms loss watchdog. |
| **rc_mapper** | [`rtl/rc_mapper.sv`](rtl/rc_mapper.sv) | Normalizes raw stick timing, provides deadband filtering, and maps to degree setpoints. |
| **safety_mgr** | [`rtl/safety_mgr.sv`](rtl/safety_mgr.sv) | Enforces arming/disarming sequence and commands failsafe motor shutdown. |
| **pid_calculator** | [`rtl/pid_calculator.sv`](rtl/pid_calculator.sv) | Q8.8 fixed-point PID pipeline with anti-windup clamping and derivative filtering. |
| **motor_mixer** | [`rtl/motor_mixer.sv`](rtl/motor_mixer.sv) | Quad-X mixing matrix resolving throttle, roll, pitch, and yaw commands into ESC ticks. |
| **pwm_generator** | [`rtl/pwm_generator.sv`](rtl/pwm_generator.sv) | 400 Hz ESC driver with synchronous double-buffered duty cycle registers. |

---

## Verification & Simulation

The design includes a comprehensive verification suite with self-checking SystemVerilog testbenches. Full defect resolution history and verification metrics are documented in [`docs/verification_summary.md`](docs/verification_summary.md).

| Testbench | Target Module | Scope & Verified Conditions | Simulation Status |
| :--- | :--- | :--- | :--- |
| `flight_core_tb` | `flight_core` | Closed-loop arming sequence, tilt auto-leveling, throttle mixing | **PASSED** (0 Errors) |
| `attitude_estimator_tb` | `attitude_estimator` | Level hover, static roll/pitch convergence, gyro drift rejection | **PASSED** (0 Errors) |
| `i2c_master_tb` | `i2c_master` | 400 kHz SCL clock generation, MPU-6050 wake-up write, burst reads | **PASSED** (0 Errors) |
| `rc_receiver_tb` | `rc_receiver` | 2-stage FF synchronization, pulse width decoding, 100 ms watchdog | **PASSED** (0 Errors) |
| `pid_calculator_tb` | `pid_calculator` | Q8.8 P/I/D responses, anti-windup clamping, dynamic `clear_i` reset | **PASSED** (0 Errors) |
| `motor_mixer_tb` | `motor_mixer` | Quad-X differential thrust matrix, disarm lockout, saturation limits | **PASSED** (0 Errors) |
| `pwm_generator_tb` | `pwm_generator` | 400 Hz frame timing, pulse fidelity, shadow register glitch prevention | **PASSED** (0 Errors) |

---

## Quickstart / Simulation Automation

A top-level [`Makefile`](Makefile) automates compilation, elaboration, and execution across all testbenches.

### 1. Run with Make (Default: AMD Vivado xsim)
```bash
# Run all testbenches
make test

# Run a specific testbench
make flight_core_tb
make attitude_estimator_tb

# Clean simulation outputs
make clean
```

*To run with Icarus Verilog instead, append `SIM=iverilog` (e.g., `make test SIM=iverilog`).*

### 2. Run Directly with AMD Vivado CLI (PowerShell / Windows)
```powershell
# Run full closed-loop system simulation
xvlog -sv rtl/*.sv tb/flight_core_tb.sv
xelab -debug typical flight_core_tb -s flight_sim
xsim flight_sim -R

# Run individual unit test (e.g., attitude estimator)
xvlog -sv rtl/attitude_estimator.sv tb/attitude_estimator_tb.sv
xelab -debug typical attitude_estimator_tb -s att_sim
xsim att_sim -R
```
