# FPGA Quadcopter Flight Controller

A synthesizable, hardware-accelerated quadcopter flight controller developed in SystemVerilog for the **Digilent Cmod A7-35T** development board (featuring the **AMD/Xilinx Artix-7 XC7A35T FPGA**).

---

## Executive Summary

Most modern commercial and hobbyist flight controllers run on general-purpose microcontrollers (e.g., ARM Cortex-M4/M7), where control loop frequencies, sensor sampling, and pulse generation are constrained by software interrupt latency, task scheduling, and bus contention.

This project implements a dedicated digital flight controller entirely in hardware RTL:
- **Zero Operating System Overhead & Deterministic Latency:** Control loops, sensor acquisition, and PWM generation run concurrently on dedicated hardware pipelines with cycle-accurate timing.
- **Cascaded Dual-Loop Architecture:** Features an outer angle P-loop for self-leveling attitude control coupled to an inner rate PID loop for agile disturbance rejection.
- **Metastability-Hardened Peripherals:** Features two-stage flip-flop synchronizers for asynchronous RC pulses, a high-throughput 6 MHz SPI master for MPU-6500 6-DOF IMU streaming, and double-buffered glitch-free PWM outputs at 400 Hz.
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
   PILOT INPUT PATH                              SENSOR PATH
   ================                              ===========
 [ RC Receiver Pins ]                         [ MPU-6500 SPI Bus ]
          |                                            |
          v                                            v
   +-------------+                              +--------------+
   | rc_receiver |                              |  spi_master  |
   +-------------+                              +--------------+
      |       |                                    |        |
      |       | (Sticks)              (Accel/Gyro) |        | (Gyro Rates)
      v       v                                    v        |
+------------+ +-----------+                +--------------+ |
| safety_mgr | | rc_mapper |                | attitude_est | |
+------------+ +-----------+                +--------------+ |
      |          |       |                         |        |
      |          |       | (Target Angles)         | (Est.) |
      |          |       +--------------+   +------+        |
      |          |                      |   |               |
      |          |                      v   v               |
      |          |               +----------------+         |
      |          |               | pid_calculator | (Outer Angle Loop)
      |          |               +----------------+         |
      |          |                       |                  |
      |          |                       | (Desired Rates)  |
      |          |                       v                  |
      |          |               +----------------+         |
      |          |               | pid_calculator |<--------+ (Inner Rate Loop)
      |          |               +----------------+
      |          |                       |
      |          | (Throttle Bypass)     | (Torque Corrections)
      |          |                       |
      |          +--------------+ +------+
      |                         | |
      | (Armed Lockout)         v v
      |                  +-------------+
      +----------------->| motor_mixer |
                         +-------------+
                                |
                                | (Motor Duty Cycles)
                                v
                        +---------------+
                        | pwm_generator |
                        +---------------+
                                |
                                v
                      [ 4x ESC PWM Outputs ]
```

*(Detailed register interfaces, pin mappings, and mathematical models are documented in [`docs/flight_controller_spec.md`](docs/flight_controller_spec.md).)*

---

## Hardware Platform & Target Peripherals

The design is targeted for physical deployment on standard quadcopter avionics hardware:

- **Target FPGA:** AMD/Xilinx Artix-7 XC7A35T-1CPG236C (Digilent Cmod A7-35T breadboard-friendly form factor).
- **Master Clock:** 12.0 MHz on-board oscillator constraint (`cmod_a7_pins.xdc`), providing an 83.33 ns base clock tick.
- **Inertial Measurement Unit (IMU):** InvenSense MPU-6500 6-DOF sensor connected via 4-wire SPI (`spi_sclk` on `M3`, `spi_mosi` on `L3`, `spi_miso` on `J1`, `spi_cs_n` on `K2`). The custom SPI master streams 14-byte sensor bursts at 6 MHz Mode 0 with sub-25 µs transport delay at a 1 kHz update rate.
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
| **spi_master** | [`rtl/spi_master.sv`](rtl/spi_master.sv) | Synthesizable 6 MHz SPI master (Mode 0) streaming 14-byte bursts (`0x3B`..`0x48`) at 1 kHz with automated MPU-6500 initialization. |
| **attitude_estimator** | [`rtl/attitude_estimator.sv`](rtl/attitude_estimator.sv) | 6-DOF complementary filter computing roll and pitch angles using Q16.16 math. |
| **rc_receiver** | [`rtl/rc_receiver.sv`](rtl/rc_receiver.sv) | 4-channel pulse decoder with 2-stage synchronization and 100 ms loss watchdog. |
| **rc_mapper** | [`rtl/rc_mapper.sv`](rtl/rc_mapper.sv) | Normalizes raw stick timing, provides deadband filtering, and maps to degree setpoints. |
| **safety_mgr** | [`rtl/safety_mgr.sv`](rtl/safety_mgr.sv) | Enforces arming/disarming sequence and commands failsafe motor shutdown. |
| **pid_calculator** | [`rtl/pid_calculator.sv`](rtl/pid_calculator.sv) | Q8.8 fixed-point PID pipeline with anti-windup clamping and derivative filtering. |
| **motor_mixer** | [`rtl/motor_mixer.sv`](rtl/motor_mixer.sv) | Quad-X mixing matrix resolving throttle, roll, pitch, and yaw commands into ESC ticks. |
| **pwm_generator** | [`rtl/pwm_generator.sv`](rtl/pwm_generator.sv) | 400 Hz ESC driver with synchronous double-buffered duty cycle registers. |

---

## Verification & Simulation Suite

The verification suite features 9 self-checking SystemVerilog testbenches built on top of a centralized verification package ([`tb/fc_tb_pkg.sv`](tb/fc_tb_pkg.sv)) that enforces safety-critical flight hardware verification standards:

- **Timing & Delta-Cycle Race Prevention:** Synchronous non-blocking drives (`<=`) on DUT inputs, synchronous sampling on clock edges, and bounded reset deassertion (`SYNC_RESET_RELEASE`).
- **Deadlock Guards & Simulator Watchdogs:** Cycle-bounded FSM handshakes (`AWAIT_SIGNAL_LEVEL`) and parallel simulator watchdog timers (`GLOBAL_WATCHDOG`) on every testbench.
- **Fixed-Point Hygiene & Saturation:** Clamping validation and epsilon-tolerant comparison functions for fixed-point and physical unit values.
- **Fail-Fast Error Handling:** Immediate assertions (`check_assert`), standardized suite exits (`finalize_test_suite`), and automatic simulation log inspection in `Makefile` (`CHECK_SIM`).

### Verification Coverage & Line Metrics

| Testbench | Target Module | Scope & Verified Conditions | Lines | Reduction | Status |
| :--- | :--- | :--- | :---: | :---: | :---: |
| [`safety_mgr_tb.sv`](tb/safety_mgr_tb.sv) | `safety_mgr` | Debounce timers, stick jitter reset, single-channel loss disarm, idle boundaries | 131 | -54.0% | **PASSED** (0 Errors) |
| [`rc_mapper_tb.sv`](tb/rc_mapper_tb.sv) | `rc_mapper` | Deadband boundaries ($\pm 99, \pm 100$ ticks), degree scaling, throttle clamping | 108 | -52.6% | **PASSED** (0 Errors) |
| [`pwm_generator_tb.sv`](tb/pwm_generator_tb.sv) | `pwm_generator` | 0%/100% duty, out-of-bounds input, shadow register glitch elimination, async reset | 105 | -51.6% | **PASSED** (0 Errors) |
| [`motor_mixer_tb.sv`](tb/motor_mixer_tb.sv) | `motor_mixer` | Quad-X thrust matrix, upper/lower saturation clamping, 16-bit extreme stress | 141 | -42.4% | **PASSED** (0 Errors) |
| [`pid_calculator_tb.sv`](tb/pid_calculator_tb.sv) | `pid_calculator` | Q8.8 P/I/D responses, anti-windup clamping, dynamic `clear_i`, enable gating | 131 | -48.8% | **PASSED** (0 Errors) |
| [`rc_receiver_tb.sv`](tb/rc_receiver_tb.sv) | `rc_receiver` | 2-stage sync, 500 ns runt glitch reject, 4 overlapping channels, 55 ms watchdog | 130 | -52.4% | **PASSED** (0 Errors) |
| [`attitude_estimator_tb.sv`](tb/attitude_estimator_tb.sv) | `attitude_estimator` | Level hover, static roll/pitch convergence, gyro rate integration, $\pm 0.25g$ vibration filter | 126 | -46.4% | **PASSED** (0 Errors) |
| [`spi_master_tb.sv`](tb/spi_master_tb.sv) | `spi_master` | 6 MHz 14-byte streaming, 25 ns MISO delay, WHO_AM_I fault injection, chip IDs | 220 | -36.4% | **PASSED** (0 Errors) |
| [`flight_core_tb.sv`](tb/flight_core_tb.sv) | `flight_core` | Closed-loop arming, roll/pitch auto-leveling, yaw rate damping, pilot disarm, failsafe | 283 | -30.5% | **PASSED** (0 Errors) |
| **Shared Package** | [`fc_tb_pkg.sv`](tb/fc_tb_pkg.sv) | Shared timing constants, watchdog macros, reset release, bounded wait, assertions | 129 | New | - |
| **Total Footprint** | - | **Complete Verification Suite** | **1,504** | **-39.6%** | **100% PASS** |

---

## Quickstart / Simulation Automation

A top-level [`Makefile`](Makefile) automates compilation, elaboration, and execution across all testbenches with automatic failure trapping (`CHECK_SIM`).

### 1. Run Complete Regression Suite (Default: AMD Vivado xsim)
```bash
# Run all 9 testbenches
mingw32-make test

# Run a specific testbench
mingw32-make flight_core_tb
mingw32-make attitude_estimator_tb
mingw32-make spi_master_tb

# Clean simulation artifacts and logs
mingw32-make clean
```

### 2. Run with Icarus Verilog
```bash
mingw32-make SIM=iverilog test
```

### 3. Run Directly with AMD Vivado CLI (PowerShell / Windows)
```powershell
# Run full closed-loop system simulation
xvlog -sv tb/fc_tb_pkg.sv rtl/*.sv tb/flight_core_tb.sv
xelab -debug typical flight_core_tb -s flight_sim
xsim flight_sim -R

# Run individual unit test (e.g., attitude estimator)
xvlog -sv tb/fc_tb_pkg.sv rtl/attitude_estimator.sv tb/attitude_estimator_tb.sv
xelab -debug typical attitude_estimator_tb -s att_sim
xsim att_sim -R
```
