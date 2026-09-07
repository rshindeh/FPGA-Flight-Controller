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
| [`docs/flight_controller_spec.md`](docs/flight_controller_spec.md) | Comprehensive engineering specification and control loop mathematics. |
| [`docs/verification_summary.md`](docs/verification_summary.md) | Testbench results, bug resolution log, and validation metrics. |

---

## Architecture & Top-Level Block Diagram

The top-level hardware entity ([`rtl/flight_core.sv`](rtl/flight_core.sv)) coordinates sensor ingestion, pilot command mapping, safety state machines, dual-loop PID processing, and motor output generation:

```
                      +--------------------------------------------------------+
                      |                      flight_core                       |
                      |                                                        |
  [RC Receiver] ----->| rc_receiver ---> rc_mapper --------+                   |
  (4x PWM In)         |     |                               | (Angle Desired)  |
                      |     +----------> safety_mgr --+     v                  |
                      |                  (Arm/Failsafe)| [Outer Angle Loop]    |
                      |                               |         |              |
                      |                               | (Rate)  v              |
  [MPU-6050 IMU] <--->| i2c_master -> attitude_est ---+---> [Inner Rate Loop]   |
  (I2C Fast Mode)     |                                         |              |
                      |                                         v (Corrections)|
                      |                                    motor_mixer         |
                      |                                         |              |
                      |                                         v              |
                      |                                   pwm_generator ------>| [4x ESC PWM Out]
                      |                                   (400 Hz Frame)       | (1.0ms - 2.0ms)
                      +--------------------------------------------------------+
```

*(Detailed block diagrams and register interface schematics can be found in [`docs/flight_controller_spec.md`](docs/flight_controller_spec.md).)*

---

## Hardware Resource Utilization & Timing

Target Device: **Xilinx Artix-7 XC7A35T-1CPG236C** (Digilent Cmod A7-35T)  
Synthesis & Implementation Toolchain: **AMD Vivado Design Suite**  
Master Clock: **12.0 MHz** (`cmod_a7_pins.xdc`)

### Post-Implementation Utilization (Placeholder)

| Resource | Used | Available | Utilization % |
| :--- | :--- | :--- | :--- |
| **LUT (Logic)** | *TBD* | 20,800 | *TBD* |
| **LUTRAM** | *TBD* | 9,600 | *TBD* |
| **Flip-Flops (FF)** | *TBD* | 41,600 | *TBD* |
| **DSP48E1 Slices** | *TBD* | 90 | *TBD* |
| **Block RAM (BRAM)** | *TBD* | 50 | *TBD* |
| **IOB Pins** | 10 | 106 | ~9.4% |

### Timing & Frequency Performance (Placeholder)

| Metric | Target Constraint | Achieved (Worst Case) | Timing Slack |
| :--- | :--- | :--- | :--- |
| **Master Clock (`clk`)** | 12.000 MHz (83.33 ns) | *TBD* | *TBD* |
| **Max Operating Frequency ($F_{\text{max}}$)** | > 12.0 MHz | *TBD* | *TBD* |
| **Worst Negative Slack (WNS)** | >= 0.000 ns | *TBD* | *TBD* |

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

All modules are verified with automated, self-checking SystemVerilog testbenches. For details on test results and previous bug resolutions, refer to [`docs/verification_summary.md`](docs/verification_summary.md).

### Run Full System Simulation
```powershell
xvlog -sv rtl/*.sv tb/flight_core_tb.sv
xelab -debug typical flight_core_tb -s flight_sim
xsim flight_sim -R
```

### Run Unit Testbenches
```powershell
# Example: Attitude Estimator unit test
xvlog -sv rtl/attitude_estimator.sv tb/attitude_estimator_tb.sv
xelab -debug typical attitude_estimator_tb -s att_sim
xsim att_sim -R
```
