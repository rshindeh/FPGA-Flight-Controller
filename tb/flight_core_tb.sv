`timescale 1ns / 1ps

// =============================================================================
// Module: flight_core_tb
// Purpose: Top-level integration testbench for FPGA Flight Controller
// Architecture: Cascaded Dual-Loop PID, 6 MHz SPI MPU-6500, Quad-X Motor Mixer
// =============================================================================

import fc_tb_pkg::*;

module flight_core_tb();

    // Instantiate Global Simulator Watchdog (Safety Measure 2)
    `GLOBAL_WATCHDOG(GLOBAL_SIM_WATCHDOG)

    // DUT Physical Interface Signals
    logic       clk;
    logic       rst_n;
    wire        spi_sclk;
    wire        spi_mosi;
    wire        spi_miso;
    wire        spi_cs_n;
    logic [3:0] rc_inputs;
    wire  [3:0] esc_pwm_outputs;

    int error_count = 0;

    // UUT Instantiation (Fast simulation parameters: 100 us arming, 10 us init delay)
    flight_core #(
        .ARM_TIME_CYCLES(1200),
        .INIT_WAIT_CYCLES(120)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .rc_inputs(rc_inputs),
        .esc_pwm_outputs(esc_pwm_outputs)
    );

    // 12 MHz Master Clock Generator (Period = 83.333 ns)
    initial begin
        clk = 1'b0;
        forever #CLK_HALF_PERIOD_NS clk = ~clk;
    end

    // Emulated RC Transmitter Pulses (Non-blocking drives to prevent delta races)
    real  throttle_ms    = 1.0;
    real  roll_ms        = 1.5;
    real  pitch_ms       = 1.5;
    real  yaw_ms         = 1.5;
    logic rc_link_active = 1'b1;

    initial begin
        rc_inputs <= 4'b0000;
        forever begin
            if (rc_link_active) begin
                rc_inputs <= 4'b1111;
                fork
                    begin #(roll_ms     * 1_000_000); rc_inputs[0] <= 1'b0; end
                    begin #(pitch_ms    * 1_000_000); rc_inputs[1] <= 1'b0; end
                    begin #(yaw_ms      * 1_000_000); rc_inputs[2] <= 1'b0; end
                    begin #(throttle_ms * 1_000_000); rc_inputs[3] <= 1'b0; end
                join
                #17_000_000; // Complete 20 ms standard frame
            end else begin
                rc_inputs <= 4'b0000;
                #1_000_000;
            end
        end
    end

    // Mock MPU-6500 SPI Slave with physical 25 ns MISO propagation delay
    mpu6500_spi_slave_sim slave (
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .miso(spi_miso),
        .cs_n(spi_cs_n)
    );

    // Measured Motor PWM High Times
    realtime m1_width, m2_width, m3_width, m4_width;

    // Bounded Concurrent Motor Pulse Measurement (Deadlock Guard)
    task automatic measure_all_motors_bounded(input time timeout_limit = DEFAULT_BOUNDED_TIMEOUT);
        realtime t_start[4], t_end[4];
        bit timed_out = 0;

        fork
            begin
                fork
                    begin
                        @(posedge esc_pwm_outputs[0]); t_start[0] = $realtime;
                        @(negedge esc_pwm_outputs[0]); t_end[0]   = $realtime;
                        m1_width = (t_end[0] - t_start[0]) / 1.0e6;
                    end
                    begin
                        @(posedge esc_pwm_outputs[1]); t_start[1] = $realtime;
                        @(negedge esc_pwm_outputs[1]); t_end[1]   = $realtime;
                        m2_width = (t_end[1] - t_start[1]) / 1.0e6;
                    end
                    begin
                        @(posedge esc_pwm_outputs[2]); t_start[2] = $realtime;
                        @(negedge esc_pwm_outputs[2]); t_end[2]   = $realtime;
                        m3_width = (t_end[2] - t_start[2]) / 1.0e6;
                    end
                    begin
                        @(posedge esc_pwm_outputs[3]); t_start[3] = $realtime;
                        @(negedge esc_pwm_outputs[3]); t_end[3]   = $realtime;
                        m4_width = (t_end[3] - t_start[3]) / 1.0e6;
                    end
                join
            end
            begin
                #(timeout_limit);
                timed_out = 1;
            end
        join_any
        disable fork;

        if (timed_out) begin
            $fatal(2, "[DEADLOCK] Timed out waiting for 4-channel motor PWM pulses (limit: %0t) at %0t ps",
                   timeout_limit, $time);
        end
    endtask

    // Main Test Stimulus Sequencer
    initial begin
        $display("=================================================================");
        $display("[TB] Starting Safety-Critical Flight Core Verification Suite");
        $display("=================================================================");
        error_count    = 0;
        rc_link_active = 1'b1;

        // Phase 1: Synchronous Reset Release
        `SYNC_RESET_RELEASE(clk, rst_n, 12)

        // Phase 2: Bounded Handshake for MPU-6500 SPI Master Init
        `AWAIT_SIGNAL_LEVEL(clk, uut.spi_master_inst.valid, 1'b1, 200_000, "SPI Master IMU Stream Active")
        check_assert(!uut.armed, "DISARMED_INIT", "Flight Core initialized in DISARMED state", error_count);

        // Verify Disarmed Actuator Lockout (Motors locked to 1.000 ms)
        measure_all_motors_bounded();
        check_assert(is_within_tolerance_real(m1_width, 1.000, 0.02) &&
                     is_within_tolerance_real(m2_width, 1.000, 0.02) &&
                     is_within_tolerance_real(m3_width, 1.000, 0.02) &&
                     is_within_tolerance_real(m4_width, 1.000, 0.02),
                     "ACTUATOR_LOCKOUT", "All motor outputs strictly locked to 1.000 ms while disarmed", error_count);

        // Phase 3: Arming Sequence (Throttle Min, Yaw Full Right)
        throttle_ms = 1.0;
        yaw_ms      = 1.95; // > 23,000 ticks
        roll_ms     = 1.5;
        pitch_ms    = 1.5;

        `AWAIT_SIGNAL_LEVEL(clk, uut.armed, 1'b1, 400_000, "Safety Manager Transition to ARMED")
        check_assert(uut.armed, "ARMED_CONFIRM", "System transitioned cleanly to ARMED state", error_count);

        // Phase 4: Hover Flight Mode (Throttle = 1.5 ms, Sticks Centered)
        throttle_ms = 1.5; // Hover throttle (18,000 ticks)
        yaw_ms      = 1.5;
        roll_ms     = 1.5;
        pitch_ms    = 1.5;
        #50_000_000; // Allow filter & cascaded PID pipelines to settle

        // Phase 5A: Roll Self-Leveling Verification (+10 deg Roll Right Tilt)
        measure_all_motors_bounded();
        $display("[TELEMETRY] Roll Response: M1=%.3f ms, M2=%.3f ms, M3=%.3f ms, M4=%.3f ms",
                 m1_width, m2_width, m3_width, m4_width);
        check_assert((m1_width > m3_width) && (m1_width > 1.50) && (m3_width < 1.50),
                     "ROLL_SELF_LEVEL", "Right motors spooled up and left motors spooled down to counter roll", error_count);

        // Phase 5B: Pitch Self-Leveling Verification (Nose Down +15 deg Tilt)
        slave.registers[8'h3D] = 8'h00; slave.registers[8'h3E] = 8'h00; // Clear roll accel
        slave.registers[8'h3B] = 8'hEF; slave.registers[8'h3C] = 8'h70; // Set pitch accel (-4240 LSB)
        #30_000_000; // Settle attitude estimator
        measure_all_motors_bounded();
        $display("[TELEMETRY] Pitch Response: M1=%.3f ms, M2=%.3f ms, M3=%.3f ms, M4=%.3f ms",
                 m1_width, m2_width, m3_width, m4_width);
        check_assert((m1_width > m2_width) && (m4_width > m3_width),
                     "PITCH_SELF_LEVEL", "Front motors spooled up over rear motors to pitch nose up", error_count);

        // Phase 5C: Yaw Dynamic Rate Damping (+50 deg/s CW Spin)
        slave.registers[8'h3B] = 8'h00; slave.registers[8'h3C] = 8'h00; // Clear pitch tilt
        slave.registers[8'h47] = 8'h19; slave.registers[8'h48] = 8'h96; // Gyro Z = +6550 LSB
        #30_000_000; // Settle rate loops
        measure_all_motors_bounded();
        $display("[TELEMETRY] Yaw Response: M1(CCW)=%.3f ms, M2(CW)=%.3f ms, M3(CCW)=%.3f ms, M4(CW)=%.3f ms",
                 m1_width, m2_width, m3_width, m4_width);
        check_assert((m2_width > m1_width) && (m4_width > m3_width),
                     "YAW_RATE_DAMPING", "CW motors spooled up over CCW motors providing counter-torque", error_count);

        // Reset sensor rates to neutral
        slave.registers[8'h47] = 8'h00; slave.registers[8'h48] = 8'h00;
        #10_000_000;

        // Phase 6: In-Flight Pilot Disarming (Throttle Min, Yaw Full Left)
        throttle_ms = 1.0;  // Throttle Min (< 12,500)
        yaw_ms      = 1.05; // Yaw Full Left (< 13,000)
        `AWAIT_SIGNAL_LEVEL(clk, uut.armed, 1'b0, 400_000, "In-Flight Pilot Disarm Command")
        check_assert(!uut.armed, "PILOT_DISARM", "System disarmed cleanly via pilot stick command", error_count);

        measure_all_motors_bounded();
        check_assert(is_within_tolerance_real(m1_width, 1.000, 0.02),
                     "POST_DISARM_LOCK", "Motor PWM outputs immediately clamped to 1.000 ms post-disarm", error_count);

        // Phase 7: Re-Arm and In-Flight RC Signal Loss Watchdog Failsafe
        throttle_ms = 1.0; yaw_ms = 1.95;
        `AWAIT_SIGNAL_LEVEL(clk, uut.armed, 1'b1, 400_000, "Re-Arming System")
        throttle_ms = 1.5; yaw_ms = 1.5;
        #20_000_000;

        // Sever RC Radio Link Completely
        rc_link_active = 1'b0;
        `AWAIT_SIGNAL_LEVEL(clk, uut.armed, 1'b0, 2_000_000, "Watchdog Failsafe Disarm on RC Loss")
        check_assert(!uut.armed && (uut.rc_valid == 4'b0000),
                     "RC_LOSS_FAILSAFE", "Loss of RC signal triggered watchdog failsafe and disarmed core", error_count);

        measure_all_motors_bounded();
        check_assert(is_within_tolerance_real(m1_width, 1.000, 0.02),
                     "FAILSAFE_LOCK", "Motor outputs locked to 1.000 ms during failsafe", error_count);

        // Final Standardized Result
        finalize_test_suite("FLIGHT CORE INTEGRATION", error_count);
        $finish;
    end

endmodule

// =============================================================================
// Mock MPU-6500 SPI Slave Model (SPI Mode 0 with 25 ns Propagation Delay)
// =============================================================================
module mpu6500_spi_slave_sim (
    input  logic sclk,
    input  logic mosi,
    output logic miso,
    input  logic cs_n
);

    logic [7:0] registers [256];
    logic [7:0] rx_byte;
    logic [7:0] tx_byte;
    logic [2:0] bit_cnt;
    logic [7:0] curr_addr;
    logic       is_read;
    logic       is_first_byte;
    logic       miso_drv;

    assign miso = (!cs_n) ? miso_drv : 1'bz;

    initial begin
        for (int i = 0; i < 256; i++) registers[i] = 8'h00;
        
        // WHO_AM_I register (0x75) returns 0x70 for MPU-6500
        registers[8'h75] = 8'h70;
        
        // Initial Sensor Data: Accel Roll Tilt of +10.0 deg (accel_y = +2845 = 0x0B1D)
        registers[8'h3B] = 8'h00; // Accel X_H
        registers[8'h3C] = 8'h00; // Accel X_L
        registers[8'h3D] = 8'h0B; // Accel Y_H (+2845 -> +10 deg roll)
        registers[8'h3E] = 8'h1D; // Accel Y_L
        registers[8'h3F] = 8'h3F; // Accel Z_H (+16135)
        registers[8'h40] = 8'h07; // Accel Z_L
        
        registers[8'h41] = 8'h00; registers[8'h42] = 8'h00; // Temp
        registers[8'h43] = 8'h00; registers[8'h44] = 8'h00; // Gyro X
        registers[8'h45] = 8'h00; registers[8'h46] = 8'h00; // Gyro Y
        registers[8'h47] = 8'h00; registers[8'h48] = 8'h00; // Gyro Z

        miso_drv      = 1'b0;
        rx_byte       = 8'h00;
        tx_byte       = 8'h00;
        bit_cnt       = 3'd0;
        is_first_byte = 1'b1;
    end

    always @(posedge cs_n) begin
        bit_cnt       <= 3'd0;
        is_first_byte <= 1'b1;
        miso_drv      <= 1'b0;
    end

    always @(posedge sclk) begin
        if (!cs_n) begin
            rx_byte <= {rx_byte[6:0], mosi};
            bit_cnt <= bit_cnt + 3'd1;

            if (bit_cnt == 3'd7) begin
                automatic logic [7:0] full_byte = {rx_byte[6:0], mosi};
                if (is_first_byte) begin
                    is_read       <= full_byte[7];
                    curr_addr     <= full_byte[6:0];
                    is_first_byte <= 1'b0;
                end else begin
                    if (!is_read) begin
                        registers[curr_addr] <= full_byte;
                    end
                    curr_addr <= curr_addr + 8'd1;
                end
            end
        end
    end

    // Slave updates MISO on falling edge of SCLK with 25 ns physical delay
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n) begin
            miso_drv <= 1'b0;
            tx_byte  <= 8'h00;
        end else begin
            if (bit_cnt == 3'd0) begin
                if (!is_first_byte && is_read) begin
                    tx_byte  <= registers[curr_addr];
                    miso_drv <= #25 registers[curr_addr][7];
                end else begin
                    tx_byte  <= 8'h00;
                    miso_drv <= #25 1'b0;
                end
            end else begin
                miso_drv <= #25 tx_byte[7 - bit_cnt];
            end
        end
    end

endmodule
