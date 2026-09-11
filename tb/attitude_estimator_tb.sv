`timescale 1ns / 1ps

// =============================================================================
// Module: attitude_estimator_tb
// Purpose: Unit verification for attitude_estimator (Complementary filter, noise)
// =============================================================================

import fc_tb_pkg::*;

module attitude_estimator_tb();

    `GLOBAL_WATCHDOG(100ms)

    logic               clk;
    logic               rst_n;
    logic               enable;
    logic signed [15:0] accel_x;
    logic signed [15:0] accel_y;
    logic signed [15:0] accel_z;
    logic signed [15:0] gyro_x;
    logic signed [15:0] gyro_y;
    logic signed [15:0] gyro_z;
    
    logic signed [15:0] roll_angle;
    logic signed [15:0] pitch_angle;
    int error_count = 0;

    // Instantiate UUT
    attitude_estimator uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .roll_angle(roll_angle),
        .pitch_angle(pitch_angle)
    );

    // 12 MHz Master Clock
    initial begin
        clk = 1'b0;
        forever #CLK_HALF_PERIOD_NS clk = ~clk;
    end

    // Task to strobe enable synchronously for N updates
    task automatic step_filter(input int steps);
        for (int i = 0; i < steps; i++) begin
            @(posedge clk);
            enable <= 1'b1;
            @(posedge clk);
            enable <= 1'b0;
        end
    endtask

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Hardened 6-DOF Attitude Estimator Verification");
        $display("=================================================================");
        error_count = 0;
        enable      <= 1'b0;
        accel_x     <= 16'sd0;
        accel_y     <= 16'sd0;
        accel_z     <= 16'sd16384; // 1g down
        gyro_x      <= 16'sd0;
        gyro_y      <= 16'sd0;
        gyro_z      <= 16'sd0;

        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        // Test 1: Flat level at rest (Expected 0.0 deg)
        step_filter(50);
        check_assert(roll_angle === 16'sd0 && pitch_angle === 16'sd0, "FLAT_LEVEL",
                     "Flat level estimation is exactly 0.0 degrees", error_count);

        // Test 2: Static Roll Tilt (+10.0 deg: accel_y = +2845)
        accel_y <= 16'sd2845;
        accel_z <= 16'sd16135;
        step_filter(200); // 200 updates to converge
        check_assert(roll_angle >= 16'sd2400 && roll_angle <= 16'sd2700, "ROLL_TILT_POS",
                     "Filter converged cleanly to +10.0 deg Roll (~2560 LSB)", error_count);

        // Test 3: Static Pitch Tilt (+15.0 deg: accel_x = -4240)
        accel_y <= 16'sd0;
        accel_x <= -16'sd4240;
        step_filter(200);
        check_assert(pitch_angle >= 16'sd3700 && pitch_angle <= 16'sd4000, "PITCH_TILT_POS",
                     "Filter converged cleanly to +15.0 deg Pitch (~3840 LSB)", error_count);

        // Test 4: Dynamic Gyro Rate Integration (+50 deg/s Roll Rate)
        accel_x <= 16'sd0;
        accel_y <= 16'sd0;
        accel_z <= 16'sd16384;
        step_filter(200); // re-level fully
        gyro_x  <= 16'sd6550; // +50 dps
        step_filter(20);
        check_assert(roll_angle >= 16'sd170 && roll_angle <= 16'sd300, "GYRO_INTEGRATION",
                     "Gyro dynamic rate integration pipeline accumulated rotation accurately", error_count);
        gyro_x  <= 16'sd0;

        // Test 5A: Negative Roll Symmetrical Tilt (-10.0 deg)
        accel_y <= -16'sd2845;
        accel_z <= 16'sd16135;
        step_filter(200);
        check_assert(roll_angle <= -16'sd2400 && roll_angle >= -16'sd2700, "ROLL_TILT_NEG",
                     "Negative Roll angle converged symmetrically without 2's complement error", error_count);

        // Test 5B: Negative Pitch Symmetrical Tilt (-15.0 deg)
        accel_y <= 16'sd0;
        accel_x <= 16'sd4240;
        step_filter(200);
        check_assert(pitch_angle <= -16'sd3700 && pitch_angle >= -16'sd4000, "PITCH_TILT_NEG",
                     "Negative Pitch angle converged symmetrically", error_count);

        // Test 6: High-Frequency Motor Vibration Noise Rejection (+/-0.25g / 14 deg raw)
        accel_x <= 16'sd0;
        accel_y <= 16'sd0;
        accel_z <= 16'sd16384;
        step_filter(200);

        for (int i = 0; i < 100; i++) begin
            accel_y <= (i % 2 == 0) ? 16'sd4000 : -16'sd4000;
            @(posedge clk); enable <= 1'b1;
            @(posedge clk); enable <= 1'b0;
        end
        check_assert(roll_angle >= -16'sd150 && roll_angle <= 16'sd150, "VIBRATION_REJECTION",
                     "High-frequency motor vibration (14 deg raw ripple) attenuated to < 0.6 deg", error_count);
        accel_y <= 16'sd0;

        // Test 7: Simultaneous Dual-Axis Dynamic Rotation (+50 dps Roll, -50 dps Pitch)
        step_filter(200);
        gyro_x <= 16'sd6550;
        gyro_y <= -16'sd6550;
        step_filter(20);
        check_assert(roll_angle >= 16'sd170 && roll_angle <= 16'sd300 &&
                     pitch_angle <= -16'sd170 && pitch_angle >= -16'sd300,
                     "DUAL_AXIS_ROTATION", "Dual-axis dynamic rates integrated with zero cross-axis leakage", error_count);

        // Standardized Exit
        finalize_test_suite("ATTITUDE ESTIMATOR", error_count);
        $finish;
    end

endmodule
