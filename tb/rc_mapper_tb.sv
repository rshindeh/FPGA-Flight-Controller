`timescale 1ns / 1ps

// =============================================================================
// Module: rc_mapper_tb
// Purpose: Unit verification for rc_mapper (Deadband, Fixed-Point Scaling, Clamping)
// =============================================================================

import fc_tb_pkg::*;

module rc_mapper_tb();

    `GLOBAL_WATCHDOG(10ms)

    logic        clk;
    logic        rst_n;
    
    logic [14:0] rc_roll;
    logic [14:0] rc_pitch;
    logic [14:0] rc_yaw;
    logic [14:0] rc_throttle;
    
    logic signed [15:0] target_roll_angle;
    logic signed [15:0] target_pitch_angle;
    logic signed [15:0] target_yaw_rate;
    logic [14:0]        throttle_out;
    int                 error_count = 0;

    // Instantiate UUT
    rc_mapper uut (
        .clk(clk),
        .rst_n(rst_n),
        .rc_roll(rc_roll),
        .rc_pitch(rc_pitch),
        .rc_yaw(rc_yaw),
        .rc_throttle(rc_throttle),
        .target_roll_angle(target_roll_angle),
        .target_pitch_angle(target_pitch_angle),
        .target_yaw_rate(target_yaw_rate),
        .throttle_out(throttle_out)
    );

    // 12 MHz Clock
    initial begin
        clk = 1'b0;
        forever #CLK_HALF_PERIOD_NS clk = ~clk;
    end

    // Concurrent SystemVerilog Assertions (SVA)
    property p_throttle_bounds;
        @(posedge clk) (throttle_out >= 15'd12000 && throttle_out <= 15'd24000);
    endproperty
    assert property (p_throttle_bounds) else $error("[SVA] throttle_out out of bounds: %0d", throttle_out);

    property p_deadband_roll;
        @(posedge clk) (rc_roll >= 15'd17901 && rc_roll <= 15'd18099) |-> (target_roll_angle == 16'sd0);
    endproperty
    assert property (p_deadband_roll) else $error("[SVA] target_roll_angle non-zero in deadband!");

    task automatic step_clk();
        @(posedge clk);
        #1;
    endtask

    initial begin
        $display("=================================================================");
        $display("[TB] Starting RC Setpoint Mapper Comprehensive Unit Verification");
        $display("=================================================================");
        error_count = 0;

        rc_roll     <= 15'd18000;
        rc_pitch    <= 15'd18000;
        rc_yaw      <= 15'd18000;
        rc_throttle <= 15'd12000;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)
        step_clk();

        // -------------------------------------------------------------
        // Test 1: Deadband Boundaries (+/-99, +/-100, +/-101 ticks)
        // -------------------------------------------------------------
        // 1A: Center (18000)
        rc_roll <= 15'd18000; rc_pitch <= 15'd18000; rc_yaw <= 15'd18000; step_clk();
        check_assert(target_roll_angle == 0 && target_pitch_angle == 0 && target_yaw_rate == 0,
                     "DEADBAND_CENTER", "Center 18000 ticks produces exact zero setpoints", error_count);

        // 1B: Inside deadband (+/-99 ticks: 18099 and 17901)
        rc_roll <= 15'd18099; rc_pitch <= 15'd18099; rc_yaw <= 15'd18099; step_clk();
        check_assert(target_roll_angle == 0 && target_pitch_angle == 0 && target_yaw_rate == 0,
                     "DEADBAND_POS_99", "+99 ticks cleanly suppressed by deadband", error_count);

        rc_roll <= 15'd17901; rc_pitch <= 15'd17901; rc_yaw <= 15'd17901; step_clk();
        check_assert(target_roll_angle == 0 && target_pitch_angle == 0 && target_yaw_rate == 0,
                     "DEADBAND_NEG_99", "-99 ticks cleanly suppressed by deadband", error_count);

        // 1C: Boundary edge (+/-100 ticks)
        // Roll/Pitch: (100 * 32)/25 = +128 (0x0080 = +0.50 deg in Q8.8)
        // Yaw: (100 * 64)/15 = +426 (0x01AA = +1.66 deg/s in Q8.8)
        rc_roll <= 15'd18100; rc_pitch <= 15'd18100; rc_yaw <= 15'd18100; step_clk();
        check_assert(target_roll_angle == 128 && target_pitch_angle == 128 && target_yaw_rate == 426,
                     "DEADBAND_POS_100", "+100 ticks steps out of deadband with exact Q8.8 scaling", error_count);

        rc_roll <= 15'd17900; rc_pitch <= 15'd17900; rc_yaw <= 15'd17900; step_clk();
        check_assert(target_roll_angle == -128 && target_pitch_angle == -128 && target_yaw_rate == -426,
                     "DEADBAND_NEG_100", "-100 ticks steps out with 2's complement negative symmetry", error_count);

        // -------------------------------------------------------------
        // Test 2: Full-Scale Range & Sign Symmetry (+/-30 deg, +/-100 deg/s)
        // -------------------------------------------------------------
        rc_roll <= 15'd24000; rc_pitch <= 15'd24000; rc_yaw <= 15'd24000; step_clk();
        check_assert(target_roll_angle == 16'sd7680 && target_pitch_angle == 16'sd7680 && target_yaw_rate == 16'sd25600,
                     "MAX_DEFLECTION_POS", "Full positive stick maps to +30.0 deg and +100.0 deg/s", error_count);

        rc_roll <= 15'd12000; rc_pitch <= 15'd12000; rc_yaw <= 15'd12000; step_clk();
        check_assert(target_roll_angle == -16'sd7680 && target_pitch_angle == -16'sd7680 && target_yaw_rate == -16'sd25600,
                     "MAX_DEFLECTION_NEG", "Full negative stick maps to -30.0 deg and -100.0 deg/s", error_count);

        // -------------------------------------------------------------
        // Test 3: Throttle Saturation Clamping [12000, 24000]
        // -------------------------------------------------------------
        rc_throttle <= 15'd0;     step_clk(); check_assert(throttle_out == 12000, "THROTTLE_CLAMP_0", "0 ticks clamped to 12000", error_count);
        rc_throttle <= 15'd11999; step_clk(); check_assert(throttle_out == 12000, "THROTTLE_CLAMP_11999", "11999 ticks clamped to 12000", error_count);
        rc_throttle <= 15'd18000; step_clk(); check_assert(throttle_out == 18000, "THROTTLE_PASS_18000", "18000 ticks passes through", error_count);
        rc_throttle <= 15'd24001; step_clk(); check_assert(throttle_out == 24000, "THROTTLE_CLAMP_24001", "24001 ticks clamped to 24000", error_count);
        rc_throttle <= 15'd32767; step_clk(); check_assert(throttle_out == 24000, "THROTTLE_CLAMP_32767", "32767 ticks clamped to 24000", error_count);

        // Standardized Exit
        finalize_test_suite("RC MAPPER", error_count);
        $finish;
    end

endmodule
