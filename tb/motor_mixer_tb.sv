`timescale 1ns / 1ps

// =============================================================================
// Module: motor_mixer_tb
// Purpose: Unit verification for motor_mixer (Quad-X mixing matrix & ESC bounds)
// =============================================================================

import fc_tb_pkg::*;

module motor_mixer_tb();

    `GLOBAL_WATCHDOG(10ms)

    logic        clk;
    logic        rst_n;
    logic        armed;
    logic [14:0] throttle_in;
    logic signed [15:0] roll_correction;
    logic signed [15:0] pitch_correction;
    logic signed [15:0] yaw_correction;
    
    logic [14:0] motor_1;
    logic [14:0] motor_2;
    logic [14:0] motor_3;
    logic [14:0] motor_4;
    int error_count = 0;

    // Instantiate UUT
    motor_mixer uut (
        .clk(clk),
        .rst_n(rst_n),
        .armed(armed),
        .throttle_in(throttle_in),
        .roll_correction(roll_correction),
        .pitch_correction(pitch_correction),
        .yaw_correction(yaw_correction),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4)
    );

    // Clock Generator (50 MHz / 20 ns period for fast combinatorial evaluation)
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Concurrent SystemVerilog Assertions (SVA)
    property p_esc_limits;
        @(posedge clk) disable iff(!rst_n)
        (motor_1 >= 15'd12000 && motor_1 <= 15'd24000) &&
        (motor_2 >= 15'd12000 && motor_2 <= 15'd24000) &&
        (motor_3 >= 15'd12000 && motor_3 <= 15'd24000) &&
        (motor_4 >= 15'd12000 && motor_4 <= 15'd24000);
    endproperty
    assert property (p_esc_limits) else $error("[SVA] Motor output violated [12000, 24000] bounds!");

    property p_disarmed_lockout;
        @(posedge clk) disable iff(!rst_n)
        !armed |-> (motor_1 == 15'd12000 && motor_2 == 15'd12000 && motor_3 == 15'd12000 && motor_4 == 15'd12000);
    endproperty
    assert property (p_disarmed_lockout) else $error("[SVA] Motor output non-idle while disarmed!");

    // Helper task to check all 4 motors against expected values
    task automatic check_motors(
        input string tc_name,
        input logic [14:0] exp_m1, exp_m2, exp_m3, exp_m4
    );
        @(posedge clk); #1;
        check_assert(motor_1 === exp_m1 && motor_2 === exp_m2 && motor_3 === exp_m3 && motor_4 === exp_m4,
                     tc_name, $sformatf("M1=%0d, M2=%0d, M3=%0d, M4=%0d", motor_1, motor_2, motor_3, motor_4),
                     error_count);
    endtask

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Hardened Motor Mixer Saturation & Matrix Suite");
        $display("=================================================================");
        error_count      = 0;
        armed            <= 1'b0;
        throttle_in      <= 15'd12000;
        roll_correction  <= 16'sd0;
        pitch_correction <= 16'sd0;
        yaw_correction   <= 16'sd0;

        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        // Test 0: Disarmed Safety Interlock
        throttle_in      <= 15'd20000;
        roll_correction  <= 16'sd5000;
        pitch_correction <= 16'sd3000;
        yaw_correction   <= 16'sd2000;
        check_motors("DISARMED_LOCKOUT", 15'd12000, 15'd12000, 15'd12000, 15'd12000);

        // Arm the mixer
        armed <= 1'b1;

        // Test 1: Neutral Hover Throttle (18,000) & Zero Corrections
        throttle_in      <= 15'd18000;
        roll_correction  <= 16'sd0;
        pitch_correction <= 16'sd0;
        yaw_correction   <= 16'sd0;
        check_motors("NEUTRAL_HOVER", 15'd18000, 15'd18000, 15'd18000, 15'd18000);

        // Test 2: Upper Saturation Clamping (Clamping to 24,000)
        // M1 = T - R - P + Y: 22000 - (-4000) - (-4000) + 4000 = 34000 -> 24000
        throttle_in      <= 15'd22000;
        roll_correction  <= -16'sd4000;
        pitch_correction <= -16'sd4000;
        yaw_correction   <= 16'sd4000;
        check_motors("UPPER_CLAMP_M1", 15'd24000, 15'd18000, 15'd18000, 15'd18000);

        // Test 3: Lower Saturation Clamping (Clamping to 12,000)
        // M1 = 14000 - 6000 - 6000 + (-6000) = -4000 -> 12000
        throttle_in      <= 15'd14000;
        roll_correction  <= 16'sd6000;
        pitch_correction <= 16'sd6000;
        yaw_correction   <= -16'sd6000;
        check_motors("LOWER_CLAMP_M1", 15'd12000, 15'd20000, 15'd20000, 15'd20000);

        // Test 4A: Pure Roll Right (Roll > 0)
        throttle_in      <= 15'd18000;
        roll_correction  <= 16'sd3000;
        pitch_correction <= 16'sd0;
        yaw_correction   <= 16'sd0;
        check_motors("ROLL_RIGHT", 15'd15000, 15'd15000, 15'd21000, 15'd21000);

        // Test 4B: Pure Pitch Up (Pitch > 0)
        roll_correction  <= 16'sd0;
        pitch_correction <= 16'sd3000;
        yaw_correction   <= 16'sd0;
        check_motors("PITCH_UP", 15'd15000, 15'd21000, 15'd21000, 15'd15000);

        // Test 4C: Pure Yaw Clockwise (Yaw > 0)
        roll_correction  <= 16'sd0;
        pitch_correction <= 16'sd0;
        yaw_correction   <= 16'sd3000;
        check_motors("YAW_CW", 15'd21000, 15'd15000, 15'd21000, 15'd15000);

        // Test 5: Full 16-bit Extreme Arithmetic Stress (+32767 / -32768)
        throttle_in      <= 15'd24000;
        roll_correction  <= 16'sd32767;
        pitch_correction <= 16'sd32767;
        yaw_correction   <= -16'sd32768;
        @(posedge clk); #1;
        check_assert(motor_1 <= 24000 && motor_1 >= 12000 &&
                     motor_2 <= 24000 && motor_2 >= 12000 &&
                     motor_3 <= 24000 && motor_3 >= 12000 &&
                     motor_4 <= 24000 && motor_4 >= 12000,
                     "ARITHMETIC_STRESS", "Extreme 16-bit signed inputs clamped to ESC bounds without wrap-around", error_count);

        // Test 6: Control Authority Saturation Analysis (Throttle = 23,000)
        throttle_in      <= 15'd23000;
        roll_correction  <= 16'sd4000;
        pitch_correction <= 16'sd0;
        yaw_correction   <= 16'sd0;
        check_motors("HIGH_THROTTLE_AUTHORITY", 15'd19000, 15'd19000, 15'd24000, 15'd24000);

        // Standardized Exit
        finalize_test_suite("MOTOR MIXER", error_count);
        $finish;
    end

endmodule
