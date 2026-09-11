`timescale 1ns / 1ps

// =============================================================================
// Module: pid_calculator_tb
// Purpose: Unit verification for pid_calculator (Q8.8 math, anti-windup, gating)
// =============================================================================

import fc_tb_pkg::*;

module pid_calculator_tb();

    `GLOBAL_WATCHDOG(10ms)

    logic               clk;
    logic               rst_n;
    logic               enable;
    logic               clear_i;
    
    logic signed [15:0] target_val;
    logic signed [15:0] actual_val;
    logic signed [15:0] p_gain;
    logic signed [15:0] i_gain;
    logic signed [15:0] d_gain;
    
    logic signed [15:0] pid_correction;
    int error_count = 0;

    // Instantiate UUT
    pid_calculator uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .clear_i(clear_i),
        .target_val(target_val),
        .actual_val(actual_val),
        .p_gain(p_gain),
        .i_gain(i_gain),
        .d_gain(d_gain),
        .pid_correction(pid_correction)
    );

    // Clock Generator (10 ns period)
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // Task to strobe enable for 1 cycle synchronously
    task automatic trigger_update();
        @(posedge clk);
        enable <= 1'b1;
        @(posedge clk);
        enable <= 1'b0;
        @(posedge clk);
    endtask

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Hardened PID Calculator Verification Suite");
        $display("=================================================================");
        error_count = 0;
        enable      <= 1'b0;
        clear_i     <= 1'b0;
        target_val  <= 16'sd0;
        actual_val  <= 16'sd0;
        p_gain      <= 16'sd0;
        i_gain      <= 16'sd0;
        d_gain      <= 16'sd0;

        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        // Test 1: Standard Proportional Tracking
        // Target = +10.0 (0x0A00), Actual = +8.5 (0x0880), Error = +1.5, P_gain = 2.0 (0x0200) -> Output = +3.0 (0x0300)
        target_val <= 16'sh0A00;
        actual_val <= 16'sh0880;
        p_gain     <= 16'sh0200;
        i_gain     <= 16'sh0000;
        d_gain     <= 16'sh0000;
        @(posedge clk); #1;
        check_assert(pid_correction === 16'sh0300, "PROPORTIONAL_TRACKING",
                     "P-term produces exact +3.0 (0x0300) correction", error_count);
        trigger_update();

        // Test 2: Derivative Response
        // Step actual to +9.5 (0x0980). New Error = +0.5. Prev Error = +1.5. Deriv = -1.0. D_gain = 1.0 -> -1.0 (0xFF00)
        actual_val <= 16'sh0980;
        p_gain     <= 16'sh0000;
        d_gain     <= 16'sh0100;
        @(posedge clk); #1;
        check_assert(pid_correction === 16'shFF00, "DERIVATIVE_RESPONSE",
                     "D-term calculates change in error (-1.0 = 0xFF00) correctly", error_count);
        trigger_update();

        // Test 3: Positive Anti-Windup Clamping
        target_val <= 16'sh0A00;
        actual_val <= 16'sh0000;
        p_gain     <= 16'sh0000;
        i_gain     <= 16'sh0100; // 1.0
        d_gain     <= 16'sh0000;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        for (int i = 0; i < 5000; i++) trigger_update();
        @(posedge clk); #1;
        check_assert(pid_correction === 16'sh7FFF, "INTEGRAL_POS_CLAMP",
                     "Integral accumulator clamped cleanly to INT_MAX (0x7FFF) without wrapping", error_count);

        // Test 3b: Negative Anti-Windup Clamping
        target_val <= -16'sh0A00;
        for (int i = 0; i < 10000; i++) trigger_update();
        @(posedge clk); #1;
        check_assert(pid_correction === -16'sh8000, "INTEGRAL_NEG_CLAMP",
                     "Integral accumulator clamped cleanly to INT_MIN (-32768) without wrapping", error_count);

        // Test 4: Integral Reset via clear_i
        clear_i <= 1'b1;
        @(posedge clk);
        clear_i <= 1'b0;
        target_val <= 16'sh0000;
        trigger_update();
        @(posedge clk); #1;
        check_assert(pid_correction === 16'sh0000, "CLEAR_I_RESET",
                     "clear_i successfully zeroed the integral accumulator", error_count);

        // Test 5: Combined Output Saturation Clamping (+/-32767 / -32768)
        target_val <= 16'sh6400; // +100.0
        actual_val <= 16'sh0000;
        p_gain     <= 16'sh2000; // 32.0
        @(posedge clk); #1;
        check_assert(pid_correction === 16'sh7FFF, "COMBINED_CLAMP_POS",
                     "Combined PID output clamped cleanly to positive limit 0x7FFF", error_count);

        target_val <= -16'sh6400; // -100.0
        @(posedge clk); #1;
        check_assert(pid_correction === -16'sh8000, "COMBINED_CLAMP_NEG",
                     "Combined PID output clamped cleanly to negative limit 0x8000", error_count);

        // Test 6: Enable Strobe Gating
        `SYNC_RESET_RELEASE(clk, rst_n, 5)
        target_val <= 16'sh0500;
        p_gain     <= 16'sh0100;
        i_gain     <= 16'sh0100;
        enable     <= 1'b0; // HOLD ENABLE LOW
        repeat (100) @(posedge clk);
        check_assert(uut.integral_error === 32'sd0 && uut.prev_error === 16'sd0, "ENABLE_GATING",
                     "Accumulator state strictly held constant while enable=0", error_count);

        // Standardized Exit
        finalize_test_suite("PID CALCULATOR", error_count);
        $finish;
    end

endmodule
