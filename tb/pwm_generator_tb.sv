`timescale 1ns / 1ps

// =============================================================================
// Module: pwm_generator_tb
// Purpose: Unit verification for pwm_generator (400 Hz ESC driver, double-buffering)
// =============================================================================

import fc_tb_pkg::*;

module pwm_generator_tb();

    `GLOBAL_WATCHDOG(60ms)

    logic        clk;
    logic        rst_n;
    logic [14:0] duty_cycle;
    logic        pwm_out;
    int          error_count = 0;

    // Instantiate UUT
    pwm_generator uut (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(duty_cycle),
        .pwm_out(pwm_out)
    );

    // 12 MHz Master Clock (Period = 83.333 ns)
    initial begin
        clk = 1'b0;
        forever #CLK_HALF_PERIOD_NS clk = ~clk;
    end

    // Realtime measurement variables
    realtime rising_edge_time;
    realtime falling_edge_time;
    realtime period;
    realtime high_time;

    always @(posedge pwm_out) begin
        if (rising_edge_time != 0) begin
            period = $realtime - rising_edge_time;
        end
        rising_edge_time = $realtime;
    end

    always @(negedge pwm_out) begin
        falling_edge_time = $realtime;
        if (rising_edge_time != 0) begin
            high_time = falling_edge_time - rising_edge_time;
        end
    end

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Hardened PWM Generator Verification Suite");
        $display("=================================================================");
        error_count      = 0;
        rising_edge_time = 0;
        falling_edge_time = 0;
        high_time        = 0;
        period           = 0;

        // Synchronous Reset
        duty_cycle <= 15'd0;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        // Test 1: 10% Throttle (3,000 / 30,000 ticks = 0.25 ms high time)
        duty_cycle <= 15'd3000;
        #5_000_000; // 2 frames (5 ms)
        check_assert(is_within_tolerance_real(period, 2_500_000.0, 10_000.0) &&
                     is_within_tolerance_real(high_time, 250_000.0, 5_000.0),
                     "DUTY_10_PCT", "10% Duty Cycle verified (Period ~2.5ms, High ~0.25ms)", error_count);

        // Test 2: 50% Throttle (15,000 / 30,000 ticks = 1.25 ms high time)
        duty_cycle <= 15'd15000;
        #5_000_000;
        check_assert(is_within_tolerance_real(period, 2_500_000.0, 10_000.0) &&
                     is_within_tolerance_real(high_time, 1_250_000.0, 10_000.0),
                     "DUTY_50_PCT", "50% Duty Cycle verified (Period ~2.5ms, High ~1.25ms)", error_count);

        // Test 3: Shadow Register Double-Buffering & Glitch Elimination
        @(posedge pwm_out);
        #500_000; // 0.5 ms into the pulse (counter ~6,000, before 15,000 end)
        duty_cycle <= 15'd3000; // lower duty cycle mid-pulse
        #500_000; // 1.0 ms from start: pwm_out must remain high
        check_assert(pwm_out === 1'b1, "SHADOW_NO_GLITCH", "pwm_out stayed HIGH mid-pulse despite input drop", error_count);

        @(negedge pwm_out);
        check_assert(is_within_tolerance_real(high_time, 1_250_000.0, 10_000.0),
                     "SHADOW_FRAME_COMPLETE", "Current frame completed full 1.25ms pulse before latching", error_count);

        @(negedge pwm_out);
        check_assert(is_within_tolerance_real(high_time, 250_000.0, 5_000.0),
                     "SHADOW_NEXT_FRAME", "Next frame adopted new 10% duty cycle cleanly at boundary", error_count);

        // Test 4: Zero Duty Cycle (0 ticks, strictly LOW)
        duty_cycle <= 15'd0;
        #5_000_000;
        check_assert(pwm_out === 1'b0, "ZERO_DUTY_CYCLE", "0% duty cycle holds pwm_out strictly LOW without glitch", error_count);

        // Test 5: 100% Duty Cycle (30,000 ticks, continuously HIGH)
        duty_cycle <= 15'd30000;
        #5_000_000;
        check_assert(pwm_out === 1'b1, "FULL_DUTY_CYCLE", "100% duty cycle holds pwm_out continuously HIGH", error_count);

        // Test 6: Out-of-Bounds Input Protection (32,767 ticks)
        duty_cycle <= 15'd32767;
        #5_000_000;
        check_assert(uut.counter < 15'd30000 && pwm_out === 1'b1,
                     "OOB_INPUT_PROTECT", "Counter rolls over cleanly at 30,000 during out-of-bounds input", error_count);

        // Test 7: Asynchronous Reset Mid-Pulse
        duty_cycle <= 15'd15000;
        @(posedge pwm_out);
        #500_000;
        rst_n <= 1'b0;
        #20;
        check_assert(pwm_out === 1'b0 && uut.counter == 15'd0 && uut.duty_cycle_buf == 15'd0,
                     "ASYNC_RESET_MIDPULSE", "Asynchronous reset immediately silenced pwm_out and zeroed registers", error_count);
        rst_n <= 1'b1;

        // Standardized Exit
        finalize_test_suite("PWM GENERATOR", error_count);
        $finish;
    end

endmodule