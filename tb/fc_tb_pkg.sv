`timescale 1ns / 1ps

// =============================================================================
// Flight Controller Verification Package (fc_tb_pkg)
// Centralizes timing parameters, math checks, deadlock guards, and reporting.
// =============================================================================

// Global Watchdog Macro to prevent hung simulations
`define GLOBAL_WATCHDOG(max_time) \
    initial begin \
        #(max_time); \
        $fatal(1, "[WATCHDOG] Global simulation timeout (%0t) reached at %0t ps.", max_time, $time); \
    end

package fc_tb_pkg;

    // -------------------------------------------------------------------------
    // 1. Timing, Physical, and ESC Constants
    // -------------------------------------------------------------------------
    localparam realtime CLK_PERIOD_12MHZ_NS     = 83.333; // 12 MHz master clock
    localparam realtime CLK_HALF_PERIOD_NS      = 41.667;
    localparam time     DEFAULT_BOUNDED_TIMEOUT = 40ms;
    localparam time     GLOBAL_SIM_WATCHDOG     = 450ms;

    // ESC Standard PWM Pulse Limits (ticks at 12 MHz)
    localparam int PWM_MIN_TICKS     = 12000; // 1.000 ms
    localparam int PWM_NEUTRAL_TICKS = 18000; // 1.500 ms
    localparam int PWM_MAX_TICKS     = 24000; // 2.000 ms

    // Math & Fixed-Point Saturation Limits
    localparam int INT16_MAX = 32767;
    localparam int INT16_MIN = -32768;

    // -------------------------------------------------------------------------
    // 2. Flight Controller Math & Fixed-Point Hygiene Functions
    // -------------------------------------------------------------------------
    function automatic int abs_diff(input int a, input int b);
        return (a >= b) ? (a - b) : (b - a);
    endfunction

    function automatic real abs_diff_real(input real a, input real b);
        return (a >= b) ? (a - b) : (b - a);
    endfunction

    function automatic bit is_within_tolerance(input int actual, input int expected, input int tol);
        return (abs_diff(actual, expected) <= tol);
    endfunction

    function automatic bit is_within_tolerance_real(input real actual, input real expected, input real tol);
        return (abs_diff_real(actual, expected) <= tol);
    endfunction

    function automatic bit is_clamped_16b(input int val);
        return (val >= INT16_MIN && val <= INT16_MAX);
    endfunction

    function automatic real q8_8_to_deg(input shortint val);
        return real'(val) / 256.0;
    endfunction

// -------------------------------------------------------------------------
// 3. Synchronous Reset & Bounded Handshake Macros
// -------------------------------------------------------------------------

// Synchronous Reset Macro: Asserts reset, holds for specified clock cycles, releases on negedge
`define SYNC_RESET_RELEASE(clk, rst_n, hold_cycles) \
    begin \
        @(negedge (clk)); \
        rst_n <= 1'b0; \
        repeat (hold_cycles) @(posedge (clk)); \
        @(negedge (clk)); \
        rst_n <= 1'b1; \
        @(posedge (clk)); \
    end

// Cycle-Bounded Signal Level Wait Guard Macro (Deadlock prevention)
`define AWAIT_SIGNAL_LEVEL(clk, sig, target_val, max_cycles, err_context) \
    begin \
        automatic int __cycles = 0; \
        while (((sig) !== (target_val)) && (__cycles < (max_cycles))) begin \
            @(posedge (clk)); \
            __cycles++; \
        end \
        if ((sig) !== (target_val)) begin \
            $fatal(2, "[DEADLOCK] Timed out waiting for %s == %b after %0d cycles at %0t ps", \
                   (err_context), (target_val), (max_cycles), $time); \
        end \
    end

// Macro to measure pulse width with bounded timeout guard (avoids ref in fork-join limitation)
`define MEASURE_PULSE_BOUNDED(sig, width_var, timeout_dur, err_name) \
    begin \
        realtime __t_rise, __t_fall; \
        bit __timed_out = 0; \
        fork \
            begin \
                @(posedge (sig)); \
                __t_rise = $realtime; \
                @(negedge (sig)); \
                __t_fall = $realtime; \
                (width_var) = (__t_fall - __t_rise) / 1.0e6; \
            end \
            begin \
                #(timeout_dur); \
                __timed_out = 1; \
            end \
        join_any \
        disable fork; \
        if (__timed_out) begin \
            $fatal(2, "[DEADLOCK] Timed out waiting for pulse on %s (limit: %0t) at %0t ps", (err_name), (timeout_dur), $time); \
        end \
    end

    // -------------------------------------------------------------------------
    // 5. Verification Reporting & Exit Status Standardization
    // -------------------------------------------------------------------------
    function automatic void check_assert(
        input bit condition,
        input string tag,
        input string msg,
        inout int err_cnt
    );
        assert (condition) begin
            $display("[PASS] %s: %s", tag, msg);
        end else begin
            err_cnt++;
            $error("[FAIL] %s: %s", tag, msg);
        end
    endfunction

    function automatic void finalize_test_suite(
        input string tb_name,
        input int err_cnt
    );
        $display("\n=================================================================");
        $display("[TB SUMMARY] %s Completed: %0d failures detected.", tb_name, err_cnt);
        if (err_cnt == 0) begin
            $display("[TB RESULT] ALL %s TESTS PASSED STRICT VERIFICATION!", tb_name);
            $display("=================================================================");
            $finish;
        end else begin
            $display("[TB RESULT] %s FAILED WITH %0d VIOLATIONS!", tb_name, err_cnt);
            $display("=================================================================");
            $fatal(2, "Aborting simulation due to verification failures in %s.", tb_name);
        end
    endfunction

endpackage
