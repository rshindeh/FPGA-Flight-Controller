`timescale 1ns / 1ps

// =============================================================================
// Module: safety_mgr_tb
// Purpose: Unit verification for safety_mgr (Arming/Disarming FSM, RC interlocks)
// =============================================================================

import fc_tb_pkg::*;

module safety_mgr_tb();

    `GLOBAL_WATCHDOG(20ms)

    localparam int TEST_ARM_CYCLES = 120; // 10 us at 12 MHz for fast simulation

    logic        clk;
    logic        rst_n;
    logic [14:0] throttle_in;
    logic [14:0] yaw_in;
    logic [3:0]  rc_valid;
    logic        armed;
    logic        idle_throttle_active;
    int          error_count = 0;

    // Instantiate UUT
    safety_mgr #(
        .ARM_TIME_CYCLES(TEST_ARM_CYCLES)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .throttle_in(throttle_in),
        .yaw_in(yaw_in),
        .rc_valid(rc_valid),
        .armed(armed),
        .idle_throttle_active(idle_throttle_active)
    );

    // 12 MHz Master Clock
    initial begin
        clk = 1'b0;
        forever #CLK_HALF_PERIOD_NS clk = ~clk;
    end

    // Concurrent SystemVerilog Assertions (SVA)
    property p_rc_loss_disarm;
        @(posedge clk) disable iff(!rst_n) !(&rc_valid) |=> !armed;
    endproperty
    assert property (p_rc_loss_disarm) else $error("[SVA] Armed asserted during RC signal loss!");

    property p_disarmed_idle;
        @(posedge clk) disable iff(!rst_n) !armed |-> idle_throttle_active;
    endproperty
    assert property (p_disarmed_idle) else $error("[SVA] idle_throttle_active deasserted while disarmed!");

    property p_flying_idle_inactive;
        @(posedge clk) disable iff(!rst_n) (armed && throttle_in >= 15'd12600) |-> !idle_throttle_active;
    endproperty
    assert property (p_flying_idle_inactive) else $error("[SVA] idle_throttle_active asserted at flight throttle!");

    // Helper task to wait clock cycles synchronously
    task automatic wait_clk(input int cycles);
        repeat (cycles) @(posedge clk);
    endtask

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Safety Manager Comprehensive Unit Verification");
        $display("=================================================================");
        error_count = 0;

        // Test 1: Synchronous Reset
        throttle_in <= 15'd12000;
        yaw_in      <= 15'd18000;
        rc_valid    <= 4'b0000;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)
        wait_clk(2);
        check_assert(!armed && idle_throttle_active, "RESET_INIT",
                     "Reset initialized armed=0 and idle_throttle_active=1", error_count);

        // Test 2: Arming Command Hold (Throttle Min, Yaw Full Right)
        rc_valid    <= 4'b1111;
        throttle_in <= 15'd12000; // < 12500
        yaw_in      <= 15'd23500; // > 23000

        wait_clk(50);
        check_assert(!armed, "ARM_PREMATURE", "System did not arm prematurely at 50 cycles", error_count);

        wait_clk(TEST_ARM_CYCLES - 50 + 5);
        check_assert(armed, "ARM_SUCCESS", "Successfully armed after timer duration", error_count);

        // Neutralize sticks in armed state
        throttle_in <= 15'd18000;
        yaw_in      <= 15'd18000;
        wait_clk(10);
        check_assert(armed, "STICK_RELEASE", "Sticks returned to neutral maintained ARMED state", error_count);

        // Test 3: Disarming Sequence (Throttle Min, Yaw Full Left)
        throttle_in <= 15'd12000; // < 12500
        yaw_in      <= 15'd12500; // < 13000
        wait_clk(TEST_ARM_CYCLES + 5);
        check_assert(!armed, "DISARM_SUCCESS", "System disarmed cleanly after disarm command hold", error_count);

        // Test 4: Arming Timer Glitch / Jitter Reset
        throttle_in <= 15'd12000;
        yaw_in      <= 15'd23500;
        wait_clk(TEST_ARM_CYCLES - 10); // 110 cycles into 120-cycle arming
        
        // 2-cycle stick jitter drops yaw below threshold
        yaw_in <= 15'd22900;
        wait_clk(2);
        yaw_in <= 15'd23500;
        wait_clk(20);
        check_assert(!armed, "DEBOUNCE_JITTER", "Arming timer restarted upon jitter, preventing false arm", error_count);

        wait_clk(TEST_ARM_CYCLES);
        check_assert(armed, "ARM_POST_JITTER", "Cleanly armed after full continuous hold post-jitter", error_count);

        // Test 5: Instant Disarm on Single-Channel Signal Loss
        rc_valid <= 4'b1101; // drop channel 1
        wait_clk(2);
        check_assert(!armed, "PARTIAL_RC_LOSS", "Partial RC loss (4'b1101) instantly disarmed system in <= 2 cycles", error_count);

        // Re-arm for idle boundary tests
        rc_valid    <= 4'b1111;
        throttle_in <= 15'd12000;
        yaw_in      <= 15'd23500;
        wait_clk(TEST_ARM_CYCLES + 5);
        check_assert(armed, "REARM_CONFIRM", "Re-armed successfully", error_count);

        // Test 6: Idle Throttle Flag Boundary Checks
        yaw_in      <= 15'd18000;
        throttle_in <= 15'd12599; // < 12600: must be active
        wait_clk(3);
        check_assert(idle_throttle_active, "IDLE_BOUND_LOW", "Throttle 12599 correctly asserted idle_throttle_active", error_count);

        throttle_in <= 15'd12600; // >= 12600: must deassert
        wait_clk(3);
        check_assert(!idle_throttle_active, "IDLE_BOUND_HIGH", "Throttle 12600 correctly deasserted idle_throttle_active", error_count);

        // Disarm and check idle throttle lockout
        throttle_in <= 15'd12000;
        yaw_in      <= 15'd12500;
        wait_clk(TEST_ARM_CYCLES + 5);
        throttle_in <= 15'd24000; // Full throttle while disarmed
        wait_clk(3);
        check_assert(idle_throttle_active && !armed, "DISARM_FULL_THROTTLE",
                     "Disarmed state strictly forced idle_throttle_active at max throttle", error_count);

        // Test 7: False Trigger Immunity
        throttle_in <= 15'd20000;
        yaw_in      <= 15'd24000;
        wait_clk(TEST_ARM_CYCLES * 2);
        check_assert(!armed, "FALSE_ARM_IMMUNITY", "High throttle + Yaw Right did not arm", error_count);

        // Standardized Exit
        finalize_test_suite("SAFETY MANAGER", error_count);
        $finish;
    end

endmodule
