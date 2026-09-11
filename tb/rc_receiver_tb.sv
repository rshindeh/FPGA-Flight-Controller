`timescale 1ns / 1ps

// =============================================================================
// Module: rc_receiver_tb
// Purpose: Unit verification for rc_receiver (4-ch PWM decoder, glitch filter, watchdog)
// =============================================================================

import fc_tb_pkg::*;

module rc_receiver_tb();

    `GLOBAL_WATCHDOG(300ms)

    logic        clk;
    logic        rst_n;
    logic [3:0]  ppm_in;
    logic [14:0] channel_out[4];
    logic [3:0]  valid;
    int error_count = 0;

    // Instantiate UUT
    rc_receiver #(
        .NUM_CHANNELS(4),
        .CLK_FREQ_HZ(12_000_000),
        .MIN_PULSE_US(1000),
        .MAX_PULSE_US(2000),
        .WD_TIMEOUT_MS(50)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .ppm_in(ppm_in),
        .channel_out(channel_out),
        .valid(valid)
    );

    // 12 MHz Master Clock
    initial begin
        clk = 1'b0;
        forever #CLK_HALF_PERIOD_NS clk = ~clk;
    end

    // Concurrent SystemVerilog Assertions (SVA)
    genvar gi;
    generate
        for (gi = 0; gi < 4; gi++) begin : gen_sva_bounds
            property p_ch_bounds;
                @(posedge clk) disable iff(!rst_n)
                valid[gi] |-> (channel_out[gi] >= 15'd12000 && channel_out[gi] <= 15'd24000);
            endproperty
            assert property (p_ch_bounds) else $error("[SVA] Ch%0d output out of bounds: %0d", gi, channel_out[gi]);
        end
    endgenerate

    // Helper task to send pulse with non-blocking drives
    task automatic send_pulse(input int channel_idx, input real duration_ms);
        ppm_in[channel_idx] <= 1'b1;
        #(duration_ms * 1_000_000);
        ppm_in[channel_idx] <= 1'b0;
    endtask

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Hardened RC Receiver Multi-Channel Verification");
        $display("=================================================================");
        error_count = 0;
        ppm_in      <= 4'b0000;

        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        // Test 1: Neutral Default Values (Disarmed/Unlinked State)
        check_assert(channel_out[0] == 18000 && channel_out[1] == 18000 &&
                     channel_out[2] == 18000 && channel_out[3] == 12000 && valid == 4'b0000,
                     "DEFAULT_NEUTRAL", "Safe default values verified on all 4 channels (valid=0000)", error_count);

        // Test 2: Standard Pulse Decoding Across All 4 Channels
        send_pulse(0, 1.5);  #5_000_000; send_pulse(0, 1.5);  #5_000_000;
        send_pulse(1, 1.75); #5_000_000; send_pulse(1, 1.75); #5_000_000;
        send_pulse(2, 1.25); #5_000_000; send_pulse(2, 1.25); #5_000_000;
        send_pulse(3, 1.0);  #5_000_000; send_pulse(3, 1.0);  #5_000_000;

        check_assert(is_within_tolerance(channel_out[0], 18000, 20) && valid[0] &&
                     is_within_tolerance(channel_out[1], 21000, 20) && valid[1] &&
                     is_within_tolerance(channel_out[2], 15000, 20) && valid[2] &&
                     is_within_tolerance(channel_out[3], 12000, 20) && valid[3],
                     "CH_DECODE", "All 4 channels decoded accurately with valid flags asserted", error_count);

        // Test 3: Limits & Clamping Check (0.95 ms and 2.05 ms)
        send_pulse(3, 0.95); #5_000_000;
        send_pulse(0, 2.05); #5_000_000;
        check_assert(channel_out[0] == 24000 && valid[0] && channel_out[3] == 12000 && valid[3],
                     "PULSE_CLAMPING", "Out-of-range pulses clamped to 12,000 and 24,000", error_count);

        // Test 4: 500 ns Runt Glitch Rejection
        ppm_in[1] <= 1'b1; #500; ppm_in[1] <= 1'b0; #5_000_000;
        check_assert(channel_out[1] == 18000 && valid[1] === 1'b0, "GLITCH_REJECTION",
                     "500 ns runt glitch rejected and channel held at safe default", error_count);

        // Test 5: Out-Of-Bounds Pulse (0.8 ms)
        send_pulse(0, 0.8); #5_000_000;
        check_assert(valid[0] === 1'b0, "OOB_LOW_REJECT",
                     "Out-of-bounds low pulse (0.8 ms) correctly de-asserted valid flag", error_count);

        // Re-establish valid signal
        send_pulse(0, 1.5); #5_000_000; send_pulse(0, 1.5); #5_000_000;

        // Test 6: Pulse Timeout Protection (4.0 ms)
        send_pulse(0, 4.0); #5_000_000;
        check_assert(channel_out[0] == 18000 && valid[0] === 1'b0, "PULSE_TIMEOUT",
                     "4.0 ms pulse triggered timeout protection, restoring safe default", error_count);

        // Restore valid pulses on all channels
        send_pulse(0, 1.5); #5_000_000; send_pulse(0, 1.5); #5_000_000;
        send_pulse(1, 1.5); #5_000_000; send_pulse(1, 1.5); #5_000_000;
        send_pulse(2, 1.5); #5_000_000; send_pulse(2, 1.5); #5_000_000;
        send_pulse(3, 1.2); #5_000_000; send_pulse(3, 1.2); #5_000_000;

        // Test 7: Watchdog Connection-Loss Failsafe (55 ms silence)
        #55_000_000;
        check_assert(channel_out[0] == 18000 && channel_out[1] == 18000 &&
                     channel_out[2] == 18000 && channel_out[3] == 12000 && valid == 4'b0000,
                     "WATCHDOG_FAILSAFE", "Watchdog timeout (55 ms) deasserted valid and restored safe defaults", error_count);

        // Test 8: Staggered Overlapping Concurrent Channels
        fork
            begin send_pulse(0, 1.3); end
            begin #400_000; send_pulse(1, 1.6); end
            begin #800_000; send_pulse(2, 1.4); end
            begin #1_200_000; send_pulse(3, 1.1); end
        join
        #5_000_000;
        fork
            begin send_pulse(0, 1.3); end
            begin #400_000; send_pulse(1, 1.6); end
            begin #800_000; send_pulse(2, 1.4); end
            begin #1_200_000; send_pulse(3, 1.1); end
        join
        #5_000_000;

        check_assert(is_within_tolerance(channel_out[0], 15600, 20) && valid[0] &&
                     is_within_tolerance(channel_out[1], 19200, 20) && valid[1] &&
                     is_within_tolerance(channel_out[2], 16800, 20) && valid[2] &&
                     is_within_tolerance(channel_out[3], 13200, 20) && valid[3],
                     "STAGGERED_CHANNELS", "All 4 concurrent overlapping channels decoded accurately", error_count);

        // Test 9: Sub-Cycle Asynchronous Clock Phase Offsets
        #37; // 37 ns sub-cycle phase offset
        send_pulse(0, 1.5); #5_000_000; send_pulse(0, 1.5); #5_000_000;
        check_assert(is_within_tolerance(channel_out[0], 18000, 20) && valid[0],
                     "METASTABLE_PHASE", "Pulse arriving with 37 ns clock offset cleanly synchronized and decoded", error_count);

        // Standardized Exit
        finalize_test_suite("RC RECEIVER", error_count);
        $finish;
    end

endmodule
