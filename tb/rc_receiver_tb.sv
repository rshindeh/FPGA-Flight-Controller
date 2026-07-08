`timescale 1ns / 1ps

module rc_receiver_tb();

    // 1. Signals
    logic        clk;
    logic        rst_n;
    logic [3:0]  ppm_in;
    logic [14:0] channel_out[4];
    logic [3:0]  valid;

    // 2. Instantiate UUT (Unit Under Test)
    rc_receiver #(
        .NUM_CHANNELS(4),
        .CLK_FREQ_HZ(12_000_000),
        .MIN_PULSE_US(1000),
        .MAX_PULSE_US(2000),
        .WD_TIMEOUT_MS(50) // Reduced watchdog timeout to 50 ms to speed up simulation
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .ppm_in(ppm_in),
        .channel_out(channel_out),
        .valid(valid)
    );

    // 3. Generate 12 MHz Clock (Period = 83.33 ns)
    always begin
        clk = 1'b1;
        #41.67;
        clk = 1'b0;
        #41.67;
    end

    // Helper task to send a pulse with specific duration on a channel
    task send_pulse(input int channel_idx, input real duration_ms);
        begin
            $display("[TB] Channel %0d: Sending %0.2f ms pulse...", channel_idx, duration_ms);
            ppm_in[channel_idx] = 1'b1;
            #(duration_ms * 1_000_000);
            ppm_in[channel_idx] = 1'b0;
        end
    endtask

    // 4. Test Stimulus Block
    initial begin
        $display("[TB] Starting RC Receiver Testbench...");
        
        // Initialize inputs
        rst_n  = 1'b0;
        ppm_in = 4'b0000;
        #200;
        
        // Release reset
        rst_n = 1'b1;
        #1000;
        $display("[TB] Reset released. Initial valid = %b", valid);
        
        // Test 1: Neutral Default Value Check (during startup, valid should be 0)
        if (channel_out[0] == 18000 && channel_out[3] == 12000 && valid == 4'b0000) begin
            $display("[TB] PASS: Initial default values correct (Roll=18000, Throttle=12000, valid=0000)");
        end else begin
            $display("[TB] FAIL: Initial default values incorrect (Roll=%0d, Throttle=%0d, valid=%b)", 
                     channel_out[0], channel_out[3], valid);
        end

        // Test 2: Normal Pulses
        // Send two consecutive clean pulses on Channel 0 (Roll, nominal 1.5 ms) and Channel 3 (Throttle, nominal 1.0 ms)
        // Note: The second pulse will lock in the valid flag since it sees consecutive clean periods
        send_pulse(0, 1.5);
        #5000000; // wait 5 ms
        send_pulse(0, 1.5);
        #5000000; // wait 5 ms
        
        send_pulse(3, 1.0);
        #5000000;
        send_pulse(3, 1.0);
        #5000000;

        $display("[TB] After normal pulses: Roll (Ch0) = %0d (valid=%b), Throttle (Ch3) = %0d (valid=%b)",
                 channel_out[0], valid[0], channel_out[3], valid[3]);

        if (channel_out[0] >= 17990 && channel_out[0] <= 18010 && valid[0] == 1'b1 && channel_out[3] == 12000 && valid[3] == 1'b1) begin
            $display("[TB] PASS: Normal pulse widths decoded successfully");
        end else begin
            $display("[TB] FAIL: Normal pulse widths decoded incorrectly");
        end

        // Test 3: Limits & Clamping Check
        // Send a 0.95 ms pulse on Ch3 (Throttle) -> should clamp to 12,000
        // Send a 2.05 ms pulse on Ch0 (Roll) -> should clamp to 24,000
        send_pulse(3, 0.95);
        #5000000;
        send_pulse(0, 2.05);
        #5000000;
        
        $display("[TB] After clamping check: Roll (Ch0) = %0d (valid=%b), Throttle (Ch3) = %0d (valid=%b)",
                 channel_out[0], valid[0], channel_out[3], valid[3]);

        if (channel_out[0] == 24000 && valid[0] == 1'b1 && channel_out[3] == 12000 && valid[3] == 1'b1) begin
            $display("[TB] PASS: Clamping limits enforced correctly");
        end else begin
            $display("[TB] FAIL: Clamping limits enforced incorrectly");
        end

        // Test 4: Out-Of-Bounds (OOB) Check
        // Send a 0.8 ms pulse on Ch0 (Roll) -> too short, should invalidate Ch0 output (de-assert valid, reset to safe default)
        send_pulse(0, 0.8);
        #5000000;
        
        $display("[TB] After OOB Low pulse: Roll (Ch0) = %0d, valid[0] = %b", channel_out[0], valid[0]);
        if (channel_out[0] == 18000 && valid[0] == 1'b0) begin
            $display("[TB] PASS: Out-of-bounds low pulse correctly handled");
        end else begin
            $display("[TB] FAIL: Out-of-bounds low pulse incorrectly handled");
        end

        // Re-establish valid signal on Ch0
        send_pulse(0, 1.5);
        #5000000;
        send_pulse(0, 1.5);
        #5000000;
        $display("[TB] Re-established Roll (Ch0) = %0d, valid[0] = %b", channel_out[0], valid[0]);

        // Test 5: Pulse Timeout Protection
        // Send a 4.0 ms pulse on Ch0 (should timeout at 3.0 ms, invalidate output, set to safe default)
        send_pulse(0, 4.0);
        #5000000;
        $display("[TB] After pulse timeout check: Roll (Ch0) = %0d, valid[0] = %b", channel_out[0], valid[0]);
        if (channel_out[0] == 18000 && valid[0] == 1'b0) begin
            $display("[TB] PASS: Pulse width timeout triggered and handled successfully");
        end else begin
            $display("[TB] FAIL: Pulse width timeout failed to trigger or handle");
        end

        // Re-establish valid signals on both Ch0 and Ch3
        send_pulse(0, 1.5); #5000000; send_pulse(0, 1.5); #5000000;
        send_pulse(3, 1.2); #5000000; send_pulse(3, 1.2); #5000000;
        $display("[TB] Re-established Ch0/Ch3 before watchdog test: valid = %b", valid);

        // Test 6: Watchdog Failsafe Check
        // No pulses sent. Wait for WD_TIMEOUT_MS = 50 ms.
        $display("[TB] Waiting 55 ms to test Watchdog failsafe...");
        #55000000; // 55 ms

        $display("[TB] After watchdog timeout: Roll (Ch0) = %0d, valid[0] = %b | Throttle (Ch3) = %0d, valid[3] = %b",
                 channel_out[0], valid[0], channel_out[3], valid[3]);

        if (channel_out[0] == 18000 && valid[0] == 1'b0 && channel_out[3] == 12000 && valid[3] == 1'b0) begin
            $display("[TB] PASS: Watchdog timeout successfully de-asserted valid outputs and forced defaults");
        end else begin
            $display("[TB] FAIL: Watchdog timeout check failed");
        end

        $display("[TB] Simulation completed successfully.");
        $finish;
    end

endmodule
