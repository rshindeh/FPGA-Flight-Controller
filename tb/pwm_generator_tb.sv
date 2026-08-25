`timescale 1ns / 1ps

module pwm_generator_tb();

    // 1. Signals
    logic        clk;
    logic        rst_n;
    logic [14:0] duty_cycle;
    logic        pwm_out;

    // 2. Instantiate UUT
    pwm_generator uut (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(duty_cycle),
        .pwm_out(pwm_out)
    );

    // 3. Generate 12 MHz Clock (Period = 83.333 ns, toggle every 41.667 ns)
    always begin
        clk = 1'b1;
        #41.667;
        clk = 1'b0;
        #41.667;
    end

    // Realtime measurement variables
    realtime rising_edge_time;
    realtime falling_edge_time;
    realtime period;
    realtime high_time;
    realtime frequency;
    realtime duty_cycle_pct;

    always @(posedge pwm_out) begin
        if (rising_edge_time != 0) begin
            period = $realtime - rising_edge_time;
            frequency = 1.0e9 / period;
            duty_cycle_pct = (high_time / period) * 100.0;
            $display("[MONITOR] Time = %0.1f ns | Period = %0.2f ns, Freq = %0.2f Hz, Duty Cycle = %0.2f%%", 
                     $realtime, period, frequency, duty_cycle_pct);
        end
        rising_edge_time = $realtime;
    end

    always @(negedge pwm_out) begin
        falling_edge_time = $realtime;
        if (rising_edge_time != 0) begin
            high_time = falling_edge_time - rising_edge_time;
        end
    end

    // 4. Test Stimulus Block
    initial begin
        $display("[TB] Starting PWM Generator Verification...");
        rising_edge_time = 0;
        falling_edge_time = 0;
        high_time = 0;
        period = 0;

        // Initialize Signals
        rst_n = 1'b0;
        duty_cycle = 15'd0;
        #100;
        rst_n = 1'b1;
        #100;

        // ========================================================
        // TEST CASE 1: 10% Throttle (3,000 / 30,000 ticks = 0.25 ms high time)
        // ========================================================
        $display("[TB] --- Test Case 1: Set 10%% Throttle (3,000 ticks) ---");
        duty_cycle = 15'd3000;
        
        // Wait for 2 full periods (5 ms)
        #5000000;
        
        if (period >= 2490000 && period <= 2510000 && high_time >= 249000 && high_time <= 251000) begin
            $display("[TB] PASS: 10%% Duty Cycle verified (Period ~2.5ms, High ~0.25ms).");
        end else begin
            $display("[TB] FAIL: 10%% Duty Cycle mismatched! Period = %0.1f ns, High = %0.1f ns", period, high_time);
        end

        // ========================================================
        // TEST CASE 2: 50% Throttle (15,000 / 30,000 ticks = 1.25 ms high time)
        // ========================================================
        $display("[TB] --- Test Case 2: Set 50%% Throttle (15,000 ticks) ---");
        duty_cycle = 15'd15000;
        
        // Wait for 2 full periods (5 ms)
        #5000000;
        
        if (period >= 2490000 && period <= 2510000 && high_time >= 1240000 && high_time <= 1260000) begin
            $display("[TB] PASS: 50%% Duty Cycle verified (Period ~2.5ms, High ~1.25ms).");
        end else begin
            $display("[TB] FAIL: 50%% Duty Cycle mismatched! Period = %0.1f ns, High = %0.1f ns", period, high_time);
        end

        // ========================================================
        // TEST CASE 3: Double-Buffering & Glitch Elimination Check
        // ========================================================
        $display("[TB] --- Test Case 3: Double-Buffering Shadow Register Check ---");
        // Wait for a new rising edge (start of frame)
        @(posedge pwm_out);
        // Wait 0.5 ms into the pulse (while counter is ~6,000 ticks, well before the 15,000 tick end)
        #500000;
        // Now abruptly drop duty_cycle to 3,000 (which is less than current counter ~6,000)
        duty_cycle = 15'd3000;
        $display("[TB] Changed duty_cycle input to 3000 mid-pulse at %0.1f ns (pwm_out is %b)", $realtime, pwm_out);
        
        // If double buffering works, pwm_out MUST remain high until 15,000 ticks (1.25 ms from rising edge)
        #500000; // 1.0 ms from rising edge
        if (pwm_out === 1'b1) begin
            $display("[TB] PASS: pwm_out stayed HIGH at 1.0 ms despite duty_cycle input dropping below current counter!");
        end else begin
            $display("[TB] FAIL: Mid-frame glitch detected! pwm_out dropped prematurely.");
        end

        // Wait for falling edge
        @(negedge pwm_out);
        if (high_time >= 1240000 && high_time <= 1260000) begin
            $display("[TB] PASS: Current frame completed full 1.25ms pulse before loading new duty cycle.");
        end else begin
            $display("[TB] FAIL: Frame high time was truncated to %0.1f ns", high_time);
        end

        // Wait for next frame to verify that the new 10% duty cycle (3,000 ticks) is now active
        @(negedge pwm_out);
        if (high_time >= 249000 && high_time <= 251000) begin
            $display("[TB] PASS: Next frame cleanly adopted new 10%% (0.25ms) duty cycle at frame boundary.");
        end else begin
            $display("[TB] FAIL: Next frame high time incorrect: %0.1f ns", high_time);
        end

        $display("[TB] PWM Generator Verification completed successfully.");
        $finish;
    end

endmodule