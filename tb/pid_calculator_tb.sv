`timescale 1ns / 1ps

module pid_calculator_tb();

    // 1. Testbench Signals
    logic               clk;
    logic               rst_n;
    logic               enable;
    
    logic signed [15:0] target_val;
    logic signed [15:0] actual_val;
    logic signed [15:0] p_gain;
    logic signed [15:0] i_gain;
    logic signed [15:0] d_gain;
    
    logic signed [15:0] pid_correction;

    // 2. Instantiate UUT (Unit Under Test)
    pid_calculator uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .target_val(target_val),
        .actual_val(actual_val),
        .p_gain(p_gain),
        .i_gain(i_gain),
        .d_gain(d_gain),
        .pid_correction(pid_correction)
    );

    // 3. Generate Clock
    always begin
        clk = 1'b1;
        #5;
        clk = 1'b0;
        #5;
    end

    // Helper task to pulse enable for 1 clock cycle
    task trigger_update();
        begin
            enable = 1'b1;
            #10;
            enable = 1'b0;
            #10;
        end
    endtask

    // 4. Test Stimulus Block
    initial begin
        $display("[TB] Starting PID Calculator Verification...");
        
        // Reset
        rst_n = 1'b0;
        enable = 1'b0;
        target_val = 16'sd0;
        actual_val = 16'sd0;
        p_gain = 16'sd0;
        i_gain = 16'sd0;
        d_gain = 16'sd0;
        #20;
        rst_n = 1'b1;
        #10;

        // ==========================================
        // TEST CASE 1: Standard Proportional Tracking
        // ==========================================
        $display("[TB] --- Test Case 1: Standard Proportional Tracking ---");
        // Target = +10.0 (0x0A00), Actual = +8.5 (0x0880)
        // Error = +1.5. P_gain = 2.0 (0x0200)
        // Expected Proportional Correction = +3.0 (0x0300)
        target_val = 16'sh0A00;
        actual_val = 16'sh0880;
        p_gain = 16'sh0200;
        i_gain = 16'sh0000;
        d_gain = 16'sh0000;
        #10;
        
        $display("[TB] Target: 0x%h, Actual: 0x%h, P-Gain: 0x%h", target_val, actual_val, p_gain);
        $display("[TB] Output Correction: 0x%h (Expected: 0x0300)", pid_correction);
        
        if (pid_correction == 16'sh0300) begin
            $display("[TB] PASS: Proportional output matches exactly +3.0 after scaling.");
        end else begin
            $display("[TB] FAIL: Proportional output mismatched.");
        end
        trigger_update(); // lock in the state

        // ==========================================
        // TEST CASE 2: Derivative Response
        // ==========================================
        $display("[TB] --- Test Case 2: Derivative Response ---");
        // Step actual value to +9.5 (0x0980). New Error = +0.5.
        // Prev Error = +1.5. Derivative Error = New - Prev = 0.5 - 1.5 = -1.0.
        // D_gain = 1.0 (0x0100).
        // Expected Derivative Correction = -1.0 (0xFF00)
        target_val = 16'sh0A00;
        actual_val = 16'sh0980;
        p_gain = 16'sh0000; // Disable P
        i_gain = 16'sh0000;
        d_gain = 16'sh0100;
        #10; // Let combinational logic settle
        
        $display("[TB] New Actual: 0x%h, D-Gain: 0x%h", actual_val, d_gain);
        $display("[TB] Output Correction: 0x%h (Expected: 0xff00)", pid_correction);

        if (pid_correction == 16'shFF00) begin
            $display("[TB] PASS: Derivative output calculates change in error (-1.0) correctly.");
        end else begin
            $display("[TB] FAIL: Derivative output mismatched.");
        end
        trigger_update();

        // ==========================================
        // TEST CASE 3: Integral Accumulation and Anti-Windup
        // ==========================================
        $display("[TB] --- Test Case 3: Integral Accumulation and Anti-Windup ---");
        
        // Reset everything to start fresh
        rst_n = 1'b0; #20; rst_n = 1'b1; #10;
        
        // Target = +10.0 (0x0A00), Actual = 0.0
        // Error = +10.0 (2560 ticks). I_gain = 1.0 (0x0100).
        target_val = 16'sh0A00;
        actual_val = 16'sh0000;
        p_gain = 16'sh0000;
        i_gain = 16'sh0100;
        d_gain = 16'sh0000;
        
        // Loop 5000 times. 10.0 * 5000 = 50,000, which exceeds 32767.
        // Integral should clamp to 32767 (0x7FFF).
        $display("[TB] Ramping up integral with continuous error (+10.0) for 5000 cycles...");
        for (int i = 0; i < 5000; i++) begin
            trigger_update();
        end
        #10;

        $display("[TB] Output Correction: 0x%h (Expected clamped to 0x7FFF)", pid_correction);
        
        if (pid_correction == 16'sh7FFF) begin
            $display("[TB] PASS: Integral accumulator saturated cleanly at its maximum limit without wrapping.");
        end else begin
            $display("[TB] FAIL: Integral accumulator anti-windup failed. Output: %d", pid_correction);
        end

        // Negative accumulation test to verify INT_MIN clamping
        $display("[TB] --- Test Case 3b: Negative Anti-Windup ---");
        target_val = -16'sh0A00; // -10.0
        actual_val = 16'sh0000;
        $display("[TB] Ramping down integral with continuous negative error (-10.0) for 10000 cycles...");
        for (int i = 0; i < 10000; i++) begin
            trigger_update();
        end
        #10;

        $display("[TB] Output Correction: 0x%h (Expected clamped to 0x8000)", pid_correction);
        
        if (pid_correction == 16'sh8000) begin
            $display("[TB] PASS: Integral accumulator saturated cleanly at its negative minimum limit.");
        end else begin
            $display("[TB] FAIL: Negative Integral anti-windup failed. Output: %d", pid_correction);
        end

        $display("[TB] PID Calculator Verification completed successfully.");
        $finish;
    end

endmodule
