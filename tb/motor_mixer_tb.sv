`timescale 1ns / 1ps

module motor_mixer_tb();

    // 1. Testbench Signals
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

    // 2. Instantiate UUT (Unit Under Test)
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

    // Clock Generator
    always begin
        clk = 1'b1;
        #5;
        clk = 1'b0;
        #5;
    end

    // 3. Test Stimulus Block
    initial begin
        $display("[TB] Starting Motor Mixer Saturation Verification...");
        
        // Reset sequence
        rst_n = 1'b0;
        armed = 1'b0;
        throttle_in = 15'd12000;
        roll_correction = 16'sd0;
        pitch_correction = 16'sd0;
        yaw_correction = 16'sd0;
        #20;
        rst_n = 1'b1;
        #10;

        // ==========================================
        // TEST CASE 0: Disarmed Safety Interlock
        // ==========================================
        $display("[TB] --- Test Case 0: Disarmed State (armed = 0) ---");
        throttle_in = 15'd20000;
        roll_correction = 16'sd5000;
        #10;
        if (motor_1 == 15'd12000 && motor_2 == 15'd12000 && 
            motor_3 == 15'd12000 && motor_4 == 15'd12000) begin
            $display("[TB] PASS: Disarmed safety lock rigidly held all motor commands at 12,000 (1.0 ms).");
        end else begin
            $display("[TB] FAIL: Disarmed safety lock failed!");
        end

        // Arm the mixer
        armed = 1'b1;
        #10;

        // ==========================================
        // TEST CASE 1: Neutral Throttle & Zero Corrections
        // ==========================================
        $display("[TB] --- Test Case 1: Neutral hover throttle (18,000) and zero corrections ---");
        throttle_in = 15'd18000;
        roll_correction = 16'sd0;
        pitch_correction = 16'sd0;
        yaw_correction = 16'sd0;
        #10;
        
        $display("[TB] Inputs: Throttle = %0d, Roll = %0d, Pitch = %0d, Yaw = %0d", 
                 throttle_in, roll_correction, pitch_correction, yaw_correction);
        $display("[TB] Outputs: Motor1 = %0d, Motor2 = %0d, Motor3 = %0d, Motor4 = %0d", 
                 motor_1, motor_2, motor_3, motor_4);

        if (motor_1 == 15'd18000 && motor_2 == 15'd18000 && 
            motor_3 == 15'd18000 && motor_4 == 15'd18000) begin
            $display("[TB] PASS: Neutral test case matched exactly 18,000.");
        end else begin
            $display("[TB] FAIL: Neutral test case failed to match 18,000!");
        end

        // ==========================================
        // TEST CASE 2: Upper Saturation Clamping (24,000)
        // ==========================================
        $display("[TB] --- Test Case 2: Extreme Over-saturation (Clamping to 24,000) ---");
        throttle_in = 15'd22000;
        roll_correction  = -16'sd4000;
        pitch_correction = -16'sd4000;
        yaw_correction   = 16'sd4000;
        #10;

        // Raw M1: 22000 - (-4000) - (-4000) + 4000 = 34000 -> Should clamp to 24000
        if (motor_1 == 15'd24000) begin
            $display("[TB] PASS: Motor 1 over-saturation clamped cleanly to 24,000 (2.0 ms) without wrap-around.");
        end else begin
            $display("[TB] FAIL: Motor 1 over-saturation clamping failed (Output: %0d)", motor_1);
        end

        // ==========================================
        // TEST CASE 3: Lower Saturation Clamping (12,000)
        // ==========================================
        $display("[TB] --- Test Case 3: Extreme Under-saturation (Clamping to 12,000) ---");
        throttle_in = 15'd14000;
        roll_correction  = 16'sd6000;
        pitch_correction = 16'sd6000;
        yaw_correction   = -16'sd6000;
        #10;

        // Raw M1: 14000 - 6000 - 6000 + (-6000) = -4000 -> Should clamp to 12000
        if (motor_1 == 15'd12000) begin
            $display("[TB] PASS: Motor 1 under-saturation clamped cleanly to 12,000 (1.0 ms) without wrap-around.");
        end else begin
            $display("[TB] FAIL: Motor 1 under-saturation clamping failed (Output: %0d)", motor_1);
        end

        $display("[TB] Motor Mixer Verification completed successfully.");
        $finish;
    end

endmodule
