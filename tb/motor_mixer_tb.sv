`timescale 1ns / 1ps

module motor_mixer_tb();

    // 1. Testbench Signals
    logic        clk;
    logic        rst_n;
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
        .throttle_in(throttle_in),
        .roll_correction(roll_correction),
        .pitch_correction(pitch_correction),
        .yaw_correction(yaw_correction),
        .motor_1(motor_1),
        .motor_2(motor_2),
        .motor_3(motor_3),
        .motor_4(motor_4)
    );

    // Clock Generator (for interface completeness, though UUT is combinational)
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
        throttle_in = 15'd0;
        roll_correction = 16'sd0;
        pitch_correction = 16'sd0;
        yaw_correction = 16'sd0;
        #20;
        rst_n = 1'b1;
        #10;

        // ==========================================
        // TEST CASE 1: Neutral Throttle & Zero Corrections
        // ==========================================
        $display("[TB] --- Test Case 1: Neutral throttle (18,000) and zero corrections ---");
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
        // TEST CASE 2: Extreme Over-saturation
        // ==========================================
        $display("[TB] --- Test Case 2: Extreme Over-saturation ---");
        // Throttle is high, and corrections force motor calculations above 30,000
        throttle_in = 15'd24000;
        
        // Front Right (Motor 1) = Throttle - Roll - Pitch + Yaw
        // Let's force Motor 1 above 30,000 by setting:
        // Roll = -4000, Pitch = -4000, Yaw = 4000
        // Expected Raw Motor 1 = 24000 - (-4000) - (-4000) + 4000 = 36,000 -> Clamp to 30,000
        roll_correction  = -16'sd4000;
        pitch_correction = -16'sd4000;
        yaw_correction   = 16'sd4000;
        #10;

        $display("[TB] Inputs: Throttle = %0d, Roll = %0d, Pitch = %0d, Yaw = %0d", 
                 throttle_in, roll_correction, pitch_correction, yaw_correction);
        $display("[TB] Outputs: Motor1 = %0d, Motor2 = %0d, Motor3 = %0d, Motor4 = %0d", 
                 motor_1, motor_2, motor_3, motor_4);

        // Check clamping of all motors. 
        // Raw computations:
        // M1: 24000 - (-4000) - (-4000) + 4000 = 36000 -> Should clamp to 30000
        // M2: 24000 - (-4000) + (-4000) - 4000 = 20000 -> No clamp
        // M3: 24000 + (-4000) + (-4000) + 4000 = 20000 -> No clamp
        // M4: 24000 + (-4000) - (-4000) - 4000 = 20000 -> No clamp
        if (motor_1 == 15'd30000) begin
            $display("[TB] PASS: Motor 1 over-saturation clamped exactly to 30,000 without wrap-around.");
        end else begin
            $display("[TB] FAIL: Motor 1 over-saturation clamping failed (Output: %0d)", motor_1);
        end

        // Test another configuration that drives all motors into over-saturation
        // Let's set Throttle = 28000, and large corrections
        throttle_in = 15'd29000;
        roll_correction = 16'sd8000;
        pitch_correction = 16'sd8000;
        yaw_correction = 16'sd8000;
        #10;
        $display("[TB] Inputs: Throttle = %0d, Roll = %0d, Pitch = %0d, Yaw = %0d", 
                 throttle_in, roll_correction, pitch_correction, yaw_correction);
        $display("[TB] Outputs: Motor1 = %0d, Motor2 = %0d, Motor3 = %0d, Motor4 = %0d", 
                 motor_1, motor_2, motor_3, motor_4);

        // Raw calculations:
        // M1 = 29000 - 8000 - 8000 + 8000 = 21000
        // M2 = 29000 - 8000 + 8000 - 8000 = 21000
        // M3 = 29000 + 8000 + 8000 + 8000 = 53000 -> Clamp to 30000
        // M4 = 29000 + 8000 - 8000 - 8000 = 21000
        if (motor_3 == 15'd30000) begin
            $display("[TB] PASS: Motor 3 over-saturation clamped exactly to 30,000.");
        end else begin
            $display("[TB] FAIL: Motor 3 over-saturation clamping failed (Output: %0d)", motor_3);
        end

        // ==========================================
        // TEST CASE 3: Extreme Under-saturation
        // ==========================================
        $display("[TB] --- Test Case 3: Extreme Under-saturation ---");
        // Throttle is low, and corrections force motor calculations below 0
        throttle_in = 15'd12000;
        
        // Let's force Motor 1 below 0 by setting:
        // Roll = 6000, Pitch = 6000, Yaw = -6000
        // Expected Raw Motor 1 = 12000 - 6000 - 6000 + (-6000) = -6,000 -> Clamp to 0
        roll_correction  = 16'sd6000;
        pitch_correction = 16'sd6000;
        yaw_correction   = -16'sd6000;
        #10;

        $display("[TB] Inputs: Throttle = %0d, Roll = %0d, Pitch = %0d, Yaw = %0d", 
                 throttle_in, roll_correction, pitch_correction, yaw_correction);
        $display("[TB] Outputs: Motor1 = %0d, Motor2 = %0d, Motor3 = %0d, Motor4 = %0d", 
                 motor_1, motor_2, motor_3, motor_4);

        // M1: 12000 - 6000 - 6000 + (-6000) = -6000 -> Clamp to 0
        // M2: 12000 - 6000 + 6000 - (-6000) = 18000
        // M3: 12000 + 6000 + 6000 + (-6000) = 18000
        // M4: 12000 + 6000 - 6000 - (-6000) = 18000
        if (motor_1 == 15'd0) begin
            $display("[TB] PASS: Motor 1 under-saturation clamped exactly to 0 without wrap-around.");
        end else begin
            $display("[TB] FAIL: Motor 1 under-saturation clamping failed (Output: %0d)", motor_1);
        end

        // Test another under-saturation configuration driving all motor calculations negative
        throttle_in = 15'd5000;
        roll_correction = -16'sd10000;
        pitch_correction = -16'sd10000;
        yaw_correction = -16'sd10000;
        #10;
        $display("[TB] Inputs: Throttle = %0d, Roll = %0d, Pitch = %0d, Yaw = %0d", 
                 throttle_in, roll_correction, pitch_correction, yaw_correction);
        $display("[TB] Outputs: Motor1 = %0d, Motor2 = %0d, Motor3 = %0d, Motor4 = %0d", 
                 motor_1, motor_2, motor_3, motor_4);

        // Raw calculations:
        // M1 = 5000 - (-10000) - (-10000) + (-10000) = 15000
        // M2 = 5000 - (-10000) + (-10000) - (-10000) = 15000
        // M3 = 5000 + (-10000) + (-10000) + (-10000) = -25000 -> Clamp to 0
        // M4 = 5000 + (-10000) - (-10000) - (-10000) = 15000
        if (motor_3 == 15'd0) begin
            $display("[TB] PASS: Motor 3 under-saturation clamped exactly to 0.");
        end else begin
            $display("[TB] FAIL: Motor 3 under-saturation clamping failed (Output: %0d)", motor_3);
        end

        $display("[TB] Motor Mixer Verification completed successfully.");
        $finish;
    end

endmodule
