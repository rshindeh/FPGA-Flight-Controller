`timescale 1ns / 1ps

module motor_mixer (
    input  logic               clk,
    input  logic               rst_n,
    
    // Core Throttle Command (15-bit Unsigned, from RC Receiver)
    input  logic [14:0]        throttle_in,
    
    // PID Correction Factors (16-bit Signed, from PID Controller)
    input  logic signed [15:0] roll_correction,
    input  logic signed [15:0] pitch_correction,
    input  logic signed [15:0] yaw_correction,
    
    // Saturated Motor Outputs (15-bit Unsigned, to PWM Generators)
    output logic [14:0]        motor_1, // Front Right (CCW)
    output logic [14:0]        motor_2, // Rear Right  (CW)
    output logic [14:0]        motor_3, // Rear Left   (CCW)
    output logic [14:0]        motor_4  // Front Left  (CW)
);

    // --- Intermediate Calculation Wires ---
    // Expanded to 32-bit signed to guarantee no intermediate arithmetic overflow/underflow wrapping
    logic signed [31:0] m1_calc;
    logic signed [31:0] m2_calc;
    logic signed [31:0] m3_calc;
    logic signed [31:0] m4_calc;

    // --- Standard Quad-X Mixing Matrix ---
    // Motor 1 (Front Right):  Throttle - Roll - Pitch + Yaw
    // Motor 2 (Rear Right) :  Throttle - Roll + Pitch - Yaw
    // Motor 3 (Rear Left)  :  Throttle + Roll + Pitch + Yaw
    // Motor 4 (Front Left) :  Throttle + Roll - Pitch - Yaw
    
    // Note: throttle_in is 15-bit unsigned, zero-extended and cast to signed 32-bit for safe addition
    always_comb begin
        m1_calc = $signed({17'b0, throttle_in}) - roll_correction - pitch_correction + yaw_correction;
        m2_calc = $signed({17'b0, throttle_in}) - roll_correction + pitch_correction - yaw_correction;
        m3_calc = $signed({17'b0, throttle_in}) + roll_correction + pitch_correction + yaw_correction;
        m4_calc = $signed({17'b0, throttle_in}) + roll_correction - pitch_correction - yaw_correction;
    end

    // --- Combinatorial Saturation Clamping ---
    // Ensures outputs rigidly stay within the 0 to 30,000 bounds of the 400Hz PWM Generator
    localparam logic signed [31:0] LIMIT_MAX = 32'sd30000;
    localparam logic signed [31:0] LIMIT_MIN = 32'sd0;

    always_comb begin
        // Motor 1 Saturation
        if (m1_calc > LIMIT_MAX) begin
            motor_1 = 15'd30000;
        end else if (m1_calc < LIMIT_MIN) begin
            motor_1 = 15'd0;
        end else begin
            motor_1 = m1_calc[14:0];
        end

        // Motor 2 Saturation
        if (m2_calc > LIMIT_MAX) begin
            motor_2 = 15'd30000;
        end else if (m2_calc < LIMIT_MIN) begin
            motor_2 = 15'd0;
        end else begin
            motor_2 = m2_calc[14:0];
        end

        // Motor 3 Saturation
        if (m3_calc > LIMIT_MAX) begin
            motor_3 = 15'd30000;
        end else if (m3_calc < LIMIT_MIN) begin
            motor_3 = 15'd0;
        end else begin
            motor_3 = m3_calc[14:0];
        end

        // Motor 4 Saturation
        if (m4_calc > LIMIT_MAX) begin
            motor_4 = 15'd30000;
        end else if (m4_calc < LIMIT_MIN) begin
            motor_4 = 15'd0;
        end else begin
            motor_4 = m4_calc[14:0];
        end
    end

endmodule
