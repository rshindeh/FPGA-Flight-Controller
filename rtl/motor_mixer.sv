`timescale 1ns / 1ps

module motor_mixer (
    input  logic               clk,             // System Clock (reserved for future filtering)
    input  logic               rst_n,           // Active-Low Reset (reserved for future filtering)
    input  logic               armed,           // Arming status flag (1 = Active, 0 = Disarmed)
    
    // Core Throttle Command (15-bit Unsigned, 12000 to 24000 ticks)
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

    // Standard ESC Pulse Bounds: 1.0 ms (12,000 ticks) to 2.0 ms (24,000 ticks)
    localparam logic signed [31:0] ESC_MIN_TICKS = 32'sd12000;
    localparam logic signed [31:0] ESC_MAX_TICKS = 32'sd24000;

    // ESC Saturation Clamping Helper Function
    function automatic logic [14:0] clamp_esc(input logic signed [31:0] calc_val);
        if (calc_val > ESC_MAX_TICKS)      return 15'd24000;
        else if (calc_val < ESC_MIN_TICKS) return 15'd12000;
        else                               return calc_val[14:0];
    endfunction

    // Zero-extend throttle to 32-bit signed for safe multi-operand arithmetic
    logic signed [31:0] throttle_signed;
    assign throttle_signed = $signed({17'b0, throttle_in});

    // Quad-X Mixing Matrix: 32-bit signed intermediate sums prevent overflow
    // Motor 1 (Front Right CCW): Throttle - Roll - Pitch + Yaw
    // Motor 2 (Rear Right CW)  : Throttle - Roll + Pitch - Yaw
    // Motor 3 (Rear Left CCW)  : Throttle + Roll + Pitch + Yaw
    // Motor 4 (Front Left CW)  : Throttle + Roll - Pitch - Yaw
    logic signed [31:0] m1_calc;
    logic signed [31:0] m2_calc;
    logic signed [31:0] m3_calc;
    logic signed [31:0] m4_calc;

    assign m1_calc = throttle_signed - roll_correction - pitch_correction + yaw_correction;
    assign m2_calc = throttle_signed - roll_correction + pitch_correction - yaw_correction;
    assign m3_calc = throttle_signed + roll_correction + pitch_correction + yaw_correction;
    assign m4_calc = throttle_signed + roll_correction - pitch_correction - yaw_correction;

    // Output Clamping & Arming Interlock
    always_comb begin
        if (!armed) begin
            motor_1 = 15'd12000;
            motor_2 = 15'd12000;
            motor_3 = 15'd12000;
            motor_4 = 15'd12000;
        end else begin
            motor_1 = clamp_esc(m1_calc);
            motor_2 = clamp_esc(m2_calc);
            motor_3 = clamp_esc(m3_calc);
            motor_4 = clamp_esc(m4_calc);
        end
    end

endmodule

