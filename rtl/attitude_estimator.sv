`timescale 1ns / 1ps

module attitude_estimator (
    input  logic               clk,        // 12 MHz System Clock
    input  logic               rst_n,      // Active-Low Reset
    input  logic               enable,     // 1-Cycle Strobe from I2C Master (1 kHz)
    
    // Raw 16-bit signed sensor readings from MPU-6050
    input  logic signed [15:0] accel_x,    // Accelerometer X (+/-2g scale)
    input  logic signed [15:0] accel_y,    // Accelerometer Y (+/-2g scale)
    input  logic signed [15:0] accel_z,    // Accelerometer Z (+/-2g scale)
    input  logic signed [15:0] gyro_x,     // Gyroscope X (+/-250 deg/s scale)
    input  logic signed [15:0] gyro_y,     // Gyroscope Y (+/-250 deg/s scale)
    input  logic signed [15:0] gyro_z,     // Gyroscope Z (+/-250 deg/s scale)
    
    // Estimated Attitude Angles (16-bit signed, Q8.8 format in degrees)
    output logic signed [15:0] roll_angle, // Positive = Roll Right
    output logic signed [15:0] pitch_angle // Positive = Pitch Up (Front Up)
);

    // Internal High-Resolution Q16.16 Angle Accumulators
    // (16 integer bits, 16 fractional bits for zero-drift integration)
    logic signed [31:0] roll_q16_reg;
    logic signed [31:0] pitch_q16_reg;

    // 1. Accelerometer Instantaneous Tilt Angle (Q16.16 format)
    // Scale factor: (180 / pi) * 65536 / 16384 = 229.183
    // 229.183 * 256 = 58671 -> (accel * 58671) >>> 8
    logic signed [31:0] accel_roll_q16;
    logic signed [31:0] accel_pitch_q16;

    assign accel_roll_q16  = ($signed(accel_y) * 32'sd58671) >>> 8;
    assign accel_pitch_q16 = (-$signed(accel_x) * 32'sd58671) >>> 8;

    // 2. Gyroscope Delta Angle per 1ms sample (Q16.16 format)
    // At 250 deg/s (131 LSB / deg/s) and 1 kHz (1/1000s):
    // delta_deg = gyro / 131000. In Q16.16: (gyro * 65536) / 131000 = gyro * 0.500275
    // 0.500275 * 65536 = 32786 -> (gyro * 32786) >>> 16
    logic signed [31:0] gyro_roll_delta_q16;
    logic signed [31:0] gyro_pitch_delta_q16;

    assign gyro_roll_delta_q16  = ($signed(gyro_x) * 32'sd32786) >>> 16;
    assign gyro_pitch_delta_q16 = ($signed(gyro_y) * 32'sd32786) >>> 16;

    // 3. Complementary Filter Update Process (96.875% Gyro / 3.125% Accel)
    // Predicted Angle = Previous Angle + Gyro Delta
    // Corrected Angle = Predicted Angle + (Accel Angle - Predicted Angle) / 32
    logic signed [31:0] roll_pred;
    logic signed [31:0] pitch_pred;
    logic signed [31:0] roll_diff;
    logic signed [31:0] pitch_diff;

    assign roll_pred  = roll_q16_reg + gyro_roll_delta_q16;
    assign pitch_pred = pitch_q16_reg + gyro_pitch_delta_q16;
    
    assign roll_diff  = accel_roll_q16 - roll_pred;
    assign pitch_diff = accel_pitch_q16 - pitch_pred;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_q16_reg  <= 32'sd0;
            pitch_q16_reg <= 32'sd0;
        end else if (enable) begin
            roll_q16_reg  <= roll_pred + (roll_diff >>> 5);
            pitch_q16_reg <= pitch_pred + (pitch_diff >>> 5);
        end
    end

    // 4. Output Scaling: Truncate Q16.16 to Standard Q8.8 Output Format
    assign roll_angle  = roll_q16_reg[23:8];
    assign pitch_angle = pitch_q16_reg[23:8];

endmodule
