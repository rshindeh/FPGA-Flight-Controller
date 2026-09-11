`timescale 1ns / 1ps

module attitude_estimator (
    input  logic               clk,        // 12 MHz System Clock
    input  logic               rst_n,      // Active-Low Reset
    input  logic               enable,     // 1-Cycle Strobe from Sensor Acquisition Loop (1 kHz)
    
    // Raw 16-bit signed sensor readings from IMU (MPU-6500)
    input  logic signed [15:0] accel_x,    // Accelerometer X (+/-2g scale, 16384 LSB/g)
    input  logic signed [15:0] accel_y,    // Accelerometer Y (+/-2g scale, 16384 LSB/g)
    input  logic signed [15:0] accel_z,    // Accelerometer Z (+/-2g scale, 16384 LSB/g)
    input  logic signed [15:0] gyro_x,     // Gyroscope X (+/-250 deg/s scale, 131 LSB/deg/s)
    input  logic signed [15:0] gyro_y,     // Gyroscope Y (+/-250 deg/s scale, 131 LSB/deg/s)
    input  logic signed [15:0] gyro_z,     // Gyroscope Z (+/-250 deg/s scale, reserved for yaw damping)
    
    // Estimated Attitude Angles (16-bit signed, Q8.8 format in degrees)
    output logic signed [15:0] roll_angle, // Positive = Roll Right
    output logic signed [15:0] pitch_angle // Positive = Pitch Up (Front Up)
);

    // -------------------------------------------------------------------------
    // Fixed-Point Mathematical Constants (Q16.16 Format)
    // -------------------------------------------------------------------------
    // Accelerometer Scale: (180 / pi) * 65536 / 16384 = 229.1831
    // In Q8.8 fixed factor: 229.1831 * 256 = 58671 -> (accel * 58671) >>> 8
    localparam logic signed [31:0] ACCEL_SCALE = 32'sd58671;

    // Gyroscope Scale per 1ms sample (1 kHz loop, 131 LSB / deg/s):
    // delta_deg = gyro / 131000. In Q16.16: (gyro * 65536) / 131000 = gyro * 0.500275
    // In Q16.16 factor: 0.500275 * 65536 = 32786 -> (gyro * 32786) >>> 16
    localparam logic signed [31:0] GYRO_SCALE  = 32'sd32786;

    // Internal High-Resolution Q16.16 Angle Accumulators (16.16 format for zero-drift integration)
    logic signed [31:0] roll_q16_reg;
    logic signed [31:0] pitch_q16_reg;

    // Sign-extended 32-bit sensor intermediates to prevent negation overflow at -32768
    logic signed [31:0] accel_x_32, accel_y_32;
    logic signed [31:0] gyro_x_32,  gyro_y_32;

    assign accel_x_32 = 32'(accel_x);
    assign accel_y_32 = 32'(accel_y);
    assign gyro_x_32  = 32'(gyro_x);
    assign gyro_y_32  = 32'(gyro_y);

    // 1. Instantaneous Accelerometer Tilt Angle (Q16.16)
    logic signed [31:0] accel_roll_q16;
    logic signed [31:0] accel_pitch_q16;

    assign accel_roll_q16  = (accel_y_32 * ACCEL_SCALE) >>> 8;
    assign accel_pitch_q16 = (-accel_x_32 * ACCEL_SCALE) >>> 8;

    // 2. Gyroscope Delta Angle per 1ms sample (Q16.16)
    logic signed [31:0] gyro_roll_delta_q16;
    logic signed [31:0] gyro_pitch_delta_q16;

    assign gyro_roll_delta_q16  = (gyro_x_32 * GYRO_SCALE) >>> 16;
    assign gyro_pitch_delta_q16 = (gyro_y_32 * GYRO_SCALE) >>> 16;

    // 3. Complementary Filter (96.875% Gyro / 3.125% Accel)
    // Angle[n] = (Angle[n-1] + Gyro_Delta) + (Accel_Angle - (Angle[n-1] + Gyro_Delta)) / 32
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

    // 4. Output Scaling: Extract Q8.8 degrees from Q16.16 accumulator (bits [23:8])
    assign roll_angle  = roll_q16_reg[23:8];
    assign pitch_angle = pitch_q16_reg[23:8];

endmodule

