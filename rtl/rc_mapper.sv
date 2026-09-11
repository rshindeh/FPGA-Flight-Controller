`timescale 1ns / 1ps

module rc_mapper (
    input  logic               clk,                // System Clock (reserved for future filtering)
    input  logic               rst_n,              // Active-Low Reset (reserved for future filtering)
    
    // Raw Decoded RC Pulses (15-bit Unsigned, 12000 to 24000 ticks)
    input  logic [14:0]        rc_roll,
    input  logic [14:0]        rc_pitch,
    input  logic [14:0]        rc_yaw,
    input  logic [14:0]        rc_throttle,
    
    // Mapped Setpoints in Standard Q8.8 Fixed-Point Format
    output logic signed [15:0] target_roll_angle,  // Target Angle: +/-30.0 deg (Q8.8, +/-7680)
    output logic signed [15:0] target_pitch_angle, // Target Angle: +/-30.0 deg (Q8.8, +/-7680)
    output logic signed [15:0] target_yaw_rate,    // Target Rate:  +/-100.0 deg/s (Q8.8, +/-25600)
    output logic [14:0]        throttle_out        // Clamped [12000, 24000] ticks
);

    // Deadband threshold (+/-100 ticks = ~8.3 us jitter suppression)
    localparam logic signed [31:0] DEADBAND     = 32'sd100;
    localparam logic signed [31:0] CENTER_TICKS = 32'sd18000;
    localparam logic [14:0]        THROTTLE_MIN = 15'd12000;
    localparam logic [14:0]        THROTTLE_MAX = 15'd24000;

    // Helper function: Zero-centered deadband filter and proportional scaler
    function automatic logic signed [15:0] scale_with_deadband(
        input logic signed [31:0] diff,
        input logic signed [31:0] mult,
        input logic signed [31:0] div
    );
        if (diff > -DEADBAND && diff < DEADBAND) begin
            return 16'sd0;
        end else begin
            return 16'((diff * mult) / div);
        end
    endfunction

    // Intermediate 32-bit signed differentials from center (18,000 ticks = 1.5 ms)
    logic signed [31:0] roll_diff;
    logic signed [31:0] pitch_diff;
    logic signed [31:0] yaw_diff;

    assign roll_diff  = $signed({17'b0, rc_roll}) - CENTER_TICKS;
    assign pitch_diff = $signed({17'b0, rc_pitch}) - CENTER_TICKS;
    assign yaw_diff   = $signed({17'b0, rc_yaw}) - CENTER_TICKS;

    // Roll & Pitch: +/-30.0 deg in Q8.8 (+/-7680). Ratio: 7680 / 6000 = 32 / 25
    // Yaw Rate    : +/-100.0 deg/s in Q8.8 (+/-25600). Ratio: 25600 / 6000 = 64 / 15
    always_comb begin
        target_roll_angle  = scale_with_deadband(roll_diff,  32'sd32, 32'sd25);
        target_pitch_angle = scale_with_deadband(pitch_diff, 32'sd32, 32'sd25);
        target_yaw_rate    = scale_with_deadband(yaw_diff,   32'sd64, 32'sd15);

        if (rc_throttle < THROTTLE_MIN) begin
            throttle_out = THROTTLE_MIN;
        end else if (rc_throttle > THROTTLE_MAX) begin
            throttle_out = THROTTLE_MAX;
        end else begin
            throttle_out = rc_throttle;
        end
    end

endmodule

