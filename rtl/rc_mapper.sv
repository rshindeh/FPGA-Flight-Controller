`timescale 1ns / 1ps

module rc_mapper (
    input  logic        clk,
    input  logic        rst_n,
    
    // Raw Decoded RC Pulses (15-bit Unsigned, 12000 to 24000 ticks)
    input  logic [14:0] rc_roll,
    input  logic [14:0] rc_pitch,
    input  logic [14:0] rc_yaw,
    input  logic [14:0] rc_throttle,
    
    // Mapped Setpoints in Standard Q8.8 Fixed-Point Format
    output logic signed [15:0] target_roll_angle,  // Target Angle: +/-30.0 deg (Q8.8)
    output logic signed [15:0] target_pitch_angle, // Target Angle: +/-30.0 deg (Q8.8)
    output logic signed [15:0] target_yaw_rate,    // Target Rate:  +/-100.0 deg/s (Q8.8)
    output logic [14:0]        throttle_out        // Clamped [12000, 24000]
);

    // Deadband threshold (+/-100 ticks = ~8.3 us jitter suppression)
    localparam logic signed [15:0] DEADBAND = 16'sd100;

    // Intermediate 32-bit signed differentials from center (18,000 ticks = 1.5 ms)
    logic signed [31:0] roll_diff;
    logic signed [31:0] pitch_diff;
    logic signed [31:0] yaw_diff;

    assign roll_diff  = $signed({17'b0, rc_roll}) - 32'sd18000;
    assign pitch_diff = $signed({17'b0, rc_pitch}) - 32'sd18000;
    assign yaw_diff   = $signed({17'b0, rc_yaw}) - 32'sd18000;

    // 1. Roll Target Angle (+/-30 deg in Q8.8 -> +/-7680)
    // Scale: diff * 7680 / 6000 = diff * 32 / 25
    always_comb begin
        if (roll_diff > -DEADBAND && roll_diff < DEADBAND) begin
            target_roll_angle = 16'sd0; // Perfect zero hover command in deadband
        end else begin
            target_roll_angle = (roll_diff * 32'sd32) / 32'sd25;
        end
    end

    // 2. Pitch Target Angle (+/-30 deg in Q8.8 -> +/-7680)
    always_comb begin
        if (pitch_diff > -DEADBAND && pitch_diff < DEADBAND) begin
            target_pitch_angle = 16'sd0;
        end else begin
            target_pitch_angle = (pitch_diff * 32'sd32) / 32'sd25;
        end
    end

    // 3. Yaw Target Rate (+/-100 deg/s in Q8.8 -> +/-25600)
    // Scale: diff * 25600 / 6000 = diff * 64 / 15
    always_comb begin
        if (yaw_diff > -DEADBAND && yaw_diff < DEADBAND) begin
            target_yaw_rate = 16'sd0;
        end else begin
            target_yaw_rate = (yaw_diff * 32'sd64) / 32'sd15;
        end
    end

    // 4. Throttle Command Clamping [12000, 24000]
    always_comb begin
        if (rc_throttle < 15'd12000) begin
            throttle_out = 15'd12000;
        end else if (rc_throttle > 15'd24000) begin
            throttle_out = 15'd24000;
        end else begin
            throttle_out = rc_throttle;
        end
    end

endmodule
