`timescale 1ns / 1ps

module safety_mgr #(
    parameter int ARM_TIME_CYCLES = 12_000_000 // 1.0 second at 12 MHz clock
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [14:0] throttle_in,
    input  logic [14:0] yaw_in,
    input  logic [3:0]  rc_valid,
    output logic        armed,
    output logic        idle_throttle_active
);

    // Thresholds for arming/disarming stick gestures
    localparam logic [14:0] THROTTLE_MIN_THRESHOLD = 15'd12500;
    localparam logic [14:0] YAW_RIGHT_THRESHOLD    = 15'd23000;
    localparam logic [14:0] YAW_LEFT_THRESHOLD     = 15'd13000;
    localparam logic [14:0] IDLE_CUTOFF_THRESHOLD  = 15'd12600;

    // Arming Stick Command: Throttle Min (< 12,500) AND Yaw Full Right (> 23,000) with Valid RC
    logic arm_stick_condition;
    assign arm_stick_condition = (throttle_in < THROTTLE_MIN_THRESHOLD) && (yaw_in > YAW_RIGHT_THRESHOLD) && (&rc_valid);

    // Disarming Stick Command: Throttle Min (< 12,500) AND Yaw Full Left (< 13,000) with Valid RC
    logic disarm_stick_condition;
    assign disarm_stick_condition = (throttle_in < THROTTLE_MIN_THRESHOLD) && (yaw_in < YAW_LEFT_THRESHOLD) && (&rc_valid);

    // Select the gesture to monitor based on current operational mode
    logic stick_transition_active;
    assign stick_transition_active = armed ? disarm_stick_condition : arm_stick_condition;

    logic [23:0] timer_cnt;

    // Unified Safety Interlock & Arming Duration FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            armed     <= 1'b0;
            timer_cnt <= 24'd0;
        end else if (!(&rc_valid)) begin
            // Instant disarm and timer reset on RC signal loss (Highest Priority Safety Action)
            armed     <= 1'b0;
            timer_cnt <= 24'd0;
        end else if (stick_transition_active) begin
            if (timer_cnt >= ARM_TIME_CYCLES[23:0]) begin
                armed     <= ~armed; // Toggle state: Disarmed -> Armed, or Armed -> Disarmed
                timer_cnt <= 24'd0;
            end else begin
                timer_cnt <= timer_cnt + 24'd1;
            end
        end else begin
            timer_cnt <= 24'd0; // Reset timer if stick is released before required hold duration
        end
    end

    // Idle throttle flag (asserted when disarmed OR when throttle is at lowest idle)
    // Used to clear/zero the PID I-terms and prevent on-ground motor spool-up
    assign idle_throttle_active = (!armed) || (throttle_in < IDLE_CUTOFF_THRESHOLD);

endmodule

