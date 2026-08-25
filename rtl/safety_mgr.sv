`timescale 1ns / 1ps

module safety_mgr #(
    parameter int ARM_TIME_CYCLES = 12_000_000 // 1.0 second at 12 MHz
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [14:0] throttle_in,
    input  logic [14:0] yaw_in,
    input  logic [3:0]  rc_valid,
    output logic        armed,
    output logic        idle_throttle_active
);

    // Arming Stick Command: Throttle Min (< 12,500) AND Yaw Full Right (> 23,000)
    logic arm_stick_condition;
    assign arm_stick_condition = (throttle_in < 15'd12500) && (yaw_in > 15'd23000) && (&rc_valid);

    // Disarming Stick Command: Throttle Min (< 12,500) AND Yaw Full Left (< 13,000)
    logic disarm_stick_condition;
    assign disarm_stick_condition = (throttle_in < 15'd12500) && (yaw_in < 15'd13000) && (&rc_valid);

    logic [23:0] timer_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            armed     <= 1'b0;
            timer_cnt <= 24'd0;
        end else begin
            // Instant disarm on RC signal loss
            if (!(&rc_valid)) begin
                armed     <= 1'b0;
                timer_cnt <= 24'd0;
            end else if (!armed) begin
                if (arm_stick_condition) begin
                    if (timer_cnt >= ARM_TIME_CYCLES[23:0]) begin
                        armed     <= 1'b1;
                        timer_cnt <= 24'd0;
                    end else begin
                        timer_cnt <= timer_cnt + 24'd1;
                    end
                end else begin
                    timer_cnt <= 24'd0;
                end
            end else begin
                // Currently armed: check for disarm command
                if (disarm_stick_condition) begin
                    if (timer_cnt >= ARM_TIME_CYCLES[23:0]) begin
                        armed     <= 1'b0;
                        timer_cnt <= 24'd0;
                    end else begin
                        timer_cnt <= timer_cnt + 24'd1;
                    end
                end else begin
                    timer_cnt <= 24'd0;
                end
            end
        end
    end

    // Idle throttle flag (asserted when disarmed OR when throttle is at lowest idle)
    // Used to clear/zero the PID I-terms and prevent on-ground motor spool-up
    assign idle_throttle_active = (!armed) || (throttle_in < 15'd12600);

endmodule
