`timescale 1ns / 1ps

module pwm_generator #(
    parameter int FRAME_PERIOD_TICKS = 30_000 // 400 Hz frame at 12 MHz clock (2.5 ms)
)(
    input  logic        clk,         // 12 MHz System Clock
    input  logic        rst_n,       // Active-Low Synchronized Reset
    input  logic [14:0] duty_cycle,  // Target high time (0 to 30,000 ticks)
    output logic        pwm_out      // Physical output signal to the ESC
);

    localparam logic [14:0] ROLLOVER_TICKS = 15'(FRAME_PERIOD_TICKS - 1);

    // Internal registers (hierarchical probes in TB)
    logic [14:0] counter;
    logic [14:0] duty_cycle_buf;

    // Frame Counter & Double-Buffered Duty Cycle Latch
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter        <= 15'd0;
            duty_cycle_buf <= 15'd0;
        end else begin
            if (counter >= ROLLOVER_TICKS) begin
                counter        <= 15'd0;
                duty_cycle_buf <= duty_cycle; // Latch new duty cycle cleanly at frame boundary
            end else begin
                counter        <= counter + 15'd1;
            end
        end
    end

    // Output comparator
    assign pwm_out = (counter < duty_cycle_buf);

endmodule