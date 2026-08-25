`timescale 1ns / 1ps

module pwm_generator (
    input  logic        clk,         // 12 MHz System Clock
    input  logic        rst_n,       // Active-Low Synchronized Reset
    input  logic [14:0] duty_cycle,  // Target high time (0 to 30,000)
    output logic        pwm_out      // Physical output signal to the ESC
);

    // Internal registers
    logic [14:0] counter;
    logic [14:0] duty_cycle_buf;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter        <= 15'd0;
            duty_cycle_buf <= 15'd0;
        end else begin
            if (counter >= 15'd29999) begin
                counter        <= 15'd0;
                duty_cycle_buf <= duty_cycle;
            end else begin
                if (counter == 15'd0) begin
                    duty_cycle_buf <= duty_cycle;
                end
                counter <= counter + 15'd1;
            end
        end
    end

    assign pwm_out = (counter < duty_cycle_buf) ? 1'b1 : 1'b0;

endmodule