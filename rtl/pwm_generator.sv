module pwm_generator (
    input  logic        clk,         // 12 MHz System Clock
    input  logic        rst_n,       // Active-Low Synchronized Reset
    input  logic [14:0] duty_cycle,  // Target high time (0 to 30,000)
    output logic        pwm_out      // Physical output signal to the ESC
);

    // Internal registers will go here
    logic [14:0] counter;
    logic [14:0] duty_cycle_buf;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 15'd0;
        end else begin
            if (counter >= 15'd29999) begin
                counter <= 15'd0;
            end else begin
                counter <= counter + 15'd1;
            end
        end
    end

    assign pwm_out = (counter < duty_cycle) ? 1'b1 : 1'b0;

endmodule