`timescale 1ns / 1ps

module pwm_generator_tb();

    // 1. Emulate Physical Inputs and Outputs as Local Signals
    logic        clk;
    logic        rst_n;
    logic [14:0] duty_cycle;
    logic        pwm_out;

    // 2. Instantiate the Unit Under Test (UUT)
    pwm_generator uut (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(duty_cycle),
        .pwm_out(pwm_out)
    );

    // 3. Generate the 12 MHz Clock Signal
    // 12 MHz period is ~83.33 ns. Flipping every 41.67 ns creates a square wave.
    always begin
        clk = 1'b1;
        #41.67;
        clk = 1'b0;
        #41.67;
    end

    // 4. Test Stimulus Vector Block
    initial begin
        // Initialize Signals
        rst_n = 1'b0;
        duty_cycle = 15'd0;
        
        // Hold Reset for 100 nanoseconds to clear registers
        #100;
        rst_n = 1'b1; // Release Reset
        #100;

        // Test Case 1: Set Motor to 10% Throttle (3,000 / 30,000 cycles)
        duty_cycle = 15'd3000;
        #3000000; // Run simulation long enough to see a few full PWM periods

        // Test Case 2: Set Motor to 50% Throttle (15,000 / 30,000 cycles)
        duty_cycle = 15'd15000;
        #6000000;

        // Stop Simulation
        $stop;
    end

    // 5. Monitor and verify the frequency and duty cycle
    realtime rising_edge_time;
    realtime falling_edge_time;
    realtime period;
    realtime high_time;
    realtime frequency;
    realtime duty_cycle_pct;

    initial begin
        rising_edge_time = 0;
        falling_edge_time = 0;
        period = 0;
        high_time = 0;
        frequency = 0;
        duty_cycle_pct = 0;
    end

    always @(posedge pwm_out) begin
        if (rising_edge_time != 0) begin
            period = $realtime - rising_edge_time;
            frequency = 1.0e9 / period; // period is in ns
            duty_cycle_pct = (high_time / period) * 100.0;
            $display("[MONITOR] Time = %0.1f ns | Period = %0.2f ns, Freq = %0.2f Hz, Duty Cycle = %0.2f%% (Duty Cycle Reg = %0d)", 
                     $realtime, period, frequency, duty_cycle_pct, duty_cycle);
        end
        rising_edge_time = $realtime;
    end

    always @(negedge pwm_out) begin
        falling_edge_time = $realtime;
        if (rising_edge_time != 0) begin
            high_time = falling_edge_time - rising_edge_time;
        end
    end

endmodule