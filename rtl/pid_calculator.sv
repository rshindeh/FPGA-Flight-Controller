`timescale 1ns / 1ps

module pid_calculator (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               enable, // Strobe high for 1 clock cycle to process a new sample
    
    // Target and Actual Values (16-bit signed, Q8.8 fixed-point format)
    input  logic signed [15:0] target_val,
    input  logic signed [15:0] actual_val,
    
    // PID Gains (16-bit signed, Q8.8 fixed-point format)
    input  logic signed [15:0] p_gain,
    input  logic signed [15:0] i_gain,
    input  logic signed [15:0] d_gain,
    
    // Output Correction (16-bit signed, Q8.8 fixed-point format)
    output logic signed [15:0] pid_correction
);

    // --- Internal State Registers ---
    logic signed [15:0] current_error;
    logic signed [15:0] prev_error;
    logic signed [15:0] derivative_error;
    logic signed [31:0] integral_error; // 32-bit register for cumulative error

    // Error Calculation
    assign current_error = target_val - actual_val;
    assign derivative_error = current_error - prev_error;

    // Integral Accumulator with Anti-Windup Limits
    // Clamped to 16-bit signed limits to prevent integral runaway and 32-bit math overflow later
    localparam logic signed [31:0] INT_MAX = 32'sd32767;
    localparam logic signed [31:0] INT_MIN = -32'sd32768;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_error     <= 16'sd0;
            integral_error <= 32'sd0;
        end else if (enable) begin
            prev_error <= current_error;
            
            // Accumulate with Anti-Windup Clamping
            if (integral_error + current_error > INT_MAX) begin
                integral_error <= INT_MAX;
            end else if (integral_error + current_error < INT_MIN) begin
                integral_error <= INT_MIN;
            end else begin
                integral_error <= integral_error + current_error;
            end
        end
    end

    // --- Multiplier & Accumulation Datapath (32-bit arithmetic) ---
    // Multiplying Q8.8 by Q8.8 yields a Q16.16 formatted result inside a 32-bit signed register
    logic signed [31:0] p_term_raw;
    logic signed [31:0] i_term_raw;
    logic signed [31:0] d_term_raw;
    logic signed [31:0] total_correction_raw;

    assign p_term_raw = $signed(current_error) * $signed(p_gain);
    // Since integral_error is clamped to INT_MAX/MIN, this multiplication fits safely in 32-bits
    assign i_term_raw = integral_error * $signed(i_gain); 
    assign d_term_raw = $signed(derivative_error) * $signed(d_gain);

    // Accumulate the Q16.16 terms
    assign total_correction_raw = p_term_raw + i_term_raw + d_term_raw;

    // --- Scaling and Output Saturation ---
    // Scale Q16.16 back down to Q24.8 by performing an arithmetic right shift by 8
    logic signed [31:0] total_correction_scaled;
    assign total_correction_scaled = total_correction_raw >>> 8;

    // Saturation clamping before truncating back to the 16-bit Q8.8 output format
    // This prevents catastrophic wrapping if the correction exceeds 16-bit limits
    always_comb begin
        if (total_correction_scaled > INT_MAX) begin
            pid_correction = 16'sd32767;
        end else if (total_correction_scaled < INT_MIN) begin
            pid_correction = -16'sd32768;
        end else begin
            pid_correction = total_correction_scaled[15:0];
        end
    end

endmodule
