`timescale 1ns / 1ps

module pid_calculator (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               enable,  // Strobe high for 1 clock cycle to process a new sample
    input  logic               clear_i, // Strobe or hold high to clear the integral accumulator
    
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

    // Anti-windup and 16-bit signed saturation limits
    localparam logic signed [31:0] INT_MAX = 32'sd32767;
    localparam logic signed [31:0] INT_MIN = -32'sd32768;

    // --- Internal State Registers (hierarchical probes in TB) ---
    logic signed [15:0] current_error;
    logic signed [15:0] prev_error;
    logic signed [15:0] derivative_error;
    logic signed [31:0] integral_error;

    // Error Differentials
    assign current_error    = target_val - actual_val;
    assign derivative_error = current_error - prev_error;

    // Candidate Integral Accumulator
    logic signed [31:0] next_integral;
    assign next_integral = integral_error + current_error;

    // Synchronous State Pipeline
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_error     <= 16'sd0;
            integral_error <= 32'sd0;
        end else begin
            // Error memory tracks input error whenever new sample strobe arrives
            if (enable) begin
                prev_error <= current_error;
            end

            // Integral accumulator with anti-windup clamping
            if (clear_i) begin
                integral_error <= 32'sd0;
            end else if (enable) begin
                if (next_integral > INT_MAX) begin
                    integral_error <= INT_MAX;
                end else if (next_integral < INT_MIN) begin
                    integral_error <= INT_MIN;
                end else begin
                    integral_error <= next_integral;
                end
            end
        end
    end

    // --- Multiplier & Accumulation Datapath (32-bit arithmetic) ---
    // Q8.8 * Q8.8 yields Q16.16 formatted terms
    logic signed [31:0] p_term;
    logic signed [31:0] i_term;
    logic signed [31:0] d_term;
    logic signed [31:0] total_correction_raw;
    logic signed [31:0] total_correction_scaled;

    assign p_term = current_error * p_gain;
    assign i_term = integral_error * i_gain;
    assign d_term = derivative_error * d_gain;

    assign total_correction_raw    = p_term + i_term + d_term;
    assign total_correction_scaled = total_correction_raw >>> 8; // Downscale Q16.16 to Q24.8

    // Saturation clamping before truncating to 16-bit Q8.8 output format
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

