`timescale 1ns / 1ps

module rc_receiver #(
    parameter int NUM_CHANNELS   = 4,
    parameter int CLK_FREQ_HZ    = 12_000_000,
    parameter int MIN_PULSE_US   = 1000,
    parameter int MAX_PULSE_US   = 2000,
    parameter int WD_TIMEOUT_MS  = 100   // Watchdog timeout to flag connection loss
)(
    input  logic                      clk,
    input  logic                      rst_n,
    input  logic [NUM_CHANNELS-1:0]   ppm_in,       // Raw asynchronous PWM inputs from receiver
    output logic [14:0]               channel_out[NUM_CHANNELS], // Standardized 15-bit output values
    output logic [NUM_CHANNELS-1:0]   valid         // Valid flags for each channel
);

    // --- Time Constant Calculations (in Clock Ticks) ---
    localparam int TICKS_MIN      = (CLK_FREQ_HZ / 1_000_000) * MIN_PULSE_US; // 12,000 ticks for 1.0 ms
    localparam int TICKS_MAX      = (CLK_FREQ_HZ / 1_000_000) * MAX_PULSE_US; // 24,000 ticks for 2.0 ms
    localparam int TICKS_MID      = (TICKS_MIN + TICKS_MAX) / 2;             // 18,000 ticks for 1.5 ms
    
    // Limits with 10% tolerance margin for validation (900 us to 2100 us)
    localparam int LIMIT_MIN      = (TICKS_MIN * 9) / 10;                     // 10,800 ticks (900 us)
    localparam int LIMIT_MAX      = (TICKS_MAX * 115) / 100;                  // 27,600 ticks (2.3 ms)
    localparam int TICKS_TIMEOUT  = (CLK_FREQ_HZ / 1_000_000) * 3000;         // 3.0 ms timeout for single pulse
    localparam int WD_LIMIT       = (CLK_FREQ_HZ / 1000) * WD_TIMEOUT_MS;     // Watchdog tick limit (1.2M ticks)

    // --- Channel Registers & Logic Generation ---
    generate
        for (genvar i = 0; i < NUM_CHANNELS; i++) begin : chan_gen
            // Safe default initialization depending on channel index:
            // Roll (0), Pitch (1), Yaw (2) default to neutral (18,000 ticks = 1.5 ms).
            // Throttle (3) and other channels default to minimum (12,000 ticks = 1.0 ms).
            localparam int SAFE_DEFAULT = (i < 3) ? TICKS_MID : TICKS_MIN;

            // 1. Metastability Synchronization (2-stage FF chain)
            logic sync_stage1;
            logic sync_stage2;
            logic sync_stage2_prev;

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    sync_stage1      <= 1'b0;
                    sync_stage2      <= 1'b0;
                    sync_stage2_prev <= 1'b0;
                end else begin
                    sync_stage1      <= ppm_in[i];
                    sync_stage2      <= sync_stage1;
                    sync_stage2_prev <= sync_stage2;
                end
            end

            // 2. Synchronous Edge Detection
            logic pos_edge;
            logic neg_edge;
            assign pos_edge = sync_stage2 && !sync_stage2_prev;
            assign neg_edge = !sync_stage2 && sync_stage2_prev;

            // 3. Pulse Width Measurement Counter & State
            logic [15:0] pulse_cnt;
            logic        pulse_active;
            logic [14:0] measure_reg;
            logic        measure_valid;

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    pulse_cnt     <= 16'd0;
                    pulse_active  <= 1'b0;
                    measure_reg   <= SAFE_DEFAULT[14:0];
                    measure_valid <= 1'b0;
                end else begin
                    if (pos_edge) begin
                        pulse_cnt    <= 16'd0;
                        pulse_active <= 1'b1;
                    end else if (pulse_active) begin
                        if (neg_edge) begin
                            pulse_active <= 1'b0;
                            // Check if pulse width is within valid tolerances
                            if (pulse_cnt >= LIMIT_MIN[15:0] && pulse_cnt <= LIMIT_MAX[15:0]) begin
                                measure_valid <= 1'b1;
                                // Clamping/Saturating to standard [12000, 24000] range
                                if (pulse_cnt < TICKS_MIN[15:0]) begin
                                    measure_reg <= TICKS_MIN[14:0];
                                end else if (pulse_cnt > TICKS_MAX[15:0]) begin
                                    measure_reg <= TICKS_MAX[14:0];
                                end else begin
                                    measure_reg <= pulse_cnt[14:0];
                                end
                            end else begin
                                // Invalid width: invalidate but hold previous clean value for filter smoothness
                                measure_valid <= 1'b0;
                            end
                        end else begin
                            // Handle single pulse timeout to prevent locking up high
                            if (pulse_cnt >= TICKS_TIMEOUT[15:0]) begin
                                pulse_active  <= 1'b0;
                                measure_valid <= 1'b0;
                                measure_reg   <= SAFE_DEFAULT[14:0]; // Go to safe default
                            end else begin
                                pulse_cnt <= pulse_cnt + 16'd1;
                            end
                        end
                    end
                end
            end

            // 4. Lost Signal/Connection Watchdog Timer
            logic [20:0] wd_cnt;
            logic        wd_valid;

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    wd_cnt   <= 21'd0;
                    wd_valid <= 1'b0;
                end else begin
                    if (pos_edge) begin
                        wd_cnt   <= 21'd0;
                        // Don't validate instantly; validate when a full clean pulse is recorded
                    end else begin
                        if (wd_cnt >= WD_LIMIT[20:0]) begin
                            wd_valid <= 1'b0; // Flag timeout
                        end else begin
                            wd_cnt <= wd_cnt + 21'd1;
                        end
                    end

                    // Sync wd_valid with measure_valid
                    if (measure_valid && pos_edge) begin
                        // A clean pulse started, keep watchdog active
                        wd_valid <= 1'b1;
                    end
                end
            end

            // 5. Output Assignment
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    channel_out[i] <= SAFE_DEFAULT[14:0];
                    valid[i]       <= 1'b0;
                end else begin
                    if (wd_valid && measure_valid) begin
                        channel_out[i] <= measure_reg;
                        valid[i]       <= 1'b1;
                    end else begin
                        // Failsafe mode: if watchdog or measurement is invalid, clamp output to safe default
                        channel_out[i] <= SAFE_DEFAULT[14:0];
                        valid[i]       <= 1'b0;
                    end
                end
            end
        end
    endgenerate

endmodule
