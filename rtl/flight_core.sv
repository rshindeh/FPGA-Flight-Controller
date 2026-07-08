`timescale 1ns / 1ps

module flight_core (
    input  logic       clk,             // 12 MHz System Clock
    input  logic       rst_n,           // Active-Low Synchronized Reset
    inout  wire        i2c_sda_io,      // I2C Serial Data Line
    inout  wire        i2c_scl,         // I2C Serial Clock Line
    input  logic [3:0] rc_inputs,       // Asynchronous PWM inputs from radio receiver
    output logic [3:0] esc_pwm_outputs  // PWM signals to the 4 ESCs
);

    // --- Internal Routing Wires ---
    
    // I2C Master Data
    logic [15:0] gyro_x_raw;
    logic [15:0] gyro_y_raw;
    logic [15:0] gyro_z_raw;
    logic        imu_valid;
    logic        imu_error;
    
    // RC Receiver Outputs
    logic [14:0] rc_channels [4]; // Unpacked array directly from rc_receiver boundary
    logic [3:0]  rc_valid;
    
    // RC Receiver Discrete Channels
    logic [14:0] rc_roll;
    logic [14:0] rc_pitch;
    logic [14:0] rc_yaw;
    logic [14:0] rc_throttle;
    
    // PID Correction Outputs
    logic signed [15:0] pid_roll_corr;
    logic signed [15:0] pid_pitch_corr;
    logic signed [15:0] pid_yaw_corr;
    
    // Motor Mixer Outputs
    logic [14:0] motor_1_cmd;
    logic [14:0] motor_2_cmd;
    logic [14:0] motor_3_cmd;
    logic [14:0] motor_4_cmd;
    
    // --- Base PID Gains ---
    // In a real-world system, these might be dynamically configurable, but for structure
    // we define default Q8.8 fixed-point gains here.
    localparam logic signed [15:0] P_GAIN = 16'sh0200; // 2.0
    localparam logic signed [15:0] I_GAIN = 16'sh0010; // Small integral
    localparam logic signed [15:0] D_GAIN = 16'sh0100; // 1.0

    // =========================================================================
    // 1. RC Receiver Instance
    // =========================================================================
    rc_receiver #(
        .NUM_CHANNELS(4),
        .CLK_FREQ_HZ(12_000_000)
    ) rc_rx_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ppm_in(rc_inputs),
        .channel_out(rc_channels),
        .valid(rc_valid)
    );
    
    // Explicitly unpack the RC array boundary into discrete named wires
    assign rc_roll     = rc_channels[0];
    assign rc_pitch    = rc_channels[1];
    assign rc_yaw      = rc_channels[2];
    assign rc_throttle = rc_channels[3];

    // =========================================================================
    // 2. I2C Master Instance
    // =========================================================================
    i2c_master i2c_master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .scl(i2c_scl),
        .sda(i2c_sda_io),
        .gyro_x(gyro_x_raw),
        .gyro_y(gyro_y_raw),
        .gyro_z(gyro_z_raw),
        .valid(imu_valid),
        .error(imu_error)
    );

    // =========================================================================
    // 3. PID Controllers (Roll, Pitch, Yaw)
    // =========================================================================
    // - Enable is directly driven by the 1-cycle 'imu_valid' pulse from I2C Master.
    // - Unsigned RC inputs (15-bit) are explicitly zero-extended and cast to signed.
    // - I2C unsigned logic outputs are explicitly cast to signed.
    
    pid_calculator pid_roll (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .target_val($signed({1'b0, rc_roll})),
        .actual_val($signed(gyro_x_raw)),
        .p_gain(P_GAIN),
        .i_gain(I_GAIN),
        .d_gain(D_GAIN),
        .pid_correction(pid_roll_corr)
    );
    
    pid_calculator pid_pitch (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .target_val($signed({1'b0, rc_pitch})),
        .actual_val($signed(gyro_y_raw)),
        .p_gain(P_GAIN),
        .i_gain(I_GAIN),
        .d_gain(D_GAIN),
        .pid_correction(pid_pitch_corr)
    );
    
    pid_calculator pid_yaw (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .target_val($signed({1'b0, rc_yaw})),
        .actual_val($signed(gyro_z_raw)),
        .p_gain(P_GAIN),
        .i_gain(I_GAIN),
        .d_gain(D_GAIN),
        .pid_correction(pid_yaw_corr)
    );

    // =========================================================================
    // 4. Motor Mixer Instance
    // =========================================================================
    motor_mixer mixer_inst (
        .clk(clk),
        .rst_n(rst_n),
        .throttle_in(rc_throttle),
        .roll_correction(pid_roll_corr),
        .pitch_correction(pid_pitch_corr),
        .yaw_correction(pid_yaw_corr),
        .motor_1(motor_1_cmd),
        .motor_2(motor_2_cmd),
        .motor_3(motor_3_cmd),
        .motor_4(motor_4_cmd)
    );

    // =========================================================================
    // 5. ESC PWM Generators
    // =========================================================================
    pwm_generator esc_pwm_1 (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(motor_1_cmd),
        .pwm_out(esc_pwm_outputs[0])
    );
    
    pwm_generator esc_pwm_2 (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(motor_2_cmd),
        .pwm_out(esc_pwm_outputs[1])
    );
    
    pwm_generator esc_pwm_3 (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(motor_3_cmd),
        .pwm_out(esc_pwm_outputs[2])
    );
    
    pwm_generator esc_pwm_4 (
        .clk(clk),
        .rst_n(rst_n),
        .duty_cycle(motor_4_cmd),
        .pwm_out(esc_pwm_outputs[3])
    );

endmodule
