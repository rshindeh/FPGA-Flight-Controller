`timescale 1ns / 1ps

module flight_core #(
    parameter int ARM_TIME_CYCLES  = 12_000_000, // 1.0s at 12MHz (override in TB for faster simulation)
    parameter int INIT_WAIT_CYCLES = 120_000     // 10ms at 12MHz for MPU-6500 stabilization
)(
    input  logic       clk,             // 12 MHz System Clock
    input  logic       rst_n,           // Active-Low Synchronized Reset
    output logic       spi_sclk,        // SPI Serial Clock (Mode 0: idles low)
    output logic       spi_mosi,        // SPI Master Out Slave In
    input  logic       spi_miso,        // SPI Master In Slave Out
    output logic       spi_cs_n,        // SPI Active-Low Chip Select
    input  logic [3:0] rc_inputs,       // Asynchronous PWM inputs from radio receiver
    output logic [3:0] esc_pwm_outputs  // PWM signals to the 4 ESCs
);

    // --- Internal Signals ---
    
    // 1. RC Receiver Outputs
    logic [14:0] rc_channels [4]; // [0]=Roll, [1]=Pitch, [2]=Yaw, [3]=Throttle
    logic [3:0]  rc_valid;
    
    // 2. RC Mapper Setpoints (Q8.8 Fixed-Point Format)
    logic signed [15:0] target_roll_angle;  // Target Tilt (+/-30.0 deg)
    logic signed [15:0] target_pitch_angle; // Target Tilt (+/-30.0 deg)
    logic signed [15:0] target_yaw_rate;    // Target Rate (+/-100.0 deg/s)
    logic [14:0]        mapped_throttle;    // Clamped [12000, 24000]

    // 3. Safety Manager Flags
    logic armed;
    logic idle_throttle_active;

    // 4. SPI Master 14-Byte Stream
    logic [15:0] accel_x_raw;
    logic [15:0] accel_y_raw;
    logic [15:0] accel_z_raw;
    logic [15:0] gyro_x_raw;
    logic [15:0] gyro_y_raw;
    logic [15:0] gyro_z_raw;
    logic        imu_valid;
    logic        imu_error;

    // 5. Attitude Estimator Outputs (Q8.8 Fixed-Point Format)
    logic signed [15:0] est_roll_angle;
    logic signed [15:0] est_pitch_angle;

    // 6. Cascaded Outer-Loop Desired Rates (Q8.8 Fixed-Point Format)
    logic signed [15:0] desired_roll_rate;
    logic signed [15:0] desired_pitch_rate;

    // 7. Inner-Loop Rate PID Corrections (Q8.8 Fixed-Point Format)
    logic signed [15:0] pid_roll_corr;
    logic signed [15:0] pid_pitch_corr;
    logic signed [15:0] pid_yaw_corr;

    // 8. Motor Mixer Commands
    logic [14:0] motor_1_cmd;
    logic [14:0] motor_2_cmd;
    logic [14:0] motor_3_cmd;
    logic [14:0] motor_4_cmd;

    // --- Control Loop Gains (Q8.8 Fixed-Point Format) ---
    // Outer Angle Loop Gain: Converts Angle Error (deg) to Desired Angular Rate (deg/s)
    localparam logic signed [15:0] P_GAIN_ANGLE = 16'sh0400; // 4.0
    
    // Inner Rate Loop Gains: Converts Rate Error to Motor PWM Correction
    localparam logic signed [15:0] P_GAIN_RATE  = 16'sh0200; // 2.0
    localparam logic signed [15:0] I_GAIN_RATE  = 16'sh0010; // 0.0625
    localparam logic signed [15:0] D_GAIN_RATE  = 16'sh0100; // 1.0

    // =========================================================================
    // 1. RC Receiver Instance (Metastability Sync & Pulse Decoding)
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

    // =========================================================================
    // 2. RC Setpoint Mapper (Stick Zero-Centering & Scaling)
    // =========================================================================
    rc_mapper rc_mapper_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rc_roll(rc_channels[0]),
        .rc_pitch(rc_channels[1]),
        .rc_yaw(rc_channels[2]),
        .rc_throttle(rc_channels[3]),
        .target_roll_angle(target_roll_angle),
        .target_pitch_angle(target_pitch_angle),
        .target_yaw_rate(target_yaw_rate),
        .throttle_out(mapped_throttle)
    );

    // =========================================================================
    // 3. Safety Manager (Arming / Disarming & Ground Idle Interlock)
    // =========================================================================
    safety_mgr #(
        .ARM_TIME_CYCLES(ARM_TIME_CYCLES)
    ) safety_inst (
        .clk(clk),
        .rst_n(rst_n),
        .throttle_in(mapped_throttle),
        .yaw_in(rc_channels[2]),
        .rc_valid(rc_valid),
        .armed(armed),
        .idle_throttle_active(idle_throttle_active)
    );

    // =========================================================================
    // 4. SPI Master Instance (MPU-6500 14-Byte Continuous IMU Stream)
    // =========================================================================
    spi_master #(
        .CLK_FREQ_HZ(12_000_000),
        .SAMPLE_RATE_HZ(1_000),
        .INIT_WAIT_CYCLES(INIT_WAIT_CYCLES)
    ) spi_master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .accel_x(accel_x_raw),
        .accel_y(accel_y_raw),
        .accel_z(accel_z_raw),
        .gyro_x(gyro_x_raw),
        .gyro_y(gyro_y_raw),
        .gyro_z(gyro_z_raw),
        .valid(imu_valid),
        .error(imu_error)
    );

    // =========================================================================
    // 5. 6-DOF Attitude Estimator (Complementary Filter)
    // =========================================================================
    attitude_estimator attitude_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .accel_x($signed(accel_x_raw)),
        .accel_y($signed(accel_y_raw)),
        .accel_z($signed(accel_z_raw)),
        .gyro_x($signed(gyro_x_raw)),
        .gyro_y($signed(gyro_y_raw)),
        .gyro_z($signed(gyro_z_raw)),
        .roll_angle(est_roll_angle),
        .pitch_angle(est_pitch_angle)
    );

    // =========================================================================
    // 6. Cascaded Outer-Loop Angle P-Controllers (Self-Leveling)
    // =========================================================================
    // Roll Angle Outer Loop -> outputs desired roll rate
    pid_calculator pid_angle_roll (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .clear_i(idle_throttle_active),
        .target_val(target_roll_angle),
        .actual_val(est_roll_angle),
        .p_gain(P_GAIN_ANGLE),
        .i_gain(16'sd0),
        .d_gain(16'sd0),
        .pid_correction(desired_roll_rate)
    );

    // Pitch Angle Outer Loop -> outputs desired pitch rate
    pid_calculator pid_angle_pitch (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .clear_i(idle_throttle_active),
        .target_val(target_pitch_angle),
        .actual_val(est_pitch_angle),
        .p_gain(P_GAIN_ANGLE),
        .i_gain(16'sd0),
        .d_gain(16'sd0),
        .pid_correction(desired_pitch_rate)
    );

    // =========================================================================
    // 7. Cascaded Inner-Loop Rate PID Controllers (Roll, Pitch, Yaw)
    // =========================================================================
    pid_calculator pid_rate_roll (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .clear_i(idle_throttle_active),
        .target_val(desired_roll_rate),
        .actual_val($signed(gyro_x_raw)),
        .p_gain(P_GAIN_RATE),
        .i_gain(I_GAIN_RATE),
        .d_gain(D_GAIN_RATE),
        .pid_correction(pid_roll_corr)
    );

    pid_calculator pid_rate_pitch (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .clear_i(idle_throttle_active),
        .target_val(desired_pitch_rate),
        .actual_val($signed(gyro_y_raw)),
        .p_gain(P_GAIN_RATE),
        .i_gain(I_GAIN_RATE),
        .d_gain(D_GAIN_RATE),
        .pid_correction(pid_pitch_corr)
    );

    pid_calculator pid_rate_yaw (
        .clk(clk),
        .rst_n(rst_n),
        .enable(imu_valid),
        .clear_i(idle_throttle_active),
        .target_val(target_yaw_rate),
        .actual_val($signed(gyro_z_raw)),
        .p_gain(P_GAIN_RATE),
        .i_gain(I_GAIN_RATE),
        .d_gain(D_GAIN_RATE),
        .pid_correction(pid_yaw_corr)
    );

    // =========================================================================
    // 8. Quad-X Motor Mixer Instance (ESC Standard Clamping [12000, 24000])
    // =========================================================================
    motor_mixer mixer_inst (
        .clk(clk),
        .rst_n(rst_n),
        .armed(armed),
        .throttle_in(mapped_throttle),
        .roll_correction(pid_roll_corr),
        .pitch_correction(pid_pitch_corr),
        .yaw_correction(pid_yaw_corr),
        .motor_1(motor_1_cmd),
        .motor_2(motor_2_cmd),
        .motor_3(motor_3_cmd),
        .motor_4(motor_4_cmd)
    );

    // =========================================================================
    // 9. ESC PWM Generators (400 Hz, Double-Buffered)
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
