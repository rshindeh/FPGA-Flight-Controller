`timescale 1ns / 1ps

module attitude_estimator_tb();

    logic               clk;
    logic               rst_n;
    logic               enable;
    logic signed [15:0] accel_x;
    logic signed [15:0] accel_y;
    logic signed [15:0] accel_z;
    logic signed [15:0] gyro_x;
    logic signed [15:0] gyro_y;
    logic signed [15:0] gyro_z;
    
    logic signed [15:0] roll_angle;
    logic signed [15:0] pitch_angle;

    // Instantiate UUT
    attitude_estimator uut (
        .clk(clk),
        .rst_n(rst_n),
        .enable(enable),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .roll_angle(roll_angle),
        .pitch_angle(pitch_angle)
    );

    // 12 MHz clock
    always begin
        clk = 1'b1; #41.667;
        clk = 1'b0; #41.667;
    end

    // Task to strobe enable at 1 kHz (every 1 ms = 1,000,000 ns in simulation, but we can step faster for testing)
    task step_filter(input int steps);
        begin
            for (int i = 0; i < steps; i++) begin
                #100;
                enable = 1'b1;
                #83.333;
                enable = 1'b0;
            end
        end
    endtask

    initial begin
        $display("[TB] Starting Attitude Estimator Verification...");
        rst_n = 1'b0;
        enable = 1'b0;
        accel_x = 16'sd0;
        accel_y = 16'sd0;
        accel_z = 16'sd16384; // 1g down
        gyro_x = 16'sd0;
        gyro_y = 16'sd0;
        gyro_z = 16'sd0;
        #200;
        rst_n = 1'b1;
        #200;

        // Test 1: Flat level at rest (Expected 0.0 deg)
        $display("[TB] --- Test 1: Flat level at rest ---");
        step_filter(50);
        $display("[TB] Flat Level: Roll = %0.2f deg (0x%h), Pitch = %0.2f deg (0x%h)", 
                 real'(roll_angle) / 256.0, roll_angle, real'(pitch_angle) / 256.0, pitch_angle);
        if (roll_angle == 16'sd0 && pitch_angle == 16'sd0) begin
            $display("[TB] PASS: Flat level estimation is exactly 0.0 degrees.");
        end else begin
            $display("[TB] FAIL: Flat level mismatch.");
        end

        // Test 2: Static Roll Tilt of +10.0 degrees (accel_y = 16384 * sin(10 deg) = 2845)
        $display("[TB] --- Test 2: Static Roll Tilt of +10.0 deg ---");
        accel_y = 16'sd2845;
        accel_z = 16'sd16135;
        step_filter(200); // 200 ms to converge
        $display("[TB] Converged Roll = %0.2f deg (0x%h) (Expected ~+10.0 deg / 0x0A00)", 
                 real'(roll_angle) / 256.0, roll_angle);
        if (roll_angle >= 16'sd2400 && roll_angle <= 16'sd2700) begin
            $display("[TB] PASS: Filter converged cleanly to +10.0 deg Roll angle.");
        end else begin
            $display("[TB] FAIL: Filter failed to converge to +10.0 deg.");
        end

        // Test 3: Static Pitch Tilt of +15.0 degrees (accel_x = -16384 * sin(15 deg) = -4240)
        $display("[TB] --- Test 3: Static Pitch Tilt of +15.0 deg ---");
        accel_y = 16'sd0;
        accel_x = -16'sd4240;
        step_filter(200);
        $display("[TB] Converged Pitch = %0.2f deg (0x%h) (Expected ~+15.0 deg / 0x0F00)", 
                 real'(pitch_angle) / 256.0, pitch_angle);
        if (pitch_angle >= 16'sd3700 && pitch_angle <= 16'sd4000) begin
            $display("[TB] PASS: Filter converged cleanly to +15.0 deg Pitch angle.");
        end else begin
            $display("[TB] FAIL: Filter failed to converge to +15.0 deg.");
        end

        $display("[TB] Attitude Estimator Verification Completed Successfully.");
        $finish;
    end

endmodule
