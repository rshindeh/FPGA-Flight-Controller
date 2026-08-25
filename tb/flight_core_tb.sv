`timescale 1ns / 1ps

module flight_core_tb();

    logic       clk;
    logic       rst_n;
    wire        i2c_sda_io;
    wire        i2c_scl;
    logic [3:0] rc_inputs;
    wire  [3:0] esc_pwm_outputs;

    // --- UUT Instantiation (Accelerated 100us arming timer for simulation) ---
    flight_core #(
        .ARM_TIME_CYCLES(1200) // 1200 cycles = 100 us
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .i2c_sda_io(i2c_sda_io),
        .i2c_scl(i2c_scl),
        .rc_inputs(rc_inputs),
        .esc_pwm_outputs(esc_pwm_outputs)
    );

    // 1. Clock Generation (12 MHz = 83.333 ns period)
    initial begin
        clk = 0;
        forever #41.667 clk = ~clk; 
    end

    // Open-drain pull-ups for I2C bus
    pullup(i2c_sda_io);
    pullup(i2c_scl);

    // 2. RC Transmitter Emulation (Dynamic Pulse Generation)
    real throttle_ms = 1.0;
    real roll_ms     = 1.5;
    real pitch_ms    = 1.5;
    real yaw_ms      = 1.5;

    initial begin
        rc_inputs = 4'b0000;
        forever begin
            rc_inputs = 4'b1111;
            
            // Fork pulses with specified widths
            fork
                begin #(roll_ms     * 1000000); rc_inputs[0] = 1'b0; end
                begin #(pitch_ms    * 1000000); rc_inputs[1] = 1'b0; end
                begin #(yaw_ms      * 1000000); rc_inputs[2] = 1'b0; end
                begin #(throttle_ms * 1000000); rc_inputs[3] = 1'b0; end
            join
            
            // Remainder of 20 ms frame
            #17000000;
        end
    end

    // 3. I2C Slave Emulation (Mock MPU-6050 with 14-Byte Burst Support)
    logic sda_drv;
    assign i2c_sda_io = sda_drv ? 1'bz : 1'b0;

    task wait_posedge_scl();
        bit stable;
        stable = 0;
        while (!stable) begin
            @(posedge i2c_scl);
            #10;
            if (i2c_scl === 1'b1) stable = 1;
        end
    endtask

    task read_byte(output logic [7:0] data);
        begin
            for (int i = 0; i < 8; i++) begin
                wait_posedge_scl();
                data[7 - i] = i2c_sda_io;
                @(negedge i2c_scl);
            end
        end
    endtask

    task send_ack();
        begin
            sda_drv = 0;
            wait_posedge_scl();
            @(negedge i2c_scl);
            sda_drv = 1;
        end
    endtask

    task send_byte(input logic [7:0] data);
        begin
            for (int i = 0; i < 8; i++) begin
                sda_drv = (data[7 - i]) ? 1 : 0;
                wait_posedge_scl();
                @(negedge i2c_scl);
            end
            sda_drv = 1; // Release bus so master can ACK/NACK
        end
    endtask

    task wait_for_ack();
        begin
            wait_posedge_scl();
            @(negedge i2c_scl);
        end
    endtask
    
    // I2C slave responder
    initial begin
        sda_drv = 1;
        forever begin
            automatic logic [7:0] addr, reg_addr;
            @(negedge i2c_sda_io iff i2c_scl === 1'b1); // START condition
            @(negedge i2c_scl);
            
            read_byte(addr);
            
            if (addr == 8'hD0) begin // Master Write
                send_ack();
                read_byte(reg_addr);
                send_ack();
                if (reg_addr == 8'h6B) begin
                    logic [7:0] data;
                    read_byte(data);
                    send_ack();
                    $display("[I2C Slave] %0.2f ms: Received MPU-6050 Wakeup Write (0x6B)", $realtime / 1.0e6);
                end else if (reg_addr == 8'h3B) begin
                    // Repeated START for 14-byte sensor read
                    @(negedge i2c_sda_io iff i2c_scl === 1'b1);
                    @(negedge i2c_scl);
                    read_byte(addr);
                    if (addr == 8'hD1) begin
                        send_ack();
                        
                        // Mock Sensor Data: Accel Roll Tilt of +10.0 deg (accel_y = +2845 = 0x0B1D)
                        send_byte(8'h00); wait_for_ack(); // Accel X_H
                        send_byte(8'h00); wait_for_ack(); // Accel X_L
                        send_byte(8'h0B); wait_for_ack(); // Accel Y_H (+2845 -> +10 deg roll)
                        send_byte(8'h1D); wait_for_ack(); // Accel Y_L
                        send_byte(8'h3F); wait_for_ack(); // Accel Z_H (+16135)
                        send_byte(8'h07); wait_for_ack(); // Accel Z_L
                        
                        send_byte(8'h00); wait_for_ack(); // Temp H
                        send_byte(8'h00); wait_for_ack(); // Temp L
                        
                        send_byte(8'h00); wait_for_ack(); // Gyro X_H (0 rate)
                        send_byte(8'h00); wait_for_ack(); // Gyro X_L
                        send_byte(8'h00); wait_for_ack(); // Gyro Y_H
                        send_byte(8'h00); wait_for_ack(); // Gyro Y_L
                        send_byte(8'h00); wait_for_ack(); // Gyro Z_H
                        send_byte(8'h00); wait_for_ack(); // Gyro Z_L (Master NACKs last byte)
                    end
                end
            end
        end
    end

    // 4. Testbench Control & Measurement
    realtime m1_start, m1_end, m1_width;
    realtime m3_start, m3_end, m3_width;

    initial begin
        $display("=================================================================");
        $display("[TB] Starting Full Cascaded Self-Leveling Flight Core Verification");
        $display("=================================================================");
        
        rst_n = 0;
        #100;
        rst_n = 1;
        $display("[TB] Reset Released. Initializing MPU-6050...");

        // -------------------------------------------------------------
        // Step 1: Wait 12 ms for MPU-6050 power-on initialization
        // -------------------------------------------------------------
        #12000000;
        $display("[TB] Time: %0.2f ms | MPU-6050 Initialized. System currently DISARMED.", $realtime / 1.0e6);

        // Verify Disarmed Output Lockout (Motors locked to 1.0 ms)
        @(posedge esc_pwm_outputs[0]);
        m1_start = $realtime;
        @(negedge esc_pwm_outputs[0]);
        m1_width = ($realtime - m1_start) / 1.0e6;
        $display("[TB] Disarmed State Motor 1 Width: %0.3f ms (Expected 1.000 ms)", m1_width);
        if (m1_width >= 0.99 && m1_width <= 1.01) begin
            $display("[TB] PASS: Disarmed safety interlock verified (1.0 ms lockout).");
        end else begin
            $display("[TB] FAIL: Disarmed safety interlock failed!");
        end

        // -------------------------------------------------------------
        // Step 2: Perform Arming Sequence (Throttle Min, Yaw Full Right)
        // -------------------------------------------------------------
        $display("\n[TB] --- Applying Arming Sequence (Throttle Min, Yaw Full Right) ---");
        throttle_ms = 1.0;
        yaw_ms      = 1.95; // > 23,000 ticks
        roll_ms     = 1.5;
        pitch_ms    = 1.5;
        
        // Wait 25 ms (exceeds ARM_TIME_CYCLES of 100 us)
        #25000000;
        $display("[TB] Time: %0.2f ms | Arming sequence applied.", $realtime / 1.0e6);

        // -------------------------------------------------------------
        // Step 3: Enter Hover Flight (Throttle 1.5 ms, Sticks Centered)
        // -------------------------------------------------------------
        $display("\n[TB] --- Entering Hover Flight Mode (Throttle = 1.5ms, Sticks Centered at 0.0 deg) ---");
        throttle_ms = 1.5; // Hover throttle (18,000 ticks)
        yaw_ms      = 1.5; // Neutral (Target Yaw Rate = 0.0 deg/s)
        roll_ms     = 1.5; // Neutral (Target Roll Angle = 0.0 deg)
        pitch_ms    = 1.5; // Neutral (Target Pitch Angle = 0.0 deg)

        // Wait 50 ms (2.5 RC frames) for RC receiver, filter, and PID controllers to settle
        #50000000;

        // -------------------------------------------------------------
        // Step 4: Measure Closed-Loop Self-Leveling Motor PWM Outputs
        // -------------------------------------------------------------
        $display("\n[TB] --- Measuring Self-Leveling PWM Corrections ---");
        $display("[DIAG] armed=%b, throttle=%0d, est_roll=%0d (%0.2f deg), desired_roll_rate=%0d (%0.2f deg/s), pid_roll_corr=%0d, m1_cmd=%0d, m3_cmd=%0d", 
                 uut.armed, uut.mapped_throttle, 
                 uut.est_roll_angle, real'(uut.est_roll_angle) / 256.0,
                 uut.desired_roll_rate, real'(uut.desired_roll_rate) / 256.0,
                 uut.pid_roll_corr, uut.motor_1_cmd, uut.motor_3_cmd);

        // Fork simultaneous measurement of Motor 1 and Motor 3
        fork
            begin
                @(posedge esc_pwm_outputs[0]);
                m1_start = $realtime;
                @(negedge esc_pwm_outputs[0]);
                m1_width = ($realtime - m1_start) / 1.0e6;
            end
            begin
                @(posedge esc_pwm_outputs[2]);
                m3_start = $realtime;
                @(negedge esc_pwm_outputs[2]);
                m3_width = ($realtime - m3_start) / 1.0e6;
            end
        join

        $display("[TB] Motor 1 (Front Right) PWM Width: %0.3f ms", m1_width);
        $display("[TB] Motor 3 (Rear Left)   PWM Width: %0.3f ms", m3_width);

        // In Angle mode: Aircraft is tilted +10 deg Roll Right.
        // Target is 0 deg. Angle Error is -10 deg.
        // Outer loop demands left-roll rotation -> Right motors (M1, M2) increase thrust,
        // Left motors (M3, M4) decrease thrust to push the right side up and restore 0 deg flat hover!
        if (m1_width > m3_width && m1_width > 1.50 && m3_width < 1.50) begin
            $display("[TB] PASS: Self-leveling active! Right motor (M1: %0.3f ms) increased above hover and Left motor (M3: %0.3f ms) decreased to right the aircraft!",
                     m1_width, m3_width);
        end else begin
            $display("[TB] FAIL: Self-leveling response did not meet expected counter-tilt direction.");
        end

        $display("=================================================================");
        $display("[TB] Full System Cascaded Verification Completed Successfully!");
        $display("=================================================================");
        $finish;
    end

endmodule
