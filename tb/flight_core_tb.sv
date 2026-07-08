`timescale 1ns / 1ps

module flight_core_tb();

    logic       clk;
    logic       rst_n;
    wire        i2c_sda_io;
    wire        i2c_scl;
    logic [3:0] rc_inputs;
    wire  [3:0] esc_pwm_outputs;

    // --- UUT Instantiation ---
    flight_core uut (
        .clk(clk),
        .rst_n(rst_n),
        .i2c_sda_io(i2c_sda_io),
        .i2c_scl(i2c_scl),
        .rc_inputs(rc_inputs),
        .esc_pwm_outputs(esc_pwm_outputs)
    );

    // 1. Clock Generation (12 MHz)
    // 12 MHz = 83.333 ns period => toggle every 41.667 ns
    initial begin
        clk = 0;
        forever #41.667 clk = ~clk; 
    end

    // Open-drain pull-ups for I2C bus
    pullup(i2c_sda_io);
    pullup(i2c_scl);



    // 2. RC Transmitter Emulation (50Hz / 20ms Frame)
    initial begin
        rc_inputs = 4'b0000;
        forever begin
            #0; 
            rc_inputs = 4'b1111;
            // Throttle (ch3) goes low after 1.2 ms (1200000 ns)
            #1200000; 
            rc_inputs[3] = 1'b0;
            // Roll, Pitch, Yaw (ch0, ch1, ch2) go low after 1.5 ms (1500000 ns total)
            #300000; 
            rc_inputs[2:0] = 3'b000;
            // Wait for end of 20 ms frame (20000000 ns total)
            #18500000; 
        end
    end

    // 3. I2C Slave Emulation (Mock MPU-6050)
    logic sda_drv;
    assign i2c_sda_io = sda_drv ? 1'bz : 1'b0;

    task wait_posedge_scl();
        bit stable;
        stable = 0;
        while (!stable) begin
            @(posedge i2c_scl);
            #10; // Wait 10 ns to filter out zero-delay glitches/race conditions on SCL
            if (i2c_scl === 1'b1) begin
                stable = 1;
            end
        end
    endtask

    task read_byte(output logic [7:0] data);
        begin
            for (int i = 0; i < 8; i++) begin
                wait_posedge_scl();
                data[7 - i] = i2c_sda_io;
                $display("[I2C Slave] %t: Bit %0d = %b", $time, 7-i, i2c_sda_io);
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
    
    // Minimal I2C slave FSM tracking the specific init/read sequence
    initial begin
        sda_drv = 1;
        forever begin
            automatic logic [7:0] addr, reg_addr;
            @(negedge i2c_sda_io iff i2c_scl === 1'b1); // Wait for START
            $display("[I2C Slave] %t: Detected START condition", $time);
            @(negedge i2c_scl); // Wait for SCL to drop before reading bits
            
            read_byte(addr);
            $display("[I2C Slave] %t: Read address %h", $time, addr);
            
            if (addr == 8'hD0) begin // Master Write
                send_ack();
                read_byte(reg_addr);
                send_ack();
                if (reg_addr == 8'h6B) begin
                    logic [7:0] data;
                    read_byte(data);
                    send_ack();
                    $display("[I2C Slave] %t: Received MPU-6050 Wakeup Write", $time);
                end else if (reg_addr == 8'h43) begin
                    $display("[I2C Slave] %t: Master requested gyro read from 0x43", $time);
                    // Expected Repeated START next
                    @(negedge i2c_sda_io iff i2c_scl === 1'b1);
                    @(negedge i2c_scl); // Wait for SCL to drop before reading bits
                    read_byte(addr);
                    if (addr == 8'hD1) begin // Master Read
                        send_ack();
                        
                        $display("[I2C Slave] %t: Master reading gyro registers", $time);
                        send_byte(8'h42); wait_for_ack(); // Roll H
                        send_byte(8'h68); wait_for_ack(); // Roll L
                        
                        send_byte(8'h46); wait_for_ack(); // Pitch H
                        send_byte(8'h50); wait_for_ack(); // Pitch L
                        
                        send_byte(8'h46); wait_for_ack(); // Yaw H
                        send_byte(8'h50); wait_for_ack(); // Master NACKs last byte
                    end
                end
            end
        end
    end

    // 4. Testbench Control and Measurement
    realtime m1_start, m1_end, m1_width;
    realtime m3_start, m3_end, m3_width;

    initial begin
        $display("=================================================");
        $display("[TB] Starting Full System Flight Core Verification");
        $display("=================================================");
        
        rst_n = 0;
        #100;
        rst_n = 1;

        $display("[TB] Reset Released. Initializing MPU-6050 (10ms wait)...");
        
        // Wait ~25ms simulation time for I2C init and RC pulses to fully stabilize
        // Print progress every 5ms
        for (int i = 0; i < 5; i++) begin
            #5000000;
            $display("[TB] Time: %0.2f ms", $realtime / 1000000.0);
        end
        
        $display("\n[TB] --- Sampling ESC PWM Outputs ---");
        // Measure Motor 1 (Front Right: Throttle - Roll - Pitch + Yaw)
        @(posedge esc_pwm_outputs[0]);
        m1_start = $realtime;
        @(negedge esc_pwm_outputs[0]);
        m1_end = $realtime;
        m1_width = (m1_end - m1_start) / 1000000.0; // ns to ms
        
        // Measure Motor 3 (Rear Left: Throttle + Roll + Pitch + Yaw)
        @(posedge esc_pwm_outputs[2]);
        m3_start = $realtime;
        @(negedge esc_pwm_outputs[2]);
        m3_end = $realtime;
        m3_width = (m3_end - m3_start) / 1000000.0; // ns to ms
        
        $display("[TB] Motor 1 (Front Right) PWM Width: %0.3f ms", m1_width);
        $display("[TB] Motor 3 (Rear Left)   PWM Width: %0.3f ms", m3_width);
        
        if (m3_width > m1_width) begin
            $display("[TB] PASS: Motor 3 duty cycle is correctly higher than Motor 1 to counter the mock positive Roll error!");
        end else begin
            $display("[TB] FAIL: Motor mixing response did not match expected dynamic shift.");
        end
        
        $display("=================================================");
        $finish;
    end

endmodule
