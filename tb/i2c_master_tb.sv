`timescale 1ns / 1ps

module i2c_master_tb();

    // 1. Emulate signals
    logic        clk;
    logic        rst_n;
    tri1         scl;
    tri1         sda;
    logic [15:0] accel_x;
    logic [15:0] accel_y;
    logic [15:0] accel_z;
    logic [15:0] gyro_x;
    logic [15:0] gyro_y;
    logic [15:0] gyro_z;
    logic        valid;
    logic        error;

    // 2. Instantiate UUT (Unit Under Test)
    i2c_master uut (
        .clk(clk),
        .rst_n(rst_n),
        .scl(scl),
        .sda(sda),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .valid(valid),
        .error(error)
    );

    // 3. Instantiate Synchronous Slave Model
    i2c_slave_model slave (
        .clk(clk),
        .rst_n(rst_n),
        .scl(scl),
        .sda(sda)
    );

    // 4. Generate 12 MHz clock (Period = 83.33 ns)
    always begin
        clk = 1'b1;
        #41.67;
        clk = 1'b0;
        #41.67;
    end

    // 5. Monitor Output Values & Automated Assertions
    int valid_read_count = 0;
    int error_count = 0;

    always @(posedge valid) begin
        valid_read_count++;
        $display("[TB MONITOR] Time = %0.3f ms | 14-Byte IMU Burst Read #%0d Received:", $realtime / 1.0e6, valid_read_count);
        $display("             Accel (X,Y,Z): %d, %d, %d (0x%h, 0x%h, 0x%h)", 
                 $signed(accel_x), $signed(accel_y), $signed(accel_z), accel_x, accel_y, accel_z);
        $display("             Gyro  (X,Y,Z): %d, %d, %d (0x%h, 0x%h, 0x%h)", 
                 $signed(gyro_x), $signed(gyro_y), $signed(gyro_z), gyro_x, gyro_y, gyro_z);

        if (accel_x === 16'h0102 && accel_y === 16'h0304 && accel_z === 16'h4000 &&
            gyro_x === 16'h1234 && gyro_y === 16'h5678 && gyro_z === 16'h9ABC) begin
            $display("[TB CHECK] PASS: Sample #%0d values match expected registers exactly!", valid_read_count);
        end else begin
            $display("[TB CHECK] FAIL: Sample #%0d data mismatch!", valid_read_count);
            error_count++;
        end
    end

    always @(posedge error) begin
        $display("[TB MONITOR] Time = %0.3f ms | Error condition detected!", $realtime / 1.0e6);
        error_count++;
    end

    // 6. Test Stimulus Block
    initial begin
        $display("[TB] Starting I2C Master Testbench...");
        
        // Apply reset
        rst_n = 1'b0;
        #200;
        rst_n = 1'b1;
        $display("[TB] Reset released at 200 ns.");

        // Run simulation long enough to observe initialization delay (10 ms)
        // wake-up stabilization (10 ms), and multiple periodic read loops (1 ms each).
        // Total simulation: 25 milliseconds
        #25000000;

        $display("\n=================================================");
        $display("[TB SUMMARY] Total valid gyro reads: %0d", valid_read_count);
        $display("[TB SUMMARY] Total errors: %0d", error_count);
        if (valid_read_count >= 3 && error_count == 0) begin
            $display("[TB] PASS: All I2C Master transactions verified successfully!");
        end else begin
            $display("[TB] FAIL: I2C Master verification failed!");
        end
        $display("=================================================");
        $finish;
    end

endmodule


// ============================================================================
// Synchronous Behavioral Slave Model of MPU-6050 for I2C Verification
// ============================================================================
module i2c_slave_model (
    input  logic clk,     // 12 MHz system clock
    input  logic rst_n,   // Active-low reset
    input  wire  scl,
    inout  wire  sda
);
    localparam [6:0] SLAVE_ADDR = 7'h68;

    logic [7:0] registers[255];
    logic [7:0] reg_ptr = 8'h00;
    logic [7:0] shift_reg = 8'h00;
    logic [3:0] bit_cnt = 4'd0;
    logic        sda_out = 1'b1; // 1 = float

    assign sda = sda_out ? 1'bz : 1'b0;

    // Registers to track rising/falling edges synchronously
    logic scl_reg, scl_prev;
    logic sda_reg, sda_prev;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_reg  <= 1'b1;
            scl_prev <= 1'b1;
            sda_reg  <= 1'b1;
            sda_prev <= 1'b1;
        end else begin
            scl_reg  <= scl;
            scl_prev <= scl_reg;
            sda_reg  <= sda;
            sda_prev <= sda_reg;
        end
    end

    // Combinational Edge/Condition Detections
    logic scl_posedge;
    logic scl_negedge;
    logic start_cond;
    logic stop_cond;

    assign scl_posedge = scl_reg && !scl_prev;
    assign scl_negedge = !scl_reg && scl_prev;
    assign start_cond  = scl_reg && (!sda_reg && sda_prev); // SDA falls while SCL high
    assign stop_cond   = scl_reg && (sda_reg && !sda_prev);  // SDA rises while SCL high

    // Slave States
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_ADDR,
        ST_WRITE_REG,
        ST_WRITE_DATA,
        ST_READ_DATA
    } state_t;

    state_t state = ST_IDLE;
    logic is_read = 1'b0;
    logic start_seen = 1'b0;

    // Diagnostic logging
    always @(posedge clk) begin
        if (scl_posedge && state != ST_IDLE) begin
            $display("[MPU-6050 SLAVE] Time = %0.3f ms | Sampled bit %0d = %b | shift_reg = %b", 
                     $realtime / 1.0e6, bit_cnt, sda_reg, {shift_reg[6:0], sda_reg});
        end
        if (scl_negedge && state != ST_IDLE) begin
            $display("[MPU-6050 SLAVE] Time = %0.3f ms | SCL fell | bit_cnt = %0d, state = %0d", 
                     $realtime / 1.0e6, bit_cnt, state);
        end
    end

    initial begin
        // Populate MPU-6050 registers with known mock data
        registers[8'h3B] = 8'h01; // ACCEL_XOUT_H
        registers[8'h3C] = 8'h02; // ACCEL_XOUT_L
        registers[8'h3D] = 8'h03; // ACCEL_YOUT_H
        registers[8'h3E] = 8'h04; // ACCEL_YOUT_L
        registers[8'h3F] = 8'h40; // ACCEL_ZOUT_H
        registers[8'h40] = 8'h00; // ACCEL_ZOUT_L
        registers[8'h41] = 8'h00; // TEMP_OUT_H
        registers[8'h42] = 8'h00; // TEMP_OUT_L
        registers[8'h43] = 8'h12; // GYRO_XOUT_H
        registers[8'h44] = 8'h34; // GYRO_XOUT_L
        registers[8'h45] = 8'h56; // GYRO_YOUT_H
        registers[8'h46] = 8'h78; // GYRO_YOUT_L
        registers[8'h47] = 8'h9A; // GYRO_ZOUT_H
        registers[8'h48] = 8'hBC; // GYRO_ZOUT_L
        registers[8'h6B] = 8'h40; // PWR_MGMT_1 (defaults to sleep mode 0x40)
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            bit_cnt    <= 4'd0;
            sda_out    <= 1'b1;
            reg_ptr    <= 8'h00;
            is_read    <= 1'b0;
            start_seen <= 1'b0;
        end else begin
            if (start_cond) begin
                state      <= ST_ADDR;
                bit_cnt    <= 4'd0;
                sda_out    <= 1'b1;
                start_seen <= 1'b1;
            end else if (stop_cond) begin
                state      <= ST_IDLE;
                sda_out    <= 1'b1;
                start_seen <= 1'b0;
            end else if (state != ST_IDLE) begin
                // 1. Input shift register sampling (on rising SCL edge)
                if (scl_posedge) begin
                    if (bit_cnt < 4'd8) begin
                        if (state == ST_ADDR || state == ST_WRITE_REG || state == ST_WRITE_DATA) begin
                            shift_reg <= {shift_reg[6:0], sda_reg};
                        end
                    end
                end

                // 2. Output driving & State transitions (on falling SCL edge)
                if (scl_negedge) begin
                    if (start_seen) begin
                        start_seen <= 1'b0;
                    end else begin
                        case (state)
                            ST_ADDR: begin
                                if (bit_cnt < 4'd7) begin
                                    sda_out <= 1'b1; // release SDA
                                    bit_cnt <= bit_cnt + 4'd1;
                                end else if (bit_cnt == 4'd7) begin
                                    // Check received address
                                    if (shift_reg[7:1] == SLAVE_ADDR) begin
                                        sda_out <= 1'b0; // Drive ACK during 9th bit
                                        is_read <= shift_reg[0];
                                        bit_cnt <= 4'd8; // Go to ACK phase
                                        $display("[MPU-6050 SLAVE] Time = %0.3f ms | Addr matches! R/W = %b", $realtime / 1.0e6, shift_reg[0]);
                                    end else begin
                                        sda_out <= 1'b1; // NACK
                                        state   <= ST_IDLE;
                                        bit_cnt <= 4'd0;
                                    end
                                end else if (bit_cnt == 4'd8) begin
                                    sda_out <= 1'b1; // Release SDA
                                    bit_cnt <= 4'd0;
                                    if (is_read) begin
                                        state   <= ST_READ_DATA;
                                        sda_out <= registers[reg_ptr][7]; // Drive MSB of read data
                                    end else begin
                                        state   <= ST_WRITE_REG;
                                    end
                                end
                            end

                            ST_WRITE_REG: begin
                                if (bit_cnt < 4'd7) begin
                                    sda_out <= 1'b1; // release SDA
                                    bit_cnt <= bit_cnt + 4'd1;
                                end else if (bit_cnt == 4'd7) begin
                                    reg_ptr <= shift_reg;
                                    sda_out <= 1'b0; // ACK
                                    bit_cnt <= 4'd8;
                                    $display("[MPU-6050 SLAVE] Time = %0.3f ms | Register pointer set to 0x%h", $realtime / 1.0e6, shift_reg);
                                end else if (bit_cnt == 4'd8) begin
                                    sda_out <= 1'b1; // Release SDA
                                    bit_cnt <= 4'd0;
                                    state   <= ST_WRITE_DATA;
                                end
                            end

                            ST_WRITE_DATA: begin
                                if (bit_cnt < 4'd7) begin
                                    sda_out <= 1'b1; // release SDA
                                    bit_cnt <= bit_cnt + 4'd1;
                                end else if (bit_cnt == 4'd7) begin
                                    registers[reg_ptr] <= shift_reg;
                                    if (reg_ptr == 8'h6B) begin
                                        $display("[MPU-6050 SLAVE] Time = %0.3f ms | PWR_MGMT_1 (0x6B) written with 0x%h. Wake Status: %s",
                                                 $realtime / 1.0e6, shift_reg, (shift_reg == 8'h00) ? "ACTIVE" : "SLEEP");
                                    end
                                    sda_out <= 1'b0; // ACK
                                    bit_cnt <= 4'd8;
                                end else if (bit_cnt == 4'd8) begin
                                    sda_out <= 1'b1; // Release SDA
                                    bit_cnt <= 4'd0;
                                    reg_ptr <= reg_ptr + 8'd1; // Auto-increment register pointer
                                end
                            end

                            ST_READ_DATA: begin
                                if (bit_cnt < 4'd7) begin
                                    sda_out <= registers[reg_ptr][6 - bit_cnt]; // Drive bits 6 down to 0
                                    bit_cnt <= bit_cnt + 4'd1;
                                end else if (bit_cnt == 4'd7) begin
                                    sda_out <= 1'b1; // Release SDA for Master ACK
                                    bit_cnt <= 4'd8;
                                end else if (bit_cnt == 4'd8) begin
                                    // Sample Master ACK status (0 = ACK, 1 = NACK)
                                    if (sda_reg == 1'b0) begin
                                        reg_ptr <= reg_ptr + 8'd1; // Auto-increment pointer
                                        bit_cnt <= 4'd0;
                                        sda_out <= registers[reg_ptr + 8'd1][7]; // Drive bit 7 of next byte immediately
                                    end else begin
                                        state   <= ST_IDLE;
                                        sda_out <= 1'b1;
                                    end
                                end
                            end
                        endcase
                    end
                end
            end
        end
    end

endmodule
