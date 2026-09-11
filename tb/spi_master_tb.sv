`timescale 1ns / 1ps

import fc_tb_pkg::*;

module spi_master_tb();

    `GLOBAL_WATCHDOG(30ms)

    logic        clk;
    logic        rst_n;
    wire         spi_sclk;
    wire         spi_mosi;
    wire         spi_miso;
    wire         spi_cs_n;

    logic [15:0] accel_x, accel_y, accel_z;
    logic [15:0] gyro_x, gyro_y, gyro_z;
    logic        valid;
    logic        error;

    int valid_read_count = 0;
    int error_count = 0;
    logic expect_error = 1'b0;

    // Accelerated simulation parameters (10 kHz burst, 10 us init delay)
    spi_master #(
        .CLK_FREQ_HZ(12_000_000),
        .SAMPLE_RATE_HZ(10_000),
        .INIT_WAIT_CYCLES(120)
    ) uut (
        .clk(clk),
        .rst_n(rst_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .spi_cs_n(spi_cs_n),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .valid(valid),
        .error(error)
    );

    // Behavioral MPU-6500 SPI Slave Model
    mpu6500_spi_slave_model slave (
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .miso(spi_miso),
        .cs_n(spi_cs_n)
    );

    // 12 MHz Master Clock (83.333 ns period)
    initial clk = 1'b0;
    always #41.667 clk = ~clk;

    // Verification Monitors & Assertions
    always @(posedge valid) begin
        valid_read_count++;
        check_assert(accel_x === 16'h0102 && accel_y === 16'h0304 && accel_z === 16'h4000 &&
                     gyro_x === 16'h1234 && gyro_y === 16'h5678 && gyro_z === 16'h9ABC,
                     "BURST_DATA", $sformatf("Sample #%0d values match expected registers", valid_read_count), error_count);
    end

    always @(posedge error) begin
        if (!expect_error) begin
            check_assert(1'b0, "UNEXPECTED_ERROR", "Unexpected error condition flagged by spi_master", error_count);
        end
    end

    // Concurrent SVA
    property p_sclk_inactive_when_cs_high;
        @(posedge clk) disable iff(!rst_n)
        spi_cs_n |-> (spi_sclk == 1'b0);
    endproperty
    assert property (p_sclk_inactive_when_cs_high) else begin
        $error("[SVA ERROR] SPI SCLK toggled or active while CS_N is high!");
    end

    property p_miso_tristate_when_cs_high;
        @(posedge clk) disable iff(!rst_n)
        spi_cs_n |-> (spi_miso === 1'bz);
    endproperty
    assert property (p_miso_tristate_when_cs_high) else begin
        $error("[SVA ERROR] MISO not tri-stated (high-Z) while CS_N is high!");
    end

    // Test Stimulus Block
    initial begin
        $display("=================================================================");
        $display("[TB] Starting Refactored MPU-6500 SPI Master Verification Suite");
        $display("=================================================================");

        // PHASE 1: Normal Operational Streaming
        $display("\n[TB] --- Phase 1: Normal Power-Up and Continuous 6 MHz Burst Streaming ---");
        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        // Wait for multiple periodic bursts (700 us yields ~4 bursts at 10 kHz)
        #700us;
        check_assert(valid_read_count >= 3 &&
                     slave.registers[8'h6A] == 8'h10 &&
                     slave.registers[8'h6B] == 8'h01,
                     "PHASE1_NORMAL", "Phase 1 normal operation and configuration registers verified", error_count);

        // PHASE 2: Fault Injection - Sensor WHO_AM_I Mismatch
        $display("\n[TB] --- Phase 2: Fault Injection - Corrupt WHO_AM_I (0xFF) ---");
        expect_error = 1'b1;
        slave.registers[8'h75] = 8'hFF;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)
        #150us;
        check_assert(error === 1'b1 && uut.state_reg === uut.STATE_ERROR,
                     "PHASE2_WHOAMI_FAULT", "spi_master detected corrupt WHO_AM_I, asserted error=1, and safely halted", error_count);
        expect_error = 1'b0;

        // PHASE 3: Mid-Burst Reset Recovery
        $display("\n[TB] --- Phase 3: Mid-Burst Reset Recovery ---");
        slave.registers[8'h75] = 8'h70;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        @(negedge spi_cs_n);
        #1000ns;
        rst_n <= 1'b0;
        #20ns;
        check_assert(spi_cs_n === 1'b1 && spi_sclk === 1'b0,
                     "PHASE3_BUS_RELEASE", "Reset immediately released SPI bus (CS_N=1, SCLK=0)", error_count);
        `SYNC_RESET_RELEASE(clk, rst_n, 5)

        valid_read_count = 0;
        #500us;
        check_assert(valid_read_count >= 1 && error === 1'b0,
                     "PHASE3_RECOVERY", "spi_master cleanly recovered from mid-burst reset and resumed valid streaming", error_count);

        // PHASE 4: Alternate Supported Chip IDs (MPU-9250 0x71 and MPU-6000 0x68)
        $display("\n[TB] --- Phase 4A: Alternate Chip ID - MPU-9250 (0x71) ---");
        slave.registers[8'h75] = 8'h71;
        valid_read_count = 0;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)
        #500us;
        check_assert(valid_read_count >= 1 && error === 1'b0,
                     "PHASE4A_MPU9250", "MPU-9250 (0x71) successfully recognized and streaming", error_count);

        $display("\n[TB] --- Phase 4B: Alternate Chip ID - MPU-6000 (0x68) ---");
        slave.registers[8'h75] = 8'h68;
        valid_read_count = 0;
        `SYNC_RESET_RELEASE(clk, rst_n, 5)
        #500us;
        check_assert(valid_read_count >= 1 && error === 1'b0,
                     "PHASE4B_MPU6000", "MPU-6000 (0x68) successfully recognized and streaming", error_count);

        finalize_test_suite("SPI MASTER", error_count);
    end

endmodule

// =============================================================================
// Behavioral MPU-6500 SPI Slave Model (SPI Mode 0)
// =============================================================================
module mpu6500_spi_slave_model (
    input  logic sclk,
    input  logic mosi,
    output logic miso,
    input  logic cs_n
);

    logic [7:0] registers [256];
    logic [7:0] rx_byte;
    logic [7:0] tx_byte;
    logic [2:0] bit_cnt;
    logic [7:0] curr_addr;
    logic       is_read;
    logic       is_first_byte;
    logic       miso_drv;

    assign miso = (!cs_n) ? miso_drv : 1'bz;

    initial begin
        for (int i = 0; i < 256; i++) registers[i] = 8'h00;
        
        registers[8'h75] = 8'h70; // WHO_AM_I
        
        registers[8'h3B] = 8'h01; registers[8'h3C] = 8'h02; // Accel X
        registers[8'h3D] = 8'h03; registers[8'h3E] = 8'h04; // Accel Y
        registers[8'h3F] = 8'h40; registers[8'h40] = 8'h00; // Accel Z (1.0g)
        registers[8'h41] = 8'h00; registers[8'h42] = 8'h00; // Temp
        registers[8'h43] = 8'h12; registers[8'h44] = 8'h34; // Gyro X
        registers[8'h45] = 8'h56; registers[8'h46] = 8'h78; // Gyro Y
        registers[8'h47] = 8'h9A; registers[8'h48] = 8'hBC; // Gyro Z

        miso_drv      = 1'b0;
        rx_byte       = 8'h00;
        tx_byte       = 8'h00;
        bit_cnt       = 3'd0;
        is_first_byte = 1'b1;
        curr_addr     = 8'h00;
        is_read       = 1'b0;
    end

    // Asynchronous reset on CS_N high
    always @(posedge cs_n) begin
        bit_cnt       <= 3'd0;
        is_first_byte <= 1'b1;
        miso_drv      <= 1'b0;
        rx_byte       <= 8'h00;
        tx_byte       <= 8'h00;
    end

    // In Mode 0: Master shifts on negedge SCLK, Slave samples on posedge SCLK
    always @(posedge sclk) begin
        if (!cs_n) begin
            rx_byte <= {rx_byte[6:0], mosi};
            bit_cnt <= bit_cnt + 3'd1;

            if (bit_cnt == 3'd7) begin
                automatic logic [7:0] full_byte;
                full_byte = {rx_byte[6:0], mosi};
                if (is_first_byte) begin
                    is_read       <= full_byte[7]; // Bit 7 of first byte: 1 = Read, 0 = Write
                    curr_addr     <= full_byte[6:0];
                    is_first_byte <= 1'b0;
                end else begin
                    if (!is_read) begin
                        registers[curr_addr] <= full_byte;
                        curr_addr            <= curr_addr + 8'd1;
                    end else begin
                        curr_addr            <= curr_addr + 8'd1;
                    end
                end
            end
        end
    end

    // Slave updates MISO on falling edge of SCLK with 25 ns physical delay
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n) begin
            miso_drv <= 1'b0;
            tx_byte  <= 8'h00;
        end else begin
            if (bit_cnt == 3'd0) begin
                if (!is_first_byte && is_read) begin
                    tx_byte  <= registers[curr_addr];
                    miso_drv <= #25 registers[curr_addr][7];
                end else begin
                    tx_byte  <= 8'h00;
                    miso_drv <= #25 1'b0;
                end
            end else begin
                miso_drv <= #25 tx_byte[7 - bit_cnt];
            end
        end
    end

endmodule
