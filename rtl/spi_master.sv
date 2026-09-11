`timescale 1ns / 1ps

module spi_master #(
    parameter int CLK_FREQ_HZ       = 12_000_000, // 12 MHz System Clock
    parameter int SAMPLE_RATE_HZ    = 1_000,      // 1 kHz Sensor Acquisition Loop
    parameter int INIT_WAIT_CYCLES  = 120_000     // 10 ms at 12 MHz (override in TB for fast simulation)
)(
    input  logic        clk,      // System Clock
    input  logic        rst_n,    // Active-Low Asynchronous Reset
    
    // Physical 4-Wire SPI Interface to MPU-6500
    output logic        spi_sclk, // SPI Serial Clock (Mode 0: idles low)
    output logic        spi_mosi, // Master Out Slave In
    input  logic        spi_miso, // Master In Slave Out
    output logic        spi_cs_n, // Active-Low Chip Select
    
    // Sensor Output Data (16-bit 2's Complement Signed Words)
    output logic [15:0] accel_x,  // Accelerometer X (+/-2g scale, 16384 LSB/g)
    output logic [15:0] accel_y,  // Accelerometer Y (+/-2g scale, 16384 LSB/g)
    output logic [15:0] accel_z,  // Accelerometer Z (+/-2g scale, 16384 LSB/g)
    output logic [15:0] gyro_x,   // Gyroscope Roll Rate (+/-250 deg/s scale, 131 LSB/deg/s)
    output logic [15:0] gyro_y,   // Gyroscope Pitch Rate (+/-250 deg/s scale, 131 LSB/deg/s)
    output logic [15:0] gyro_z,   // Gyroscope Yaw Rate (+/-250 deg/s scale, 131 LSB/deg/s)
    
    // Status & Handshake
    output logic        valid,    // 1-Cycle Pulse: outputs are updated and stable
    output logic        error     // Communication or device identification failure
);

    // -------------------------------------------------------------------------
    // Constants & Timing Parameters
    // -------------------------------------------------------------------------
    localparam int SAMPLE_INTERVAL_CYCLES = CLK_FREQ_HZ / SAMPLE_RATE_HZ; // 12,000 cycles for 1 kHz
    
    // SPI Clock Dividers (Half-Period ticks of 12 MHz clk):
    // Init: 1 MHz SCLK -> Half-period = 6 cycles
    // Read: 6 MHz SCLK -> Half-period = 1 cycle
    localparam logic [3:0] DIV_HALF_PERIOD_INIT = 4'd6;
    localparam logic [3:0] DIV_HALF_PERIOD_READ = 4'd1;

    // MPU-6500 Registers & Commands
    localparam logic [7:0] REG_CONFIG        = 8'h1A;
    localparam logic [7:0] REG_GYRO_CONFIG   = 8'h1B;
    localparam logic [7:0] REG_ACCEL_CONFIG  = 8'h1C;
    localparam logic [7:0] REG_ACCEL_XOUT_H  = 8'h3B;
    localparam logic [7:0] REG_USER_CTRL     = 8'h6A;
    localparam logic [7:0] REG_PWR_MGMT_1    = 8'h6B;
    localparam logic [7:0] REG_WHO_AM_I      = 8'h75;

    localparam logic [7:0] SPI_READ_FLAG     = 8'h80; // Bit 7 = 1 for SPI Read

    // -------------------------------------------------------------------------
    // Finite State Machine Definition (names preserved for hierarchical TB probes)
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        STATE_POWER_ON_WAIT,
        STATE_WRITE_USER_CTRL,   // Disable I2C interface (I2C_IF_DIS = 1)
        STATE_WRITE_PWR_MGMT_1,  // Clear SLEEP bit, select PLL gyro reference
        STATE_WRITE_GYRO_CFG,    // Set Gyro +/-250 deg/s scale
        STATE_WRITE_ACCEL_CFG,   // Set Accel +/-2g scale
        STATE_READ_WHOAMI,       // Verify MPU-6500 WHO_AM_I (0x70)
        STATE_INIT_SETTLE_WAIT,  // Settle PLL and digital filters
        STATE_IDLE_WAIT_SAMPLE,  // Wait for 1 kHz sample interval
        STATE_BURST_READ_STREAM, // 15-byte continuous read (1 addr + 14 data)
        STATE_LATCH_OUTPUTS,     // Strobe valid flag, update registers
        STATE_ERROR              // Error state if WHO_AM_I mismatch (probed in TB)
    } state_t;

    state_t state_reg;

    // -------------------------------------------------------------------------
    // Internal Registers
    // -------------------------------------------------------------------------
    logic [19:0] delay_cnt;
    logic [15:0] sample_timer;

    // SPI Bit-Level Engine Signals
    logic        spi_start;
    logic        spi_done;
    logic        transaction_active;
    logic [4:0]  spi_total_bytes;  // Total bytes in current transaction (1 to 15)
    logic [3:0]  spi_div_limit;    // Half-period divider
    
    // Shift buffers
    logic [7:0]  tx_buf [15];      // Bytes to send on MOSI
    logic [7:0]  rx_buf [15];      // Bytes received on MISO

    // Bit engine state
    logic [3:0]  clk_div_cnt;
    logic        sclk_reg;
    logic        mosi_reg;
    logic        cs_n_reg;
    logic [4:0]  byte_idx;
    logic [2:0]  bit_idx;
    logic        engine_busy;
    logic [7:0]  rx_shift;

    assign spi_sclk = sclk_reg;
    assign spi_mosi = mosi_reg;
    assign spi_cs_n = cs_n_reg;

    // -------------------------------------------------------------------------
    // 1. SPI Low-Level Bit/Byte Serialization Engine (SPI Mode 0)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_reg     <= 1'b0;
            mosi_reg     <= 1'b0;
            cs_n_reg     <= 1'b1;
            clk_div_cnt  <= 4'd0;
            byte_idx     <= 5'd0;
            bit_idx      <= 3'd7;
            engine_busy  <= 1'b0;
            spi_done     <= 1'b0;
            rx_shift     <= 8'd0;
            rx_buf       <= '{default: 8'h00};
        end else begin
            spi_done <= 1'b0;

            if (!engine_busy) begin
                sclk_reg    <= 1'b0;
                cs_n_reg    <= 1'b1;
                clk_div_cnt <= 4'd0;
                byte_idx    <= 5'd0;
                bit_idx     <= 3'd7;

                if (spi_start) begin
                    engine_busy <= 1'b1;
                    cs_n_reg    <= 1'b0;         // Assert Chip Select
                    mosi_reg    <= tx_buf[0][7]; // Present first MSB immediately
                    clk_div_cnt <= 4'd0;
                end
            end else begin
                // Running SCLK divider
                if (clk_div_cnt < spi_div_limit - 4'd1) begin
                    clk_div_cnt <= clk_div_cnt + 4'd1;
                end else begin
                    clk_div_cnt <= 4'd0;

                    if (!sclk_reg) begin
                        // Rising Edge of SCLK: Sample MISO
                        sclk_reg <= 1'b1;
                        rx_shift <= {rx_shift[6:0], spi_miso};
                    end else begin
                        // Falling Edge of SCLK: Shift out next MOSI bit
                        sclk_reg <= 1'b0;

                        if (bit_idx == 3'd0) begin
                            // Completed current byte
                            rx_buf[byte_idx] <= rx_shift;
                            bit_idx          <= 3'd7;

                            if (byte_idx == spi_total_bytes - 5'd1) begin
                                // Entire transaction finished
                                engine_busy <= 1'b0;
                                cs_n_reg    <= 1'b1; // Deassert Chip Select
                                spi_done    <= 1'b1;
                            end else begin
                                byte_idx <= byte_idx + 5'd1;
                                mosi_reg <= tx_buf[byte_idx + 5'd1][7];
                            end
                        end else begin
                            bit_idx  <= bit_idx - 3'd1;
                            mosi_reg <= tx_buf[byte_idx][bit_idx - 3'd1];
                        end
                    end
                end
            end
        end
    end

    // Helper task to initiate 2-byte register write/read transactions cleanly
    task automatic start_2byte_spi(input logic [7:0] reg_addr, input logic [7:0] reg_data);
        spi_div_limit      <= DIV_HALF_PERIOD_INIT;
        spi_total_bytes    <= 5'd2;
        tx_buf[0]          <= reg_addr;
        tx_buf[1]          <= reg_data;
        spi_start          <= 1'b1;
        transaction_active <= 1'b1;
    endtask

    // -------------------------------------------------------------------------
    // 2. High-Level Protocol FSM & Sample Sequencer
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg          <= STATE_POWER_ON_WAIT;
            delay_cnt          <= INIT_WAIT_CYCLES[19:0];
            sample_timer       <= 16'd0;
            spi_start          <= 1'b0;
            transaction_active <= 1'b0;
            spi_div_limit      <= DIV_HALF_PERIOD_INIT;
            spi_total_bytes    <= 5'd0;
            valid              <= 1'b0;
            error              <= 1'b0;
            
            accel_x            <= 16'd0;
            accel_y            <= 16'd0;
            accel_z            <= 16'd0;
            gyro_x             <= 16'd0;
            gyro_y             <= 16'd0;
            gyro_z             <= 16'd0;

            tx_buf             <= '{default: 8'h00};
        end else begin
            valid     <= 1'b0;
            spi_start <= 1'b0;

            case (state_reg)
                // Wait for MPU-6500 internal power-on reset & regulator settling
                STATE_POWER_ON_WAIT: begin
                    if (delay_cnt == 20'd0) begin
                        state_reg <= STATE_WRITE_USER_CTRL;
                    end else begin
                        delay_cnt <= delay_cnt - 20'd1;
                    end
                end

                // Step 1: Write USER_CTRL (0x6A) = 0x10 (Disable I2C interface)
                STATE_WRITE_USER_CTRL: begin
                    if (!transaction_active) begin
                        start_2byte_spi(REG_USER_CTRL, 8'h10); // I2C_IF_DIS = 1
                    end else if (spi_done) begin
                        transaction_active <= 1'b0;
                        state_reg          <= STATE_WRITE_PWR_MGMT_1;
                    end
                end

                // Step 2: Write PWR_MGMT_1 (0x6B) = 0x01 (Auto-select PLL gyro clock, wake from sleep)
                STATE_WRITE_PWR_MGMT_1: begin
                    if (!transaction_active) begin
                        start_2byte_spi(REG_PWR_MGMT_1, 8'h01); // CLKSEL = 1, SLEEP = 0
                    end else if (spi_done) begin
                        transaction_active <= 1'b0;
                        state_reg          <= STATE_WRITE_GYRO_CFG;
                    end
                end

                // Step 3: Write GYRO_CONFIG (0x1B) = 0x00 (+/-250 deg/s scale, 131 LSB/deg/s)
                STATE_WRITE_GYRO_CFG: begin
                    if (!transaction_active) begin
                        start_2byte_spi(REG_GYRO_CONFIG, 8'h00); // FS_SEL = 00 (+/-250 dps)
                    end else if (spi_done) begin
                        transaction_active <= 1'b0;
                        state_reg          <= STATE_WRITE_ACCEL_CFG;
                    end
                end

                // Step 4: Write ACCEL_CONFIG (0x1C) = 0x00 (+/-2g scale, 16384 LSB/g)
                STATE_WRITE_ACCEL_CFG: begin
                    if (!transaction_active) begin
                        start_2byte_spi(REG_ACCEL_CONFIG, 8'h00); // AFS_SEL = 00 (+/-2g)
                    end else if (spi_done) begin
                        transaction_active <= 1'b0;
                        state_reg          <= STATE_READ_WHOAMI;
                    end
                end

                // Step 5: Read WHO_AM_I register (0x75)
                STATE_READ_WHOAMI: begin
                    if (!transaction_active) begin
                        start_2byte_spi(REG_WHO_AM_I | SPI_READ_FLAG, 8'h00); // Read WHO_AM_I
                    end else if (spi_done) begin
                        transaction_active <= 1'b0;
                        // MPU-6500 (0x70), MPU-9250 (0x71), MPU-6000 (0x68)
                        if (rx_buf[1] == 8'h70 || rx_buf[1] == 8'h71 || rx_buf[1] == 8'h68) begin
                            state_reg <= STATE_INIT_SETTLE_WAIT;
                            delay_cnt <= 20'd1200; // 100 us PLL settle delay
                        end else begin
                            error     <= 1'b1;
                            state_reg <= STATE_ERROR;
                        end
                    end
                end

                // Wait for PLL stabilization before starting continuous sampling
                STATE_INIT_SETTLE_WAIT: begin
                    if (delay_cnt == 20'd0) begin
                        state_reg    <= STATE_IDLE_WAIT_SAMPLE;
                        sample_timer <= 16'd0;
                    end else begin
                        delay_cnt <= delay_cnt - 20'd1;
                    end
                end

                // Periodic 1 kHz Sample Interval Counter
                STATE_IDLE_WAIT_SAMPLE: begin
                    if (sample_timer >= SAMPLE_INTERVAL_CYCLES - 1) begin
                        sample_timer <= 16'd0;
                        state_reg    <= STATE_BURST_READ_STREAM;
                    end else begin
                        sample_timer <= sample_timer + 16'd1;
                    end
                end

                // High-Speed 6 MHz Burst Read: 1 Command byte + 14 Data bytes = 15 bytes
                STATE_BURST_READ_STREAM: begin
                    if (!transaction_active) begin
                        spi_div_limit      <= DIV_HALF_PERIOD_READ; // 6 MHz SCLK
                        spi_total_bytes    <= 5'd15;
                        tx_buf[0]          <= REG_ACCEL_XOUT_H | SPI_READ_FLAG; // 0xBB (Read Accel X_H)
                        for (int k = 1; k < 15; k++) tx_buf[k] <= 8'h00;    // 14 dummy bytes to clock in data
                        spi_start          <= 1'b1;
                        transaction_active <= 1'b1;
                    end else if (spi_done) begin
                        transaction_active <= 1'b0;
                        state_reg          <= STATE_LATCH_OUTPUTS;
                    end
                end

                // Latch 16-bit Sensor Data and strobe valid pulse
                STATE_LATCH_OUTPUTS: begin
                    accel_x   <= {rx_buf[1],  rx_buf[2]};
                    accel_y   <= {rx_buf[3],  rx_buf[4]};
                    accel_z   <= {rx_buf[5],  rx_buf[6]};
                    // rx_buf[7], rx_buf[8] is Temperature (unused in PID pipelines)
                    gyro_x    <= {rx_buf[9],  rx_buf[10]};
                    gyro_y    <= {rx_buf[11], rx_buf[12]};
                    gyro_z    <= {rx_buf[13], rx_buf[14]};
                    
                    valid     <= 1'b1;
                    state_reg <= STATE_IDLE_WAIT_SAMPLE;
                end

                // Fatal Identification Error: Halt permanently until external reset
                STATE_ERROR: begin
                    error <= 1'b1;
                end

                default: state_reg <= STATE_POWER_ON_WAIT;
            endcase
        end
    end

endmodule

