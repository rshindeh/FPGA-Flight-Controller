`timescale 1ns / 1ps

module i2c_master (
    input  logic        clk,      // 12 MHz System Clock
    input  logic        rst_n,    // Active-Low Synchronized Reset
    inout  wire         scl,      // I2C Serial Clock Line (Open-Drain)
    inout  wire         sda,      // I2C Serial Data Line (Open-Drain)
    output logic [15:0] accel_x,  // Accelerometer X (16-bit Signed)
    output logic [15:0] accel_y,  // Accelerometer Y (16-bit Signed)
    output logic [15:0] accel_z,  // Accelerometer Z (16-bit Signed)
    output logic [15:0] gyro_x,   // Roll Rate (16-bit Signed Gyroscope Data)
    output logic [15:0] gyro_y,   // Pitch Rate (16-bit Signed Gyroscope Data)
    output logic [15:0] gyro_z,   // Yaw Rate (16-bit Signed Gyroscope Data)
    output logic        valid,    // 1-Cycle Pulse: Outputs are stable and updated
    output logic        error     // Asserted on Slave NACK / Communication Fault
);

    // 1. High-Level Sequencer States
    typedef enum logic [5:0] {
        SEQ_IDLE,
        SEQ_INIT_WAIT,
        SEQ_INIT_WAKE_START,
        SEQ_INIT_WAKE_DEV_ADDR,
        SEQ_INIT_WAKE_REG_ADDR,
        SEQ_INIT_WAKE_DATA,
        SEQ_INIT_WAKE_STOP,
        
        SEQ_READ_START,
        SEQ_READ_DEV_ADDR_W,
        SEQ_READ_REG_ADDR,
        SEQ_READ_REP_START,
        SEQ_READ_DEV_ADDR_R,
        
        SEQ_READ_ACCEL_X_H,
        SEQ_READ_ACCEL_X_L,
        SEQ_READ_ACCEL_Y_H,
        SEQ_READ_ACCEL_Y_L,
        SEQ_READ_ACCEL_Z_H,
        SEQ_READ_ACCEL_Z_L,
        SEQ_READ_TEMP_H,
        SEQ_READ_TEMP_L,
        SEQ_READ_GYRO_X_H,
        SEQ_READ_GYRO_X_L,
        SEQ_READ_GYRO_Y_H,
        SEQ_READ_GYRO_Y_L,
        SEQ_READ_GYRO_Z_H,
        SEQ_READ_GYRO_Z_L,
        SEQ_READ_STOP,
        SEQ_READ_DELAY,
        
        SEQ_WAIT_DONE,
        SEQ_ERROR_STOP,
        SEQ_ERROR_WAIT
    } seq_state_t;

    // 2. Bit Controller Commands
    typedef enum logic [2:0] {
        SUB_CMD_NONE,
        SUB_CMD_START,
        SUB_CMD_REP_START,
        SUB_CMD_WRITE,
        SUB_CMD_READ_ACK,
        SUB_CMD_READ_NACK,
        SUB_CMD_STOP
    } sub_cmd_t;

    // 3. Bit Controller States
    typedef enum logic [2:0] {
        SUB_STATE_IDLE,
        SUB_STATE_START,
        SUB_STATE_REP_START,
        SUB_STATE_WRITE,
        SUB_STATE_READ,
        SUB_STATE_STOP,
        SUB_STATE_DONE
    } sub_state_t;

    // --- Sequencer Registers ---
    seq_state_t state_reg;
    seq_state_t seq_next_state;
    seq_state_t seq_after_wait_state;
    logic [19:0] delay_timer;
    logic [7:0]  tx_data;
    logic [7:0]  reg_ax_h, reg_ax_l;
    logic [7:0]  reg_ay_h, reg_ay_l;
    logic [7:0]  reg_az_h, reg_az_l;
    logic [7:0]  reg_temp_h, reg_temp_l;
    logic [7:0]  reg_gx_h, reg_gx_l;
    logic [7:0]  reg_gy_h, reg_gy_l;
    logic [7:0]  reg_gz_h, reg_gz_l;

    // --- Bit Controller Signals/Registers ---
    sub_cmd_t   sub_cmd;
    sub_state_t sub_state;
    logic [3:0] bit_cnt;     // Counts bits (0 to 8: 9 bits total)
    logic [4:0] clk_cnt;     // Divider count (0 to 29 for 400kHz from 12MHz)
    logic [1:0] q_cnt;       // Quarter phase (0 to 3)
    logic [7:0] rx_data;     // Shift register for data read
    logic        rx_ack;      // Sampled slave ACK status
    logic        sub_done;    // Handshake strobe when command finishes
    logic        scl_out;     // Combinational SCL level (1 = float, 0 = pull low)
    logic        sda_out;     // Combinational SDA level (1 = float, 0 = pull low)
    logic        scl_out_mux;
    logic        sda_out_mux;
    logic [7:0]  tx_data_buf;
    logic        read_nack_buf;

    // --- Bi-directional Open-Drain Output Buffer Drivers ---
    assign scl = scl_out_mux ? 1'bz : 1'b0;
    assign sda = sda_out_mux ? 1'bz : 1'b0;

    // 4. Multiplexer for Idle/Active Bus Driving
    always_comb begin
        if (sub_state != SUB_STATE_IDLE && sub_state != SUB_STATE_DONE) begin
            scl_out_mux = scl_out;
            sda_out_mux = sda_out;
        end else begin
            // When sub-FSM is idle, float SCL/SDA during idle states or hold SCL low during active sequences
            if (state_reg == SEQ_IDLE || state_reg == SEQ_INIT_WAIT || state_reg == SEQ_ERROR_WAIT || 
                state_reg == SEQ_INIT_WAKE_START || state_reg == SEQ_READ_START) begin
                scl_out_mux = 1'b1;
                sda_out_mux = 1'b1;
            end else begin
                scl_out_mux = 1'b0; // Hold SCL low to extend low period between bytes
                sda_out_mux = 1'b1; // Keep SDA released
            end
        end
    end

    // 5. Bit-Level I2C Controller - Combinational Outputs
    always_comb begin
        scl_out = 1'b1;
        sda_out = 1'b1;

        case (sub_state)
            SUB_STATE_IDLE, SUB_STATE_DONE: begin
                scl_out = 1'b1;
                sda_out = 1'b1;
            end

            SUB_STATE_START: begin
                case (q_cnt)
                    2'd0: begin scl_out = 1'b1; sda_out = 1'b1; end
                    2'd1: begin scl_out = 1'b1; sda_out = 1'b1; end
                    2'd2: begin scl_out = 1'b1; sda_out = 1'b0; end
                    2'd3: begin scl_out = 1'b0; sda_out = 1'b0; end
                endcase
            end

            SUB_STATE_REP_START: begin
                case (q_cnt)
                    2'd0: begin scl_out = 1'b0; sda_out = 1'b1; end
                    2'd1: begin scl_out = 1'b1; sda_out = 1'b1; end
                    2'd2: begin scl_out = 1'b1; sda_out = 1'b0; end
                    2'd3: begin scl_out = 1'b0; sda_out = 1'b0; end
                endcase
            end

            SUB_STATE_WRITE: begin
                if (bit_cnt < 4'd8) begin
                    logic sda_val;
                    sda_val = tx_data_buf[7 - bit_cnt];
                    case (q_cnt)
                        2'd0: begin scl_out = 1'b0; sda_out = sda_val; end
                        2'd1: begin scl_out = 1'b1; sda_out = sda_val; end
                        2'd2: begin scl_out = 1'b1; sda_out = sda_val; end
                        2'd3: begin scl_out = 1'b0; sda_out = sda_val; end
                    endcase
                end else begin
                    case (q_cnt)
                        2'd0: begin scl_out = 1'b0; sda_out = 1'b1; end
                        2'd1: begin scl_out = 1'b1; sda_out = 1'b1; end
                        2'd2: begin scl_out = 1'b1; sda_out = 1'b1; end
                        2'd3: begin scl_out = 1'b0; sda_out = 1'b1; end
                    endcase
                end
            end

            SUB_STATE_READ: begin
                if (bit_cnt < 4'd8) begin
                    case (q_cnt)
                        2'd0: begin scl_out = 1'b0; sda_out = 1'b1; end
                        2'd1: begin scl_out = 1'b1; sda_out = 1'b1; end
                        2'd2: begin scl_out = 1'b1; sda_out = 1'b1; end
                        2'd3: begin scl_out = 1'b0; sda_out = 1'b1; end
                    endcase
                end else begin
                    logic ack_val;
                    ack_val = read_nack_buf ? 1'b1 : 1'b0;
                    case (q_cnt)
                        2'd0: begin scl_out = 1'b0; sda_out = ack_val; end
                        2'd1: begin scl_out = 1'b1; sda_out = ack_val; end
                        2'd2: begin scl_out = 1'b1; sda_out = ack_val; end
                        2'd3: begin scl_out = 1'b0; sda_out = ack_val; end
                    endcase
                end
            end

            SUB_STATE_STOP: begin
                case (q_cnt)
                    2'd0: begin scl_out = 1'b0; sda_out = 1'b0; end
                    2'd1: begin scl_out = 1'b1; sda_out = 1'b0; end
                    2'd2: begin scl_out = 1'b1; sda_out = 1'b1; end
                    2'd3: begin scl_out = 1'b1; sda_out = 1'b1; end
                endcase
            end
        endcase
    end

    // 5b. Bit-Level I2C Controller FSM (Clocked Process)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_state     <= SUB_STATE_IDLE;
            bit_cnt       <= 4'd0;
            clk_cnt       <= 5'd0;
            q_cnt         <= 2'd0;
            rx_data       <= 8'd0;
            rx_ack        <= 1'b0;
            sub_done      <= 1'b0;
            tx_data_buf   <= 8'd0;
            read_nack_buf <= 1'b0;
        end else begin
            case (sub_state)
                SUB_STATE_IDLE: begin
                    sub_done <= 1'b0;
                    clk_cnt  <= 5'd0;
                    q_cnt    <= 2'd0;
                    bit_cnt  <= 4'd0;

                    if (sub_cmd == SUB_CMD_START) begin
                        sub_state <= SUB_STATE_START;
                    end else if (sub_cmd == SUB_CMD_REP_START) begin
                        sub_state <= SUB_STATE_REP_START;
                    end else if (sub_cmd == SUB_CMD_WRITE) begin
                        sub_state   <= SUB_STATE_WRITE;
                        tx_data_buf <= tx_data;
                    end else if (sub_cmd == SUB_CMD_READ_ACK || sub_cmd == SUB_CMD_READ_NACK) begin
                        sub_state     <= SUB_STATE_READ;
                        read_nack_buf <= (sub_cmd == SUB_CMD_READ_NACK);
                    end else if (sub_cmd == SUB_CMD_STOP) begin
                        sub_state <= SUB_STATE_STOP;
                    end
                end

                SUB_STATE_START, SUB_STATE_REP_START, SUB_STATE_STOP: begin
                    if (clk_cnt == 5'd29) begin
                        clk_cnt   <= 5'd0;
                        q_cnt     <= 2'd0;
                        sub_state <= SUB_STATE_DONE;
                    end else begin
                        clk_cnt <= clk_cnt + 5'd1;
                        if (clk_cnt == 5'd6)  q_cnt <= 2'd1;
                        if (clk_cnt == 5'd14) q_cnt <= 2'd2;
                        if (clk_cnt == 5'd21) q_cnt <= 2'd3;
                    end
                end

                SUB_STATE_WRITE: begin
                    if (clk_cnt == 5'd29) begin
                        clk_cnt <= 5'd0;
                        q_cnt   <= 2'd0;
                        if (bit_cnt == 4'd8) begin
                            bit_cnt   <= 4'd0;
                            sub_state <= SUB_STATE_DONE;
                        end else begin
                            bit_cnt <= bit_cnt + 4'd1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 5'd1;
                        if (clk_cnt == 5'd6)  q_cnt <= 2'd1;
                        if (clk_cnt == 5'd14) q_cnt <= 2'd2;
                        if (clk_cnt == 5'd21) q_cnt <= 2'd3;
                    end

                    if (bit_cnt == 4'd8 && q_cnt == 2'd2 && clk_cnt == 5'd18) begin
                        rx_ack <= sda;
                    end
                end

                SUB_STATE_READ: begin
                    if (clk_cnt == 5'd29) begin
                        clk_cnt <= 5'd0;
                        q_cnt   <= 2'd0;
                        if (bit_cnt == 4'd8) begin
                            bit_cnt   <= 4'd0;
                            sub_state <= SUB_STATE_DONE;
                        end else begin
                            bit_cnt <= bit_cnt + 4'd1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 5'd1;
                        if (clk_cnt == 5'd6)  q_cnt <= 2'd1;
                        if (clk_cnt == 5'd14) q_cnt <= 2'd2;
                        if (clk_cnt == 5'd21) q_cnt <= 2'd3;
                    end

                    if (bit_cnt < 4'd8 && q_cnt == 2'd2 && clk_cnt == 5'd18) begin
                        rx_data <= {rx_data[6:0], sda};
                    end
                end

                SUB_STATE_DONE: begin
                    sub_done  <= 1'b1;
                    sub_state <= SUB_STATE_IDLE;
                end
            endcase
        end
    end

    // 6. High-Level Sequencer FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg            <= SEQ_IDLE;
            seq_next_state       <= SEQ_IDLE;
            seq_after_wait_state <= SEQ_IDLE;
            delay_timer          <= 20'd0;
            sub_cmd              <= SUB_CMD_NONE;
            tx_data              <= 8'd0;
            reg_ax_h             <= 8'd0;
            reg_ax_l             <= 8'd0;
            reg_ay_h             <= 8'd0;
            reg_ay_l             <= 8'd0;
            reg_az_h             <= 8'd0;
            reg_az_l             <= 8'd0;
            reg_temp_h           <= 8'd0;
            reg_temp_l           <= 8'd0;
            reg_gx_h             <= 8'd0;
            reg_gx_l             <= 8'd0;
            reg_gy_h             <= 8'd0;
            reg_gy_l             <= 8'd0;
            reg_gz_h             <= 8'd0;
            reg_gz_l             <= 8'd0;
            accel_x              <= 16'd0;
            accel_y              <= 16'd0;
            accel_z              <= 16'd0;
            gyro_x               <= 16'd0;
            gyro_y               <= 16'd0;
            gyro_z               <= 16'd0;
            valid                <= 1'b0;
            error                <= 1'b0;
        end else begin
            valid <= 1'b0; // Default single-cycle pulse

            case (state_reg)
                SEQ_IDLE: begin
                    // Wait 10 ms at startup for MPU-6050 power rails to stabilize
                    delay_timer          <= 20'd120000; 
                    seq_after_wait_state <= SEQ_INIT_WAKE_START;
                    state_reg            <= SEQ_INIT_WAIT;
                end

                SEQ_INIT_WAIT: begin
                    if (delay_timer == 20'd0) begin
                        state_reg <= seq_after_wait_state;
                    end else begin
                        delay_timer <= delay_timer - 20'd1;
                    end
                end

                // --- Initialization Sequence (Wake up MPU-6050) ---
                SEQ_INIT_WAKE_START: begin
                    sub_cmd        <= SUB_CMD_START;
                    seq_next_state <= SEQ_INIT_WAKE_DEV_ADDR;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_INIT_WAKE_DEV_ADDR: begin
                    sub_cmd        <= SUB_CMD_WRITE;
                    tx_data        <= 8'hD0; // MPU-6050 Device Address Write
                    seq_next_state <= SEQ_INIT_WAKE_REG_ADDR;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_INIT_WAKE_REG_ADDR: begin
                    sub_cmd        <= SUB_CMD_WRITE;
                    tx_data        <= 8'h6B; // PWR_MGMT_1 register
                    seq_next_state <= SEQ_INIT_WAKE_DATA;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_INIT_WAKE_DATA: begin
                    sub_cmd        <= SUB_CMD_WRITE;
                    tx_data        <= 8'h00; // Set sleep bit to 0 (wake up)
                    seq_next_state <= SEQ_INIT_WAKE_STOP;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_INIT_WAKE_STOP: begin
                    sub_cmd              <= SUB_CMD_STOP;
                    delay_timer          <= 20'd120000; // Wait 10 ms for PLL stabilization
                    seq_after_wait_state <= SEQ_READ_START;
                    seq_next_state       <= SEQ_INIT_WAIT;
                    state_reg            <= SEQ_WAIT_DONE;
                end

                // --- Periodic Sensor Burst Read Loop (14 Bytes starting at 0x3B) ---
                SEQ_READ_START: begin
                    sub_cmd        <= SUB_CMD_START;
                    seq_next_state <= SEQ_READ_DEV_ADDR_W;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_DEV_ADDR_W: begin
                    sub_cmd        <= SUB_CMD_WRITE;
                    tx_data        <= 8'hD0; // MPU-6050 Device Address Write
                    seq_next_state <= SEQ_READ_REG_ADDR;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_REG_ADDR: begin
                    sub_cmd        <= SUB_CMD_WRITE;
                    tx_data        <= 8'h3B; // ACCEL_XOUT_H register (burst read starting address)
                    seq_next_state <= SEQ_READ_REP_START;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_REP_START: begin
                    sub_cmd        <= SUB_CMD_REP_START; // Repeated START condition
                    seq_next_state <= SEQ_READ_DEV_ADDR_R;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_DEV_ADDR_R: begin
                    sub_cmd        <= SUB_CMD_WRITE;
                    tx_data        <= 8'hD1; // MPU-6050 Device Address Read
                    seq_next_state <= SEQ_READ_ACCEL_X_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_ACCEL_X_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_ACCEL_X_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_ACCEL_X_L: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_ACCEL_Y_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_ACCEL_Y_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_ACCEL_Y_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_ACCEL_Y_L: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_ACCEL_Z_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_ACCEL_Z_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_ACCEL_Z_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_ACCEL_Z_L: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_TEMP_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_TEMP_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_TEMP_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_TEMP_L: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_GYRO_X_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_GYRO_X_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_GYRO_X_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_GYRO_X_L: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_GYRO_Y_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_GYRO_Y_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_GYRO_Y_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_GYRO_Y_L: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_GYRO_Z_H;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_GYRO_Z_H: begin
                    sub_cmd        <= SUB_CMD_READ_ACK;
                    seq_next_state <= SEQ_READ_GYRO_Z_L;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_GYRO_Z_L: begin
                    sub_cmd        <= SUB_CMD_READ_NACK; // NACK last byte (14th byte)
                    seq_next_state <= SEQ_READ_STOP;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_STOP: begin
                    sub_cmd        <= SUB_CMD_STOP;
                    delay_timer    <= 20'd12000; // 1 ms interval between reads (1 kHz rate)
                    seq_next_state <= SEQ_READ_DELAY;
                    state_reg      <= SEQ_WAIT_DONE;
                end

                SEQ_READ_DELAY: begin
                    if (delay_timer == 20'd0) begin
                        state_reg <= SEQ_READ_START;
                    end else begin
                        delay_timer <= delay_timer - 20'd1;
                    end
                end

                SEQ_WAIT_DONE: begin
                    sub_cmd <= SUB_CMD_NONE; // Handshake clear cmd to avoid trigger loop
                    if (sub_done) begin
                        // Latch intermediate registers when the corresponding byte finishes reading
                        if (seq_next_state == SEQ_READ_ACCEL_X_L) reg_ax_h   <= rx_data;
                        else if (seq_next_state == SEQ_READ_ACCEL_Y_H) reg_ax_l <= rx_data;
                        else if (seq_next_state == SEQ_READ_ACCEL_Y_L) reg_ay_h <= rx_data;
                        else if (seq_next_state == SEQ_READ_ACCEL_Z_H) reg_ay_l <= rx_data;
                        else if (seq_next_state == SEQ_READ_ACCEL_Z_L) reg_az_h <= rx_data;
                        else if (seq_next_state == SEQ_READ_TEMP_H)    reg_az_l <= rx_data;
                        else if (seq_next_state == SEQ_READ_TEMP_L)    reg_temp_h <= rx_data;
                        else if (seq_next_state == SEQ_READ_GYRO_X_H)  reg_temp_l <= rx_data;
                        else if (seq_next_state == SEQ_READ_GYRO_X_L)  reg_gx_h <= rx_data;
                        else if (seq_next_state == SEQ_READ_GYRO_Y_H)  reg_gx_l <= rx_data;
                        else if (seq_next_state == SEQ_READ_GYRO_Y_L)  reg_gy_h <= rx_data;
                        else if (seq_next_state == SEQ_READ_GYRO_Z_H)  reg_gy_l <= rx_data;
                        else if (seq_next_state == SEQ_READ_GYRO_Z_L)  reg_gz_h <= rx_data;
                        else if (seq_next_state == SEQ_READ_STOP)      reg_gz_l <= rx_data;
                        
                        // Latch all outputs when read sequence completes STOP phase
                        if (seq_next_state == SEQ_READ_DELAY) begin
                            accel_x <= {reg_ax_h, reg_ax_l};
                            accel_y <= {reg_ay_h, reg_ay_l};
                            accel_z <= {reg_az_h, reg_az_l};
                            gyro_x  <= {reg_gx_h, reg_gx_l};
                            gyro_y  <= {reg_gy_h, reg_gy_l};
                            gyro_z  <= {reg_gz_h, reg_gz_l};
                            valid   <= 1'b1;
                            error   <= 1'b0;
                        end

                        // Check for Slave ACK/NACK errors on write operations
                        if ((seq_next_state == SEQ_INIT_WAKE_REG_ADDR ||
                             seq_next_state == SEQ_INIT_WAKE_DATA     ||
                             seq_next_state == SEQ_INIT_WAKE_STOP     ||
                             seq_next_state == SEQ_READ_REG_ADDR      ||
                             seq_next_state == SEQ_READ_REP_START     ||
                             seq_next_state == SEQ_READ_ACCEL_X_H)    && rx_ack) begin
                            state_reg <= SEQ_ERROR_STOP;
                        end else begin
                            state_reg <= seq_next_state;
                        end
                    end
                end

                // --- Error Recovery State ---
                SEQ_ERROR_STOP: begin
                    sub_cmd        <= SUB_CMD_STOP;
                    seq_next_state <= SEQ_ERROR_WAIT;
                    state_reg      <= SEQ_WAIT_DONE;
                    error          <= 1'b1;
                end

                SEQ_ERROR_WAIT: begin
                    delay_timer          <= 20'd12000; // Retry reading in 1 ms
                    seq_after_wait_state <= SEQ_READ_START;
                    state_reg            <= SEQ_INIT_WAIT;
                end
            endcase
        end
    end

endmodule
