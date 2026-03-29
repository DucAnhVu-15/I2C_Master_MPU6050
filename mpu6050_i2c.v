module mpu6050_i2c(
    input  wire        clk_sys,
    input  wire        rst,
    output wire [15:0] accel_x,
    output wire [15:0] accel_y,
    output wire [15:0] accel_z,
    output wire [15:0] gyro_x,
    output wire [15:0] gyro_y,
    output wire [15:0] gyro_z,
    output wire        data_valid,
    output wire        init_done,
    output wire        error,
    inout  wire        sda,
    inout  wire        scl
);
    parameter divider      = 125;       // 50MHz/400kHz
    parameter sample_delay = 500000;    // so chu ky cho giua 2 lan doc

    // Tham so cau hinh MPU6050
    parameter [7:0] CFG_SMPLRT_DIV   = 8'd7;   // Fs = 1kHz/(1+CFG_SMPLRT_DIV) khi DLPF bat
    parameter [2:0] CFG_DLPF_CFG     = 3'd3;   // DLPF ~44Hz cho accel, ~42Hz cho gyro
    parameter [1:0] CFG_ACCEL_FS_SEL = 2'd0;   // 0:+/-2g, 1:+/-4g, 2:+/-8g, 3:+/-16g
    parameter [1:0] CFG_GYRO_FS_SEL  = 2'd0;   // 0:+/-250dps, 1:+/-500dps, 2:+/-1000dps, 3:+/-2000dps
    parameter [7:0] CFG_INT_ENABLE   = 8'h01;  // bit0=DATA_RDY_EN

    parameter REG_SMPLRT_DIV   = 8'h19;
    parameter REG_CONFIG       = 8'h1A;
    parameter REG_GYRO_CONFIG  = 8'h1B;
    parameter REG_ACCEL_CONFIG = 8'h1C;
    parameter REG_ACCEL_XOUT_H = 8'h3B;
    parameter REG_INT_ENABLE   = 8'h38;
    parameter REG_PWR_MGMT_1   = 8'h6B;

    parameter IDLE        = 3'd0;
    parameter CFG_LOAD    = 3'd1;
    parameter CFG_REQ     = 3'd2;
    parameter CFG_WAIT    = 3'd3;
    parameter WAIT_SAMPLE = 3'd4;
    parameter READ_REQ    = 3'd5;
    parameter READ_WAIT   = 3'd6;
    parameter ERROR_ST    = 3'd7;

    parameter [2:0] CFG_STEP_PWR_MGMT_1   = 3'd0;
    parameter [2:0] CFG_STEP_SMPLRT_DIV   = 3'd1;
    parameter [2:0] CFG_STEP_CONFIG       = 3'd2;
    parameter [2:0] CFG_STEP_GYRO_CONFIG  = 3'd3;
    parameter [2:0] CFG_STEP_ACCEL_CONFIG = 3'd4;
    parameter [2:0] CFG_STEP_INT_ENABLE   = 3'd5;

    reg [2:0]  state;
    reg [2:0]  cfg_step;
    reg [31:0] cnt_delay;

    reg        r_w;
    reg [7:0]  reg_addr;
    reg [7:0]  data_wr;
    reg        startfr;
    reg        stopfr;

    // Doc 14 bytes: Accel(6) + Temp(2) + Gyro(6)
    wire [111:0] data_rd;
    wire        writebytedone;
    wire        readbytedone;
    wire        done;
    wire        ack_error;

    reg [15:0] accel_x_r;
    reg [15:0] accel_y_r;
    reg [15:0] accel_z_r;
    reg [15:0] gyro_x_r;
    reg [15:0] gyro_y_r;
    reg [15:0] gyro_z_r;
    reg        data_valid_r;
    reg        init_done_r;
    reg        error_r;

    i2c_master #(
        .divider      (divider),
        .MPU6050_ADDR (7'h68),
        .READ_BYTES   (14)       // Accel(6) + Temp(2) + Gyro(6)
    ) i2c_inst (
        .clk_sys      (clk_sys),
        .rst          (rst),
        .reg_addr     (reg_addr),
        .data_wr      (data_wr),
        .r_w          (r_w),
        .startfr      (startfr),
        .stopfr       (stopfr),
        .sda          (sda),
        .scl          (scl),
        .data_rd      (data_rd),
        .writebytedone(writebytedone),
        .readbytedone (readbytedone),
        .ack_error    (ack_error),
        .done         (done)
    );

    assign accel_x    = accel_x_r;
    assign accel_y    = accel_y_r;
    assign accel_z    = accel_z_r;
    assign gyro_x     = gyro_x_r;
    assign gyro_y     = gyro_y_r;
    assign gyro_z     = gyro_z_r;
    assign data_valid = data_valid_r;
    assign init_done  = init_done_r;
    assign error      = error_r;

    always @(posedge clk_sys or negedge rst) begin
        if (!rst) begin
            state        <= IDLE;
            cfg_step     <= CFG_STEP_PWR_MGMT_1;
            cnt_delay    <= 32'd0;
            r_w          <= 1'b0;
            reg_addr     <= 8'h00;
            data_wr      <= 8'h00;
            startfr      <= 1'b0;
            stopfr       <= 1'b0;
            accel_x_r    <= 16'd0;
            accel_y_r    <= 16'd0;
            accel_z_r    <= 16'd0;
            gyro_x_r     <= 16'd0;
            gyro_y_r     <= 16'd0;
            gyro_z_r     <= 16'd0;
            data_valid_r <= 1'b0;
            init_done_r  <= 1'b0;
            error_r      <= 1'b0;
        end else begin
            startfr      <= 1'b0;
            stopfr       <= 1'b0;
            data_valid_r <= 1'b0;

            case (state)
                IDLE: begin
                    cfg_step <= CFG_STEP_PWR_MGMT_1;
                    state    <= CFG_LOAD;
                end

                CFG_LOAD: begin
                    r_w <= 1'b0;

                    case (cfg_step)
                        CFG_STEP_PWR_MGMT_1: begin
                            reg_addr <= REG_PWR_MGMT_1;
                            data_wr  <= 8'h00;
                        end
                        CFG_STEP_SMPLRT_DIV: begin
                            reg_addr <= REG_SMPLRT_DIV;
                            data_wr  <= CFG_SMPLRT_DIV;
                        end
                        CFG_STEP_CONFIG: begin
                            reg_addr <= REG_CONFIG;
                            data_wr  <= {5'b00000, CFG_DLPF_CFG};
                        end
                        CFG_STEP_GYRO_CONFIG: begin
                            reg_addr <= REG_GYRO_CONFIG;
                            data_wr  <= {3'b000, CFG_GYRO_FS_SEL, 3'b000};
                        end
                        CFG_STEP_ACCEL_CONFIG: begin
                            reg_addr <= REG_ACCEL_CONFIG;
                            data_wr  <= {3'b000, CFG_ACCEL_FS_SEL, 3'b000};
                        end
                        default: begin
                            reg_addr <= REG_INT_ENABLE;
                            data_wr  <= CFG_INT_ENABLE;
                        end
                    endcase

                    state <= CFG_REQ;
                end

                CFG_REQ: begin
                    r_w     <= 1'b0;
                    stopfr  <= 1'b1;
                    startfr <= 1'b1;
                    state   <= CFG_WAIT;
                end

                CFG_WAIT: begin
                    if (ack_error) begin
                        error_r <= 1'b1;
                        state   <= ERROR_ST;
                    end else if (done) begin
                        if (cfg_step == CFG_STEP_INT_ENABLE) begin
                            init_done_r <= 1'b1;
                            cnt_delay   <= 32'd0;
                            state       <= WAIT_SAMPLE;
                        end else begin
                            cfg_step <= cfg_step + 1'b1;
                            state    <= CFG_LOAD;
                        end
                    end
                end

                WAIT_SAMPLE: begin
                    if (cnt_delay >= sample_delay - 1) begin
                        cnt_delay <= 32'd0;
                        state     <= READ_REQ;
                    end else begin
                        cnt_delay <= cnt_delay + 1'b1;
                    end
                end

                READ_REQ: begin
                    reg_addr <= REG_ACCEL_XOUT_H;
                    r_w      <= 1'b1;
                    stopfr   <= 1'b1;
                    startfr  <= 1'b1;
                    state    <= READ_WAIT;
                end

                READ_WAIT: begin
                    if (ack_error) begin
                        error_r <= 1'b1;
                        state   <= ERROR_ST;
                    end else if (done) begin
                        // data_rd[111:0]: 14 bytes, MSB first
                        // Byte 0-5: Accel X,Y,Z (bits 111:64)
                        // Byte 6-7: Temperature (bits 63:48) - bo qua
                        // Byte 8-13: Gyro X,Y,Z (bits 47:0)
                        accel_x_r    <= data_rd[111:96];
                        accel_y_r    <= data_rd[95:80];
                        accel_z_r    <= data_rd[79:64];
                        gyro_x_r     <= data_rd[47:32];
                        gyro_y_r     <= data_rd[31:16];
                        gyro_z_r     <= data_rd[15:0];
                        data_valid_r <= 1'b1;
                        state        <= WAIT_SAMPLE;
                    end
                end

                ERROR_ST: begin
                    error_r <= 1'b1;
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
