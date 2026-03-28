module mpu6050_i2c(
    input  wire        clk_sys,
    input  wire        rst,
    output wire [15:0] accel_x,
    output wire [15:0] accel_y,
    output wire [15:0] accel_z,
    output wire        data_valid,
    output wire        init_done,
    output wire        error,
    inout  wire        sda,
    inout  wire        scl
);
    parameter divider      = 125;       // 50MHz/400kHz
    parameter sample_delay = 500000;    // so chu ky cho giua 2 lan doc

    parameter REG_PWR_MGMT_1   = 8'h6B;
    parameter REG_ACCEL_XOUT_H = 8'h3B;

    parameter IDLE        = 3'd0;
    parameter WAKEUP      = 3'd1;
    parameter WAKEUP_WAIT = 3'd2;
    parameter WAIT_SAMPLE = 3'd3;
    parameter READ_REQ    = 3'd4;
    parameter READ_WAIT   = 3'd5;
    parameter ERROR_ST    = 3'd6;

    reg [2:0]  state;
    reg [31:0] cnt_delay;

    reg        r_w;
    reg [7:0]  reg_addr;
    reg [7:0]  data_wr;
    reg        startfr;
    reg        stopfr;

    wire [47:0] data_rd;
    wire        writebytedone;
    wire        readbytedone;
    wire        done;
    wire        ack_error;

    reg [15:0] accel_x_r;
    reg [15:0] accel_y_r;
    reg [15:0] accel_z_r;
    reg        data_valid_r;
    reg        init_done_r;
    reg        error_r;

    i2c_master #(
        .divider      (divider),
        .MPU6050_ADDR (7'h68),
        .READ_BYTES   (6)
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
    assign data_valid = data_valid_r;
    assign init_done  = init_done_r;
    assign error      = error_r;

    always @(posedge clk_sys or negedge rst) begin
        if (!rst) begin
            state        <= IDLE;
            cnt_delay    <= 32'd0;
            r_w          <= 1'b0;
            reg_addr     <= 8'h00;
            data_wr      <= 8'h00;
            startfr      <= 1'b0;
            stopfr       <= 1'b0;
            accel_x_r    <= 16'd0;
            accel_y_r    <= 16'd0;
            accel_z_r    <= 16'd0;
            data_valid_r <= 1'b0;
            init_done_r  <= 1'b0;
            error_r      <= 1'b0;
        end else begin
            startfr      <= 1'b0;
            stopfr       <= 1'b0;
            data_valid_r <= 1'b0;

            case (state)
                IDLE: begin
                    r_w      <= 1'b0;
                    reg_addr <= REG_PWR_MGMT_1;
                    data_wr  <= 8'h00;
                    state    <= WAKEUP;
                end

                WAKEUP: begin
                    r_w     <= 1'b0;
                    stopfr  <= 1'b1;
                    startfr <= 1'b1;
                    state   <= WAKEUP_WAIT;
                end

                WAKEUP_WAIT: begin
                    if (ack_error) begin
                        error_r <= 1'b1;
                        state   <= ERROR_ST;
                    end else if (done) begin
                        init_done_r <= 1'b1;
                        cnt_delay   <= 32'd0;
                        state       <= WAIT_SAMPLE;
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
                        accel_x_r    <= data_rd[47:32];
                        accel_y_r    <= data_rd[31:16];
                        accel_z_r    <= data_rd[15:0];
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
