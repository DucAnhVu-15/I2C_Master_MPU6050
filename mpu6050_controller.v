module mpu6050_controller #(
    parameter divider = 256,
    parameter STARTUP_DELAY = 16'd5000,
    parameter READ_INTERVAL = 24'd500000
) (
    input clk_sys,
    input rst,
    inout sda,
    inout scl,
    output reg init_done,
    output reg init_error,
    output reg sample_valid,
    output reg read_error,
    output reg [15:0] accel_x,
    output reg [15:0] accel_y,
    output reg [15:0] accel_z,
    output reg [47:0] burst_data
);

    localparam [7:0] REG_SMPLRT_DIV  = 8'h19;
    localparam [7:0] REG_CONFIG      = 8'h1A;
    localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;
    localparam [7:0] REG_PWR_MGMT_1  = 8'h6B;

    localparam [4:0] ST_BOOT_WAIT       = 5'd0;
    localparam [4:0] ST_INIT_PWR        = 5'd1;
    localparam [4:0] ST_INIT_PWR_WAIT   = 5'd2;
    localparam [4:0] ST_INIT_SMPL       = 5'd3;
    localparam [4:0] ST_INIT_SMPL_WAIT  = 5'd4;
    localparam [4:0] ST_INIT_CFG        = 5'd5;
    localparam [4:0] ST_INIT_CFG_WAIT   = 5'd6;
    localparam [4:0] ST_INIT_ACCEL      = 5'd7;
    localparam [4:0] ST_INIT_ACCEL_WAIT = 5'd8;
    localparam [4:0] ST_READ_REQ        = 5'd9;
    localparam [4:0] ST_READ_WAIT       = 5'd10;
    localparam [4:0] ST_READ_GAP        = 5'd11;
    localparam [4:0] ST_INIT_FAIL       = 5'd12;

    reg [4:0] state = ST_BOOT_WAIT;
    reg [15:0] startup_cnt = 16'd0;
    reg [23:0] read_gap_cnt = 24'd0;
    reg [7:0] reg_addr_cmd = 8'h00;
    reg [7:0] data_wr_cmd = 8'h00;
    reg r_w_cmd = 1'b0;
    reg startfr_cmd = 1'b0;
    reg stopfr_cmd = 1'b0;

    wire [47:0] data_rd;
    wire writebytedone;
    wire readbytedone;
    wire ack_error;
    wire done;

    i2c_master #(
        .divider(divider),
        .READ_BYTES(6)
    ) u_i2c_master (
        .clk_sys(clk_sys),
        .rst(rst),
        .reg_addr(reg_addr_cmd),
        .data_wr(data_wr_cmd),
        .r_w(r_w_cmd),
        .startfr(startfr_cmd),
        .stopfr(stopfr_cmd),
        .sda(sda),
        .scl(scl),
        .data_rd(data_rd),
        .writebytedone(writebytedone),
        .readbytedone(readbytedone),
        .ack_error(ack_error),
        .done(done)
    );

    always @(posedge clk_sys or negedge rst) begin
        if (!rst) begin
            state <= ST_BOOT_WAIT;
            startup_cnt <= 16'd0;
            read_gap_cnt <= 24'd0;
            reg_addr_cmd <= 8'h00;
            data_wr_cmd <= 8'h00;
            r_w_cmd <= 1'b0;
            startfr_cmd <= 1'b0;
            stopfr_cmd <= 1'b0;
            init_done <= 1'b0;
            init_error <= 1'b0;
            sample_valid <= 1'b0;
            read_error <= 1'b0;
            accel_x <= 16'd0;
            accel_y <= 16'd0;
            accel_z <= 16'd0;
            burst_data <= 48'd0;
        end else begin
            startfr_cmd <= 1'b0;
            stopfr_cmd <= 1'b0;
            sample_valid <= 1'b0;
            read_error <= 1'b0;

            case (state)
                ST_BOOT_WAIT: begin
                    if (startup_cnt < STARTUP_DELAY - 1'b1)
                        startup_cnt <= startup_cnt + 1'b1;
                    else begin
                        startup_cnt <= 16'd0;
                        state <= ST_INIT_PWR;
                    end
                end

                ST_INIT_PWR: begin
                    reg_addr_cmd <= REG_PWR_MGMT_1;
                    data_wr_cmd <= 8'h00;
                    r_w_cmd <= 1'b0;
                    startfr_cmd <= 1'b1;
                    stopfr_cmd <= 1'b1;
                    state <= ST_INIT_PWR_WAIT;
                end

                ST_INIT_PWR_WAIT: begin
                    if (done) begin
                        if (ack_error) begin
                            init_error <= 1'b1;
                            state <= ST_INIT_FAIL;
                        end else begin
                            state <= ST_INIT_SMPL;
                        end
                    end
                end

                ST_INIT_SMPL: begin
                    reg_addr_cmd <= REG_SMPLRT_DIV;
                    data_wr_cmd <= 8'h07;
                    r_w_cmd <= 1'b0;
                    startfr_cmd <= 1'b1;
                    stopfr_cmd <= 1'b1;
                    state <= ST_INIT_SMPL_WAIT;
                end

                ST_INIT_SMPL_WAIT: begin
                    if (done) begin
                        if (ack_error) begin
                            init_error <= 1'b1;
                            state <= ST_INIT_FAIL;
                        end else begin
                            state <= ST_INIT_CFG;
                        end
                    end
                end

                ST_INIT_CFG: begin
                    reg_addr_cmd <= REG_CONFIG;
                    data_wr_cmd <= 8'h03;
                    r_w_cmd <= 1'b0;
                    startfr_cmd <= 1'b1;
                    stopfr_cmd <= 1'b1;
                    state <= ST_INIT_CFG_WAIT;
                end

                ST_INIT_CFG_WAIT: begin
                    if (done) begin
                        if (ack_error) begin
                            init_error <= 1'b1;
                            state <= ST_INIT_FAIL;
                        end else begin
                            state <= ST_INIT_ACCEL;
                        end
                    end
                end

                ST_INIT_ACCEL: begin
                    reg_addr_cmd <= REG_ACCEL_CONFIG;
                    data_wr_cmd <= 8'h00;
                    r_w_cmd <= 1'b0;
                    startfr_cmd <= 1'b1;
                    stopfr_cmd <= 1'b1;
                    state <= ST_INIT_ACCEL_WAIT;
                end

                ST_INIT_ACCEL_WAIT: begin
                    if (done) begin
                        if (ack_error) begin
                            init_error <= 1'b1;
                            state <= ST_INIT_FAIL;
                        end else begin
                            init_done <= 1'b1;
                            state <= ST_READ_REQ;
                        end
                    end
                end

                ST_READ_REQ: begin
                    reg_addr_cmd <= REG_ACCEL_XOUT_H;
                    data_wr_cmd <= 8'h00;
                    r_w_cmd <= 1'b1;
                    startfr_cmd <= 1'b1;
                    stopfr_cmd <= 1'b1;
                    state <= ST_READ_WAIT;
                end

                ST_READ_WAIT: begin
                    if (done) begin
                        if (ack_error) begin
                            read_error <= 1'b1;
                            read_gap_cnt <= 24'd0;
                            state <= ST_READ_GAP;
                        end else begin
                            burst_data <= data_rd;
                            accel_x <= data_rd[47:32];
                            accel_y <= data_rd[31:16];
                            accel_z <= data_rd[15:0];
                            sample_valid <= 1'b1;
                            read_gap_cnt <= 24'd0;
                            state <= ST_READ_GAP;
                        end
                    end
                end

                ST_READ_GAP: begin
                    if (read_gap_cnt < READ_INTERVAL - 1'b1)
                        read_gap_cnt <= read_gap_cnt + 1'b1;
                    else begin
                        read_gap_cnt <= 24'd0;
                        state <= ST_READ_REQ;
                    end
                end

                ST_INIT_FAIL: begin
                    init_error <= 1'b1;
                    state <= ST_INIT_FAIL;
                end

                default: begin
                    state <= ST_BOOT_WAIT;
                end
            endcase
        end
    end

endmodule