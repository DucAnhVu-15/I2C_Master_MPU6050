`timescale 1ns/1ps

module tb_top_de10_lite_mpu6050;

    // Tang toc mo phong bang cach override parameter ben trong top
    localparam DIVIDER_SIM      = 16;
    localparam SAMPLE_DELAY_SIM = 200;
    localparam NUM_READS_SIM    = 4;

    localparam [7:0] DEV_ADDR_W       = 8'hD0;
    localparam [7:0] DEV_ADDR_R       = 8'hD1;
    localparam [7:0] REG_SMPLRT_DIV   = 8'h19;
    localparam [7:0] REG_CONFIG       = 8'h1A;
    localparam [7:0] REG_ACCEL_CONFIG = 8'h1C;
    localparam [7:0] REG_INT_ENABLE   = 8'h38;
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;
    localparam [7:0] REG_PWR_MGMT_1   = 8'h6B;

    localparam [7:0] CFG_SMPLRT_DIV   = 8'd7;
    localparam [2:0] CFG_DLPF_CFG     = 3'd3;
    localparam [1:0] CFG_ACCEL_FS_SEL = 2'd0;
    localparam [7:0] CFG_INT_ENABLE   = 8'h01;

    reg        clk_50;
    reg [1:0]  key;

    tri1 sda;
    tri1 scl;

    reg sda_slave_drive_low;
    reg [7:0] rx_byte;

    integer error_count;
    integer read_idx;

    reg [15:0] accel_x_seq [0:NUM_READS_SIM-1];
    reg [15:0] accel_y_seq [0:NUM_READS_SIM-1];
    reg [15:0] accel_z_seq [0:NUM_READS_SIM-1];

    assign sda = sda_slave_drive_low ? 1'b0 : 1'bz;

    top_de10_lite_mpu6050 dut (
        .MAX10_CLK1_50 (clk_50),
        .KEY           (key),
        .I2C_SDA       (sda),
        .I2C_SCL       (scl)
    );

    // Override de mo phong nhanh hon
    defparam dut.u_mpu6050_i2c.divider      = DIVIDER_SIM;
    defparam dut.u_mpu6050_i2c.sample_delay = SAMPLE_DELAY_SIM;

    initial begin
        clk_50 = 1'b0;
        forever #10 clk_50 = ~clk_50;
    end

    task automatic check_flag;
        input actual;
        input expected;
        input [255:0] msg;
        begin
            if (actual !== expected) begin
                error_count = error_count + 1;
                $display("[ERROR] %0s got=%b exp=%b t=%0t", msg, actual, expected, $time);
            end
        end
    endtask

    task automatic check_8;
        input [7:0] actual;
        input [7:0] expected;
        input [255:0] msg;
        begin
            if (actual !== expected) begin
                error_count = error_count + 1;
                $display("[ERROR] %0s got=%02h exp=%02h t=%0t", msg, actual, expected, $time);
            end
        end
    endtask

    task automatic wait_start;
        begin : wait_start_blk
            forever begin
                @(negedge sda);
                if (scl === 1'b1)
                    disable wait_start_blk;
            end
        end
    endtask

    task automatic wait_stop;
        begin : wait_stop_blk
            forever begin
                @(posedge sda);
                if (scl === 1'b1)
                    disable wait_stop_blk;
            end
        end
    endtask

    task automatic recv_byte;
        output [7:0] val;
        integer i;
        begin
            val = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                @(posedge scl);
                val[i] = sda;
                @(negedge scl);
            end
        end
    endtask

    task automatic send_ack;
        begin
            sda_slave_drive_low = 1'b1;
            @(posedge scl);
            @(negedge scl);
            sda_slave_drive_low = 1'b0;
        end
    endtask

    task automatic send_byte;
        input [7:0] val;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda_slave_drive_low = ~val[i];
                @(posedge scl);
                @(negedge scl);
            end
            sda_slave_drive_low = 1'b0;
        end
    endtask

    task automatic recv_master_ack;
        output is_ack;
        begin
            @(posedge scl);
            is_ack = (sda === 1'b0) ? 1'b1 : 1'b0;
            @(negedge scl);
        end
    endtask

    task automatic handle_write_reg;
        input [7:0] reg_expect;
        input [7:0] data_expect;
        input [255:0] tag;
        begin
            wait_start();

            recv_byte(rx_byte);
            check_8(rx_byte, DEV_ADDR_W, {tag, " addr+W"});
            send_ack();

            recv_byte(rx_byte);
            check_8(rx_byte, reg_expect, {tag, " reg"});
            send_ack();

            recv_byte(rx_byte);
            check_8(rx_byte, data_expect, {tag, " data"});
            send_ack();

            wait_stop();
        end
    endtask

    task automatic handle_accel_read;
        input [15:0] ax;
        input [15:0] ay;
        input [15:0] az;
        reg ack;
        begin
            wait_start();

            recv_byte(rx_byte);
            check_8(rx_byte, DEV_ADDR_W, "read phase-W addr");
            send_ack();

            recv_byte(rx_byte);
            check_8(rx_byte, REG_ACCEL_XOUT_H, "read phase-W reg");
            send_ack();

            wait_start();

            recv_byte(rx_byte);
            check_8(rx_byte, DEV_ADDR_R, "read phase-R addr");
            send_ack();

            send_byte(ax[15:8]); recv_master_ack(ack); check_flag(ack, 1'b1, "ack byte1");
            send_byte(ax[ 7:0]); recv_master_ack(ack); check_flag(ack, 1'b1, "ack byte2");
            send_byte(ay[15:8]); recv_master_ack(ack); check_flag(ack, 1'b1, "ack byte3");
            send_byte(ay[ 7:0]); recv_master_ack(ack); check_flag(ack, 1'b1, "ack byte4");
            send_byte(az[15:8]); recv_master_ack(ack); check_flag(ack, 1'b1, "ack byte5");
            send_byte(az[ 7:0]); recv_master_ack(ack); check_flag(ack, 1'b0, "nack byte6");

            wait_stop();
        end
    endtask

    initial begin
        error_count = 0;
        sda_slave_drive_low = 1'b0;
        key = 2'b00;

        accel_x_seq[0] = 16'h1234; accel_y_seq[0] = 16'hFE00; accel_z_seq[0] = 16'h0064;
        accel_x_seq[1] = 16'h1240; accel_y_seq[1] = 16'hFE10; accel_z_seq[1] = 16'h0070;
        accel_x_seq[2] = 16'h1250; accel_y_seq[2] = 16'hFE20; accel_z_seq[2] = 16'h0080;
        accel_x_seq[3] = 16'h1260; accel_y_seq[3] = 16'hFE30; accel_z_seq[3] = 16'h0090;

        repeat (20) @(posedge clk_50);
        key[0] = 1'b1;
        key[1] = 1'b1;

        // Cho slave phuc vu xong tat ca giao dich mong doi
        wait (read_idx == NUM_READS_SIM);

        repeat (50) @(posedge clk_50);

        if (error_count == 0)
            $display("TB PASS: top_de10_lite_mpu6050");
        else
            $display("TB FAIL: error_count=%0d", error_count);

        $stop;
    end

    initial begin
        #10_000_000;
        $display("[ERROR] TB timeout");
        $stop;
    end

    initial begin
        read_idx = 0;
        wait (key[0] === 1'b1);

        handle_write_reg(REG_PWR_MGMT_1,   8'h00,                        "cfg pwr_mgmt_1");
        handle_write_reg(REG_SMPLRT_DIV,   CFG_SMPLRT_DIV,               "cfg smplrt_div");
        handle_write_reg(REG_CONFIG,       {5'b00000, CFG_DLPF_CFG},     "cfg config");
        handle_write_reg(REG_ACCEL_CONFIG, {3'b000, CFG_ACCEL_FS_SEL, 3'b000}, "cfg accel_config");
        handle_write_reg(REG_INT_ENABLE,   CFG_INT_ENABLE,               "cfg int_enable");

        for (read_idx = 0; read_idx < NUM_READS_SIM; read_idx = read_idx + 1)
            handle_accel_read(accel_x_seq[read_idx], accel_y_seq[read_idx], accel_z_seq[read_idx]);
    end

endmodule
