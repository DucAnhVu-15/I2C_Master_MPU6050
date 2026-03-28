`timescale 1ns/1ps

module tb_mpu6050_i2c;

    // Sim nhanh: giam divider va sample_delay
    localparam DIVIDER_SIM      = 16;
    localparam SAMPLE_DELAY_SIM = 200;
    localparam NUM_READS_SIM    = 6;

    reg clk_sys;
    reg rst;

    tri1 sda;
    tri1 scl;

    wire [15:0] accel_x;
    wire [15:0] accel_y;
    wire [15:0] accel_z;
    wire        data_valid;
    wire        init_done;
    wire        error;

    reg sda_slave_drive_low;
    reg [7:0] rx_byte;
    integer error_count;
    integer read_idx_chk;
    integer read_idx_slv;

    reg [15:0] accel_x_seq [0:NUM_READS_SIM-1];
    reg [15:0] accel_y_seq [0:NUM_READS_SIM-1];
    reg [15:0] accel_z_seq [0:NUM_READS_SIM-1];

    assign sda = sda_slave_drive_low ? 1'b0 : 1'bz;

    mpu6050_i2c #(
        .divider      (DIVIDER_SIM),
        .sample_delay (SAMPLE_DELAY_SIM)
    ) dut (
        .clk_sys    (clk_sys),
        .rst        (rst),
        .accel_x    (accel_x),
        .accel_y    (accel_y),
        .accel_z    (accel_z),
        .data_valid (data_valid),
        .init_done  (init_done),
        .error      (error),
        .sda        (sda),
        .scl        (scl)
    );

    // 50 MHz
    initial begin
        clk_sys = 1'b0;
        forever #10 clk_sys = ~clk_sys;
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

    task automatic check_16;
        input [15:0] actual;
        input [15:0] expected;
        input [255:0] msg;
        begin
            if (actual !== expected) begin
                error_count = error_count + 1;
                $display("[ERROR] %0s got=%04h exp=%04h t=%0t", msg, actual, expected, $time);
            end
        end
    endtask

    // Doi START: SDA xuong khi SCL dang 1
    task automatic wait_start;
        begin : wait_start_blk
            forever begin
                @(negedge sda);
                if (scl === 1'b1)
                    disable wait_start_blk;
            end
        end
    endtask

    // Doi STOP: SDA len khi SCL dang 1
    task automatic wait_stop;
        begin : wait_stop_blk
            forever begin
                @(posedge sda);
                if (scl === 1'b1)
                    disable wait_stop_blk;
            end
        end
    endtask

    // Nhan 1 byte tu master (MSB first)
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

    // Slave gui ACK
    task automatic send_ack;
        begin
            sda_slave_drive_low = 1'b1;
            @(posedge scl);
            @(negedge scl);
            sda_slave_drive_low = 1'b0;
        end
    endtask

    // Slave gui 1 byte du lieu
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

    // Doc ACK/NACK cua master sau moi byte slave gui
    task automatic recv_master_ack;
        output is_ack;
        begin
            @(posedge scl);
            is_ack = (sda === 1'b0) ? 1'b1 : 1'b0;
            @(negedge scl);
        end
    endtask

    // Giao dich wake-up: START D0 6B 00 STOP
    task automatic handle_wakeup_write;
        begin
            wait_start();

            recv_byte(rx_byte);
            check_16({8'h00, rx_byte}, 16'h00D0, "wake addr+W");
            send_ack();

            recv_byte(rx_byte);
            check_16({8'h00, rx_byte}, 16'h006B, "wake reg");
            send_ack();

            recv_byte(rx_byte);
            check_16({8'h00, rx_byte}, 16'h0000, "wake data");
            send_ack();

            wait_stop();
        end
    endtask

    // Giao dich read burst: START D0 3B RSTART D1 [6 bytes] STOP
    task automatic handle_accel_read;
        input [15:0] ax;
        input [15:0] ay;
        input [15:0] az;
        reg ack;
        begin
            wait_start();

            recv_byte(rx_byte);
            check_16({8'h00, rx_byte}, 16'h00D0, "read phase-W addr");
            send_ack();

            recv_byte(rx_byte);
            check_16({8'h00, rx_byte}, 16'h003B, "read phase-W reg");
            send_ack();

            wait_start();

            recv_byte(rx_byte);
            check_16({8'h00, rx_byte}, 16'h00D1, "read phase-R addr");
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

    // Kich ban chinh
    initial begin
        error_count = 0;
        sda_slave_drive_low = 1'b0;
        rst = 1'b0;

        // Du lieu MPU gia lap cho 6 lan doc (moi lan khac nhau)
        accel_x_seq[0] = 16'h1234; accel_y_seq[0] = 16'hFE00; accel_z_seq[0] = 16'h0064;
        accel_x_seq[1] = 16'h1240; accel_y_seq[1] = 16'hFE10; accel_z_seq[1] = 16'h0070;
        accel_x_seq[2] = 16'h1250; accel_y_seq[2] = 16'hFE20; accel_z_seq[2] = 16'h0080;
        accel_x_seq[3] = 16'h1260; accel_y_seq[3] = 16'hFE30; accel_z_seq[3] = 16'h0090;
        accel_x_seq[4] = 16'h1270; accel_y_seq[4] = 16'hFE40; accel_z_seq[4] = 16'h00A0;
        accel_x_seq[5] = 16'h1280; accel_y_seq[5] = 16'hFE50; accel_z_seq[5] = 16'h00B0;

        repeat (20) @(posedge clk_sys);
        rst = 1'b1;

        // Cho wake-up xong
        wait (init_done === 1'b1);
        check_flag(error, 1'b0, "error sau init");

        // Kiem tra NUM_READS_SIM lan data_valid lien tiep
        for (read_idx_chk = 0; read_idx_chk < NUM_READS_SIM; read_idx_chk = read_idx_chk + 1) begin
            wait (data_valid === 1'b1);
            check_16(accel_x, accel_x_seq[read_idx_chk], "accel_x");
            check_16(accel_y, accel_y_seq[read_idx_chk], "accel_y");
            check_16(accel_z, accel_z_seq[read_idx_chk], "accel_z");
            $display("[INFO] Read #%0d AX=%04h AY=%04h AZ=%04h t=%0t",
                     read_idx_chk + 1, accel_x, accel_y, accel_z, $time);
            if (read_idx_chk < NUM_READS_SIM - 1)
                @(negedge data_valid);
        end

        repeat (10) @(posedge clk_sys);

        if (error_count == 0)
            $display("TB PASS: mpu6050_i2c");
        else
            $display("TB FAIL: error_count=%0d", error_count);

        $stop;
    end

    // Gia lap slave MPU6050
    initial begin
        wait (rst === 1'b1);
        handle_wakeup_write();
        for (read_idx_slv = 0; read_idx_slv < NUM_READS_SIM; read_idx_slv = read_idx_slv + 1)
            handle_accel_read(accel_x_seq[read_idx_slv], accel_y_seq[read_idx_slv], accel_z_seq[read_idx_slv]);
    end

endmodule
