`timescale 1ns/1ps

module tb_i2c_master;

    reg clk_sys;
    reg rst;
    reg [7:0] reg_addr;
    reg [7:0] data_wr;
    reg r_w;
    reg startfr;
    reg stopfr;

    tri1 sda;
    tri1 scl;

    wire [47:0] data_rd;
    wire writebytedone;
    wire readbytedone;
    wire ack_error;
    wire done;

    reg sda_slave_drive_low;
    reg [7:0] rx_byte = 0 ;
    integer error_count;
    integer write_done_count;
    integer read_done_count;

    localparam [15:0] ACCEL_X = 16'h1234;
    localparam [15:0] ACCEL_Y = 16'h5678;
    localparam [15:0] ACCEL_Z = 16'h9ABC;
    localparam [47:0] EXPECTED_BURST = {ACCEL_X, ACCEL_Y, ACCEL_Z};

    assign sda = sda_slave_drive_low ? 1'b0 : 1'bz;

    i2c_master #(
        .divider(16),
        .READ_BYTES(6)
    ) uut (
        .clk_sys(clk_sys),
        .rst(rst),
        .reg_addr(reg_addr),
        .data_wr(data_wr),
        .r_w(r_w),
        .startfr(startfr),
        .stopfr(stopfr),
        .sda(sda),
        .scl(scl),
        .data_rd(data_rd),
        .writebytedone(writebytedone),
        .readbytedone(readbytedone),
        .ack_error(ack_error),
        .done(done)
    );

    initial begin
        clk_sys = 1'b0;
        forever #10 clk_sys = ~clk_sys;
    end

    task automatic check_equal_8;
        input [7:0] actual;
        input [7:0] expected;
        input [255:0] message;
        begin
            if (actual !== expected) begin
                error_count = error_count + 1;
                $display("ERROR: %0s actual=%h expected=%h time=%0t", message, actual, expected, $time);
            end
        end
    endtask

    task automatic check_equal_48;
        input [47:0] actual;
        input [47:0] expected;
        input [255:0] message;
        begin
            if (actual !== expected) begin
                error_count = error_count + 1;
                $display("ERROR: %0s actual=%h expected=%h time=%0t", message, actual, expected, $time);
            end
        end
    endtask

    task automatic check_flag;
        input actual;
        input expected;
        input [255:0] message;
        begin
            if (actual !== expected) begin
                error_count = error_count + 1;
                $display("ERROR: %0s actual=%b expected=%b time=%0t", message, actual, expected, $time);
            end
        end
    endtask

    task automatic pulse_start;
        begin
            @(negedge clk_sys);
            startfr = 1'b1;
            @(negedge clk_sys);
            startfr = 1'b0;
        end
    endtask

    task automatic pulse_stop;
        begin
            @(negedge clk_sys);
            stopfr = 1'b1;
            @(negedge clk_sys);
            stopfr = 1'b0;
        end
    endtask

    task automatic wait_done_and_check_ok;
        input expect_write_done;
        input expect_read_done;
        input [255:0] message;
        begin
            wait(done);
            check_flag(ack_error, 1'b0, {message, " ack_error"});
            check_flag(writebytedone, expect_write_done, {message, " writebytedone"});
            check_flag(readbytedone, expect_read_done, {message, " readbytedone"});
            @(posedge clk_sys);
        end
    endtask

    task automatic wait_i2c_start;
        begin : wait_start_block
            forever begin
                @(negedge sda);
                if (scl === 1'b1)
                    disable wait_start_block;
            end
        end
    endtask

    task automatic wait_i2c_stop;
        begin : wait_stop_block
            forever begin
                @(posedge sda);
                if (scl === 1'b1)
                    disable wait_stop_block;
            end
        end
    endtask

    task automatic slave_recv_byte;
        output [7:0] value;
        integer i;
        begin
            value = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                @(posedge scl);
                value[i] = sda;
                @(negedge scl);
            end
        end
    endtask

    task automatic slave_send_ack;
        begin
            sda_slave_drive_low = 1'b1;
            @(posedge scl);
            @(negedge scl);
            sda_slave_drive_low = 1'b0;
        end
    endtask

    task automatic slave_send_byte;
        input [7:0] value;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda_slave_drive_low = ~value[i];
                @(posedge scl);
                @(negedge scl);
            end
            sda_slave_drive_low = 1'b0;
        end
    endtask

    task automatic expect_master_ack;
        input expect_ack;
        begin
            @(posedge scl);
            if (expect_ack) begin
                if (sda !== 1'b0) begin
                    error_count = error_count + 1;
                    $display("ERROR: master khong ACK byte doc du lieu tai time=%0t", $time);
                end
            end else begin
                if (sda !== 1'b1) begin
                    error_count = error_count + 1;
                    $display("ERROR: master khong NACK byte cuoi tai time=%0t", $time);
                end
            end
            @(negedge scl);
        end
    endtask

    task automatic master_write_reg;
        input [7:0] reg_address;
        input [7:0] reg_value;
        input [255:0] message;
        begin
            reg_addr = reg_address;
            data_wr = reg_value;
            r_w = 1'b0;
            @(negedge clk_sys);
            startfr = 1'b1;
            stopfr  = 1'b1;
            @(negedge clk_sys);
            startfr = 1'b0;
            stopfr  = 1'b0;
            wait_done_and_check_ok(1'b1, 1'b0, message);
        end
    endtask

    task automatic master_read_accel;
        input [7:0] reg_address;
        begin
            reg_addr = reg_address;
            data_wr = 8'h00;
            r_w = 1'b1;
            @(negedge clk_sys);
            startfr = 1'b1;
            stopfr  = 1'b1;
            @(negedge clk_sys);
            startfr = 1'b0;
            stopfr  = 1'b0;
            wait_done_and_check_ok(1'b0, 1'b1, "read accel burst");
            check_equal_48(data_rd, EXPECTED_BURST, "read burst data");
        end
    endtask

    task automatic expect_write_transaction;
        input [7:0] reg_address;
        input [7:0] reg_value;
        input [255:0] message;
        begin
            $display("CHECK WRITE: %0s", message);

            wait_i2c_start();

            slave_recv_byte(rx_byte);
            check_equal_8(rx_byte, 8'hD0, {message, " device address"});
            slave_send_ack();

            slave_recv_byte(rx_byte);
            check_equal_8(rx_byte, reg_address, {message, " register address"});
            slave_send_ack();

            slave_recv_byte(rx_byte);
            check_equal_8(rx_byte, reg_value, {message, " register data"});
            slave_send_ack();

            wait_i2c_stop();
        end
    endtask

    task automatic expect_read_transaction;
        begin
            $display("CHECK READ: accel burst");

            wait_i2c_start();

            slave_recv_byte(rx_byte);
            check_equal_8(rx_byte, 8'hD0, "read write-phase device address");
            slave_send_ack();

            slave_recv_byte(rx_byte);
            check_equal_8(rx_byte, 8'h3B, "read register address");
            slave_send_ack();

            wait_i2c_start();

            slave_recv_byte(rx_byte);
            check_equal_8(rx_byte, 8'hD1, "read device address");
            slave_send_ack();

            slave_send_byte(ACCEL_X[15:8]);
            expect_master_ack(1'b1);
            slave_send_byte(ACCEL_X[7:0]);
            expect_master_ack(1'b1);
            slave_send_byte(ACCEL_Y[15:8]);
            expect_master_ack(1'b1);
            slave_send_byte(ACCEL_Y[7:0]);
            expect_master_ack(1'b1);
            slave_send_byte(ACCEL_Z[15:8]);
            expect_master_ack(1'b1);
            slave_send_byte(ACCEL_Z[7:0]);
            expect_master_ack(1'b0);

            wait_i2c_stop();
        end
    endtask

    reg writebytedone_prev;
    reg readbytedone_prev;

    always @(posedge clk_sys) begin
        if (!rst) begin
            write_done_count    <= 0;
            read_done_count     <= 0;
            writebytedone_prev  <= 1'b0;
            readbytedone_prev   <= 1'b0;
        end else begin
            writebytedone_prev <= writebytedone;
            readbytedone_prev  <= readbytedone;
            if (writebytedone && !writebytedone_prev)
                write_done_count <= write_done_count + 1;
            if (readbytedone && !readbytedone_prev)
                read_done_count <= read_done_count + 1;
        end
    end

    initial begin
        rst = 1'b0;
        reg_addr = 8'h00;
        data_wr = 8'h00;
        r_w = 1'b0;
        startfr = 1'b0;
        stopfr = 1'b0;
        sda_slave_drive_low = 1'b0;
        error_count = 0;

        #100;
        rst = 1'b1;
        #100;

        $display("TEST: init MPU6050 without gyro config, then read accel data");

        master_write_reg(8'h6B, 8'h00, "init PWR_MGMT_1");
        master_write_reg(8'h19, 8'h07, "init SMPLRT_DIV");
        master_write_reg(8'h1A, 8'h03, "init CONFIG");
        master_write_reg(8'h1C, 8'h00, "init ACCEL_CONFIG");
        master_read_accel(8'h3B);

        #200;

        if (write_done_count !== 4) begin
            error_count = error_count + 1;
            $display("ERROR: so lan write transaction=%0d, expected=4", write_done_count);
        end

        if (read_done_count !== 1) begin
            error_count = error_count + 1;
            $display("ERROR: so lan read transaction=%0d, expected=1", read_done_count);
        end

        if (error_count == 0)
            $display("TB PASS: init bo qua gyro va doc accel dung");
        else
            $display("TB FAIL: error_count=%0d", error_count);

        $stop;
    end

    initial begin
        wait(rst == 1'b1);

        expect_write_transaction(8'h6B, 8'h00, "init PWR_MGMT_1");
        expect_write_transaction(8'h19, 8'h07, "init SMPLRT_DIV");
        expect_write_transaction(8'h1A, 8'h03, "init CONFIG");
        expect_write_transaction(8'h1C, 8'h00, "init ACCEL_CONFIG");
        $display("EXPECTED_BURST: X=%0h, Y=%0h, Z=%0h", ACCEL_X, ACCEL_Y, ACCEL_Z);
        expect_read_transaction();
    end

endmodule
