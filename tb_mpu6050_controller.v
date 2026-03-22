`timescale 1ns/1ps

module tb_mpu6050_controller;

    reg clk_sys;
    reg rst;

    tri1 sda;
    tri1 scl;

    wire init_done;
    wire sample_valid;
    wire [15:0] accel_x;
    wire [15:0] accel_y;
    wire [15:0] accel_z;
    wire [47:0] burst_data;

    reg sda_slave_drive_low;
    reg [7:0] rx_byte;

    localparam [15:0] EXP_ACCEL_X = 16'h1234;
    localparam [15:0] EXP_ACCEL_Y = 16'h5678;
    localparam [15:0] EXP_ACCEL_Z = 16'h9ABC;
    localparam [47:0] EXP_BURST  = {
        EXP_ACCEL_X,
        EXP_ACCEL_Y,
        EXP_ACCEL_Z
    };

    assign sda = sda_slave_drive_low ? 1'b0 : 1'bz;

    mpu6050_controller #(
        .divider(16),
        .STARTUP_DELAY(8),
        .READ_INTERVAL(32)
    ) uut (
        .clk_sys(clk_sys),
        .rst(rst),
        .sda(sda),
        .scl(scl),
        .init_done(init_done),
        .sample_valid(sample_valid),
        .accel_x(accel_x),
        .accel_y(accel_y),
        .accel_z(accel_z),
        .burst_data(burst_data)
    );

    initial begin
        clk_sys = 1'b0;
        forever #10 clk_sys = ~clk_sys;
    end

    task automatic wait_i2c_start;
        begin : wait_start_block
            forever begin
                @(negedge sda);
                if (scl === 1'b1)
                    disable wait_start_block;
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
                if (sda !== 1'b0)
                    $display("ERROR: master khong ACK byte doc du lieu tai time=%0t", $time);
            end else begin
                if (sda !== 1'b1)
                    $display("ERROR: master khong NACK byte cuoi tai time=%0t", $time);
            end
            @(negedge scl);
        end
    endtask

    task automatic expect_write_transaction;
        input [7:0] reg_address;
        input [7:0] reg_value;
        begin
            wait_i2c_start();

            slave_recv_byte(rx_byte);
            if (rx_byte !== 8'hD0)
                $display("ERROR: dia chi ghi MPU6050 sai, nhan=%h", rx_byte);
            slave_send_ack();

            slave_recv_byte(rx_byte);
            if (rx_byte !== reg_address)
                $display("ERROR: thanh ghi ghi sai, expected=%h nhan=%h", reg_address, rx_byte);
            slave_send_ack();

            slave_recv_byte(rx_byte);
            if (rx_byte !== reg_value)
                $display("ERROR: du lieu ghi sai, expected=%h nhan=%h", reg_value, rx_byte);
            slave_send_ack();
        end
    endtask

    task automatic expect_read_burst;
        begin
            wait_i2c_start();

            slave_recv_byte(rx_byte);
            if (rx_byte !== 8'hD0)
                $display("ERROR: dia chi ghi MPU6050 sai truoc burst doc, nhan=%h", rx_byte);
            slave_send_ack();

            slave_recv_byte(rx_byte);
            if (rx_byte !== 8'h3B)
                $display("ERROR: thanh ghi bat dau burst sai, nhan=%h", rx_byte);
            slave_send_ack();

            wait_i2c_start();

            slave_recv_byte(rx_byte);
            if (rx_byte !== 8'hD1)
                $display("ERROR: dia chi doc MPU6050 sai, nhan=%h", rx_byte);
            slave_send_ack();

            slave_send_byte(EXP_ACCEL_X[15:8]);
            expect_master_ack(1'b1);
            slave_send_byte(EXP_ACCEL_X[7:0]);
            expect_master_ack(1'b1);
            slave_send_byte(EXP_ACCEL_Y[15:8]);
            expect_master_ack(1'b1);
            slave_send_byte(EXP_ACCEL_Y[7:0]);
            expect_master_ack(1'b1);
            slave_send_byte(EXP_ACCEL_Z[15:8]);
            expect_master_ack(1'b1);
            slave_send_byte(EXP_ACCEL_Z[7:0]);
            expect_master_ack(1'b0);
        end
    endtask

    initial begin
        rst = 1'b0;
        sda_slave_drive_low = 1'b0;

        #100;
        rst = 1'b1;

        wait(init_done);
        wait(sample_valid);

        if ({accel_x, accel_y, accel_z} !== EXP_BURST)
            $display("ERROR: frame du lieu sai, nhan=%h, expected=%h",
                     {accel_x, accel_y, accel_z}, EXP_BURST);
        else begin
            $display("ACCEL_X = %h", accel_x);
            $display("ACCEL_Y = %h", accel_y);
            $display("ACCEL_Z = %h", accel_z);
        end

        if (burst_data !== EXP_BURST)
            $display("ERROR: burst_data=%h, expected=%h", burst_data, EXP_BURST);

        #200;
        $stop;
    end

    initial begin
        wait(rst == 1'b1);
        expect_write_transaction(8'h6B, 8'h00);
        expect_write_transaction(8'h19, 8'h07);
        expect_write_transaction(8'h1A, 8'h03);
        expect_write_transaction(8'h1C, 8'h00);
        expect_read_burst();
    end

endmodule