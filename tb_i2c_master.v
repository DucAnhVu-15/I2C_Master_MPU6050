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
    wire done;

    reg sda_slave_drive_low;
    reg [7:0] rx_byte;

    localparam [15:0] ACCEL_X = 16'h1234;
    localparam [15:0] ACCEL_Y = 16'h5678;
    localparam [15:0] ACCEL_Z = 16'h9ABC;
    localparam [47:0] EXPECTED_BURST = {ACCEL_X, ACCEL_Y, ACCEL_Z};

    assign sda = sda_slave_drive_low ? 1'b0 : 1'bz;

    i2c_master #(
        .divider(16)
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
        .done(done)
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

    initial begin
        rst = 1'b0;
        reg_addr = 8'h3B;
        data_wr = 8'h00;
        r_w = 1'b1;
        startfr = 1'b0;
        stopfr = 1'b0;
        sda_slave_drive_low = 1'b0;

        #100;
        rst = 1'b1;

        #100;
        startfr = 1'b1;
        #20;
        startfr = 1'b0;

        wait(done);

        if (data_rd !== EXPECTED_BURST)
            $display("ERROR: data_rd=%h, expected=%h", data_rd, EXPECTED_BURST);
        else begin
            $display("ACCEL_X = %h", data_rd[47:32]);
            $display("ACCEL_Y = %h", data_rd[31:16]);
            $display("ACCEL_Z = %h", data_rd[15:0]);
        end

        #200;
        $stop;
    end

    initial begin
        wait(rst == 1'b1);

        wait_i2c_start();

        slave_recv_byte(rx_byte);
        if (rx_byte !== 8'hD0)
            $display("ERROR: dia chi ghi MPU6050 sai, nhan=%h", rx_byte);
        slave_send_ack();

        slave_recv_byte(rx_byte);
        if (rx_byte !== 8'h3B)
            $display("ERROR: thanh ghi bat dau sai, nhan=%h", rx_byte);
        slave_send_ack();

        wait_i2c_start();

        slave_recv_byte(rx_byte);
        if (rx_byte !== 8'hD1)
            $display("ERROR: dia chi doc MPU6050 sai, nhan=%h", rx_byte);
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
    end

endmodule
