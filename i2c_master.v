// DAV - I2C Master Controller for MPU6050
module i2c_master (
    input clk_sys,
    input rst,
    input [7:0] reg_addr,
    input [7:0] data_wr,
    input r_w,
    input startfr,
    input stopfr,
    inout sda,
    inout scl,
    output [47:0] data_rd,
    output reg writebytedone,
    output reg readbytedone,
    output reg done
);

    parameter divider = 256;
    parameter MPU6050_ADDR = 7'h68;
    parameter READ_BYTES = 6;

    parameter IDLE             = 4'd0;
    parameter START            = 4'd1;
    parameter REPEATED_START   = 4'd2;
    parameter WRITE_BYTE       = 4'd3;
    parameter RECEIVER_ACK     = 4'd4;
    parameter READ_BYTE        = 4'd5;
    parameter SEND_MASTER_ACK  = 4'd6;
    parameter SEND_MASTER_NACK = 4'd7;
    parameter STOP             = 4'd8;

    parameter PHASE_DEV_ADDR_W = 2'd0;
    parameter PHASE_REG_ADDR   = 2'd1;
    parameter PHASE_DEV_ADDR_R = 2'd2;
    parameter PHASE_WRITE_DATA = 2'd3;

    reg [3:0] state = IDLE;
    reg [2:0] bit_cnt = 3'd7;
    reg [2:0] read_byte_cnt = 3'd0;
    reg [1:0] phase = PHASE_DEV_ADDR_W;
    reg [7:0] tx_byte = 8'h00;
    reg [7:0] reg_addr_latched = 8'h00;
    reg [7:0] data_wr_latched = 8'h00;
    reg op_read = 1'b0;
    reg [47:0] data_rd_reg = 48'd0;
    reg [15:0] clock_divider = 16'd0;
    reg sda_out = 1'b1;
    reg sda_en = 1'b1;
    reg scl_out = 1'b1;
    reg scl_en = 1'b1;
    reg ack_flag = 1'b0;

    assign sda = sda_en ? sda_out : 1'bz;
    assign scl = scl_en ? scl_out : 1'bz;
    assign data_rd = data_rd_reg;

    always @(posedge clk_sys or negedge rst) begin
        if (!rst) begin
            state <= IDLE;
            bit_cnt <= 3'd7;
            read_byte_cnt <= 3'd0;
            phase <= PHASE_DEV_ADDR_W;
            tx_byte <= 8'h00;
            reg_addr_latched <= 8'h00;
            data_wr_latched <= 8'h00;
            op_read <= 1'b0;
            data_rd_reg <= 48'd0;
            clock_divider <= 16'd0;
            sda_out <= 1'b1;
            sda_en <= 1'b1;
            scl_out <= 1'b1;
            scl_en <= 1'b1;
            ack_flag <= 1'b0;
            writebytedone <= 1'b0;
            readbytedone <= 1'b0;
            done <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    writebytedone <= 1'b0;
                    readbytedone <= 1'b0;
                    ack_flag <= 1'b0;
                    clock_divider <= 16'd0;
                    bit_cnt <= 3'd7;
                    read_byte_cnt <= 3'd0;
                    sda_en <= 1'b1;
                    sda_out <= 1'b1;
                    scl_en <= 1'b1;
                    scl_out <= 1'b1;

                    if (startfr) begin
                        reg_addr_latched <= reg_addr;
                        data_wr_latched <= data_wr;
                        op_read <= r_w;
                        tx_byte <= {MPU6050_ADDR, 1'b0};
                        phase <= PHASE_DEV_ADDR_W;
                        data_rd_reg <= 48'd0;
                        state <= START;
                    end
                end

                START: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b0;
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider/2 - 1) begin
                        clock_divider <= 16'd0;
                        state <= WRITE_BYTE;
                    end
                end

                REPEATED_START: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0) begin
                        sda_en <= 1'b1;
                        sda_out <= 1'b1;
                        scl_out <= 1'b0;
                    end else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider/2 - 1)
                        sda_out <= 1'b0;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        state <= WRITE_BYTE;
                    end
                end

                WRITE_BYTE: begin
                    writebytedone <= 1'b0;
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= tx_byte[bit_cnt];
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        if (bit_cnt > 0) begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end else begin
                            bit_cnt <= 3'd7;
                            sda_en <= 1'b0;
                            state <= RECEIVER_ACK;
                        end
                    end
                end

                RECEIVER_ACK: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        if (ack_flag) begin
                            ack_flag <= 1'b0;
                            clock_divider <= 16'd0;

                            case (phase)
                                PHASE_DEV_ADDR_W: begin
                                    tx_byte <= reg_addr_latched;
                                    phase <= PHASE_REG_ADDR;
                                    sda_en <= 1'b1;
                                    state <= WRITE_BYTE;
                                end

                                PHASE_REG_ADDR: begin
                                    if (op_read) begin
                                        tx_byte <= {MPU6050_ADDR, 1'b1};
                                        phase <= PHASE_DEV_ADDR_R;
                                        sda_en <= 1'b1;
                                        sda_out <= 1'b1;
                                        state <= REPEATED_START;
                                    end else begin
                                        tx_byte <= data_wr_latched;
                                        phase <= PHASE_WRITE_DATA;
                                        sda_en <= 1'b1;
                                        state <= WRITE_BYTE;
                                    end
                                end

                                PHASE_DEV_ADDR_R: begin
                                    sda_en <= 1'b0;
                                    state <= READ_BYTE;
                                end

                                PHASE_WRITE_DATA: begin
                                    writebytedone <= 1'b1;
                                    sda_en <= 1'b1;
                                    state <= STOP;
                                end

                                default: begin
                                    state <= IDLE;
                                end
                            endcase
                        end else begin
                            clock_divider <= divider*3/4;
                        end
                    end else if (clock_divider > divider/4 - 1) begin
                        if (sda == 1'b0)
                            ack_flag <= 1'b1;
                    end
                end

                READ_BYTE: begin
                    readbytedone <= 1'b0;
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider/2 - 1)
                        data_rd_reg[((READ_BYTES - 1 - read_byte_cnt) * 8) + bit_cnt] <= sda;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        if (bit_cnt > 0) begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end else begin
                            bit_cnt <= 3'd7;
                            sda_en <= 1'b1;
                            if (read_byte_cnt < READ_BYTES - 1) begin
                                read_byte_cnt <= read_byte_cnt + 1'b1;
                                state <= SEND_MASTER_ACK;
                            end else begin
                                readbytedone <= 1'b1;
                                state <= SEND_MASTER_NACK;
                            end
                        end
                    end
                end

                SEND_MASTER_ACK: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b0;
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        sda_en <= 1'b0;
                        state <= READ_BYTE;
                    end
                end

                SEND_MASTER_NACK: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b1;
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        sda_en <= 1'b1;
                        state <= STOP;
                    end
                end

                STOP: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b0;
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider/2 - 1) begin
                        sda_out <= 1'b1;
                        clock_divider <= 16'd0;
                        done <= 1'b1;
                        state <= IDLE;
                    end
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
