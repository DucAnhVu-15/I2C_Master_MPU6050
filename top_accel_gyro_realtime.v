// =============================================================================
// Module: top_accel_gyro_realtime
// Description: Gui du lieu 6 truc (Accel X,Y,Z + Gyro X,Y,Z) realtime qua UART
// Format: "AX:±xxxxx AY:±xxxxx AZ:±xxxxx GX:±xxxxx GY:±xxxxx GZ:±xxxxx\r\n"
// =============================================================================
module top_accel_gyro_realtime (
    input  wire        MAX10_CLK1_50,
    input  wire [1:0]  KEY,
    inout  wire        I2C_SDA,
    inout  wire        I2C_SCL,
    output wire        UART_TX
);

    // ========================================
    // Reset
    // ========================================
    wire rst_n = KEY[0];
    wire rst   = ~rst_n;

    // ========================================
    // MPU6050 I2C
    // ========================================
    wire [15:0] accel_x, accel_y, accel_z;
    wire [15:0] gyro_x, gyro_y, gyro_z;
    wire        data_valid;
    wire        init_done;
    wire        error;

    mpu6050_i2c #(
        .divider      (500),
        .sample_delay (500000)
    ) u_mpu6050_i2c (
        .clk_sys    (MAX10_CLK1_50),
        .rst        (rst_n),
        .accel_x    (accel_x),
        .accel_y    (accel_y),
        .accel_z    (accel_z),
        .gyro_x     (gyro_x),
        .gyro_y     (gyro_y),
        .gyro_z     (gyro_z),
        .data_valid (data_valid),
        .init_done  (init_done),
        .error      (error),
        .sda        (I2C_SDA),
        .scl        (I2C_SCL)
    );

    // ========================================
    // Baud Generator - 115200 baud
    // ========================================
    wire baud_tick;

    baud_gen #(
        .CLK_FREQ (50_000_000),
        .BAUD     (115200)
    ) u_baud_gen (
        .clk       (MAX10_CLK1_50),
        .rst       (rst),
        .baud_tick (baud_tick)
    );

    // ========================================
    // FIFO TX
    // ========================================
    wire       fifo_wr_en;
    wire       fifo_rd_en;
    wire [7:0] fifo_din;
    wire [7:0] fifo_dout;
    wire       fifo_full;
    wire       fifo_empty;

    fifo_sync #(
        .DATA_WIDTH (8),
        .ADDR_WIDTH (7)   // 128 bytes
    ) u_fifo_tx (
        .clk   (MAX10_CLK1_50),
        .rst   (rst),
        .wr_en (fifo_wr_en),
        .rd_en (fifo_rd_en),
        .din   (fifo_din),
        .dout  (fifo_dout),
        .full  (fifo_full),
        .empty (fifo_empty),
        .count ()
    );

    // ========================================
    // UART TX
    // ========================================
    wire tx_busy;
    reg  tx_start;

    uart_tx #(
        .DATA_BITS (8)
    ) u_uart_tx (
        .clk        (MAX10_CLK1_50),
        .rst        (rst),
        .baud_tick  (baud_tick),
        .data_in    (fifo_dout),
        .data_valid (tx_start),
        .tx         (UART_TX),
        .busy       (tx_busy)
    );

    assign fifo_rd_en = tx_start;

    always @(posedge MAX10_CLK1_50 or posedge rst) begin
        if (rst)
            tx_start <= 1'b0;
        else
            tx_start <= !fifo_empty && !tx_busy && !tx_start;
    end

    // ========================================
    // Sender FSM - Gui 6 truc
    // Format: "AX:±xxxxx AY:±xxxxx AZ:±xxxxx GX:±xxxxx GY:±xxxxx GZ:±xxxxx\r\n"
    // ========================================
    localparam IDLE      = 2'd0;
    localparam SEND_CHAR = 2'd1;
    localparam NEXT_CHAR = 2'd2;
    localparam WAIT_DONE = 2'd3;
    
    reg [1:0]  state;
    reg [6:0]  char_pos;
    reg        wr_en_r;
    reg [7:0]  wr_data;
    reg        busy;
    
    // Latch du lieu
    reg signed [15:0] latch_ax, latch_ay, latch_az;
    reg signed [15:0] latch_gx, latch_gy, latch_gz;
    
    assign fifo_wr_en = wr_en_r && !fifo_full;
    assign fifo_din   = wr_data;
    
    // Chuyen so thanh ASCII
    function [7:0] digit_to_ascii;
        input [3:0] d;
        begin
            digit_to_ascii = 8'h30 + d;
        end
    endfunction
    
    // Lay ky tu value (±xxxxx, 6 chars)
    function [7:0] get_value_char;
        input [2:0] pos;
        input signed [15:0] val;
        reg [15:0] abs_val;
        begin
            abs_val = (val[15]) ? (~val + 1'b1) : val;
            case (pos)
                3'd0: get_value_char = (val[15]) ? "-" : "+";
                3'd1: get_value_char = digit_to_ascii((abs_val / 10000) % 10);
                3'd2: get_value_char = digit_to_ascii((abs_val / 1000) % 10);
                3'd3: get_value_char = digit_to_ascii((abs_val / 100) % 10);
                3'd4: get_value_char = digit_to_ascii((abs_val / 10) % 10);
                3'd5: get_value_char = digit_to_ascii(abs_val % 10);
                default: get_value_char = " ";
            endcase
        end
    endfunction
    
    // Message: "AX:±xxxxx AY:±xxxxx AZ:±xxxxx GX:±xxxxx GY:±xxxxx GZ:±xxxxx\r\n"
    // Positions: AX(0-8) space(9) AY(10-18) space(19) AZ(20-28) space(29) 
    //            GX(30-38) space(39) GY(40-48) space(49) GZ(50-58) CR(59) LF(60)
    function [7:0] get_msg_char;
        input [6:0] pos;
        input signed [15:0] ax, ay, az, gx, gy, gz;
        begin
            case (pos)
                // AX:±xxxxx
                7'd0:  get_msg_char = "A";
                7'd1:  get_msg_char = "X";
                7'd2:  get_msg_char = ":";
                7'd3:  get_msg_char = get_value_char(0, ax);
                7'd4:  get_msg_char = get_value_char(1, ax);
                7'd5:  get_msg_char = get_value_char(2, ax);
                7'd6:  get_msg_char = get_value_char(3, ax);
                7'd7:  get_msg_char = get_value_char(4, ax);
                7'd8:  get_msg_char = get_value_char(5, ax);
                7'd9:  get_msg_char = " ";
                // AY:±xxxxx
                7'd10: get_msg_char = "A";
                7'd11: get_msg_char = "Y";
                7'd12: get_msg_char = ":";
                7'd13: get_msg_char = get_value_char(0, ay);
                7'd14: get_msg_char = get_value_char(1, ay);
                7'd15: get_msg_char = get_value_char(2, ay);
                7'd16: get_msg_char = get_value_char(3, ay);
                7'd17: get_msg_char = get_value_char(4, ay);
                7'd18: get_msg_char = get_value_char(5, ay);
                7'd19: get_msg_char = " ";
                // AZ:±xxxxx
                7'd20: get_msg_char = "A";
                7'd21: get_msg_char = "Z";
                7'd22: get_msg_char = ":";
                7'd23: get_msg_char = get_value_char(0, az);
                7'd24: get_msg_char = get_value_char(1, az);
                7'd25: get_msg_char = get_value_char(2, az);
                7'd26: get_msg_char = get_value_char(3, az);
                7'd27: get_msg_char = get_value_char(4, az);
                7'd28: get_msg_char = get_value_char(5, az);
                7'd29: get_msg_char = " ";
                // GX:±xxxxx
                7'd30: get_msg_char = "G";
                7'd31: get_msg_char = "X";
                7'd32: get_msg_char = ":";
                7'd33: get_msg_char = get_value_char(0, gx);
                7'd34: get_msg_char = get_value_char(1, gx);
                7'd35: get_msg_char = get_value_char(2, gx);
                7'd36: get_msg_char = get_value_char(3, gx);
                7'd37: get_msg_char = get_value_char(4, gx);
                7'd38: get_msg_char = get_value_char(5, gx);
                7'd39: get_msg_char = " ";
                // GY:±xxxxx
                7'd40: get_msg_char = "G";
                7'd41: get_msg_char = "Y";
                7'd42: get_msg_char = ":";
                7'd43: get_msg_char = get_value_char(0, gy);
                7'd44: get_msg_char = get_value_char(1, gy);
                7'd45: get_msg_char = get_value_char(2, gy);
                7'd46: get_msg_char = get_value_char(3, gy);
                7'd47: get_msg_char = get_value_char(4, gy);
                7'd48: get_msg_char = get_value_char(5, gy);
                7'd49: get_msg_char = " ";
                // GZ:±xxxxx
                7'd50: get_msg_char = "G";
                7'd51: get_msg_char = "Z";
                7'd52: get_msg_char = ":";
                7'd53: get_msg_char = get_value_char(0, gz);
                7'd54: get_msg_char = get_value_char(1, gz);
                7'd55: get_msg_char = get_value_char(2, gz);
                7'd56: get_msg_char = get_value_char(3, gz);
                7'd57: get_msg_char = get_value_char(4, gz);
                7'd58: get_msg_char = get_value_char(5, gz);
                // CR LF
                7'd59: get_msg_char = 8'h0D;
                7'd60: get_msg_char = 8'h0A;
                default: get_msg_char = " ";
            endcase
        end
    endfunction
    
    always @(posedge MAX10_CLK1_50 or posedge rst) begin
        if (rst) begin
            state    <= IDLE;
            char_pos <= 7'd0;
            wr_en_r  <= 1'b0;
            wr_data  <= 8'd0;
            busy     <= 1'b0;
            latch_ax <= 16'd0;
            latch_ay <= 16'd0;
            latch_az <= 16'd0;
            latch_gx <= 16'd0;
            latch_gy <= 16'd0;
            latch_gz <= 16'd0;
        end else begin
            wr_en_r <= 1'b0;
            
            case (state)
                IDLE: begin
                    if (data_valid && init_done && !error && !busy) begin
                        latch_ax <= accel_x;
                        latch_ay <= accel_y;
                        latch_az <= accel_z;
                        latch_gx <= gyro_x;
                        latch_gy <= gyro_y;
                        latch_gz <= gyro_z;
                        char_pos <= 7'd0;
                        busy     <= 1'b1;
                        state    <= SEND_CHAR;
                    end
                end
                
                SEND_CHAR: begin
                    if (!fifo_full) begin
                        wr_data <= get_msg_char(char_pos, latch_ax, latch_ay, latch_az,
                                                         latch_gx, latch_gy, latch_gz);
                        wr_en_r <= 1'b1;
                        state   <= NEXT_CHAR;
                    end
                end
                
                NEXT_CHAR: begin
                    if (char_pos == 7'd60) begin
                        busy  <= 1'b0;
                        state <= IDLE;
                    end else begin
                        char_pos <= char_pos + 1'b1;
                        state    <= SEND_CHAR;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

    // Unused signals
    wire unused_ok;
    assign unused_ok = &{1'b0, error, KEY[1]};

endmodule
