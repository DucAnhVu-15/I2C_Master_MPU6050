module top_de10_lite_mpu6050 (
    input  wire        MAX10_CLK1_50,
    input  wire [1:0]  KEY,
    inout  wire        I2C_SDA,
    inout  wire        I2C_SCL,
    output wire        UART_TX
);

    // ========================================
    // Reset: KEY[0] la active-low tren DE10-Lite
    // ========================================
    wire rst_n = KEY[0];
    wire rst   = ~rst_n;  // active-high cho UART/FIFO

    // ========================================
    // MPU6050 I2C - Doc ca Accel va Gyro
    // ========================================
    wire [15:0] accel_x;
    wire [15:0] accel_y;
    wire [15:0] accel_z;
    wire [15:0] gyro_x;
    wire [15:0] gyro_y;
    wire [15:0] gyro_z;
    wire        data_valid;
    wire        init_done;
    wire        error;

    mpu6050_i2c #(
        .divider      (500),       // 50MHz/500 = 100kHz
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
    // FIFO TX - buffer du lieu truoc khi gui
    // ========================================
    wire       fifo_wr_en;
    wire       fifo_rd_en;
    wire [7:0] fifo_din;
    wire [7:0] fifo_dout;
    wire       fifo_full;
    wire       fifo_empty;

    fifo_sync #(
        .DATA_WIDTH (8),
        .ADDR_WIDTH (6)   // 64 bytes buffer (message dai hon)
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

    // Doc tu FIFO khi TX khong ban
    assign fifo_rd_en = tx_start;

    always @(posedge MAX10_CLK1_50 or posedge rst) begin
        if (rst)
            tx_start <= 1'b0;
        else
            tx_start <= !fifo_empty && !tx_busy && !tx_start;
    end

    // ========================================
    // UART TX Controller - Gui Accel (m/s²) va Gyro (deg/s)
    // Format: "AX:±xx.xx AY:±yy.yy AZ:±zz.zz GX:±xxx.x GY:±yyy.y GZ:±zzz.z\r\n"
    // ========================================
    
    // Chuyen doi:
    // Accel: raw * 599 / 10000 -> m/s² x100 (±2g, sensitivity 16384)
    // Gyro:  raw * 763 / 100000 -> deg/s x10 (±250dps, sensitivity 131)
    //        hoac raw / 131 * 10 = raw * 10 / 131
    
    reg [6:0]  msg_idx;        // index trong message (0-55)
    reg signed [15:0] tx_accel_x, tx_accel_y, tx_accel_z;
    reg signed [15:0] tx_gyro_x, tx_gyro_y, tx_gyro_z;
    reg        wr_req;
    reg [7:0]  wr_data;
    reg        tx_busy_flag;
    reg        wr_pending;
    
    // Gia tri da chuyen doi
    reg signed [15:0] accel_x_conv, accel_y_conv, accel_z_conv;  // m/s² x100
    reg signed [15:0] gyro_x_conv, gyro_y_conv, gyro_z_conv;     // deg/s x10
    reg        conv_done;
    reg [2:0]  conv_step;
    
    assign fifo_wr_en = wr_req && !fifo_full;
    assign fifo_din   = wr_data;

    // Chuyen so thanh ASCII digit
    function [7:0] digit_to_ascii;
        input [3:0] d;
        begin
            digit_to_ascii = 8'h30 + d;  // '0'-'9'
        end
    endfunction
    
    // Lay byte cho Accel value (±xx.xx format, 6 chars: sign + 2digit + . + 2digit)
    function [7:0] get_accel_byte;
        input [2:0] pos;  // 0=sign, 1-2=integer, 3=dot, 4-5=decimal
        input signed [15:0] val;
        reg [15:0] abs_val;
        begin
            abs_val = (val[15]) ? (~val + 1'b1) : val;
            case (pos)
                3'd0: get_accel_byte = (val[15]) ? "-" : "+";
                3'd1: get_accel_byte = digit_to_ascii((abs_val / 1000) % 10);
                3'd2: get_accel_byte = digit_to_ascii((abs_val / 100) % 10);
                3'd3: get_accel_byte = ".";
                3'd4: get_accel_byte = digit_to_ascii((abs_val / 10) % 10);
                3'd5: get_accel_byte = digit_to_ascii(abs_val % 10);
                default: get_accel_byte = " ";
            endcase
        end
    endfunction
    
    // Lay byte cho Gyro value (±xxx.x format, 6 chars: sign + 3digit + . + 1digit)
    function [7:0] get_gyro_byte;
        input [2:0] pos;  // 0=sign, 1-3=integer, 4=dot, 5=decimal
        input signed [15:0] val;
        reg [15:0] abs_val;
        begin
            abs_val = (val[15]) ? (~val + 1'b1) : val;
            case (pos)
                3'd0: get_gyro_byte = (val[15]) ? "-" : "+";
                3'd1: get_gyro_byte = digit_to_ascii((abs_val / 1000) % 10);
                3'd2: get_gyro_byte = digit_to_ascii((abs_val / 100) % 10);
                3'd3: get_gyro_byte = digit_to_ascii((abs_val / 10) % 10);
                3'd4: get_gyro_byte = ".";
                3'd5: get_gyro_byte = digit_to_ascii(abs_val % 10);
                default: get_gyro_byte = " ";
            endcase
        end
    endfunction
    
    // Lay byte tuong ung voi index trong message
    // Format: "AX:±xx.xx AY:±yy.yy AZ:±zz.zz GX:±xxx.x GY:±yyy.y GZ:±zzz.z\r\n"
    function [7:0] get_msg_byte;
        input [6:0] idx;
        input signed [15:0] ax, ay, az, gx, gy, gz;
        begin
            case (idx)
                // AX:±xx.xx (9 chars: AX: + 6 value)
                7'd0:  get_msg_byte = "A";
                7'd1:  get_msg_byte = "X";
                7'd2:  get_msg_byte = ":";
                7'd3:  get_msg_byte = get_accel_byte(0, ax);
                7'd4:  get_msg_byte = get_accel_byte(1, ax);
                7'd5:  get_msg_byte = get_accel_byte(2, ax);
                7'd6:  get_msg_byte = get_accel_byte(3, ax);
                7'd7:  get_msg_byte = get_accel_byte(4, ax);
                7'd8:  get_msg_byte = get_accel_byte(5, ax);
                7'd9:  get_msg_byte = " ";
                
                // AY:±yy.yy
                7'd10: get_msg_byte = "A";
                7'd11: get_msg_byte = "Y";
                7'd12: get_msg_byte = ":";
                7'd13: get_msg_byte = get_accel_byte(0, ay);
                7'd14: get_msg_byte = get_accel_byte(1, ay);
                7'd15: get_msg_byte = get_accel_byte(2, ay);
                7'd16: get_msg_byte = get_accel_byte(3, ay);
                7'd17: get_msg_byte = get_accel_byte(4, ay);
                7'd18: get_msg_byte = get_accel_byte(5, ay);
                7'd19: get_msg_byte = " ";
                
                // AZ:±zz.zz
                7'd20: get_msg_byte = "A";
                7'd21: get_msg_byte = "Z";
                7'd22: get_msg_byte = ":";
                7'd23: get_msg_byte = get_accel_byte(0, az);
                7'd24: get_msg_byte = get_accel_byte(1, az);
                7'd25: get_msg_byte = get_accel_byte(2, az);
                7'd26: get_msg_byte = get_accel_byte(3, az);
                7'd27: get_msg_byte = get_accel_byte(4, az);
                7'd28: get_msg_byte = get_accel_byte(5, az);
                7'd29: get_msg_byte = " ";
                
                // GX:±xxx.x
                7'd30: get_msg_byte = "G";
                7'd31: get_msg_byte = "X";
                7'd32: get_msg_byte = ":";
                7'd33: get_msg_byte = get_gyro_byte(0, gx);
                7'd34: get_msg_byte = get_gyro_byte(1, gx);
                7'd35: get_msg_byte = get_gyro_byte(2, gx);
                7'd36: get_msg_byte = get_gyro_byte(3, gx);
                7'd37: get_msg_byte = get_gyro_byte(4, gx);
                7'd38: get_msg_byte = get_gyro_byte(5, gx);
                7'd39: get_msg_byte = " ";
                
                // GY:±yyy.y
                7'd40: get_msg_byte = "G";
                7'd41: get_msg_byte = "Y";
                7'd42: get_msg_byte = ":";
                7'd43: get_msg_byte = get_gyro_byte(0, gy);
                7'd44: get_msg_byte = get_gyro_byte(1, gy);
                7'd45: get_msg_byte = get_gyro_byte(2, gy);
                7'd46: get_msg_byte = get_gyro_byte(3, gy);
                7'd47: get_msg_byte = get_gyro_byte(4, gy);
                7'd48: get_msg_byte = get_gyro_byte(5, gy);
                7'd49: get_msg_byte = " ";
                
                // GZ:±zzz.z
                7'd50: get_msg_byte = "G";
                7'd51: get_msg_byte = "Z";
                7'd52: get_msg_byte = ":";
                7'd53: get_msg_byte = get_gyro_byte(0, gz);
                7'd54: get_msg_byte = get_gyro_byte(1, gz);
                7'd55: get_msg_byte = get_gyro_byte(2, gz);
                7'd56: get_msg_byte = get_gyro_byte(3, gz);
                7'd57: get_msg_byte = get_gyro_byte(4, gz);
                7'd58: get_msg_byte = get_gyro_byte(5, gz);
                
                7'd59: get_msg_byte = 8'h0D;  // CR
                7'd60: get_msg_byte = 8'h0A;  // LF
                default: get_msg_byte = 8'h00;
            endcase
        end
    endfunction

    // Phep nhan cho chuyen doi
    wire signed [31:0] mult_ax = tx_accel_x * 16'sd599;
    wire signed [31:0] mult_ay = tx_accel_y * 16'sd599;
    wire signed [31:0] mult_az = tx_accel_z * 16'sd599;
    wire signed [31:0] mult_gx = tx_gyro_x * 16'sd10;
    wire signed [31:0] mult_gy = tx_gyro_y * 16'sd10;
    wire signed [31:0] mult_gz = tx_gyro_z * 16'sd10;
    
    always @(posedge MAX10_CLK1_50 or posedge rst) begin
        if (rst) begin
            msg_idx      <= 7'd0;
            tx_accel_x   <= 16'sd0;
            tx_accel_y   <= 16'sd0;
            tx_accel_z   <= 16'sd0;
            tx_gyro_x    <= 16'sd0;
            tx_gyro_y    <= 16'sd0;
            tx_gyro_z    <= 16'sd0;
            wr_req       <= 1'b0;
            wr_data      <= 8'd0;
            tx_busy_flag <= 1'b0;
            wr_pending   <= 1'b0;
            accel_x_conv <= 16'sd0;
            accel_y_conv <= 16'sd0;
            accel_z_conv <= 16'sd0;
            gyro_x_conv  <= 16'sd0;
            gyro_y_conv  <= 16'sd0;
            gyro_z_conv  <= 16'sd0;
            conv_done    <= 1'b0;
            conv_step    <= 3'd0;
        end else begin
            // Sau khi ghi xong, cho 1 cycle
            if (wr_req && !fifo_full) begin
                wr_req     <= 1'b0;
                wr_pending <= 1'b1;
            end else if (wr_pending) begin
                wr_pending <= 1'b0;
                if (msg_idx == 7'd60) begin
                    tx_busy_flag <= 1'b0;
                    msg_idx      <= 7'd0;
                    conv_done    <= 1'b0;
                end else begin
                    msg_idx <= msg_idx + 1'b1;
                end
            end else if (!tx_busy_flag) begin
                // IDLE: cho data_valid
                if (data_valid && init_done && !error) begin
                    tx_accel_x   <= accel_x;
                    tx_accel_y   <= accel_y;
                    tx_accel_z   <= accel_z;
                    tx_gyro_x    <= gyro_x;
                    tx_gyro_y    <= gyro_y;
                    tx_gyro_z    <= gyro_z;
                    msg_idx      <= 7'd0;
                    tx_busy_flag <= 1'b1;
                    conv_step    <= 3'd0;
                    conv_done    <= 1'b0;
                end
            end else if (!conv_done) begin
                // Chuyen doi: pipeline chia
                case (conv_step)
                    3'd0: begin
                        accel_x_conv <= mult_ax / 10000;
                        conv_step    <= 3'd1;
                    end
                    3'd1: begin
                        accel_y_conv <= mult_ay / 10000;
                        conv_step    <= 3'd2;
                    end
                    3'd2: begin
                        accel_z_conv <= mult_az / 10000;
                        conv_step    <= 3'd3;
                    end
                    3'd3: begin
                        gyro_x_conv <= mult_gx / 131;  // sensitivity 131 LSB/(deg/s)
                        conv_step   <= 3'd4;
                    end
                    3'd4: begin
                        gyro_y_conv <= mult_gy / 131;
                        conv_step   <= 3'd5;
                    end
                    3'd5: begin
                        gyro_z_conv <= mult_gz / 131;
                        conv_step   <= 3'd6;
                    end
                    3'd6: begin
                        conv_done <= 1'b1;
                        wr_data   <= "A";
                        wr_req    <= 1'b1;
                    end
                    default: conv_step <= 3'd0;
                endcase
            end else if (!fifo_full && !wr_req) begin
                // BUSY: gui byte tiep theo
                wr_data <= get_msg_byte(msg_idx, accel_x_conv, accel_y_conv, accel_z_conv,
                                                 gyro_x_conv, gyro_y_conv, gyro_z_conv);
                wr_req  <= 1'b1;
            end
        end
    end

    // Tie off unused signals
    wire unused_ok;
    assign unused_ok = &{1'b0, error};

endmodule
