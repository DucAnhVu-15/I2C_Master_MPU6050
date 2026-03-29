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
    // MPU6050 I2C
    // ========================================
    wire [15:0] accel_x;
    wire [15:0] accel_y;
    wire [15:0] accel_z;
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
        .ADDR_WIDTH (5)   // 32 bytes buffer
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
    // UART TX Controller - Gui du lieu khi data_valid
    // Format: "X:xxxx Y:yyyy Z:zzzz\r\n" (hex)
    // Tong cong 22 bytes: X:xxxx Y:yyyy Z:zzzz\r\n
    // ========================================
    reg [4:0]  msg_idx;        // 0-21: index trong message
    reg [15:0] tx_accel_x, tx_accel_y, tx_accel_z;
    reg        wr_req;
    reg [7:0]  wr_data;
    reg        tx_busy_flag;
    reg        wr_pending;     // cho 1 cycle sau khi ghi

    assign fifo_wr_en = wr_req && !fifo_full;
    assign fifo_din   = wr_data;

    // Chuyen 4-bit sang ASCII hex
    function [7:0] hex_to_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 10)
                hex_to_ascii = 8'h30 + nibble;  // '0'-'9'
            else
                hex_to_ascii = 8'h41 + nibble - 10;  // 'A'-'F'
        end
    endfunction

    // Lay byte tuong ung voi index
    function [7:0] get_msg_byte;
        input [4:0] idx;
        input [15:0] ax, ay, az;
        begin
            case (idx)
                5'd0:  get_msg_byte = "X";
                5'd1:  get_msg_byte = ":";
                5'd2:  get_msg_byte = hex_to_ascii(ax[15:12]);
                5'd3:  get_msg_byte = hex_to_ascii(ax[11:8]);
                5'd4:  get_msg_byte = hex_to_ascii(ax[7:4]);
                5'd5:  get_msg_byte = hex_to_ascii(ax[3:0]);
                5'd6:  get_msg_byte = " ";
                5'd7:  get_msg_byte = "Y";
                5'd8:  get_msg_byte = ":";
                5'd9:  get_msg_byte = hex_to_ascii(ay[15:12]);
                5'd10: get_msg_byte = hex_to_ascii(ay[11:8]);
                5'd11: get_msg_byte = hex_to_ascii(ay[7:4]);
                5'd12: get_msg_byte = hex_to_ascii(ay[3:0]);
                5'd13: get_msg_byte = " ";
                5'd14: get_msg_byte = "Z";
                5'd15: get_msg_byte = ":";
                5'd16: get_msg_byte = hex_to_ascii(az[15:12]);
                5'd17: get_msg_byte = hex_to_ascii(az[11:8]);
                5'd18: get_msg_byte = hex_to_ascii(az[7:4]);
                5'd19: get_msg_byte = hex_to_ascii(az[3:0]);
                5'd20: get_msg_byte = 8'h0D;  // CR
                5'd21: get_msg_byte = 8'h0A;  // LF
                default: get_msg_byte = 8'h00;
            endcase
        end
    endfunction

    always @(posedge MAX10_CLK1_50 or posedge rst) begin
        if (rst) begin
            msg_idx      <= 5'd0;
            tx_accel_x   <= 16'd0;
            tx_accel_y   <= 16'd0;
            tx_accel_z   <= 16'd0;
            wr_req       <= 1'b0;
            wr_data      <= 8'd0;
            tx_busy_flag <= 1'b0;
            wr_pending   <= 1'b0;
        end else begin
            // Sau khi ghi xong, cho 1 cycle
            if (wr_req && !fifo_full) begin
                wr_req     <= 1'b0;
                wr_pending <= 1'b1;
            end else if (wr_pending) begin
                wr_pending <= 1'b0;
                // Tang index sau khi da ghi thanh cong
                if (msg_idx == 5'd21) begin
                    tx_busy_flag <= 1'b0;
                    msg_idx      <= 5'd0;
                end else begin
                    msg_idx <= msg_idx + 1'b1;
                end
            end else if (!tx_busy_flag) begin
                // IDLE: cho data_valid
                if (data_valid && init_done && !error) begin
                    tx_accel_x   <= accel_x;
                    tx_accel_y   <= accel_y;
                    tx_accel_z   <= accel_z;
                    msg_idx      <= 5'd0;
                    tx_busy_flag <= 1'b1;
                    wr_data      <= "X";  // Chuan bi byte dau tien
                    wr_req       <= 1'b1;
                end
            end else if (!fifo_full && !wr_req) begin
                // BUSY: gui byte tiep theo
                wr_data <= get_msg_byte(msg_idx, tx_accel_x, tx_accel_y, tx_accel_z);
                wr_req  <= 1'b1;
            end
        end
    end

    // Tie off unused signals
    wire unused_ok;
    assign unused_ok = &{1'b0, error};

endmodule
