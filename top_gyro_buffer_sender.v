// =============================================================================
// Module: top_gyro_buffer_sender
// Description: Luu 256 mau gyro vao buffer, khi day thi gui qua UART
// Format: "GX[000]:±xxxxx\r\nGY[000]:±xxxxx\r\nGZ[000]:±xxxxx\r\n..."
// =============================================================================
module top_gyro_buffer_sender (
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
    // Buffer Gyro X, Y, Z - 256 mau
    // ========================================
    wire        buf_rd_en;
    wire [15:0] gyro_x_buf, gyro_y_buf, gyro_z_buf;
    wire        buf_full, buf_empty, buf_data_ready;
    wire [8:0]  buf_count;
    
    wire        sender_busy;
    
    buffer_gyro_x_y_z #(
        .BUFFER_DEPTH (256),
        .ADDR_WIDTH   (8)
    ) u_gyro_buffer (
        .clk        (MAX10_CLK1_50),
        .rst        (rst),
        .gyro_x_in  (gyro_x),
        .gyro_y_in  (gyro_y),
        .gyro_z_in  (gyro_z),
        .wr_en      (data_valid && init_done && !error && !sender_busy),
        .rd_en      (buf_rd_en),
        .gyro_x_out (gyro_x_buf),
        .gyro_y_out (gyro_y_buf),
        .gyro_z_out (gyro_z_buf),
        .data_ready (buf_data_ready),
        .full       (buf_full),
        .empty      (buf_empty),
        .count      (buf_count)
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
    
    wire       sender_done;
    wire       sender_fifo_wr_en;
    wire [7:0] sender_fifo_din;
    
    assign fifo_wr_en = sender_fifo_wr_en;
    assign fifo_din   = sender_fifo_din;

    fifo_sync #(
        .DATA_WIDTH (8),
        .ADDR_WIDTH (8)   // 256 bytes
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
    // UART Gyro Buffer Sender
    // Tu dong gui khi buffer day 256 mau
    // ========================================
    uart_gyro_buffer_sender #(
        .BUFFER_DEPTH (256),
        .ADDR_WIDTH   (8)
    ) u_gyro_sender (
        .clk          (MAX10_CLK1_50),
        .rst          (rst),
        .start_send   (buf_full && !sender_busy),
        .busy         (sender_busy),
        .done         (sender_done),
        .buf_rd_en    (buf_rd_en),
        .gyro_x_data  (gyro_x_buf),
        .gyro_y_data  (gyro_y_buf),
        .gyro_z_data  (gyro_z_buf),
        .buf_empty    (buf_empty),
        .buf_count    (buf_count),
        .fifo_wr_en   (sender_fifo_wr_en),
        .fifo_din     (sender_fifo_din),
        .fifo_full    (fifo_full)
    );

    // Unused signals
    wire unused_ok;
    assign unused_ok = &{1'b0, error, sender_done, buf_data_ready,
                         accel_x, accel_y, accel_z, KEY[1]};

endmodule
