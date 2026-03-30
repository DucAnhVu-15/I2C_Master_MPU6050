// =============================================================================
// Module: top_distance_calculator
// Description: Tinh Distance trung binh tu 256 mau gyro
//              D = sqrt(X² + Y² + Z²), gui qua UART khi vuot nguong
// =============================================================================
module top_distance_calculator (
    input  wire        MAX10_CLK1_50,
    input  wire [1:0]  KEY,
    inout  wire        I2C_SDA,
    inout  wire        I2C_SCL,
    output wire        UART_TX
);

    // ========================================
    // Reset: KEY[0] la active-low tren DE10-Lite
    // KEY[1] de gui buffer qua UART
    // ========================================
    wire rst_n = KEY[0];
    wire rst   = ~rst_n;  // active-high cho UART/FIFO
    
    // Debounce KEY[1] - gui buffer
    reg [19:0] key1_cnt;
    reg key1_sync, key1_prev, key1_pulse;
    
    always @(posedge MAX10_CLK1_50 or posedge rst) begin
        if (rst) begin
            key1_cnt   <= 20'd0;
            key1_sync  <= 1'b1;
            key1_prev  <= 1'b1;
            key1_pulse <= 1'b0;
        end else begin
            key1_pulse <= 1'b0;
            if (KEY[1] != key1_sync) begin
                key1_cnt <= key1_cnt + 1'b1;
                if (key1_cnt == 20'hFFFFF) begin
                    key1_sync <= KEY[1];
                    key1_cnt  <= 20'd0;
                end
            end else begin
                key1_cnt <= 20'd0;
            end
            key1_prev <= key1_sync;
            if (key1_prev && !key1_sync)  // falling edge (nhan nut)
                key1_pulse <= 1'b1;
        end
    end

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
    // Buffer Gyro X, Y, Z - 256 mau
    // ========================================
    wire        buf_rd_en;
    wire [15:0] gyro_x_buf, gyro_y_buf, gyro_z_buf;
    wire        buf_full, buf_empty, buf_data_ready;
    wire [8:0]  buf_count;
    
    // Signals for distance calculator
    wire        calc_rd_en;
    wire        calc_busy, calc_done;
    wire [31:0] distance_avg;
    wire        threshold_exceeded;
    
    // Buffer read from distance calculator only
    assign buf_rd_en = calc_rd_en;
    
    buffer_gyro_x_y_z #(
        .BUFFER_DEPTH (256),
        .ADDR_WIDTH   (8)
    ) u_gyro_buffer (
        .clk        (MAX10_CLK1_50),
        .rst        (rst),
        .gyro_x_in  (gyro_x),
        .gyro_y_in  (gyro_y),
        .gyro_z_in  (gyro_z),
        .wr_en      (data_valid && init_done && !error && !calc_busy && !sender_busy),
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
    // Distance Calculator
    // Tinh D = sqrt(X² + Y² + Z²) trung binh
    // ========================================
    distance_calculator #(
        .BUFFER_DEPTH (256),
        .ADDR_WIDTH   (8),
        .THRESHOLD    (1000)
    ) u_distance_calc (
        .clk               (MAX10_CLK1_50),
        .rst               (rst),
        .start_calc        (buf_full && !calc_busy && !sender_busy),
        .busy              (calc_busy),
        .done              (calc_done),
        .buf_rd_en         (calc_rd_en),
        .gyro_x_data       (gyro_x_buf),
        .gyro_y_data       (gyro_y_buf),
        .gyro_z_data       (gyro_z_buf),
        .buf_empty         (buf_empty),
        .buf_count         (buf_count),
        .distance_avg      (distance_avg),
        .threshold_exceeded(threshold_exceeded)
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
    
    // Distance sender signals
    wire       sender_busy;
    wire       sender_done;
    wire       sender_fifo_wr_en;
    wire [7:0] sender_fifo_din;
    
    // FIFO input from distance sender
    assign fifo_wr_en = sender_fifo_wr_en;
    assign fifo_din   = sender_fifo_din;

    fifo_sync #(
        .DATA_WIDTH (8),
        .ADDR_WIDTH (4)   // 16 bytes buffer (du cho "D:xxxxx\r\n")
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
    // UART Distance Sender
    // Gui gia tri D khi vuot nguong
    // ========================================
    uart_distance_sender u_dist_sender (
        .clk            (MAX10_CLK1_50),
        .rst            (rst),
        .start_send     (calc_done && threshold_exceeded && !sender_busy),
        .distance_value (distance_avg),
        .busy           (sender_busy),
        .done           (sender_done),
        .fifo_wr_en     (sender_fifo_wr_en),
        .fifo_din       (sender_fifo_din),
        .fifo_full      (fifo_full)
    );

    // Tie off unused signals
    wire unused_ok;
    assign unused_ok = &{1'b0, error, sender_done, buf_data_ready,
                         accel_x, accel_y, accel_z, init_done,
                         key1_pulse, gyro_x_buf, gyro_y_buf, gyro_z_buf};

endmodule
