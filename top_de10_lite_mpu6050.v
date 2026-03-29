module top_de10_lite_mpu6050 (
    input  wire        MAX10_CLK1_50,
    input  wire [1:0]  KEY,
    inout  wire        I2C_SDA,
    inout  wire        I2C_SCL
);

    wire [15:0] accel_x;
    wire [15:0] accel_y;
    wire [15:0] accel_z;
    wire        data_valid;
    wire        init_done;
    wire        error;

    mpu6050_i2c #(
        .divider      (500),       // 50MHz/500 = 100kHz (giam tu 400kHz)
        .sample_delay (500000)
    ) u_mpu6050_i2c (
        .clk_sys    (MAX10_CLK1_50),
        .rst        (KEY[0]),
        .accel_x    (accel_x),
        .accel_y    (accel_y),
        .accel_z    (accel_z),
        .data_valid (data_valid),
        .init_done  (init_done),
        .error      (error),
        .sda        (I2C_SDA),
        .scl        (I2C_SCL)
    );

    // Khong dung cac output debug tren kit, chi can giao tiep I2C.
    // Tie off de tranh canh bao synthesis ve tin hieu khong su dung.
    wire unused_ok;
    assign unused_ok = &{1'b0, accel_x, accel_y, accel_z, data_valid, init_done, error};

endmodule
