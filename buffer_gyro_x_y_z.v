// =============================================================================
// Module: buffer_gyro_x_y_z
// Description: 3 bo dem du lieu cho gyro X, Y, Z tu MPU6050
//              Moi bo dem luu tru BUFFER_DEPTH mau 16-bit
// =============================================================================
module buffer_gyro_x_y_z #(
    parameter BUFFER_DEPTH = 256,          // So mau luu tru (power of 2)
    parameter ADDR_WIDTH   = 8             // log2(BUFFER_DEPTH)
)(
    input  wire        clk,
    input  wire        rst,
    
    // Giao dien ghi du lieu (tu mpu6050_i2c)
    input  wire [15:0] gyro_x_in,
    input  wire [15:0] gyro_y_in,
    input  wire [15:0] gyro_z_in,
    input  wire        wr_en,              // Cho phep ghi (data_valid)
    
    // Giao dien doc du lieu
    input  wire        rd_en,              // Cho phep doc
    output wire [15:0] gyro_x_out,
    output wire [15:0] gyro_y_out,
    output wire [15:0] gyro_z_out,
    output wire        data_ready,         // Co du lieu de doc
    
    // Trang thai bo dem
    output wire        full,
    output wire        empty,
    output wire [ADDR_WIDTH:0] count       // So mau hien tai trong buffer
);

    // =========================================================================
    // Bo dem cho Gyro X
    // =========================================================================
    reg [15:0] buffer_x [0:BUFFER_DEPTH-1];
    
    // =========================================================================
    // Bo dem cho Gyro Y
    // =========================================================================
    reg [15:0] buffer_y [0:BUFFER_DEPTH-1];
    
    // =========================================================================
    // Bo dem cho Gyro Z
    // =========================================================================
    reg [15:0] buffer_z [0:BUFFER_DEPTH-1];
    
    // =========================================================================
    // Con tro doc/ghi va bo dem trang thai
    // =========================================================================
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   fifo_count;
    
    // Trang thai
    assign full       = (fifo_count == BUFFER_DEPTH);
    assign empty      = (fifo_count == 0);
    assign count      = fifo_count;
    assign data_ready = !empty;
    
    // Dau ra du lieu
    assign gyro_x_out = buffer_x[rd_ptr];
    assign gyro_y_out = buffer_y[rd_ptr];
    assign gyro_z_out = buffer_z[rd_ptr];
    
    // =========================================================================
    // Logic ghi/doc
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr     <= {ADDR_WIDTH{1'b0}};
            rd_ptr     <= {ADDR_WIDTH{1'b0}};
            fifo_count <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            // Xu ly ghi
            if (wr_en && !full) begin
                buffer_x[wr_ptr] <= gyro_x_in;
                buffer_y[wr_ptr] <= gyro_y_in;
                buffer_z[wr_ptr] <= gyro_z_in;
                wr_ptr <= wr_ptr + 1'b1;
            end
            
            // Xu ly doc
            if (rd_en && !empty) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
            
            // Cap nhat so luong
            case ({wr_en && !full, rd_en && !empty})
                2'b10:   fifo_count <= fifo_count + 1'b1;  // Chi ghi
                2'b01:   fifo_count <= fifo_count - 1'b1;  // Chi doc
                default: fifo_count <= fifo_count;         // Dong thoi hoac khong lam gi
            endcase
        end
    end

endmodule
