// =============================================================================
// Module: distance_calculator
// Description: Tinh Euclidean Distance trung binh tu buffer gyro
//              D = sqrt(X² + Y² + Z²) cho moi mau, roi tinh trung binh
//              Su dung CORDIC/approximation de tinh sqrt
// =============================================================================
module distance_calculator #(
    parameter BUFFER_DEPTH = 256,
    parameter ADDR_WIDTH   = 8,
    parameter THRESHOLD    = 1000           // Nguong so sanh
)(
    input  wire        clk,
    input  wire        rst,
    
    // Dieu khien
    input  wire        start_calc,          // Bat dau tinh toan
    output reg         busy,                // Dang tinh
    output reg         done,                // Hoan thanh
    
    // Giao dien doc buffer
    output reg         buf_rd_en,
    input  wire [15:0] gyro_x_data,
    input  wire [15:0] gyro_y_data,
    input  wire [15:0] gyro_z_data,
    input  wire        buf_empty,
    input  wire [ADDR_WIDTH:0] buf_count,
    
    // Ket qua
    output reg [31:0]  distance_avg,        // D trung binh
    output reg         threshold_exceeded   // D > THRESHOLD
);

    // =========================================================================
    // Trang thai FSM
    // =========================================================================
    localparam IDLE       = 3'd0;
    localparam READ_SAMPLE = 3'd1;
    localparam WAIT_READ  = 3'd2;
    localparam CALC_SUM   = 3'd3;
    localparam CALC_SQRT  = 3'd4;
    localparam ACCUM      = 3'd5;
    localparam CALC_AVG   = 3'd6;
    localparam FINISH     = 3'd7;
    
    reg [2:0] state;
    reg [ADDR_WIDTH:0] sample_idx;
    reg [ADDR_WIDTH:0] total_samples;
    
    // Latch du lieu
    reg signed [15:0] latch_x, latch_y, latch_z;
    
    // Tinh toan trung gian
    reg [31:0] x_sq, y_sq, z_sq;       // X², Y², Z²
    reg [31:0] sum_sq;                  // X² + Y² + Z²
    reg [31:0] sqrt_result;             // sqrt(sum_sq)
    reg [47:0] distance_sum;            // Tong D cua tat ca mau
    
    // Bien cho sqrt
    reg [4:0]  sqrt_step;
    reg [31:0] sqrt_rem;
    reg [31:0] sqrt_root;
    
    // =========================================================================
    // Tinh gia tri tuyet doi va binh phuong
    // =========================================================================
    wire [15:0] abs_x = latch_x[15] ? (~latch_x + 1'b1) : latch_x;
    wire [15:0] abs_y = latch_y[15] ? (~latch_y + 1'b1) : latch_y;
    wire [15:0] abs_z = latch_z[15] ? (~latch_z + 1'b1) : latch_z;
    
    // =========================================================================
    // FSM chinh
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state              <= IDLE;
            sample_idx         <= 0;
            total_samples      <= 0;
            busy               <= 1'b0;
            done               <= 1'b0;
            buf_rd_en          <= 1'b0;
            latch_x            <= 16'd0;
            latch_y            <= 16'd0;
            latch_z            <= 16'd0;
            x_sq               <= 32'd0;
            y_sq               <= 32'd0;
            z_sq               <= 32'd0;
            sum_sq             <= 32'd0;
            sqrt_result        <= 32'd0;
            distance_sum       <= 48'd0;
            distance_avg       <= 32'd0;
            threshold_exceeded <= 1'b0;
            sqrt_step          <= 5'd0;
            sqrt_rem           <= 32'd0;
            sqrt_root          <= 32'd0;
        end else begin
            buf_rd_en <= 1'b0;
            done      <= 1'b0;
            
            case (state)
                IDLE: begin
                    if (start_calc && !buf_empty) begin
                        busy          <= 1'b1;
                        sample_idx    <= 0;
                        total_samples <= buf_count;
                        distance_sum  <= 48'd0;
                        state         <= READ_SAMPLE;
                    end
                end
                
                READ_SAMPLE: begin
                    if (!buf_empty) begin
                        buf_rd_en <= 1'b1;
                        state     <= WAIT_READ;
                    end else begin
                        state <= CALC_AVG;
                    end
                end
                
                WAIT_READ: begin
                    // Latch du lieu
                    latch_x <= gyro_x_data;
                    latch_y <= gyro_y_data;
                    latch_z <= gyro_z_data;
                    state   <= CALC_SUM;
                end
                
                CALC_SUM: begin
                    // Tinh X² + Y² + Z²
                    x_sq   <= abs_x * abs_x;
                    y_sq   <= abs_y * abs_y;
                    z_sq   <= abs_z * abs_z;
                    state  <= CALC_SQRT;
                    sqrt_step <= 5'd0;
                end
                
                CALC_SQRT: begin
                    // Tinh sqrt bang phuong phap Newton-Raphson don gian
                    // Hoac su dung bit-by-bit method
                    if (sqrt_step == 5'd0) begin
                        sum_sq    <= x_sq + y_sq + z_sq;
                        sqrt_rem  <= 32'd0;
                        sqrt_root <= 32'd0;
                        sqrt_step <= 5'd1;
                    end else if (sqrt_step <= 5'd16) begin
                        // Bit-by-bit sqrt algorithm
                        sqrt_rem <= {sqrt_rem[29:0], sum_sq[31:30]};
                        sum_sq   <= {sum_sq[29:0], 2'b00};
                        
                        if (sqrt_rem >= {sqrt_root, 1'b1}) begin
                            sqrt_rem  <= sqrt_rem - {sqrt_root, 1'b1};
                            sqrt_root <= {sqrt_root[30:0], 1'b1};
                        end else begin
                            sqrt_root <= {sqrt_root[30:0], 1'b0};
                        end
                        sqrt_step <= sqrt_step + 1'b1;
                    end else begin
                        sqrt_result <= sqrt_root;
                        state       <= ACCUM;
                    end
                end
                
                ACCUM: begin
                    // Cong vao tong
                    distance_sum <= distance_sum + sqrt_result;
                    sample_idx   <= sample_idx + 1'b1;
                    
                    if (sample_idx + 1'b1 >= total_samples) begin
                        state <= CALC_AVG;
                    end else begin
                        state <= READ_SAMPLE;
                    end
                end
                
                CALC_AVG: begin
                    // Tinh trung binh: distance_sum / total_samples
                    // Chia cho 256 = shift right 8 bit
                    if (total_samples == 256) begin
                        distance_avg <= distance_sum[39:8];
                    end else begin
                        // Chia don gian cho truong hop khac
                        distance_avg <= distance_sum[31:0] / total_samples;
                    end
                    state <= FINISH;
                end
                
                FINISH: begin
                    threshold_exceeded <= (distance_avg > THRESHOLD);
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
