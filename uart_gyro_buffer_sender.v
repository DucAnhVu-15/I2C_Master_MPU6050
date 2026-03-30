// =============================================================================
// Module: uart_gyro_buffer_sender
// Description: Doc du lieu tu buffer gyro va gui qua UART
//              Thu tu gui: Moi mau gui GX -> GY -> GZ roi chuyen mau tiep
//              Format moi dong: "GX[idx]:±xxxxx\r\n"
// =============================================================================
module uart_gyro_buffer_sender #(
    parameter BUFFER_DEPTH = 256,
    parameter ADDR_WIDTH   = 8
)(
    input  wire        clk,
    input  wire        rst,
    
    // Tin hieu dieu khien
    input  wire        start_send,         // Kich hoat gui du lieu
    output reg         busy,               // Dang gui du lieu
    output reg         done,               // Hoan thanh gui
    
    // Giao dien voi buffer gyro
    output reg         buf_rd_en,
    input  wire [15:0] gyro_x_data,
    input  wire [15:0] gyro_y_data,
    input  wire [15:0] gyro_z_data,
    input  wire        buf_empty,
    input  wire [ADDR_WIDTH:0] buf_count,
    
    // Giao dien voi FIFO UART TX
    output reg         fifo_wr_en,
    output reg  [7:0]  fifo_din,
    input  wire        fifo_full
);

    // =========================================================================
    // Trang thai FSM
    // =========================================================================
    localparam IDLE         = 4'd0;
    localparam READ_BUFFER  = 4'd1;
    localparam WAIT_READ    = 4'd2;
    localparam SEND_CHAR    = 4'd3;
    localparam WAIT_FIFO    = 4'd4;
    localparam NEXT_CHAR    = 4'd5;
    localparam NEXT_AXIS    = 4'd6;
    localparam NEXT_SAMPLE  = 4'd7;
    localparam FINISH       = 4'd8;
    
    reg [3:0] state;
    reg [1:0] axis;              // 0=X, 1=Y, 2=Z
    reg [ADDR_WIDTH:0] sample_idx;      // 9 bit de dem toi 256
    reg [ADDR_WIDTH:0] total_samples;   // 9 bit de luu 256
    reg [3:0] char_pos;          // Vi tri ky tu trong dong (0-14)
    
    // Latch du lieu khi doc tu buffer
    reg signed [15:0] latched_x, latched_y, latched_z;
    
    // Message format: "GX[000]:+00000\r\n" = 16 chars
    // Pos: 0=G, 1=X/Y/Z, 2=[, 3-5=idx, 6=], 7=:, 8=sign, 9-13=value, 14=CR, 15=LF
    
    // =========================================================================
    // Chuyen so thanh ASCII
    // =========================================================================
    function [7:0] digit_to_ascii;
        input [3:0] d;
        begin
            digit_to_ascii = 8'h30 + d;
        end
    endfunction
    
    // Lay ky tu tuong ung vi tri trong message
    function [7:0] get_char;
        input [3:0] pos;
        input [1:0] ax;
        input [ADDR_WIDTH:0] idx;
        input signed [15:0] val;
        reg [15:0] abs_val;
        begin
            abs_val = (val[15]) ? (~val + 1'b1) : val;
            case (pos)
                4'd0:  get_char = "G";
                4'd1:  begin
                    case (ax)
                        2'd0: get_char = "X";
                        2'd1: get_char = "Y";
                        2'd2: get_char = "Z";
                        default: get_char = "?";
                    endcase
                end
                4'd2:  get_char = "[";
                4'd3:  get_char = digit_to_ascii((idx / 100) % 10);
                4'd4:  get_char = digit_to_ascii((idx / 10) % 10);
                4'd5:  get_char = digit_to_ascii(idx % 10);
                4'd6:  get_char = "]";
                4'd7:  get_char = ":";
                4'd8:  get_char = (val[15]) ? "-" : "+";
                4'd9:  get_char = digit_to_ascii((abs_val / 10000) % 10);
                4'd10: get_char = digit_to_ascii((abs_val / 1000) % 10);
                4'd11: get_char = digit_to_ascii((abs_val / 100) % 10);
                4'd12: get_char = digit_to_ascii((abs_val / 10) % 10);
                4'd13: get_char = digit_to_ascii(abs_val % 10);
                4'd14: get_char = 8'h0D;  // CR
                4'd15: get_char = 8'h0A;  // LF
                default: get_char = " ";
            endcase
        end
    endfunction
    
    // Chon gia tri theo truc
    reg signed [15:0] current_value;
    always @(*) begin
        case (axis)
            2'd0: current_value = latched_x;
            2'd1: current_value = latched_y;
            2'd2: current_value = latched_z;
            default: current_value = 16'd0;
        endcase
    end
    
    // =========================================================================
    // FSM chinh
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state         <= IDLE;
            axis          <= 2'd0;
            sample_idx    <= {(ADDR_WIDTH+1){1'b0}};
            total_samples <= {(ADDR_WIDTH+1){1'b0}};
            char_pos      <= 4'd0;
            busy          <= 1'b0;
            done          <= 1'b0;
            buf_rd_en     <= 1'b0;
            fifo_wr_en    <= 1'b0;
            fifo_din      <= 8'd0;
            latched_x     <= 16'd0;
            latched_y     <= 16'd0;
            latched_z     <= 16'd0;
        end else begin
            // Mac dinh
            buf_rd_en  <= 1'b0;
            fifo_wr_en <= 1'b0;
            done       <= 1'b0;
            
            case (state)
                IDLE: begin
                    if (start_send && !buf_empty) begin
                        busy          <= 1'b1;
                        axis          <= 2'd0;
                        sample_idx    <= {(ADDR_WIDTH+1){1'b0}};
                        total_samples <= buf_count;  // Lay du 9 bit
                        state         <= READ_BUFFER;
                    end
                end
                
                READ_BUFFER: begin
                    if (!buf_empty) begin
                        buf_rd_en <= 1'b1;
                        state     <= WAIT_READ;
                    end else begin
                        state <= FINISH;
                    end
                end
                
                WAIT_READ: begin
                    // Cho 1 cycle de du lieu stable
                    latched_x <= gyro_x_data;
                    latched_y <= gyro_y_data;
                    latched_z <= gyro_z_data;
                    char_pos  <= 4'd0;
                    axis      <= 2'd0;
                    state     <= SEND_CHAR;
                end
                
                SEND_CHAR: begin
                    if (!fifo_full) begin
                        fifo_din   <= get_char(char_pos, axis, sample_idx, current_value);
                        fifo_wr_en <= 1'b1;
                        state      <= NEXT_CHAR;
                    end
                end
                
                NEXT_CHAR: begin
                    if (char_pos == 4'd15) begin
                        // Xong 1 dong, chuyen truc
                        state <= NEXT_AXIS;
                    end else begin
                        char_pos <= char_pos + 1'b1;
                        state    <= SEND_CHAR;
                    end
                end
                
                NEXT_AXIS: begin
                    char_pos <= 4'd0;
                    if (axis == 2'd2) begin
                        // Da gui xong GX, GY, GZ -> chuyen mau tiep
                        state <= NEXT_SAMPLE;
                    end else begin
                        axis  <= axis + 1'b1;
                        state <= SEND_CHAR;
                    end
                end
                
                NEXT_SAMPLE: begin
                    if (sample_idx + 1'b1 >= total_samples) begin
                        state <= FINISH;
                    end else begin
                        sample_idx <= sample_idx + 1'b1;
                        axis       <= 2'd0;
                        state      <= READ_BUFFER;
                    end
                end
                
                FINISH: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
