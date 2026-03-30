// =============================================================================
// Module: uart_distance_sender
// Description: Gui gia tri Distance trung binh qua UART
//              Format: "D:xxxxx\r\n"
// =============================================================================
module uart_distance_sender (
    input  wire        clk,
    input  wire        rst,
    
    // Dieu khien
    input  wire        start_send,
    input  wire [31:0] distance_value,
    output reg         busy,
    output reg         done,
    
    // Giao dien FIFO
    output reg         fifo_wr_en,
    output reg  [7:0]  fifo_din,
    input  wire        fifo_full
);

    // =========================================================================
    // FSM
    // =========================================================================
    localparam IDLE      = 2'd0;
    localparam SEND_CHAR = 2'd1;
    localparam NEXT_CHAR = 2'd2;
    localparam FINISH    = 2'd3;
    
    reg [1:0] state;
    reg [3:0] char_pos;      // 0-8: "D:xxxxx\r\n"
    reg [31:0] latched_val;
    
    // =========================================================================
    // Chuyen so thanh ASCII
    // =========================================================================
    function [7:0] digit_to_ascii;
        input [3:0] d;
        begin
            digit_to_ascii = 8'h30 + d;
        end
    endfunction
    
    // Lay ky tu tai vi tri
    // Format: "D:xxxxx\r\n" (9 chars)
    function [7:0] get_char;
        input [3:0] pos;
        input [31:0] val;
        begin
            case (pos)
                4'd0: get_char = "D";
                4'd1: get_char = ":";
                4'd2: get_char = digit_to_ascii((val / 10000) % 10);
                4'd3: get_char = digit_to_ascii((val / 1000) % 10);
                4'd4: get_char = digit_to_ascii((val / 100) % 10);
                4'd5: get_char = digit_to_ascii((val / 10) % 10);
                4'd6: get_char = digit_to_ascii(val % 10);
                4'd7: get_char = 8'h0D;  // CR
                4'd8: get_char = 8'h0A;  // LF
                default: get_char = " ";
            endcase
        end
    endfunction
    
    // =========================================================================
    // FSM
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= IDLE;
            char_pos    <= 4'd0;
            busy        <= 1'b0;
            done        <= 1'b0;
            fifo_wr_en  <= 1'b0;
            fifo_din    <= 8'd0;
            latched_val <= 32'd0;
        end else begin
            fifo_wr_en <= 1'b0;
            done       <= 1'b0;
            
            case (state)
                IDLE: begin
                    if (start_send) begin
                        busy        <= 1'b1;
                        char_pos    <= 4'd0;
                        latched_val <= distance_value;
                        state       <= SEND_CHAR;
                    end
                end
                
                SEND_CHAR: begin
                    if (!fifo_full) begin
                        fifo_din   <= get_char(char_pos, latched_val);
                        fifo_wr_en <= 1'b1;
                        state      <= NEXT_CHAR;
                    end
                end
                
                NEXT_CHAR: begin
                    if (char_pos == 4'd8) begin
                        state <= FINISH;
                    end else begin
                        char_pos <= char_pos + 1'b1;
                        state    <= SEND_CHAR;
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
