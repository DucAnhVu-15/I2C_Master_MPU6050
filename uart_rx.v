// uart_rx.v
// UART RX 8N1
module uart_rx
#(
    parameter DATA_BITS = 8,
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD      = 115200
)
(
    input  wire clk,
    input  wire rst,
    input  wire rx,                     // chan RX ngoai vao
    output reg  [DATA_BITS-1:0] data_out,
    output reg  data_valid              // pulse 1 chu ky khi nhan xong 1 byte
);

    localparam integer BAUD_DIV     = CLK_FREQ / BAUD;
    localparam integer HALF_BAUDDIV = BAUD_DIV / 2;

    localparam IDLE  = 2'b00;
    localparam START = 2'b01;
    localparam DATA  = 2'b10;
    localparam STOP  = 2'b11;

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [3:0]  bit_cnt;
    reg [DATA_BITS-1:0] shifter;

    // dong bo RX
    reg rx_sync1, rx_sync2;
    always @(posedge clk) begin
        rx_sync1 <= rx;
        rx_sync2 <= rx_sync1;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state     <= IDLE;
            cnt       <= 16'd0;
            bit_cnt   <= 4'd0;
            shifter   <= {DATA_BITS{1'b0}};
            data_out  <= {DATA_BITS{1'b0}};
            data_valid<= 1'b0;
        end else begin
            data_valid <= 1'b0;
            case (state)
                IDLE: begin
                    if (rx_sync2 == 1'b0) begin  // start bit
                        state <= START;
                        cnt   <= 16'd0;
                    end
                end

                START: begin
                    if (cnt == HALF_BAUDDIV) begin
                        if (rx_sync2 == 1'b0) begin
                            cnt     <= 16'd0;
                            bit_cnt <= 4'd0;
                            state   <= DATA;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                DATA: begin
                    if (cnt == BAUD_DIV - 1) begin
                        cnt     <= 16'd0;
                        shifter <= {rx_sync2, shifter[DATA_BITS-1:1]};
                        if (bit_cnt == DATA_BITS - 1) begin
                            state <= STOP;
                        end
                        bit_cnt <= bit_cnt + 4'd1;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                STOP: begin
                    if (cnt == BAUD_DIV - 1) begin
                        cnt        <= 16'd0;
                        data_out   <= shifter;
                        data_valid <= 1'b1;
                        state      <= IDLE;
                    end else begin
                        cnt <= cnt + 16'd1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
