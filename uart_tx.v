// uart_tx.v
// UART TX 8N1
module uart_tx
#(
    parameter DATA_BITS = 8
)
(
    input  wire clk,
    input  wire rst,
    input  wire baud_tick,                 // xung baud
    input  wire [DATA_BITS-1:0] data_in,   // byte can gui
    input  wire data_valid,                // pulse 1 chu ky de bat dau gui
    output reg  tx,                        // chan TX
    output reg  busy                       // = 1 khi dang gui
);

    localparam IDLE  = 2'b00;
    localparam START = 2'b01;
    localparam DATA  = 2'b10;
    localparam STOP  = 2'b11;

    reg [1:0] state;
    reg [DATA_BITS-1:0] shifter;
    reg [2:0] bit_cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state   <= IDLE;
            tx      <= 1'b1;  // idle = 1
            busy    <= 1'b0;
            shifter <= {DATA_BITS{1'b0}};
            bit_cnt <= 3'd0;
        end else begin
            case (state)
                IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    if (data_valid) begin
                        busy    <= 1'b1;
                        shifter <= data_in;
                        bit_cnt <= 3'd0;
                        state   <= START;
                    end
                end

                START: begin
                    if (baud_tick) begin
                        tx    <= 1'b0;   // start bit
                        state <= DATA;
                    end
                end

                DATA: begin
                    if (baud_tick) begin
                        tx      <= shifter[0];
                        shifter <= {1'b0, shifter[DATA_BITS-1:1]};
                        if (bit_cnt == DATA_BITS - 1) begin
                            state <= STOP;
                        end
                        bit_cnt <= bit_cnt + 3'd1;
                    end
                end

                STOP: begin
                    if (baud_tick) begin
                        tx    <= 1'b1;  // stop bit
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
