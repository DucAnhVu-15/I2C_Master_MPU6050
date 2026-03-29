// baud_gen.v
// Tao xung baud_tick tu clock he thong
module baud_gen
#(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)
(
    input  wire clk,
    input  wire rst,
    output reg  baud_tick      // 1 xung / bit
);

    // so chia
    localparam integer BAUD_DIV = CLK_FREQ / BAUD;

    reg [31:0] cnt;  

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cnt       <= 32'd0;
            baud_tick <= 1'b0;
        end else begin
            if (cnt == BAUD_DIV - 1) begin
                cnt       <= 32'd0;
                baud_tick <= 1'b1;
            end else begin
                cnt       <= cnt + 32'd1;
                baud_tick <= 1'b0;
            end
        end
    end

endmodule
