// fifo_sync.v
module fifo_sync
#(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 4    
)
(
    input  wire clk,
    input  wire rst,
    input  wire wr_en,
    input  wire rd_en,
    input  wire [DATA_WIDTH-1:0] din,
    output wire [DATA_WIDTH-1:0] dout,
    output wire full,
    output wire empty,
    output reg  [ADDR_WIDTH:0] count 
);

    localparam DEPTH = (1 << ADDR_WIDTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] w_ptr;
    reg [ADDR_WIDTH-1:0] r_ptr;

    assign dout  = mem[r_ptr];
    assign full  = (count == DEPTH);
    assign empty = (count == 0);

    // Tinh toan thuc su co ghi/doc hay khong
    wire do_write = wr_en && !full;
    wire do_read  = rd_en && !empty;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            w_ptr <= {ADDR_WIDTH{1'b0}};
            r_ptr <= {ADDR_WIDTH{1'b0}};
            count <= {ADDR_WIDTH+1{1'b0}};
        end else begin
            // Ghi
            if (do_write) begin
                mem[w_ptr] <= din;
                w_ptr      <= w_ptr + 1'b1;
            end

            // Doc
            if (do_read) begin
                r_ptr <= r_ptr + 1'b1;
            end

            // Cap nhat count - xu ly dong thoi read/write
            if (do_write && !do_read)
                count <= count + 1'b1;
            else if (do_read && !do_write)
                count <= count - 1'b1;
            // Neu ca hai hoac khong co gi -> count giu nguyen
        end
    end

endmodule
