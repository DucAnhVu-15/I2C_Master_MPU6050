// I2C Master Controller for MPU6050
// Fixed version - corrections listed below:
//
// [FIX 1] RECEIVER_ACK: sample ack_flag tai dung dinh SCL (divider/2-1)
//         thay vi sample lien tuc tu divider/4 tro di (co the bat noise)
//
// [FIX 2] STOP: SCL phai len HIGH truoc, sau do SDA moi len HIGH
//         Code goc: clock_divider==divider/4-1 -> scl_out<=0  (SAI)
//         Fixed:    clock_divider==divider/4-1 -> scl_out<=1  (DUNG)
//
// [FIX 3] COMPLETE: giu SCL=1 (bus free) thay vi SCL=0
//         De dam bao bus duoc giai phong sau khi giao dich hoan tat
//
// [FIX 4] STOP: them hold time day du (them 1 phase) truoc khi vao IDLE
//         SCL rise -> hold div/4 -> SDA rise -> hold div/4 -> IDLE
//
// [FIX 5] RECEIVER_ACK: reset ack_flag ngay khi vao state,
//         tranh ack_flag cu tu chu ky truoc anh huong ket qua

module i2c_master #(
    parameter divider        = 256,
    parameter MPU6050_ADDR   = 7'h68,
    parameter READ_BYTES     = 6
) (
    input  wire                    clk_sys,
    input  wire                    rst,
    input  wire [7:0]              reg_addr,
    input  wire [7:0]              data_wr,
    input  wire                    r_w,
    input  wire                    startfr,
    input  wire                    stopfr,
    inout  wire                    sda,
    inout  wire                    scl,
    output wire [(READ_BYTES*8)-1:0] data_rd,
    output reg                     writebytedone,
    output reg                     readbytedone,
    output reg                     ack_error,
    output reg                     done
);

    // ----------------------------------------------------------------
    // State encoding
    // ----------------------------------------------------------------
    localparam IDLE             = 4'd0;
    localparam START            = 4'd1;
    localparam REPEATED_START   = 4'd2;
    localparam WRITE_BYTE       = 4'd3;
    localparam RECEIVER_ACK     = 4'd4;
    localparam READ_BYTE        = 4'd5;
    localparam SEND_MASTER_ACK  = 4'd6;
    localparam SEND_MASTER_NACK = 4'd7;
    localparam STOP             = 4'd8;
    localparam COMPLETE         = 4'd9;

    // ----------------------------------------------------------------
    // Transaction phase encoding
    // ----------------------------------------------------------------
    localparam PHASE_DEV_ADDR_W = 2'd0;
    localparam PHASE_REG_ADDR   = 2'd1;
    localparam PHASE_DEV_ADDR_R = 2'd2;
    localparam PHASE_WRITE_DATA = 2'd3;

    // ----------------------------------------------------------------
    // Registers
    // ----------------------------------------------------------------
    reg [3:0]  state;
    reg [2:0]  bit_cnt;
    reg [4:0]  read_byte_cnt;
    reg [1:0]  phase;
    reg [7:0]  tx_byte;
    reg [7:0]  reg_addr_latched;
    reg [7:0]  data_wr_latched;
    reg        op_read;
    reg        stop_latched;
    reg [(READ_BYTES*8)-1:0] data_rd_reg;
    reg [15:0] clock_divider;
    reg        sda_out;
    reg        sda_en;
    reg        scl_out;
    reg        scl_en;
    reg        ack_flag;

    // ----------------------------------------------------------------
    // Open-drain outputs
    // SDA/SCL: chi keo xuong 0 khi en=1 va out=0
    //          nguoc lai tha (high-Z) de pull-up keo len 1
    // ----------------------------------------------------------------
    assign sda = (sda_en && !sda_out) ? 1'b0 : 1'bz;
    assign scl = (scl_en && !scl_out) ? 1'b0 : 1'bz;
    assign data_rd = data_rd_reg;

    // ----------------------------------------------------------------
    // Main FSM
    // ----------------------------------------------------------------
    always @(posedge clk_sys or negedge rst) begin
        if (!rst) begin
            state          <= IDLE;
            bit_cnt        <= 3'd7;
            read_byte_cnt  <= 5'd0;
            phase          <= PHASE_DEV_ADDR_W;
            tx_byte        <= 8'h00;
            reg_addr_latched <= 8'h00;
            data_wr_latched  <= 8'h00;
            op_read        <= 1'b0;
            stop_latched   <= 1'b0;
            data_rd_reg    <= {(READ_BYTES*8){1'b0}};
            clock_divider  <= 16'd0;
            sda_out        <= 1'b1;
            sda_en         <= 1'b1;
            scl_out        <= 1'b1;
            scl_en         <= 1'b1;
            ack_flag       <= 1'b0;
            writebytedone  <= 1'b0;
            readbytedone   <= 1'b0;
            ack_error      <= 1'b0;
            done           <= 1'b0;
        end else begin
            case (state)

                // --------------------------------------------------------
                // IDLE
                // Bus free: SCL=1, SDA=1
                // Cho tin hieu startfr de bat dau giao dich
                // --------------------------------------------------------
                IDLE: begin
                    done          <= 1'b0;
                    writebytedone <= 1'b0;
                    readbytedone  <= 1'b0;
                    ack_error     <= 1'b0;
                    ack_flag      <= 1'b0;
                    clock_divider <= 16'd0;
                    bit_cnt       <= 3'd7;
                    read_byte_cnt <= 5'd0;
                    sda_en        <= 1'b1;
                    sda_out       <= 1'b1;   // SDA = 1 (bus free)
                    scl_en        <= 1'b1;
                    scl_out       <= 1'b1;   // SCL = 1 (bus free)

                    if (startfr) begin
                        reg_addr_latched <= reg_addr;
                        data_wr_latched  <= data_wr;
                        op_read          <= r_w;
                        stop_latched     <= stopfr;
                        tx_byte          <= {MPU6050_ADDR, 1'b0};
                        phase            <= PHASE_DEV_ADDR_W;
                        data_rd_reg      <= {(READ_BYTES*8){1'b0}};
                        state            <= START;
                    end
                end

                // --------------------------------------------------------
                // START
                // Tao dieu kien START: SDA xuong trong khi SCL = 1
                //
                // Timeline (1 don vi = 1 clock tick):
                //   div==0           : SDA xuong 0 (SCL van 1 tu IDLE)
                //   div==divider/4-1 : SCL xuong 0 (bat dau gui bit)
                //   div==divider/2-1 : reset div, chuyen WRITE_BYTE
                // --------------------------------------------------------
                START: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b0;                  // START: SDA fall while SCL=1
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b0;                  // SCL fall -> chuan bi gui bit
                    else if (clock_divider == divider/2 - 1) begin
                        clock_divider <= 16'd0;
                        state         <= WRITE_BYTE;
                    end
                end

                // --------------------------------------------------------
                // REPEATED_START
                // Tao Repeated START giua read va write trong 1 giao dich
                //
                // Timeline (SCL dang o muc thap khi vao state nay):
                //   div==0           : SDA len 1, SCL giu 0
                //   div==divider/4-1 : SCL len 1 (SCL=1, SDA=1)
                //   div==divider/2-1 : SDA xuong 0 (START condition)
                //   div==3*div/4-1   : SCL xuong 0 (chuan bi gui bit)
                //   div==divider-1   : reset, chuyen WRITE_BYTE
                // --------------------------------------------------------
                REPEATED_START: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0) begin
                        sda_en  <= 1'b1;
                        sda_out <= 1'b1;                  // SDA len 1
                        scl_out <= 1'b0;                  // SCL giu thap
                    end else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;                  // SCL len 1 (SCL=SDA=1)
                    else if (clock_divider == divider/2 - 1)
                        sda_out <= 1'b0;                  // SDA xuong -> START
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;                  // SCL xuong -> chuan bi bit
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        state         <= WRITE_BYTE;
                    end
                end

                // --------------------------------------------------------
                // WRITE_BYTE
                // Gui 8 bit (MSB first) cua tx_byte
                //
                // Timeline moi bit:
                //   div==0           : SDA = tx_byte[bit_cnt]
                //   div==divider/4-1 : SCL len 1  (slave sample SDA)
                //   div==3*div/4-1   : SCL xuong 0
                //   div==divider-1   : next bit hoac sang RECEIVER_ACK
                // --------------------------------------------------------
                WRITE_BYTE: begin
                    writebytedone <= 1'b0;
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= tx_byte[bit_cnt];      // setup SDA truoc SCL rise
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;                  // SCL len -> slave sample
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;                  // SCL xuong
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        if (bit_cnt > 0) begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end else begin
                            bit_cnt  <= 3'd7;
                            sda_en   <= 1'b0;             // tha SDA de slave drive ACK
                            ack_flag <= 1'b0;             // [FIX 5] xoa flag cu
                            state    <= RECEIVER_ACK;
                        end
                    end
                end

                // --------------------------------------------------------
                // RECEIVER_ACK
                // Doc ACK/NACK tu slave
                //
                // [FIX 1] Sample SDA tai dung dinh SCL (divider/2-1)
                //         thay vi sample lien tuc tu divider/4-1 tro di.
                //         Ly do: tranh bat noise, dam bao SDA da on dinh.
                //
                // Timeline:
                //   div==divider/4-1 : SCL len 1
                //   div==divider/2-1 : sample SDA (DUNG DINH SCL) <- FIX
                //   div==3*div/4-1   : SCL xuong 0
                //   div==divider-1   : xu ly ACK/NACK, chuyen state
                // --------------------------------------------------------
                RECEIVER_ACK: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;                  // SCL len
                    else if (clock_divider == divider/2 - 1) begin
                        // [FIX 1] Sample tai dinh SCL, SDA da on dinh
                        if (sda == 1'b0)
                            ack_flag <= 1'b1;             // ACK nhan duoc
                    end else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;                  // SCL xuong
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;

                        if (ack_flag) begin
                            // ---- ACK nhan duoc: tiep tuc giao dich ----
                            ack_flag <= 1'b0;

                            case (phase)
                                PHASE_DEV_ADDR_W: begin
                                    // Gui dia chi register
                                    tx_byte  <= reg_addr_latched;
                                    phase    <= PHASE_REG_ADDR;
                                    sda_en   <= 1'b1;
                                    state    <= WRITE_BYTE;
                                end

                                PHASE_REG_ADDR: begin
                                    if (op_read) begin
                                        // Doc: gui Repeated START + addr+R
                                        tx_byte  <= {MPU6050_ADDR, 1'b1};
                                        phase    <= PHASE_DEV_ADDR_R;
                                        sda_en   <= 1'b1;
                                        sda_out  <= 1'b1;
                                        state    <= REPEATED_START;
                                    end else begin
                                        // Ghi: gui data byte
                                        tx_byte  <= data_wr_latched;
                                        phase    <= PHASE_WRITE_DATA;
                                        sda_en   <= 1'b1;
                                        state    <= WRITE_BYTE;
                                    end
                                end

                                PHASE_DEV_ADDR_R: begin
                                    // Chuyen sang doc du lieu
                                    sda_en <= 1'b0;
                                    state  <= READ_BYTE;
                                end

                                PHASE_WRITE_DATA: begin
                                    // Ghi hoan tat
                                    writebytedone <= 1'b1;
                                    sda_en        <= 1'b1;
                                    sda_out       <= 1'b1;
                                    if (stop_latched)
                                        state <= STOP;
                                    else
                                        state <= COMPLETE;
                                end

                                default: state <= IDLE;
                            endcase

                        end else begin
                            // ---- NACK hoac khong co ACK: bao loi ----
                            ack_error <= 1'b1;
                            sda_en    <= 1'b1;
                            sda_out   <= 1'b1;
                            if (stop_latched)
                                state <= STOP;
                            else
                                state <= COMPLETE;
                        end
                    end
                end

                // --------------------------------------------------------
                // READ_BYTE
                // Doc 1 byte tu slave (MSB first)
                // SDA duoc tha (sda_en=0) de slave drive
                //
                // Timeline moi bit:
                //   div==divider/4-1 : SCL len 1
                //   div==divider/2-1 : sample SDA (giua xung SCL)
                //   div==3*div/4-1   : SCL xuong 0
                //   div==divider-1   : next bit hoac sang ACK/NACK
                //
                // Layout data_rd_reg (MSB first, byte 0 o bit cao nhat):
                //   Byte 0: [(READ_BYTES*8-1) : (READ_BYTES-1)*8]
                //   Byte N: bit = ((READ_BYTES-1-read_byte_cnt)*8 + bit_cnt)
                // --------------------------------------------------------
                READ_BYTE: begin
                    readbytedone  <= 1'b0;
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider/2 - 1)
                        data_rd_reg[((READ_BYTES - 1 - read_byte_cnt) * 8) + bit_cnt] <= sda;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        if (bit_cnt > 0) begin
                            bit_cnt <= bit_cnt - 1'b1;
                        end else begin
                            bit_cnt <= 3'd7;
                            sda_en  <= 1'b1;
                            if (read_byte_cnt < READ_BYTES - 1) begin
                                read_byte_cnt <= read_byte_cnt + 1'b1;
                                state         <= SEND_MASTER_ACK;
                            end else begin
                                readbytedone <= 1'b1;
                                state        <= SEND_MASTER_NACK;
                            end
                        end
                    end
                end

                // --------------------------------------------------------
                // SEND_MASTER_ACK
                // Master gui ACK sau moi byte doc (tru byte cuoi)
                // Keo SDA = 0 trong 1 SCL cycle
                // --------------------------------------------------------
                SEND_MASTER_ACK: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b0;                  // SDA = 0 (ACK)
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        sda_en        <= 1'b0;            // tha SDA, slave se drive
                        state         <= READ_BYTE;
                    end
                end

                // --------------------------------------------------------
                // SEND_MASTER_NACK
                // Master gui NACK sau byte cuoi cung
                // Tha SDA = 1 trong 1 SCL cycle
                // --------------------------------------------------------
                SEND_MASTER_NACK: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0)
                        sda_out <= 1'b1;                  // SDA = 1 (NACK)
                    else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;
                    else if (clock_divider == divider*3/4 - 1)
                        scl_out <= 1'b0;
                    else if (clock_divider == divider - 1) begin
                        clock_divider <= 16'd0;
                        sda_en        <= 1'b1;
                        if (stop_latched)
                            state <= STOP;
                        else
                            state <= COMPLETE;
                    end
                end

                // --------------------------------------------------------
                // COMPLETE
                // Giao dich hoan tat, khong phat STOP (bus giu cho master)
                //
                // [FIX 3] scl_out <= 1 (bus free: SCL cao)
                //         Code goc giu scl_out=0 -> bus bi chiem giu sai
                //
                // Cho lenh tiep theo:
                //   - stopfr: phat STOP roi vao IDLE
                //   - startfr: phat Repeated START cho giao dich moi
                // --------------------------------------------------------
                COMPLETE: begin
                    done          <= 1'b1;
                    clock_divider <= 16'd0;
                    sda_en        <= 1'b1;
                    sda_out       <= 1'b1;
                    scl_en        <= 1'b1;
                    scl_out       <= 1'b1;                // [FIX 3] SCL=1 (bus free)

                    if (stopfr) begin
                        done  <= 1'b0;
                        state <= STOP;
                    end else if (startfr) begin
                        done          <= 1'b0;
                        writebytedone <= 1'b0;
                        readbytedone  <= 1'b0;
                        ack_error     <= 1'b0;
                        reg_addr_latched <= reg_addr;
                        data_wr_latched  <= data_wr;
                        op_read       <= r_w;
                        stop_latched  <= stopfr;
                        tx_byte       <= {MPU6050_ADDR, 1'b0};
                        phase         <= PHASE_DEV_ADDR_W;
                        data_rd_reg   <= {(READ_BYTES*8){1'b0}};
                        state         <= REPEATED_START;
                    end
                end

                // --------------------------------------------------------
                // STOP
                // Phat dieu kien STOP: SDA len trong khi SCL = 1
                //
                // [FIX 2] Timeline dung I2C spec:
                //   div==0           : SDA = 0 (chuan bi, SCL dang thap)
                //   div==divider/4-1 : SCL len 1  <- FIX: goc la scl=0 (SAI)
                //   div==divider/2-1 : SDA len 1  -> day chinh la STOP condition
                //   div==3*div/4-1   : giu SCL=1 SDA=1 (bus free hold time)
                //   div==divider-1   : done=1, vao IDLE
                //
                // [FIX 4] Them phase hold sau STOP truoc khi vao IDLE
                // --------------------------------------------------------
                STOP: begin
                    clock_divider <= clock_divider + 1'b1;

                    if (clock_divider == 0) begin
                        sda_en  <= 1'b1;
                        sda_out <= 1'b0;                  // SDA = 0 (SCL dang thap)
                    end else if (clock_divider == divider/4 - 1)
                        scl_out <= 1'b1;                  // [FIX 2] SCL len 1
                    else if (clock_divider == divider/2 - 1)
                        sda_out <= 1'b1;                  // SDA len 1 -> STOP
                    else if (clock_divider == divider - 1) begin
                        // [FIX 4] Bus free sau STOP: giu SCL=SDA=1 du hold time
                        clock_divider <= 16'd0;
                        done          <= 1'b1;
                        state         <= IDLE;
                    end
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule