// =============================================================================
// Module: i2c_slave
// Description: I2C Slave nhan cau hinh tu MCU
//              - Cau hinh MPU6050 (sample rate, gyro/accel range)
//              - Threshold cho Distance
//              - So luong phan tu trong buffer
//              - Enable/Disable he thong
// =============================================================================
module i2c_slave #(
    parameter SLAVE_ADDR = 7'h50           // Dia chi I2C cua FPGA
)(
    input  wire        clk,
    input  wire        rst,
    
    // I2C interface
    inout  wire        sda,
    input  wire        scl,
    
    // Output IRQ to MCU
    output reg         irq,
    
    // Cau hinh outputs
    output reg         enable,             // Enable he thong
    output reg         trigger,            // Trigger bat dau doc/tinh
    
    // MPU6050 config
    output reg [7:0]   cfg_sample_div,     // SMPLRT_DIV register
    output reg [2:0]   cfg_dlpf,           // DLPF config
    output reg [1:0]   cfg_gyro_fs,        // Gyro full scale
    output reg [1:0]   cfg_accel_fs,       // Accel full scale
    
    // Distance config
    output reg [15:0]  cfg_threshold,      // Nguong Distance
    output reg [8:0]   cfg_buffer_size,    // So phan tu buffer (max 512)
    
    // Status inputs (de MCU doc)
    input  wire        calc_done,          // Tinh xong Distance
    input  wire        threshold_exceeded, // Vuot nguong
    input  wire [31:0] distance_value,     // Gia tri Distance
    input  wire [8:0]  buffer_count        // So mau hien tai trong buffer
);

    // =========================================================================
    // Register Map (8-bit address, 8-bit data)
    // =========================================================================
    // Addr | Name           | R/W | Description
    // -----|----------------|-----|------------------------------------------
    // 0x00 | CTRL           | R/W | [0]=Enable, [1]=Trigger, [7:2]=Reserved
    // 0x01 | STATUS         | R   | [0]=calc_done, [1]=threshold_exceeded
    // 0x02 | IRQ_CTRL       | R/W | [0]=IRQ enable, [1]=Clear IRQ
    // 0x03 | SAMPLE_DIV     | R/W | MPU6050 SMPLRT_DIV
    // 0x04 | MPU_CFG        | R/W | [2:0]=DLPF, [4:3]=GYRO_FS, [6:5]=ACCEL_FS
    // 0x05 | THRESHOLD_L    | R/W | Threshold low byte
    // 0x06 | THRESHOLD_H    | R/W | Threshold high byte
    // 0x07 | BUFFER_SIZE_L  | R/W | Buffer size low byte
    // 0x08 | BUFFER_SIZE_H  | R/W | Buffer size high byte [0] only
    // 0x09 | BUFFER_CNT_L   | R   | Current buffer count low byte
    // 0x0A | BUFFER_CNT_H   | R   | Current buffer count high byte
    // 0x0B | DISTANCE_0     | R   | Distance byte 0 (LSB)
    // 0x0C | DISTANCE_1     | R   | Distance byte 1
    // 0x0D | DISTANCE_2     | R   | Distance byte 2
    // 0x0E | DISTANCE_3     | R   | Distance byte 3 (MSB)
    // =========================================================================
    
    localparam REG_CTRL          = 8'h00;
    localparam REG_STATUS        = 8'h01;
    localparam REG_IRQ_CTRL      = 8'h02;
    localparam REG_SAMPLE_DIV    = 8'h03;
    localparam REG_MPU_CFG       = 8'h04;
    localparam REG_THRESHOLD_L   = 8'h05;
    localparam REG_THRESHOLD_H   = 8'h06;
    localparam REG_BUFFER_SIZE_L = 8'h07;
    localparam REG_BUFFER_SIZE_H = 8'h08;
    localparam REG_BUFFER_CNT_L  = 8'h09;
    localparam REG_BUFFER_CNT_H  = 8'h0A;
    localparam REG_DISTANCE_0    = 8'h0B;
    localparam REG_DISTANCE_1    = 8'h0C;
    localparam REG_DISTANCE_2    = 8'h0D;
    localparam REG_DISTANCE_3    = 8'h0E;
    
    // =========================================================================
    // I2C FSM
    // =========================================================================
    localparam I2C_IDLE       = 4'd0;
    localparam I2C_ADDR       = 4'd1;
    localparam I2C_ADDR_ACK   = 4'd2;
    localparam I2C_REG_ADDR   = 4'd3;
    localparam I2C_REG_ACK    = 4'd4;
    localparam I2C_WRITE_DATA = 4'd5;
    localparam I2C_WRITE_ACK  = 4'd6;
    localparam I2C_READ_DATA  = 4'd7;
    localparam I2C_READ_ACK   = 4'd8;
    
    reg [3:0]  state;
    reg [3:0]  bit_cnt;
    reg [7:0]  shift_reg;
    reg [7:0]  reg_addr;
    reg        rw_bit;           // 0=Write, 1=Read
    reg        addr_match;
    reg        sda_out;
    reg        sda_oe;           // Output enable for SDA
    
    // Registers
    reg        irq_en;
    reg        irq_flag;
    
    // SCL edge detection
    reg [2:0]  scl_sync;
    reg [2:0]  sda_sync;
    wire       scl_posedge = (scl_sync[2:1] == 2'b01);
    wire       scl_negedge = (scl_sync[2:1] == 2'b10);
    wire       sda_posedge = (sda_sync[2:1] == 2'b01);
    wire       sda_negedge = (sda_sync[2:1] == 2'b10);
    wire       scl_high    = scl_sync[1];
    wire       sda_val     = sda_sync[1];
    
    // Start/Stop detection
    wire       start_cond = scl_high && sda_negedge;
    wire       stop_cond  = scl_high && sda_posedge;
    
    // SDA tristate
    assign sda = sda_oe ? sda_out : 1'bz;
    
    // =========================================================================
    // Synchronizers
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl};
            sda_sync <= {sda_sync[1:0], sda};
        end
    end
    
    // =========================================================================
    // Read register value
    // =========================================================================
    function [7:0] read_reg;
        input [7:0] addr;
        begin
            case (addr)
                REG_CTRL:          read_reg = {6'b0, trigger, enable};
                REG_STATUS:        read_reg = {6'b0, threshold_exceeded, calc_done};
                REG_IRQ_CTRL:      read_reg = {6'b0, irq_flag, irq_en};
                REG_SAMPLE_DIV:    read_reg = cfg_sample_div;
                REG_MPU_CFG:       read_reg = {1'b0, cfg_accel_fs, cfg_gyro_fs, cfg_dlpf};
                REG_THRESHOLD_L:   read_reg = cfg_threshold[7:0];
                REG_THRESHOLD_H:   read_reg = cfg_threshold[15:8];
                REG_BUFFER_SIZE_L: read_reg = cfg_buffer_size[7:0];
                REG_BUFFER_SIZE_H: read_reg = {7'b0, cfg_buffer_size[8]};
                REG_BUFFER_CNT_L:  read_reg = buffer_count[7:0];
                REG_BUFFER_CNT_H:  read_reg = {7'b0, buffer_count[8]};
                REG_DISTANCE_0:    read_reg = distance_value[7:0];
                REG_DISTANCE_1:    read_reg = distance_value[15:8];
                REG_DISTANCE_2:    read_reg = distance_value[23:16];
                REG_DISTANCE_3:    read_reg = distance_value[31:24];
                default:           read_reg = 8'hFF;
            endcase
        end
    endfunction
    
    // =========================================================================
    // I2C FSM
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state           <= I2C_IDLE;
            bit_cnt         <= 4'd0;
            shift_reg       <= 8'd0;
            reg_addr        <= 8'd0;
            rw_bit          <= 1'b0;
            addr_match      <= 1'b0;
            sda_out         <= 1'b1;
            sda_oe          <= 1'b0;
            
            // Default config
            enable          <= 1'b0;
            trigger         <= 1'b0;
            cfg_sample_div  <= 8'd7;        // Default: Fs = 1kHz / 8 = 125Hz
            cfg_dlpf        <= 3'd3;        // Default: DLPF ~44Hz
            cfg_gyro_fs     <= 2'd0;        // Default: ±250 dps
            cfg_accel_fs    <= 2'd0;        // Default: ±2g
            cfg_threshold   <= 16'd1000;    // Default threshold
            cfg_buffer_size <= 9'd256;      // Default 256 samples
            
            irq             <= 1'b0;
            irq_en          <= 1'b0;
            irq_flag        <= 1'b0;
        end else begin
            // Clear trigger after 1 cycle
            trigger <= 1'b0;
            
            // IRQ generation
            if (irq_en && threshold_exceeded && calc_done) begin
                irq_flag <= 1'b1;
                irq      <= 1'b1;
            end
            
            // Start condition - reset FSM
            if (start_cond) begin
                state    <= I2C_ADDR;
                bit_cnt  <= 4'd0;
                sda_oe   <= 1'b0;
            end
            // Stop condition
            else if (stop_cond) begin
                state   <= I2C_IDLE;
                sda_oe  <= 1'b0;
            end
            else begin
                case (state)
                    I2C_IDLE: begin
                        sda_oe <= 1'b0;
                    end
                    
                    I2C_ADDR: begin
                        if (scl_posedge) begin
                            shift_reg <= {shift_reg[6:0], sda_val};
                            bit_cnt   <= bit_cnt + 1'b1;
                            
                            if (bit_cnt == 4'd7) begin
                                // Check address match
                                if (shift_reg[6:0] == SLAVE_ADDR) begin
                                    addr_match <= 1'b1;
                                    rw_bit     <= sda_val;  // R/W bit
                                    state      <= I2C_ADDR_ACK;
                                end else begin
                                    addr_match <= 1'b0;
                                    state      <= I2C_IDLE;
                                end
                            end
                        end
                    end
                    
                    I2C_ADDR_ACK: begin
                        if (scl_negedge) begin
                            sda_out <= 1'b0;  // ACK
                            sda_oe  <= 1'b1;
                            bit_cnt <= 4'd0;
                            
                            if (rw_bit) begin
                                // Read mode - load data
                                shift_reg <= read_reg(reg_addr);
                                state     <= I2C_READ_DATA;
                            end else begin
                                state <= I2C_REG_ADDR;
                            end
                        end
                    end
                    
                    I2C_REG_ADDR: begin
                        if (scl_negedge && bit_cnt == 4'd0) begin
                            sda_oe <= 1'b0;  // Release SDA
                        end
                        
                        if (scl_posedge) begin
                            shift_reg <= {shift_reg[6:0], sda_val};
                            bit_cnt   <= bit_cnt + 1'b1;
                            
                            if (bit_cnt == 4'd7) begin
                                reg_addr <= {shift_reg[6:0], sda_val};
                                state    <= I2C_REG_ACK;
                            end
                        end
                    end
                    
                    I2C_REG_ACK: begin
                        if (scl_negedge) begin
                            sda_out <= 1'b0;  // ACK
                            sda_oe  <= 1'b1;
                            bit_cnt <= 4'd0;
                            state   <= I2C_WRITE_DATA;
                        end
                    end
                    
                    I2C_WRITE_DATA: begin
                        if (scl_negedge && bit_cnt == 4'd0) begin
                            sda_oe <= 1'b0;  // Release SDA
                        end
                        
                        if (scl_posedge) begin
                            shift_reg <= {shift_reg[6:0], sda_val};
                            bit_cnt   <= bit_cnt + 1'b1;
                            
                            if (bit_cnt == 4'd7) begin
                                // Write to register
                                case (reg_addr)
                                    REG_CTRL: begin
                                        enable  <= shift_reg[0];
                                        trigger <= shift_reg[1];
                                    end
                                    REG_IRQ_CTRL: begin
                                        irq_en <= shift_reg[0];
                                        if (shift_reg[1]) begin
                                            irq_flag <= 1'b0;
                                            irq      <= 1'b0;
                                        end
                                    end
                                    REG_SAMPLE_DIV:    cfg_sample_div <= {shift_reg[6:0], sda_val};
                                    REG_MPU_CFG: begin
                                        cfg_dlpf     <= shift_reg[2:0];
                                        cfg_gyro_fs  <= shift_reg[4:3];
                                        cfg_accel_fs <= shift_reg[6:5];
                                    end
                                    REG_THRESHOLD_L:   cfg_threshold[7:0]  <= {shift_reg[6:0], sda_val};
                                    REG_THRESHOLD_H:   cfg_threshold[15:8] <= {shift_reg[6:0], sda_val};
                                    REG_BUFFER_SIZE_L: cfg_buffer_size[7:0] <= {shift_reg[6:0], sda_val};
                                    REG_BUFFER_SIZE_H: cfg_buffer_size[8]   <= sda_val;
                                endcase
                                
                                state <= I2C_WRITE_ACK;
                            end
                        end
                    end
                    
                    I2C_WRITE_ACK: begin
                        if (scl_negedge) begin
                            sda_out  <= 1'b0;  // ACK
                            sda_oe   <= 1'b1;
                            bit_cnt  <= 4'd0;
                            reg_addr <= reg_addr + 1'b1;  // Auto-increment
                            state    <= I2C_WRITE_DATA;
                        end
                    end
                    
                    I2C_READ_DATA: begin
                        if (scl_negedge) begin
                            sda_out <= shift_reg[7];
                            sda_oe  <= 1'b1;
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            bit_cnt <= bit_cnt + 1'b1;
                            
                            if (bit_cnt == 4'd7) begin
                                state <= I2C_READ_ACK;
                            end
                        end
                    end
                    
                    I2C_READ_ACK: begin
                        if (scl_negedge) begin
                            sda_oe <= 1'b0;  // Release for master ACK/NACK
                        end
                        
                        if (scl_posedge) begin
                            if (sda_val) begin
                                // NACK - stop reading
                                state <= I2C_IDLE;
                            end else begin
                                // ACK - continue reading
                                reg_addr  <= reg_addr + 1'b1;
                                shift_reg <= read_reg(reg_addr + 1'b1);
                                bit_cnt   <= 4'd0;
                                state     <= I2C_READ_DATA;
                            end
                        end
                    end
                    
                    default: state <= I2C_IDLE;
                endcase
            end
        end
    end

endmodule