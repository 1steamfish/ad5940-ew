// =============================================================================
// ad5940_spi_master.v
//
// AD5940 SPI register read/write master.
//
// Implements the two-phase SPI protocol required by the AD5940:
//
//   Phase 1 – Set address (always first):
//     CS# low → send SPICMD_SETADDR (8-bit) + addr (16-bit) → CS# high
//
//   Phase 2 – Transfer data:
//     CS# low → send SPICMD_WRITEREG or SPICMD_READREG (8-bit)
//              [+ 1 dummy byte for reads] + data (16 or 32 bit) → CS# high
//
// Data width rule (from ad5940.c AD5940_SPIWriteReg / AD5940_SPIReadReg):
//   addr in [0x1000 .. 0x3014] → 32-bit data
//   otherwise                  → 16-bit data
//
// SPI electrical standard: Mode 0 (CPOL=0 CPHA=0)
//   – SCLK idles LOW
//   – MOSI is valid before the rising edge
//   – MISO is captured on the rising edge
//
// Interface:
//   wr_en / rd_en – one-cycle pulse to start a write or read transaction.
//   busy          – high from the start until done.
//   done          – one-cycle pulse when the transaction has finished.
//   rdata         – read-back data; valid while done is asserted.
//
// Parameter:
//   CLK_DIV – system clock cycles per SCLK half-period.
//             SCLK_freq = sys_clk / (2 * CLK_DIV).
//             Default 4 → 6.25 MHz for a 50 MHz system clock (max is 16 MHz).
// =============================================================================
`timescale 1ns/1ps

module ad5940_spi_master #(
    parameter CLK_DIV = 4
)(
    input  wire        clk,
    input  wire        rst_n,

    // User interface
    input  wire        wr_en,       // pulse: start a register write
    input  wire        rd_en,       // pulse: start a register read
    input  wire [15:0] addr,        // AD5940 register address
    input  wire [31:0] wdata,       // data to write (ignored for reads)
    output reg  [31:0] rdata,       // data read back (valid on done)
    output reg         busy,        // high while transaction is in progress
    output reg         done,        // one-cycle pulse on completion

    // SPI pins (Mode 0)
    output reg         spi_cs_n,
    output reg         spi_clk,
    output reg         spi_mosi,
    input  wire        spi_miso
);

    // -------------------------------------------------------------------------
    // SPI command bytes (from ad5940.h)
    // -------------------------------------------------------------------------
    localparam [7:0] SPICMD_SETADDR  = 8'h20;
    localparam [7:0] SPICMD_READREG  = 8'h6d;
    localparam [7:0] SPICMD_WRITEREG = 8'h2d;

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    localparam [2:0]
        S_IDLE     = 3'd0,
        S_P1_START = 3'd1,   // assert CS#, load Phase-1 shift register
        S_P1_SHIFT = 3'd2,   // clock out SETADDR + addr (24 bits)
        S_P2_START = 3'd3,   // CS# gap + assert CS#, load Phase-2 shift register
        S_P2_SHIFT = 3'd4,   // clock out/in command + data
        S_DONE     = 3'd5;   // assert done for one cycle, return to IDLE

    reg [2:0] state;

    // -------------------------------------------------------------------------
    // Registered copies of caller inputs
    // -------------------------------------------------------------------------
    reg        is_read;
    reg [15:0] addr_r;
    reg [31:0] wdata_r;

    // Address range that uses 32-bit data words (from ad5940.c)
    wire is_32bit = (addr_r >= 16'h1000) && (addr_r <= 16'h3014);

    // -------------------------------------------------------------------------
    // Clock divider
    // -------------------------------------------------------------------------
    // clk_cnt counts 0 … CLK_DIV-1; tick fires every CLK_DIV system clocks.
    localparam CNT_W = $clog2(CLK_DIV) + 1; // +1 avoids zero-width when CLK_DIV=1
    reg [CNT_W-1:0] clk_cnt;
    wire tick = (clk_cnt == CNT_W'(CLK_DIV - 1));

    // -------------------------------------------------------------------------
    // 48-bit shift registers
    // -------------------------------------------------------------------------
    // TX: data to clock out, MSB first, left-justified in bit [47].
    // RX: incoming MISO bits shifted in from bit [0].
    // After N rising edges the first received bit is at shift_rx[N-1].
    reg [47:0] shift_tx;
    reg [47:0] shift_rx;

    // bit_cnt: number of SCLK rising edges remaining in this phase.
    reg [5:0] bit_cnt;

    // gap_cnt: system-clock counter used for the inter-phase CS# gap.
    reg [7:0] gap_cnt;

    // =========================================================================
    // Main state machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            spi_cs_n <= 1'b1;
            spi_clk  <= 1'b0;
            spi_mosi <= 1'b0;
            rdata    <= 32'h0;
            busy     <= 1'b0;
            done     <= 1'b0;
            clk_cnt  <= 0;
            bit_cnt  <= 6'd0;
            shift_tx <= 48'h0;
            shift_rx <= 48'h0;
            gap_cnt  <= 8'd0;
            is_read  <= 1'b0;
            addr_r   <= 16'h0;
            wdata_r  <= 32'h0;
        end else begin
            // Default: clear single-cycle outputs
            done <= 1'b0;

            // Clock divider always running
            if (clk_cnt == CNT_W'(CLK_DIV - 1))
                clk_cnt <= 0;
            else
                clk_cnt <= clk_cnt + 1'b1;

            case (state)
                // -----------------------------------------------------------------
                S_IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_clk  <= 1'b0;
                    busy     <= 1'b0;
                    if (wr_en || rd_en) begin
                        busy    <= 1'b1;
                        is_read <= rd_en;
                        addr_r  <= addr;
                        wdata_r <= wdata;
                        state   <= S_P1_START;
                    end
                end

                // -----------------------------------------------------------------
                // Phase 1 – send SETADDR + 16-bit address (24 bits total).
                // TX layout: [47:40]=SETADDR [39:24]=addr [23:0]=don't-care
                // MOSI is pre-loaded with the MSB before the first rising edge.
                // -----------------------------------------------------------------
                S_P1_START: begin
                    spi_cs_n <= 1'b0;
                    spi_clk  <= 1'b0;
                    shift_tx <= {SPICMD_SETADDR, addr_r, 24'h000000};
                    shift_rx <= 48'h0;
                    spi_mosi <= SPICMD_SETADDR[7]; // first bit (= 0 for 0x20)
                    bit_cnt  <= 6'd24;
                    state    <= S_P1_SHIFT;
                end

                // -----------------------------------------------------------------
                S_P1_SHIFT: begin
                    if (tick) begin
                        if (!spi_clk) begin
                            // ----- Rising edge: sample MISO -----
                            spi_clk  <= 1'b1;
                            shift_rx <= {shift_rx[46:0], spi_miso};
                            bit_cnt  <= bit_cnt - 1'b1;
                        end else begin
                            // ----- Falling edge: update MOSI or finish -----
                            spi_clk <= 1'b0;
                            if (bit_cnt == 6'd0) begin
                                // All 24 bits have been clocked – end Phase 1
                                spi_cs_n <= 1'b1;
                                gap_cnt  <= 8'd0;
                                state    <= S_P2_START;
                            end else begin
                                // Advance TX shift register and drive next bit.
                                // Both assignments use the OLD value of shift_tx,
                                // so spi_mosi = old bit[46] = new bit[47] after shift.
                                shift_tx <= {shift_tx[46:0], 1'b0};
                                spi_mosi <= shift_tx[46];
                            end
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Phase 2 – send READ or WRITE command + data.
                //
                // Write TX layout (32-bit addr range):
                //   [47:40]=WRITEREG [39:8]=wdata[31:0] [7:0]=don't-care (40 bits used)
                // Write TX layout (16-bit addr range):
                //   [47:40]=WRITEREG [39:24]=wdata[15:0] [23:0]=don't-care (24 bits used)
                //
                // Read TX layout (all zeros beyond command – clocks in MISO data):
                //   [47:40]=READREG [39:0]=0 (32-bit: 48 bits; 16-bit: 32 bits)
                //
                // After a 32-bit read: shift_rx[31:0] = received data.
                // After a 16-bit read: shift_rx[15:0] = received data.
                //   (first 8+8 bits in shift_rx are the cmd-echo and dummy byte)
                // -----------------------------------------------------------------
                S_P2_START: begin
                    // Wait a few system clocks as inter-phase CS# gap
                    if (gap_cnt < 8'(CLK_DIV)) begin
                        gap_cnt <= gap_cnt + 1'b1;
                    end else begin
                        spi_cs_n <= 1'b0;
                        spi_clk  <= 1'b0;
                        shift_rx <= 48'h0;

                        if (is_read) begin
                            // READREG + 1 dummy byte + N data bytes (all 0 on TX)
                            shift_tx <= {SPICMD_READREG, 40'h0};
                            spi_mosi <= SPICMD_READREG[7]; // = 0
                            bit_cnt  <= is_32bit ? 6'd48 : 6'd32;
                        end else if (is_32bit) begin
                            // WRITEREG + 32-bit data
                            shift_tx <= {SPICMD_WRITEREG, wdata_r, 8'h00};
                            spi_mosi <= SPICMD_WRITEREG[7]; // = 0
                            bit_cnt  <= 6'd40;
                        end else begin
                            // WRITEREG + 16-bit data
                            shift_tx <= {SPICMD_WRITEREG, wdata_r[15:0], 24'h0};
                            spi_mosi <= SPICMD_WRITEREG[7]; // = 0
                            bit_cnt  <= 6'd24;
                        end

                        state <= S_P2_SHIFT;
                    end
                end

                // -----------------------------------------------------------------
                S_P2_SHIFT: begin
                    if (tick) begin
                        if (!spi_clk) begin
                            // ----- Rising edge: sample MISO -----
                            spi_clk  <= 1'b1;
                            shift_rx <= {shift_rx[46:0], spi_miso};
                            bit_cnt  <= bit_cnt - 1'b1;
                        end else begin
                            // ----- Falling edge: update MOSI or finish -----
                            spi_clk <= 1'b0;
                            if (bit_cnt == 6'd0) begin
                                // End Phase 2
                                spi_cs_n <= 1'b1;
                                // Extract read data from shift_rx:
                                //   After 48 risings: cmd-echo=[47:40] dummy=[39:32] data=[31:0]
                                //   After 32 risings: cmd-echo=[31:24] dummy=[23:16] data=[15:0]
                                if (is_read) begin
                                    rdata <= is_32bit
                                        ? shift_rx[31:0]
                                        : {16'h0, shift_rx[15:0]};
                                end
                                state <= S_DONE;
                            end else begin
                                shift_tx <= {shift_tx[46:0], 1'b0};
                                spi_mosi <= shift_tx[46];
                            end
                        end
                    end
                end

                // -----------------------------------------------------------------
                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
