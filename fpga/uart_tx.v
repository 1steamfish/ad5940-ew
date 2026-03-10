// =============================================================================
// uart_tx.v
//
// Simple 8N1 UART transmitter.
//
// When valid is pulsed for one clock cycle the byte in 'data' is serialised
// on the 'tx' output using the standard 8N1 frame:
//   – 1 start bit (logic 0)
//   – 8 data bits (LSB first)
//   – 1 stop bit  (logic 1)
//
// 'busy' is high from the first clock of the start bit through to the last
// clock of the stop bit.  A new byte must not be presented until busy is low.
//
// Parameters:
//   CLK_FREQ  – system clock frequency in Hz (default 50 MHz)
//   BAUD_RATE – baud rate in bits/s        (default 115 200)
// =============================================================================
`timescale 1ns/1ps

module uart_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,   // byte to transmit
    input  wire       valid,  // one-cycle pulse: load and send 'data'
    output reg        tx,     // UART TX line (idle high)
    output wire       busy    // high while transmission is in progress
);

    // Number of system-clock cycles per UART bit
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam CNT_W        = $clog2(CLKS_PER_BIT) + 1;
    localparam [CNT_W-1:0] CNT_MAX = CNT_W'(CLKS_PER_BIT - 1);

    // State encoding
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0]       state;
    reg [CNT_W-1:0] cnt;      // bit-period counter (0 … CLKS_PER_BIT-1)
    reg [2:0]       bit_idx;  // data-bit index (0 = LSB … 7 = MSB)
    reg [7:0]       shift;    // working copy of 'data', shifted right as bits go out

    assign busy = (state != S_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            tx      <= 1'b1;
            cnt     <= 0;
            bit_idx <= 3'd0;
            shift   <= 8'd0;
        end else begin
            case (state)
                // -----------------------------------------------------------------
                // Idle – TX line high; wait for a byte to send.
                // -----------------------------------------------------------------
                S_IDLE: begin
                    tx <= 1'b1;
                    if (valid) begin
                        shift <= data;
                        cnt   <= 0;
                        state <= S_START;
                    end
                end

                // -----------------------------------------------------------------
                // Start bit – TX low for one full bit period.
                // -----------------------------------------------------------------
                S_START: begin
                    tx <= 1'b0;
                    if (cnt == CNT_MAX) begin
                        cnt     <= 0;
                        bit_idx <= 3'd0;
                        state   <= S_DATA;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                // Data bits – shift out LSB first, one bit per bit period.
                // -----------------------------------------------------------------
                S_DATA: begin
                    tx <= shift[0];
                    if (cnt == CNT_MAX) begin
                        cnt   <= 0;
                        shift <= {1'b0, shift[7:1]};
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                // Stop bit – TX high for one full bit period.
                // -----------------------------------------------------------------
                S_STOP: begin
                    tx <= 1'b1;
                    if (cnt == CNT_MAX) begin
                        cnt   <= 0;
                        state <= S_IDLE;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
