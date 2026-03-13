// =============================================================================
// uart_rx.v
//
// Simple 8N1 UART receiver.
//
// Monitors the 'rx' input line.  When a start bit is detected the receiver
// advances to the centre of the start bit, verifies it is still low (glitch
// rejection), then samples each of the 8 data bits at the centre of their
// respective bit periods (LSB first).  After a valid stop bit the received
// byte is presented on 'data' and 'valid' is pulsed for one clock cycle.
//
// A two-flip-flop synchroniser is included on the 'rx' input to prevent
// metastability when the transmitting clock domain differs from the FPGA
// system clock.
//
// Parameters:
//   CLK_FREQ  – system clock frequency in Hz (default 50 MHz)
//   BAUD_RATE – baud rate in bits/s        (default 115 200)
// =============================================================================
`timescale 1ns/1ps

module uart_rx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,       // UART RX line (idle high)
    output reg  [7:0] data,     // received byte (valid when valid=1)
    output reg        valid     // one-cycle pulse: data holds a new byte
);

    // Number of system-clock cycles per UART bit
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    localparam CNT_W        = $clog2(CLKS_PER_BIT) + 1;
    localparam [CNT_W-1:0] CNT_MAX  = CNT_W'(CLKS_PER_BIT - 1);
    localparam [CNT_W-1:0] CNT_HALF = CNT_W'(CLKS_PER_BIT / 2);

    // State encoding
    localparam [1:0]
        S_IDLE  = 2'd0,
        S_START = 2'd1,
        S_DATA  = 2'd2,
        S_STOP  = 2'd3;

    reg [1:0]       state;
    reg [CNT_W-1:0] cnt;
    reg [2:0]       bit_idx;
    reg [7:0]       shift;

    // Two-FF synchroniser – prevents metastability on the RX input
    reg rx_s0, rx_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_s0 <= 1'b1;
            rx_s1 <= 1'b1;
        end else begin
            rx_s0 <= rx;
            rx_s1 <= rx_s0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            cnt     <= 0;
            bit_idx <= 3'd0;
            shift   <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
        end else begin
            valid <= 1'b0;  // default: no new byte this cycle

            case (state)
                // -----------------------------------------------------------------
                // Idle – wait for a start bit (RX falling edge, i.e. RX goes low).
                // -----------------------------------------------------------------
                S_IDLE: begin
                    if (!rx_s1) begin
                        cnt   <= 0;
                        state <= S_START;
                    end
                end

                // -----------------------------------------------------------------
                // Start bit – count to the centre of the start-bit period.
                // If RX is still low at the mid-point it is a genuine start bit;
                // otherwise treat it as a glitch and return to idle.
                // -----------------------------------------------------------------
                S_START: begin
                    if (cnt == CNT_HALF) begin
                        if (!rx_s1) begin
                            cnt     <= 0;
                            bit_idx <= 3'd0;
                            shift   <= 8'd0;
                            state   <= S_DATA;
                        end else begin
                            state <= S_IDLE;    // glitch – abort
                        end
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                // Data bits – sample at the centre of each bit period (LSB first).
                // Bits are shifted right into 'shift': the first received bit (LSB)
                // eventually lands at shift[0]; the last (MSB) stays at shift[7].
                // -----------------------------------------------------------------
                S_DATA: begin
                    if (cnt == CNT_MAX) begin
                        cnt   <= 0;
                        shift <= {rx_s1, shift[7:1]};  // shift right; MSB ← new bit, existing bits move towards LSB
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
                // Stop bit – wait one full bit period; if the stop bit is high
                // (valid) present the received byte on 'data' and pulse 'valid'.
                // -----------------------------------------------------------------
                S_STOP: begin
                    if (cnt == CNT_MAX) begin
                        cnt <= 0;
                        if (rx_s1) begin    // valid stop bit
                            data  <= shift;
                            valid <= 1'b1;
                        end
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
