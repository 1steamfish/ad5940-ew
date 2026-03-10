// =============================================================================
// ad5940_top.v
//
// Top-level FPGA module.
//
// Instantiates ad5940_adc_polling to continuously read the AD5940 ADC via SPI,
// then serialises each 16-bit result over a UART port (8N1).
//
// Each ADC sample is transmitted as two consecutive UART bytes, MSB first:
//   Byte 0 : adc_data[15:8]  (high byte)
//   Byte 1 : adc_data[7:0]   (low byte)
//
// SPI clock:
//   SCLK = CLK_FREQ / (2 × CLK_DIV)
//   Default CLK_DIV=3 → ~8.33 MHz for a 50 MHz system clock.
//   For exactly 10 MHz set CLK_FREQ=100_000_000 and CLK_DIV=5.
//
// Parameters:
//   CLK_FREQ     – system clock frequency in Hz (default 50 MHz)
//   CLK_DIV      – SPI clock divider (default 3 → ~8.33 MHz @ 50 MHz;
//                  use 5 with CLK_FREQ=100_000_000 for exactly 10 MHz)
//   BAUD_RATE    – UART baud rate in bits/s (default 115 200)
//   RESET_CYCLES – afe_rst_n low duration in clock cycles (~100 µs @ 50 MHz)
//   BOOT_CYCLES  – post-reset boot wait in clock cycles  (~10 ms  @ 50 MHz)
// =============================================================================
`timescale 1ns/1ps

module ad5940_top #(
    parameter CLK_FREQ     = 50_000_000,
    parameter CLK_DIV      = 3,          // SPI: ~8.33 MHz @ 50 MHz; 10 MHz @ 100 MHz w/ CLK_DIV=5
    parameter BAUD_RATE    = 115_200,
    parameter RESET_CYCLES = 5_000,      // ~100 µs @ 50 MHz
    parameter BOOT_CYCLES  = 500_000     // ~10 ms  @ 50 MHz
)(
    input  wire clk,
    input  wire rst_n,

    // UART output
    output wire uart_tx_pin,

    // AD5940 hardware pins
    output wire afe_rst_n,
    output wire spi_cs_n,
    output wire spi_clk,
    output wire spi_mosi,
    input  wire spi_miso
);

    // =========================================================================
    // ADC polling core
    // =========================================================================
    wire        adc_valid;
    wire [15:0] adc_data;
    wire [3:0]  state_out;

    ad5940_adc_polling #(
        .CLK_DIV      (CLK_DIV),
        .RESET_CYCLES (RESET_CYCLES),
        .BOOT_CYCLES  (BOOT_CYCLES)
    ) u_adc_polling (
        .clk       (clk),
        .rst_n     (rst_n),
        .adc_valid (adc_valid),
        .adc_data  (adc_data),
        .state_out (state_out),
        .afe_rst_n (afe_rst_n),
        .spi_cs_n  (spi_cs_n),
        .spi_clk   (spi_clk),
        .spi_mosi  (spi_mosi),
        .spi_miso  (spi_miso)
    );

    // =========================================================================
    // UART transmitter
    // =========================================================================
    reg  [7:0] uart_data;
    reg        uart_valid;
    wire       uart_busy;

    // High when the UART transmitter is idle and no valid pulse is in flight
    wire uart_ready = !uart_busy && !uart_valid;

    uart_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_uart_tx (
        .clk   (clk),
        .rst_n (rst_n),
        .data  (uart_data),
        .valid (uart_valid),
        .tx    (uart_tx_pin),
        .busy  (uart_busy)
    );

    // =========================================================================
    // Serialise 16-bit ADC result as two UART bytes: MSB first, then LSB.
    //
    // State machine:
    //   TX_IDLE – waiting for adc_valid; new samples are ignored while busy.
    //   TX_HIGH – high byte handed to uart_tx; wait for transmission to finish.
    //   TX_LOW  – low byte handed to uart_tx; wait for transmission to finish.
    // =========================================================================
    localparam [1:0]
        TX_IDLE = 2'd0,
        TX_HIGH = 2'd1,
        TX_LOW  = 2'd2;

    reg [1:0]  tx_state;
    reg [15:0] adc_hold;   // captured copy of adc_data

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state   <= TX_IDLE;
            adc_hold   <= 16'h0;
            uart_data  <= 8'h0;
            uart_valid <= 1'b0;
        end else begin
            uart_valid <= 1'b0;   // default: no new byte this cycle

            case (tx_state)
                // -----------------------------------------------------------------
                // Wait for an ADC result; capture it and start sending the high byte.
                // Any adc_valid that arrives while TX_HIGH or TX_LOW is in progress
                // is silently dropped (ADC rate << UART byte rate, so this never
                // happens under normal operating conditions).
                // -----------------------------------------------------------------
                TX_IDLE: begin
                    if (adc_valid) begin
                        adc_hold   <= adc_data;
                        uart_data  <= adc_data[15:8];  // high byte first
                        uart_valid <= 1'b1;
                        tx_state   <= TX_HIGH;
                    end
                end

                // -----------------------------------------------------------------
                // High byte has been handed to uart_tx; wait until it finishes,
                // then queue the low byte.
                // -----------------------------------------------------------------
                TX_HIGH: begin
                    if (uart_ready) begin
                        uart_data  <= adc_hold[7:0];   // low byte
                        uart_valid <= 1'b1;
                        tx_state   <= TX_LOW;
                    end
                end

                // -----------------------------------------------------------------
                // Low byte has been handed to uart_tx; wait until it finishes,
                // then return to idle.
                // -----------------------------------------------------------------
                TX_LOW: begin
                    if (uart_ready) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
