// =============================================================================
// ad5940_top.v
//
// Top-level FPGA module.
//
// Instantiates ad5940_adc_polling to continuously read the AD5940 ADC via SPI,
// then serialises each 16-bit result over a UART TX port (8N1).
//
// A UART RX port accepts 3-byte configuration commands at the same baud rate:
//
//   Byte 0 : 0xA5          (sync / packet-start marker)
//   Byte 1 : CMD           (which parameter to change)
//   Byte 2 : VAL           (new value)
//
//   CMD = 0x01 – Set measurement type (selects ADC input mux):
//     VAL = 0  Voltage, internal references
//               MuxP = VREF1P8DAC (0x12), MuxN = VSET1P1 (0x08)  ← default
//     VAL = 1  Voltage, external pins AIN0 / AIN1
//               MuxP = AIN0 (0x04),       MuxN = AIN1 (0x05)
//     VAL = 2  Current via LPTIA0 transimpedance amplifier
//               MuxP = LPTIA0_P (0x21),   MuxN = VSET1P1 (0x08)
//
//   CMD = 0x02 – Set PGA gain (ADCCON bits [18:16]):
//     VAL = 0  ×1   (ADCPGA_1)
//     VAL = 1  ×1.5 (ADCPGA_1P5)  ← default
//     VAL = 2  ×2   (ADCPGA_2)
//     VAL = 3  ×4   (ADCPGA_4)
//     VAL = 4  ×9   (ADCPGA_9)
//
//   CMD = 0x03 – Select digital filter preset (ADCFILTERCON):
//     VAL = 0  SINC3(OSR=4) + SINC2(OSR=1333), notch bypassed  → ~150 Hz  ← default
//     VAL = 1  SINC3(OSR=4) + SINC2(OSR=1333), notch enabled   → ~150 Hz, 50/60 Hz rejection
//     VAL = 2  SINC3(OSR=4) + SINC2(OSR=667),  notch bypassed  → ~300 Hz
//     VAL = 3  SINC3(OSR=4) + SINC2(OSR=178),  notch bypassed  → ~890 Hz
//
// Any other CMD or out-of-range VAL is silently ignored.
// A new configuration triggers a full re-initialisation of the AD5940 (all
// 22 SPI writes) so it takes effect on the first ADC sample after init.
//
// UART TX output (adc_data + converted voltage over serial):
//   Each ADC sample is transmitted as four consecutive UART bytes, MSB first:
//   Byte 0 : adc_data[15:8]   raw ADC code high byte
//   Byte 1 : adc_data[7:0]    raw ADC code low byte
//   Byte 2 : diff_mv[15:8]    signed differential voltage, high byte (mV)
//   Byte 3 : diff_mv[7:0]     signed differential voltage, low byte  (mV)
//
//   diff_mv is the output of ad5940_adc_code2volt: it represents the
//   differential voltage V_MuxP − V_MuxN in signed 16-bit millivolts.
//   The conversion mirrors AD5940_ADCCode2Volt() from ad5940lib/ad5940.c:
//     diff_mv = (adc_code − 32768) × 1.835 / (32768 × pga_gain)   [mV]
//   To recover the single-ended voltage, add the VSET1P1 reference:
//     volt_mv = diff_mv + 1110   [mV]   (valid when MuxN = VSET1P1)
//   For current measurements (mode 2, MuxP = LPTIA0_P), the current in
//   microamps is recovered by: I_uA = diff_mv * 1000 / Rtia_ohm
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

    // UART TX output – ADC results
    output wire uart_tx_pin,

    // UART RX input – configuration commands
    input  wire uart_rx_pin,

    // AD5940 hardware pins
    output wire afe_rst_n,
    output wire spi_cs_n,
    output wire spi_clk,
    output wire spi_mosi,
    input  wire spi_miso
);

    // =========================================================================
    // Configuration registers
    // =========================================================================
    // These are updated by the UART command decoder below and passed to the
    // ADC polling core.  Their reset values match the original hardcoded values
    // so the module is fully backward-compatible when no commands are received.
    //
    // ADCCON  : bits[5:0]=MuxP  bits[12:8]=MuxN  bits[18:16]=PGA
    // ADCFILTERCON : bit[0]=Rate bit[4]=BpNotch bits[11:8]=SINC2OSR bits[13:12]=SINC3OSR
    // =========================================================================

    // MuxP / MuxN defaults: VREF1P8DAC / VSET1P1
    localparam [5:0] MUXP_INT_VOLT = 6'h12; // VREF1P8DAC – internal voltage ref
    localparam [5:0] MUXP_EXT_VOLT = 6'h04; // AIN0       – external pin
    localparam [5:0] MUXP_CURR     = 6'h21; // LPTIA0_P   – current via LPTIA0
    localparam [4:0] MUXN_INT_VOLT = 5'h08; // VSET1P1    – internal 1.11 V
    localparam [4:0] MUXN_EXT_VOLT = 5'h05; // AIN1       – external pin
    localparam [4:0] MUXN_CURR     = 5'h08; // VSET1P1    – internal 1.11 V

    reg [1:0] mux_type_r; // 0=voltage internal (default), 1=voltage external, 2=current
    reg [2:0] pga_r;      // PGA gain code 0–4 (default 1 = ×1.5)

    // ADCFILTERCON presets
    // BpNotch field (bit[4]): 1 = bypass notch, 0 = enable notch (50/60 Hz rejection).
    // Preset 0 (default): SINC3(OSR=4)+SINC2(OSR=1333), notch bypassed   → ~150 Hz output
    // Preset 1           : SINC3(OSR=4)+SINC2(OSR=1333), notch enabled   → ~150 Hz + 50/60 Hz rejection
    // Preset 2           : SINC3(OSR=4)+SINC2(OSR=667),  notch bypassed  → ~300 Hz output
    // Preset 3           : SINC3(OSR=4)+SINC2(OSR=178),  notch bypassed  → ~890 Hz output
    localparam [31:0] FLTCON_0 = 32'h00001B11; // OSR4/1333, bypass notch (default)
    localparam [31:0] FLTCON_1 = 32'h00001B01; // OSR4/1333, with notch
    localparam [31:0] FLTCON_2 = 32'h00001711; // OSR4/667,  bypass notch
    localparam [31:0] FLTCON_3 = 32'h00001311; // OSR4/178,  bypass notch

    reg [31:0] cfg_adcfiltercon_r;

    // Build cfg_adccon combinationally from mux_type_r and pga_r
    reg [5:0] muxp_sel;
    reg [4:0] muxn_sel;

    always @(*) begin
        case (mux_type_r)
            2'd1:    begin muxp_sel = MUXP_EXT_VOLT; muxn_sel = MUXN_EXT_VOLT; end
            2'd2:    begin muxp_sel = MUXP_CURR;     muxn_sel = MUXN_CURR;     end
            default: begin muxp_sel = MUXP_INT_VOLT; muxn_sel = MUXN_INT_VOLT; end
        endcase
    end

    // ADCCON[5:0]=MuxP  [12:8]=MuxN  [18:16]=PGA  (all other bits = 0)
    // pga_r is 3 bits wide but is only written when rx_data ≤ 4 (see CMD_PGA handler),
    // so the value is always in the valid range 0–4.
    wire [31:0] cfg_adccon_w = {13'd0, pga_r, 3'd0, muxn_sel, 2'd0, muxp_sel};

    // Reconfig pulse to ADC polling core
    reg cfg_reconfig;

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
        .clk              (clk),
        .rst_n            (rst_n),
        .cfg_adccon       (cfg_adccon_w),
        .cfg_adcfiltercon (cfg_adcfiltercon_r),
        .cfg_reconfig     (cfg_reconfig),
        .adc_valid        (adc_valid),
        .adc_data         (adc_data),
        .state_out        (state_out),
        .afe_rst_n        (afe_rst_n),
        .spi_cs_n         (spi_cs_n),
        .spi_clk          (spi_clk),
        .spi_mosi         (spi_mosi),
        .spi_miso         (spi_miso)
    );

    // =========================================================================
    // ADC code to voltage converter
    //
    // Combinational module that converts the raw 16-bit ADC code to a signed
    // 16-bit differential voltage in millivolts (mV).  The conversion uses
    // the same formula as AD5940_ADCCode2Volt() in ad5940lib/ad5940.c.
    // =========================================================================
    wire [15:0] diff_mv;    // signed 16-bit differential voltage (mV)
    wire [15:0] single_mv;  // signed 16-bit single-ended voltage (mV, = diff_mv + 1110)

    ad5940_adc_code2volt u_code2volt (
        .adc_code  (adc_data),
        .pga_sel   (pga_r),
        .diff_mv   (diff_mv),
        .single_mv (single_mv)
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
    // Serialise each ADC sample as four UART bytes (MSB first):
    //   Byte 0 : adc_data[15:8]  raw ADC code, high byte
    //   Byte 1 : adc_data[7:0]   raw ADC code, low byte
    //   Byte 2 : diff_mv[15:8]   signed differential voltage (mV), high byte
    //   Byte 3 : diff_mv[7:0]    signed differential voltage (mV), low byte
    //
    // State machine:
    //   TX_IDLE   – waiting for adc_valid; samples arriving while busy are dropped.
    //   TX_RAW_HI – raw-code high byte handed to uart_tx; wait for completion.
    //   TX_RAW_LO – raw-code low byte handed to uart_tx; wait for completion.
    //   TX_MV_HI  – diff_mv high byte handed to uart_tx; wait for completion.
    //   TX_MV_LO  – diff_mv low byte handed to uart_tx; wait for completion.
    // =========================================================================
    localparam [2:0]
        TX_IDLE   = 3'd0,
        TX_RAW_HI = 3'd1,
        TX_RAW_LO = 3'd2,
        TX_MV_HI  = 3'd3,
        TX_MV_LO  = 3'd4;

    reg [2:0]  tx_state;
    reg [15:0] adc_hold;   // captured copy of adc_data
    reg [15:0] mv_hold;    // captured copy of diff_mv

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state   <= TX_IDLE;
            adc_hold   <= 16'h0;
            mv_hold    <= 16'h0;
            uart_data  <= 8'h0;
            uart_valid <= 1'b0;
        end else begin
            uart_valid <= 1'b0;   // default: no new byte this cycle

            case (tx_state)
                // -----------------------------------------------------------------
                // Wait for an ADC result.  Capture the raw code and the converted
                // voltage at the same clock edge, then start the raw high byte.
                // Any adc_valid that arrives while transmission is in progress is
                // silently dropped (ADC rate << UART byte rate in normal use).
                // -----------------------------------------------------------------
                TX_IDLE: begin
                    if (adc_valid) begin
                        adc_hold   <= adc_data;
                        mv_hold    <= diff_mv;    // latch converted voltage
                        uart_data  <= adc_data[15:8];  // raw code high byte first
                        uart_valid <= 1'b1;
                        tx_state   <= TX_RAW_HI;
                    end
                end

                // -----------------------------------------------------------------
                // Raw code high byte queued; wait for uart_tx to finish, then
                // queue the raw code low byte.
                // -----------------------------------------------------------------
                TX_RAW_HI: begin
                    if (uart_ready) begin
                        uart_data  <= adc_hold[7:0];   // raw code low byte
                        uart_valid <= 1'b1;
                        tx_state   <= TX_RAW_LO;
                    end
                end

                // -----------------------------------------------------------------
                // Raw code low byte queued; wait for uart_tx to finish, then
                // queue the converted-voltage high byte.
                // -----------------------------------------------------------------
                TX_RAW_LO: begin
                    if (uart_ready) begin
                        uart_data  <= mv_hold[15:8];   // diff_mv high byte
                        uart_valid <= 1'b1;
                        tx_state   <= TX_MV_HI;
                    end
                end

                // -----------------------------------------------------------------
                // diff_mv high byte queued; wait for uart_tx to finish, then
                // queue the converted-voltage low byte.
                // -----------------------------------------------------------------
                TX_MV_HI: begin
                    if (uart_ready) begin
                        uart_data  <= mv_hold[7:0];    // diff_mv low byte
                        uart_valid <= 1'b1;
                        tx_state   <= TX_MV_LO;
                    end
                end

                // -----------------------------------------------------------------
                // diff_mv low byte queued; wait for uart_tx to finish, then
                // return to idle ready for the next sample.
                // -----------------------------------------------------------------
                TX_MV_LO: begin
                    if (uart_ready) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // =========================================================================
    // UART receiver
    // =========================================================================
    wire       rx_valid;
    wire [7:0] rx_data;

    uart_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_uart_rx (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (uart_rx_pin),
        .data  (rx_data),
        .valid (rx_valid)
    );

    // =========================================================================
    // Configuration command decoder
    //
    // Parses 3-byte packets:  [0xA5] [CMD] [VAL]
    //
    // States:
    //   CFG_SYNC – waiting for the 0xA5 sync byte.
    //   CFG_CMD  – received 0xA5; waiting for CMD byte.
    //   CFG_VAL  – received CMD; waiting for VAL byte.
    //
    // On a complete, valid packet the relevant config register is updated
    // and cfg_reconfig is asserted for one clock cycle to trigger
    // re-initialisation of the AD5940.
    // =========================================================================
    localparam [1:0]
        CFG_SYNC = 2'd0,
        CFG_CMD  = 2'd1,
        CFG_VAL  = 2'd2;

    localparam [7:0] SYNC_BYTE   = 8'hA5;
    localparam [7:0] CMD_MUX     = 8'h01; // set measurement type
    localparam [7:0] CMD_PGA     = 8'h02; // set PGA gain
    localparam [7:0] CMD_FILTER  = 8'h03; // set digital filter preset

    reg [1:0] cfg_state;
    reg [7:0] cfg_cmd_r;   // latched CMD byte

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_state          <= CFG_SYNC;
            cfg_cmd_r          <= 8'h0;
            cfg_reconfig       <= 1'b0;
            mux_type_r         <= 2'd0;          // default: voltage internal
            pga_r              <= 3'd1;          // default: ×1.5
            cfg_adcfiltercon_r <= FLTCON_0;      // default: SINC3+SINC2, bypass notch
        end else begin
            cfg_reconfig <= 1'b0;   // default: no reconfig pulse

            case (cfg_state)
                // -----------------------------------------------------------------
                // Wait for the 0xA5 sync byte.
                // -----------------------------------------------------------------
                CFG_SYNC: begin
                    if (rx_valid && rx_data == SYNC_BYTE)
                        cfg_state <= CFG_CMD;
                end

                // -----------------------------------------------------------------
                // Wait for the CMD byte; accept only recognised commands.
                // -----------------------------------------------------------------
                CFG_CMD: begin
                    if (rx_valid) begin
                        if (rx_data == CMD_MUX   ||
                            rx_data == CMD_PGA   ||
                            rx_data == CMD_FILTER) begin
                            cfg_cmd_r <= rx_data;
                            cfg_state <= CFG_VAL;
                        end else begin
                            cfg_state <= CFG_SYNC; // unknown CMD – resync
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Wait for the VAL byte; apply it to the relevant register and
                // trigger re-initialisation.
                // -----------------------------------------------------------------
                CFG_VAL: begin
                    if (rx_valid) begin
                        cfg_state <= CFG_SYNC;

                        case (cfg_cmd_r)
                            // CMD 0x01 – measurement type
                            CMD_MUX: begin
                                if (rx_data <= 8'd2) begin
                                    mux_type_r   <= rx_data[1:0];
                                    cfg_reconfig <= 1'b1;
                                end
                            end

                            // CMD 0x02 – PGA gain (0–4)
                            CMD_PGA: begin
                                if (rx_data <= 8'd4) begin
                                    pga_r        <= rx_data[2:0];
                                    cfg_reconfig <= 1'b1;
                                end
                            end

                            // CMD 0x03 – digital filter preset (0–3)
                            CMD_FILTER: begin
                                case (rx_data)
                                    8'd0: cfg_adcfiltercon_r <= FLTCON_0;
                                    8'd1: cfg_adcfiltercon_r <= FLTCON_1;
                                    8'd2: cfg_adcfiltercon_r <= FLTCON_2;
                                    8'd3: cfg_adcfiltercon_r <= FLTCON_3;
                                    default: ;  // out-of-range: ignore
                                endcase
                                if (rx_data <= 8'd3)
                                    cfg_reconfig <= 1'b1;
                            end

                            default: ;  // should not reach here
                        endcase
                    end
                end

                default: cfg_state <= CFG_SYNC;
            endcase
        end
    end

endmodule
