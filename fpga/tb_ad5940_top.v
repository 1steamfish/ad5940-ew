// =============================================================================
// tb_ad5940_top.v
//
// Testbench for ad5940_top.v.
//
// Uses the same behavioural AD5940 SPI slave model as tb_ad5940_adc_polling.v.
// A simple UART-receive task samples the uart_tx_pin output and reconstructs
// each received byte.
//
// Each ADC sample is now transmitted as four consecutive UART bytes:
//   Byte 0 : adc_data[15:8]  raw ADC code high byte
//   Byte 1 : adc_data[7:0]   raw ADC code low byte
//   Byte 2 : diff_mv[15:8]   signed differential voltage (mV), high byte
//   Byte 3 : diff_mv[7:0]    signed differential voltage (mV), low byte
//
// The SPI slave always returns SINC2DAT = 0x8000 (mid-scale), so the expected
// values are:
//   raw word  = 0x8000  (mid-scale ADC code)
//   diff_mv   = 0x0000  (0 mV differential; (0x8000 - 32768) * gain = 0)
//
// Simulation speed-up:
//   CLK_DIV=2, RESET_CYCLES=10, BOOT_CYCLES=20 to shorten init time.
//   BAUD_RATE=5_000_000 → CLKS_PER_BIT=10, so each 4-byte packet takes only
//   400 system-clock cycles to transmit.
// =============================================================================
`timescale 1ns/1ps

// =============================================================================
// Behavioural SPI slave model for AD5940
// (identical logic to the one in tb_ad5940_adc_polling.v, different module name
//  to avoid compilation conflicts when both testbenches exist in the same project)
// =============================================================================
module ad5940_spi_slave_top (
    input  wire spi_cs_n,
    input  wire spi_clk,
    input  wire spi_mosi,
    output reg  spi_miso
);

    localparam [7:0] SPICMD_SETADDR  = 8'h20;
    localparam [7:0] SPICMD_READREG  = 8'h6d;
    localparam [7:0] SPICMD_WRITEREG = 8'h2d;

    reg [15:0] latched_addr;
    reg        in_phase2;
    reg [47:0] rx_shift;
    reg [47:0] tx_shift;
    integer    bit_count;

    function [31:0] reg_response;
        input [15:0] addr;
        begin
            case (addr)
                16'h3014: reg_response = 32'h00000004; // INTCFLAG1: SINC2RDY
                16'h2080: reg_response = 32'h00008000; // SINC2DAT:  mid-scale
                default:  reg_response = 32'h00000000;
            endcase
        end
    endfunction

    initial begin
        spi_miso     = 1'b0;
        latched_addr = 16'h0;
        in_phase2    = 1'b1;
        rx_shift     = 48'h0;
        tx_shift     = 48'h0;
        bit_count    = 0;
    end

    always @(negedge spi_cs_n) begin
        rx_shift  = 48'h0;
        bit_count = 0;
        spi_miso  = 1'b0;
        if (in_phase2)
            tx_shift = {15'h0000, reg_response(latched_addr), 1'b0};
        else
            tx_shift = 48'h0;
    end

    always @(posedge spi_cs_n) begin
        if (!in_phase2) begin
            if (rx_shift[23:16] == SPICMD_SETADDR)
                latched_addr = rx_shift[15:0];
            in_phase2 = 1'b1;
        end else begin
            in_phase2 = 1'b0;
        end
    end

    always @(posedge spi_clk) begin
        if (!spi_cs_n) begin
            rx_shift  = {rx_shift[46:0], spi_mosi};
            bit_count = bit_count + 1;
        end
    end

    always @(negedge spi_clk) begin
        if (!spi_cs_n) begin
            spi_miso = tx_shift[47];
            tx_shift = {tx_shift[46:0], 1'b0};
        end
    end

endmodule


// =============================================================================
// Testbench top
// =============================================================================
module tb_ad5940_top;

    // -------------------------------------------------------------------------
    // Simulation parameters
    // -------------------------------------------------------------------------
    localparam CLK_FREQ     = 50_000_000;
    localparam BAUD_RATE    = 5_000_000;   // fast baud for simulation
    localparam CLK_DIV      = 2;           // fast SPI for simulation
    localparam RESET_CYCLES = 10;
    localparam BOOT_CYCLES  = 20;
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;  // = 10

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk;
    reg  rst_n;

    wire uart_tx_pin;
    reg  uart_rx_pin;        // driven by tb to send config commands to DUT
    wire afe_rst_n;
    wire spi_cs_n;
    wire spi_clk;
    wire spi_mosi;
    wire spi_miso;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
    ad5940_top #(
        .CLK_FREQ     (CLK_FREQ),
        .CLK_DIV      (CLK_DIV),
        .BAUD_RATE    (BAUD_RATE),
        .RESET_CYCLES (RESET_CYCLES),
        .BOOT_CYCLES  (BOOT_CYCLES)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .uart_tx_pin (uart_tx_pin),
        .uart_rx_pin (uart_rx_pin),
        .afe_rst_n   (afe_rst_n),
        .spi_cs_n    (spi_cs_n),
        .spi_clk     (spi_clk),
        .spi_mosi    (spi_mosi),
        .spi_miso    (spi_miso)
    );

    // -------------------------------------------------------------------------
    // Behavioural SPI slave
    // -------------------------------------------------------------------------
    ad5940_spi_slave_top slave (
        .spi_cs_n (spi_cs_n),
        .spi_clk  (spi_clk),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );

    // -------------------------------------------------------------------------
    // 50 MHz clock (20 ns period)
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always  #10 clk = ~clk;

    // -------------------------------------------------------------------------
    // UART receive task (DUT → TB)
    //
    // Detects the falling edge of uart_tx_pin (start bit), then samples each
    // of the 8 data bits at the centre of their respective bit periods (LSB
    // first), and returns the reconstructed byte in 'data'.
    // -------------------------------------------------------------------------
    task uart_recv;
        output reg [7:0] data;
        integer i;
        begin
            // Wait for start bit (TX goes low)
            @(negedge uart_tx_pin);
            // Advance to the centre of the start bit
            repeat (CLKS_PER_BIT / 2) @(posedge clk);
            // Sample 8 data bits at the centre of each bit period
            for (i = 0; i < 8; i = i + 1) begin
                repeat (CLKS_PER_BIT) @(posedge clk);
                data[i] = uart_tx_pin;  // LSB first
            end
            // Consume stop bit
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // UART transmit task (TB → DUT via uart_rx_pin)
    //
    // Sends a single byte as an 8N1 UART frame at the simulation baud rate.
    // The byte is sent LSB-first.
    // -------------------------------------------------------------------------
    task uart_send;
        input [7:0] byte_val;
        integer i;
        begin
            // Start bit
            uart_rx_pin = 1'b0;
            repeat (CLKS_PER_BIT) @(posedge clk);
            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_pin = byte_val[i];
                repeat (CLKS_PER_BIT) @(posedge clk);
            end
            // Stop bit
            uart_rx_pin = 1'b1;
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // Send a 3-byte config packet: [0xA5][CMD][VAL]
    // -------------------------------------------------------------------------
    task cfg_send;
        input [7:0] cmd;
        input [7:0] val;
        begin
            uart_send(8'hA5);
            uart_send(cmd);
            uart_send(val);
        end
    endtask

    // -------------------------------------------------------------------------
    // Test stimulus
    // -------------------------------------------------------------------------
    integer sample_count;
    integer pass_count;

    reg [7:0]  recv_b0, recv_b1, recv_b2, recv_b3;
    reg [15:0] recv_raw;    // bytes 0-1: raw ADC code
    reg [15:0] recv_mv;     // bytes 2-3: diff_mv (signed mV)

    initial begin
        sample_count  = 0;
        pass_count    = 0;
        uart_rx_pin   = 1'b1;   // idle high

        // Apply reset
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        $display("[TB-TOP] Reset released, waiting for UART samples...");

        // --- Phase 1: collect 3 samples with the default configuration ---
        // Slave returns SINC2DAT = 0x8000 (mid-scale).
        // Expected: raw = 0x8000, diff_mv = 0x0000 (0 mV differential).
        $display("[TB-TOP] Phase 1: default configuration (PGA x1.5)");
        repeat (3) begin
            uart_recv(recv_b0);
            uart_recv(recv_b1);
            uart_recv(recv_b2);
            uart_recv(recv_b3);
            recv_raw     = {recv_b0, recv_b1};
            recv_mv      = {recv_b2, recv_b3};
            sample_count = sample_count + 1;

            $display("[TB-TOP] Sample #%0d : raw=0x%04X  diff_mv=0x%04X (%0d mV)",
                     sample_count, recv_raw, recv_mv, $signed(recv_mv));

            if (recv_raw === 16'h8000 && recv_mv === 16'h0000) begin
                $display("[TB-TOP]   PASS – raw=0x8000  diff_mv=0 mV");
                pass_count = pass_count + 1;
            end else begin
                $display("[TB-TOP]   FAIL – expected raw=0x8000 diff_mv=0x0000, got raw=0x%04X diff_mv=0x%04X",
                         recv_raw, recv_mv);
            end
        end

        // --- Phase 2: send a config command to change PGA to ×2 (CMD=0x02, VAL=2) ---
        $display("[TB-TOP] Sending config: CMD=0x02 (PGA), VAL=0x02 (x2)");
        cfg_send(8'h02, 8'h02);
        $display("[TB-TOP] Config command sent, waiting for re-init and new samples...");

        // --- Phase 3: collect 2 samples after reconfiguration ---
        // Mid-scale code 0x8000 still gives diff_mv = 0 regardless of PGA.
        $display("[TB-TOP] Phase 3: after PGA reconfiguration (PGA x2)");
        repeat (2) begin
            uart_recv(recv_b0);
            uart_recv(recv_b1);
            uart_recv(recv_b2);
            uart_recv(recv_b3);
            recv_raw     = {recv_b0, recv_b1};
            recv_mv      = {recv_b2, recv_b3};
            sample_count = sample_count + 1;

            $display("[TB-TOP] Sample #%0d : raw=0x%04X  diff_mv=0x%04X (%0d mV)",
                     sample_count, recv_raw, recv_mv, $signed(recv_mv));

            if (recv_raw === 16'h8000 && recv_mv === 16'h0000) begin
                $display("[TB-TOP]   PASS – raw=0x8000  diff_mv=0 mV");
                pass_count = pass_count + 1;
            end else begin
                $display("[TB-TOP]   FAIL – expected raw=0x8000 diff_mv=0x0000, got raw=0x%04X diff_mv=0x%04X",
                         recv_raw, recv_mv);
            end
        end

        $display("[TB-TOP] ----------------------------------------");
        $display("[TB-TOP] %0d / %0d samples PASSED", pass_count, sample_count);
        if (pass_count === sample_count)
            $display("[TB-TOP] ALL TESTS PASSED");
        else
            $display("[TB-TOP] SOME TESTS FAILED");
        $display("[TB-TOP] ----------------------------------------");

        #200;
        $finish;
    end

    // -------------------------------------------------------------------------
    // Simulation timeout guard (10 ms simulated time)
    // -------------------------------------------------------------------------
    initial begin
        #10_000_000;
        $display("[TB-TOP] TIMEOUT: simulation exceeded 10 ms, aborting");
        $finish;
    end

    // -------------------------------------------------------------------------
    // Optional: dump waveforms for post-processing
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_ad5940_top.vcd");
        $dumpvars(0, tb_ad5940_top);
    end

endmodule
