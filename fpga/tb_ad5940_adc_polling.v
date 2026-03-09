// =============================================================================
// tb_ad5940_adc_polling.v
//
// Testbench for ad5940_adc_polling.v.
//
// A simple behavioural SPI slave (ad5940_spi_slave_model) is included.
// It mimics the AD5940 SPI protocol:
//   • Phase 1: receives SETADDR + 16-bit address → latches the address.
//   • Phase 2: for READREG returns a canned register value:
//                INTCFLAG1 (0x3014) → 0x00000004  (SINC2RDY = 1)
//                SINC2DAT  (0x2080) → 0x00008000  (mid-scale ADC code)
//              for WRITEREG: silently discards the data.
//
// Simulation speed-up: RESET_CYCLES and BOOT_CYCLES are overridden to small
// values via parameter override.
//
// Expected behaviour:
//   • Module resets the AD5940, writes 22 init registers, then polls.
//   • The slave always returns SINC2RDY=1, so every poll immediately triggers
//     a flag-clear + SINC2DAT read.
//   • adc_data should read back 0x8000 on every valid pulse.
// =============================================================================
`timescale 1ns/1ps

// =============================================================================
// Behavioural SPI slave model for AD5940
// =============================================================================
module ad5940_spi_slave_model (
    input  wire spi_cs_n,
    input  wire spi_clk,
    input  wire spi_mosi,
    output reg  spi_miso
);

    localparam [7:0] SPICMD_SETADDR  = 8'h20;
    localparam [7:0] SPICMD_READREG  = 8'h6d;
    localparam [7:0] SPICMD_WRITEREG = 8'h2d;

    // State across CS# phases
    reg [15:0] latched_addr;  // address from Phase 1
    reg        in_phase2;     // 0 = expecting Phase1, 1 = expecting Phase2

    // Per-phase shift registers (simulation only – blocking assignments fine)
    reg [47:0] rx_shift;  // MOSI bits shifted in
    reg [47:0] tx_shift;  // MISO bits to shift out
    integer    bit_count;

    // Canned register responses
    function [31:0] reg_response;
        input [15:0] addr;
        begin
            case (addr)
                // INTCFLAG1: AFEINTSRC_SINC2RDY (bit 2) always set
                16'h3014: reg_response = 32'h00000004;
                // SINC2DAT: mid-scale = 0x8000 (differential zero)
                16'h2080: reg_response = 32'h00008000;
                // All other registers: return 0
                default:  reg_response = 32'h00000000;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Initialise
    // ------------------------------------------------------------------
    // in_phase2 is set to 1 so the unavoidable spurious posedge spi_cs_n
    // at simulation time 0 (when the DUT drives CS# X→1 during reset)
    // correctly resets it to 0 via the else branch, leaving the slave
    // ready to process a real Phase-1 transaction.
    // ------------------------------------------------------------------
    initial begin
        spi_miso    = 1'b0;
        latched_addr = 16'h0;
        in_phase2   = 1'b1;   // absorbs the t=0 X→1 spurious posedge
        rx_shift    = 48'h0;
        tx_shift    = 48'h0;
        bit_count   = 0;
    end

    // ------------------------------------------------------------------
    // CS# falling edge – start of a new SPI phase
    // ------------------------------------------------------------------
    always @(negedge spi_cs_n) begin
        rx_shift  = 48'h0;
        bit_count = 0;
        spi_miso  = 1'b0;

        if (in_phase2) begin
            // Pre-load tx_shift for a potential READREG response.
            //
            // The slave drives MISO on the falling SCLK edge; the master
            // samples on the following rising SCLK edge.  The very first
            // rising edge of Phase-2 occurs *before* any falling edge, so
            // it samples the spi_miso=0 that was set at the negedge-CS# event.
            // This creates a 1-cycle offset: data bit [j] exits tx_shift[47]
            // at falling edge #(47-j) and is sampled at rising edge #(48-j).
            // After 48 total risings the master extracts rdata = shift_rx[31:0]
            // where shift_rx[k] = MISO from rising #(48-k).
            // Therefore shift_rx[k] = data[k]  iff  data[k] starts at
            // tx_shift bit position k+1.
            // Placing data at tx_shift[32:1] (= left-shifted by 1) achieves this.
            tx_shift = {15'h0000, reg_response(latched_addr), 1'b0};
        end else begin
            tx_shift = 48'h0;
        end
    end

    // ------------------------------------------------------------------
    // CS# rising edge – end of phase, decode received data
    //
    // After N rising SCLK edges with shift_in = {shift_in[46:0], mosi},
    // the first received bit ends up at shift_in[N-1], i.e. the bits are
    // stored MSB-first starting from bit N-1 downward.
    //
    // Phase 1 = 24 bits:  rx_shift[23:16] = SETADDR cmd
    //                     rx_shift[15:0]  = 16-bit register address
    // ------------------------------------------------------------------
    always @(posedge spi_cs_n) begin
        if (!in_phase2) begin
            // End of Phase 1: latch address
            if (rx_shift[23:16] == SPICMD_SETADDR)
                latched_addr = rx_shift[15:0];
            in_phase2 = 1'b1;
        end else begin
            // End of Phase 2
            in_phase2 = 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // Rising edge of SCLK – shift MOSI into rx_shift
    // ------------------------------------------------------------------
    always @(posedge spi_clk) begin
        if (!spi_cs_n) begin
            rx_shift  = {rx_shift[46:0], spi_mosi};
            bit_count = bit_count + 1;
        end
    end

    // ------------------------------------------------------------------
    // Falling edge of SCLK – drive MISO with next tx_shift bit
    // ------------------------------------------------------------------
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
module tb_ad5940_adc_polling;

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk;
    reg  rst_n;

    wire        adc_valid;
    wire [15:0] adc_data;
    wire [3:0]  state_out;

    wire        afe_rst_n;
    wire        spi_cs_n;
    wire        spi_clk;
    wire        spi_mosi;
    wire        spi_miso;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // Use tiny RESET_CYCLES / BOOT_CYCLES to make the simulation fast.
    // CLK_DIV=2 → SCLK = sys_clk/4 (plenty of timing margin).
    // -------------------------------------------------------------------------
    ad5940_adc_polling #(
        .CLK_DIV      (2),
        .RESET_CYCLES (10),
        .BOOT_CYCLES  (20)
    ) dut (
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

    // -------------------------------------------------------------------------
    // Behavioural SPI slave
    // -------------------------------------------------------------------------
    ad5940_spi_slave_model slave (
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
    // Test stimulus
    // -------------------------------------------------------------------------
    integer valid_count;
    integer pass_count;

    initial begin
        valid_count = 0;
        pass_count  = 0;

        // Apply reset
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        $display("[TB] Reset released, waiting for ADC valid pulses...");

        // Collect 5 valid samples
        repeat (5) begin
            @(posedge adc_valid);
            valid_count = valid_count + 1;
            $display("[TB] adc_valid pulse #%0d : adc_data = 0x%04X  (state=%0d)",
                     valid_count, adc_data, state_out);
            if (adc_data === 16'h8000) begin
                $display("[TB]   PASS – adc_data matches expected 0x8000");
                pass_count = pass_count + 1;
            end else begin
                $display("[TB]   FAIL – expected 0x8000, got 0x%04X", adc_data);
            end
        end

        $display("[TB] ----------------------------------------");
        $display("[TB] %0d / %0d samples PASSED", pass_count, valid_count);
        if (pass_count === valid_count)
            $display("[TB] ALL TESTS PASSED");
        else
            $display("[TB] SOME TESTS FAILED");
        $display("[TB] ----------------------------------------");

        #200;
        $finish;
    end

    // -------------------------------------------------------------------------
    // Simulation timeout guard (10 ms simulated time)
    // -------------------------------------------------------------------------
    initial begin
        #10_000_000;
        $display("[TB] TIMEOUT: simulation exceeded 10 ms, aborting");
        $finish;
    end

    // -------------------------------------------------------------------------
    // Optional: dump waveforms for post-processing
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("tb_ad5940_adc_polling.vcd");
        $dumpvars(0, tb_ad5940_adc_polling);
    end

endmodule
