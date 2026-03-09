// =============================================================================
// ad5940_adc_polling.v
//
// FPGA top-level module that reproduces the AD5940_ADCPolling.c example
// in synthesisable Verilog RTL.
//
// What this module does (mirrors AD5940_Main() in AD5940_ADCPolling.c):
//
//  1. Hardware-reset the AD5940 (drive afe_rst_n low then release).
//  2. Write the initialization register table (from AD5940_Initialize()).
//  3. Configure power/bandwidth (AD5940_AFEPwrBW – low-power, 250 kHz).
//  4. Enable the internal 1.8 V DAC reference power supply.
//  5. Select ADC inputs:
//       Positive  → VREF1P8DAC (internal 1.8 V reference)
//       Negative  → VSET1P1    (internal 1.11 V reference)
//       PGA gain  → 1.5×
//  6. Configure the digital filter chain:
//       ADC rate    : 800 kSPS   (16 MHz ADC clock)
//       SINC3 OSR   : 4          → 200 kSPS after SINC3
//       SINC2 OSR   : 1333       → ~150 Hz output rate
//       Notch bypass: enabled    (use SINC2 output directly)
//  7. Enable SINC2+Notch block, INTC1 (all interrupt sources).
//  8. Power up the ADC and start continuous conversion.
//  9. Poll INTCFLAG1 for SINC2RDY (bit 2).
//     When set:
//       a. Clear the flag (write INTCCLR).
//       b. Read SINC2DAT (16-bit result in bits [15:0]).
//       c. Assert adc_valid for one clock cycle and put the code in adc_data.
//
// The output code can be converted to voltage by the formula from AD5940_ADCPolling.c:
//   diff_volt = (code / 32768.0 - 1.0) / 1.5 * 1.82   [V, differential]
//   volt      = diff_volt + 1.11                        [V, single-ended]
//
// Parameters
//   CLK_DIV      – passed to ad5940_spi_master; controls SPI clock speed.
//   RESET_CYCLES – how many clk cycles afe_rst_n is held low (default: 100 µs @ 50 MHz).
//   BOOT_CYCLES  – how many clk cycles to wait after reset before SPI access
//                  (default: 10 ms @ 50 MHz).
//
// Register values computed from the C source:
//   Step 14  : PMBW  (0x22F0) ← 0x0000000C  [LP mode, 250 kHz BW]
//   Step 15  : AFECON(0x2000) ← 0x00180040  [DACREFEN + DACEN, ALDOILIMITEN kept]
//   Step 16  : ADCCON(0x21A8) ← 0x00010812  [MuxP=0x12 MuxN=0x08 PGA=1 (×1.5)]
//   Step 17  : ADCFILTERCON(0x2044) ← 0x00001B11  [SINC3OSR=4 SINC2OSR=1333 BpNotch]
//   Step 18  : AFECON(0x2000) ← 0x00190040  [+ SINC2NOTCH]
//   Step 19  : INTCSEL1(0x300C) ← 0xFFFFFFFF [all interrupts → INTC1]
//   Step 20  : AFECON(0x2000) ← 0x001900C0  [+ ADCEN]
//   Step 21  : AFECON(0x2000) ← 0x001901C0  [+ ADCCONVEN]
// =============================================================================
`timescale 1ns/1ps

module ad5940_adc_polling #(
    parameter CLK_DIV      = 4,         // SPI clock: sys_clk / (2*CLK_DIV)
    parameter RESET_CYCLES = 5_000,     // afe_rst_n low duration  (default ~100 µs @ 50 MHz)
    parameter BOOT_CYCLES  = 500_000    // wait after reset        (default ~10 ms @ 50 MHz)
)(
    input  wire        clk,
    input  wire        rst_n,

    // ADC result output
    output reg         adc_valid,   // one-cycle pulse: adc_data holds a new sample
    output reg  [15:0] adc_data,    // ADC code (0–65535); use formula above for voltage

    // Debug
    output wire [3:0]  state_out,   // current main-state (for debug / ILA)

    // AD5940 hardware pins
    output reg         afe_rst_n,   // hardware reset to AD5940 (active-low)
    output wire        spi_cs_n,
    output wire        spi_clk,
    output wire        spi_mosi,
    input  wire        spi_miso
);

    // =========================================================================
    // Main-FSM state encoding
    // =========================================================================
    localparam [3:0]
        MS_RESET      = 4'd0,   // hold afe_rst_n low
        MS_BOOT_WAIT  = 4'd1,   // wait for AD5940 internal boot
        MS_INIT_ISSUE = 4'd2,   // issue one init write
        MS_INIT_WAIT  = 4'd3,   // wait for write to complete
        MS_POLL_ISSUE = 4'd4,   // read INTCFLAG1
        MS_POLL_WAIT  = 4'd5,   // wait for read, check SINC2RDY
        MS_CLR_ISSUE  = 4'd6,   // write INTCCLR to clear SINC2RDY
        MS_CLR_WAIT   = 4'd7,   // wait for clear to complete
        MS_RES_ISSUE  = 4'd8,   // read SINC2DAT
        MS_RES_WAIT   = 4'd9;   // wait for read, output ADC code

    reg [3:0] main_state;
    assign state_out = main_state;

    // =========================================================================
    // Delay counter (shared for reset-hold and boot-wait phases)
    // =========================================================================
    reg [31:0] delay_cnt;

    // =========================================================================
    // Init-sequence ROM
    // =========================================================================
    // 22 sequential register writes that reproduce:
    //   AD5940_Initialize() + AD5940_AFEPwrBW() + ADC/filter setup + start.
    // Combinational lookup indexed by init_step.
    // =========================================================================
    localparam INIT_STEPS = 22;
    reg [4:0] init_step;

    reg [15:0] rom_addr;
    reg [31:0] rom_data;

    always @(*) begin
        case (init_step)
            // --- AD5940_Initialize() magic registers ---
            5'd0 : begin rom_addr = 16'h0908; rom_data = 32'h000002C9; end
            5'd1 : begin rom_addr = 16'h0C08; rom_data = 32'h0000206C; end
            5'd2 : begin rom_addr = 16'h21F0; rom_data = 32'h00000010; end
            5'd3 : begin rom_addr = 16'h0410; rom_data = 32'h000002C9; end
            5'd4 : begin rom_addr = 16'h0A28; rom_data = 32'h00000009; end
            5'd5 : begin rom_addr = 16'h238C; rom_data = 32'h00000104; end
            5'd6 : begin rom_addr = 16'h0A04; rom_data = 32'h00004859; end
            5'd7 : begin rom_addr = 16'h0A04; rom_data = 32'h0000F27B; end
            5'd8 : begin rom_addr = 16'h0A00; rom_data = 32'h00008009; end
            5'd9 : begin rom_addr = 16'h22F0; rom_data = 32'h00000000; end // PMBW init
            5'd10: begin rom_addr = 16'h2230; rom_data = 32'hDE87A5AF; end
            5'd11: begin rom_addr = 16'h2250; rom_data = 32'h0000103F; end
            5'd12: begin rom_addr = 16'h22B0; rom_data = 32'h0000203C; end
            5'd13: begin rom_addr = 16'h2230; rom_data = 32'hDE87A5A0; end
            // --- AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ) ---
            // PMBW = AFEPWR_LP(0) | (AFEBW_250KHZ(3) << BITP_SYSBW(2)) = 0x0C
            5'd14: begin rom_addr = 16'h22F0; rom_data = 32'h0000000C; end
            // --- AD5940_AFECtrlS(DACREFPWR|HSDACPWR, bTRUE) ---
            // AFECON reset=0x00080000 | DACREFEN(bit20)=0x100000 | DACEN(bit6)=0x40
            5'd15: begin rom_addr = 16'h2000; rom_data = 32'h00180040; end
            // --- AD5940_ADCBaseCfgS: MuxP=0x12 MuxN=0x8 PGA=1(×1.5) ---
            // ADCCON = MuxP[5:0] | (MuxN<<8) | (PGA<<16) = 0x12|0x800|0x10000
            5'd16: begin rom_addr = 16'h21A8; rom_data = 32'h00010812; end
            // --- AD5940_ADCFilterCfgS: SINC3OSR=4(1) SINC2OSR=1333(11) Rate=800k BpNotch ---
            // ADCFILTERCON = Rate(1)|BpNotch(0x10)|(SINC2OSR(11)<<8)|(SINC3OSR(1)<<12)
            //              = 0x1 | 0x10 | 0xB00 | 0x1000 = 0x1B11
            5'd17: begin rom_addr = 16'h2044; rom_data = 32'h00001B11; end
            // --- AD5940_AFECtrlS(SINC2NOTCH, bTRUE): set bit16 in AFECON ---
            5'd18: begin rom_addr = 16'h2000; rom_data = 32'h00190040; end
            // --- AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE) ---
            5'd19: begin rom_addr = 16'h300C; rom_data = 32'hFFFFFFFF; end
            // --- AD5940_ADCPowerCtrlS(bTRUE): set ADCEN(bit7) ---
            5'd20: begin rom_addr = 16'h2000; rom_data = 32'h001900C0; end
            // --- AD5940_ADCConvtCtrlS(bTRUE): set ADCCONVEN(bit8) ---
            5'd21: begin rom_addr = 16'h2000; rom_data = 32'h001901C0; end

            default: begin rom_addr = 16'h0000; rom_data = 32'h00000000; end
        endcase
    end

    // =========================================================================
    // SPI master interface wires
    // =========================================================================
    reg        spi_wr_en;
    reg        spi_rd_en;
    reg [15:0] spi_addr;
    reg [31:0] spi_wdata;
    wire [31:0] spi_rdata;
    wire        spi_busy;
    wire        spi_done;

    ad5940_spi_master #(
        .CLK_DIV (CLK_DIV)
    ) u_spi_master (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_en    (spi_wr_en),
        .rd_en    (spi_rd_en),
        .addr     (spi_addr),
        .wdata    (spi_wdata),
        .rdata    (spi_rdata),
        .busy     (spi_busy),
        .done     (spi_done),
        .spi_cs_n (spi_cs_n),
        .spi_clk  (spi_clk),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );

    // =========================================================================
    // Main state machine
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            main_state <= MS_RESET;
            afe_rst_n  <= 1'b0;
            delay_cnt  <= 32'd0;
            init_step  <= 5'd0;
            spi_wr_en  <= 1'b0;
            spi_rd_en  <= 1'b0;
            spi_addr   <= 16'h0;
            spi_wdata  <= 32'h0;
            adc_valid  <= 1'b0;
            adc_data   <= 16'h0;
        end else begin
            // Default: clear single-cycle signals
            spi_wr_en <= 1'b0;
            spi_rd_en <= 1'b0;
            adc_valid <= 1'b0;

            case (main_state)
                // -----------------------------------------------------------------
                // Hold afe_rst_n low for RESET_CYCLES system clocks.
                // -----------------------------------------------------------------
                MS_RESET: begin
                    afe_rst_n <= 1'b0;
                    if (delay_cnt >= RESET_CYCLES - 1) begin
                        delay_cnt  <= 32'd0;
                        main_state <= MS_BOOT_WAIT;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                // Release reset, wait BOOT_CYCLES for AD5940 internal initialisation.
                // -----------------------------------------------------------------
                MS_BOOT_WAIT: begin
                    afe_rst_n <= 1'b1;
                    if (delay_cnt >= BOOT_CYCLES - 1) begin
                        delay_cnt  <= 32'd0;
                        init_step  <= 5'd0;
                        main_state <= MS_INIT_ISSUE;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                // Issue one write from the init ROM, then wait.
                // After all 22 writes transition to polling.
                // -----------------------------------------------------------------
                MS_INIT_ISSUE: begin
                    if (init_step < INIT_STEPS) begin
                        spi_wr_en  <= 1'b1;
                        spi_addr   <= rom_addr;
                        spi_wdata  <= rom_data;
                        main_state <= MS_INIT_WAIT;
                    end else begin
                        main_state <= MS_POLL_ISSUE;
                    end
                end

                MS_INIT_WAIT: begin
                    if (spi_done) begin
                        init_step  <= init_step + 1'b1;
                        main_state <= MS_INIT_ISSUE;
                    end
                end

                // -----------------------------------------------------------------
                // Poll: read INTCFLAG1 (0x3014) and check SINC2RDY (bit 2).
                // -----------------------------------------------------------------
                MS_POLL_ISSUE: begin
                    spi_rd_en  <= 1'b1;
                    spi_addr   <= 16'h3014; // REG_INTC_INTCFLAG1
                    main_state <= MS_POLL_WAIT;
                end

                MS_POLL_WAIT: begin
                    if (spi_done) begin
                        if (spi_rdata[2]) begin
                            // AFEINTSRC_SINC2RDY (bit 2) is set → data ready
                            main_state <= MS_CLR_ISSUE;
                        end else begin
                            // Not ready yet – poll again
                            main_state <= MS_POLL_ISSUE;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Clear the SINC2RDY flag in INTCCLR (0x3004), bit 2.
                // -----------------------------------------------------------------
                MS_CLR_ISSUE: begin
                    spi_wr_en  <= 1'b1;
                    spi_addr   <= 16'h3004; // REG_INTC_INTCCLR
                    spi_wdata  <= 32'h00000004; // AFEINTSRC_SINC2RDY
                    main_state <= MS_CLR_WAIT;
                end

                MS_CLR_WAIT: begin
                    if (spi_done) begin
                        main_state <= MS_RES_ISSUE;
                    end
                end

                // -----------------------------------------------------------------
                // Read SINC2DAT (0x2080) – 16-bit ADC result in bits [15:0].
                // -----------------------------------------------------------------
                MS_RES_ISSUE: begin
                    spi_rd_en  <= 1'b1;
                    spi_addr   <= 16'h2080; // REG_AFE_SINC2DAT
                    main_state <= MS_RES_WAIT;
                end

                MS_RES_WAIT: begin
                    if (spi_done) begin
                        adc_valid  <= 1'b1;
                        adc_data   <= spi_rdata[15:0];
                        main_state <= MS_POLL_ISSUE; // back to polling
                    end
                end

                default: main_state <= MS_RESET;
            endcase
        end
    end

endmodule
