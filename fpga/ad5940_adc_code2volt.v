// =============================================================================
// ad5940_adc_code2volt.v
//
// Combinational fixed-point converter: 16-bit AD5940 ADC code → signed
// differential voltage in millivolts (mV).
//
// Implements the same algorithm as AD5940_ADCCode2Volt() in ad5940lib/ad5940.c:
//
//   kFactor   = 1.835 / 1.82
//   tmp       = (int32_t)adc_code - 32768          // remove offset-binary bias
//   tmp       = tmp / pga_gain                      // undo ADC PGA amplification
//   diff_volt = tmp * VRef1p82 * kFactor / 32768    // scale to volts
//             = tmp * 1.835 / 32768                 // (VRef1p82 * kFactor = 1.835 V)
//
// This module uses Q20 fixed-point arithmetic to avoid floating-point hardware:
//
//   diff_mv = (adc_code - 32768) * COEFF_PGA >>> 20
//
// where COEFF_PGA = round(1835 * 2^20 / (32768 * pga_gain)):
//   PGA ×1   → 58720    (max error < 0.1 mV)
//   PGA ×1.5 → 39147    (max error < 0.2 mV)
//   PGA ×2   → 29360    (max error < 0.1 mV)
//   PGA ×4   → 14680    (max error < 0.1 mV)
//   PGA ×9   →  6524    (max error < 0.1 mV)
//
// Ports:
//   adc_code  [15:0]  – raw 16-bit ADC result (offset-binary; 0x8000 = 0 V diff.)
//   pga_sel   [2:0]   – PGA gain selection:
//                         0 → ×1      (ADCPGA_1)
//                         1 → ×1.5    (ADCPGA_1P5, default)
//                         2 → ×2      (ADCPGA_2)
//                         3 → ×4      (ADCPGA_4)
//                         4 → ×9      (ADCPGA_9)
//                         5-7 → ×1.5  (reserved, treated as default)
//   diff_mv   [15:0]  – signed 16-bit differential voltage in mV
//                       (= V_MuxP − V_MuxN; range ≈ ±1835 mV at PGA ×1)
//   single_mv [15:0]  – signed 16-bit single-ended voltage in mV
//                       (= diff_mv + 1110; adds the VSET1P1 reference, ~1.11 V)
//                       Meaningful when MuxN = VSET1P1 (modes 0 and 1).
// =============================================================================
`timescale 1ns/1ps

module ad5940_adc_code2volt (
    input  wire [15:0] adc_code,   // raw ADC result (offset-binary)
    input  wire [2:0]  pga_sel,    // PGA gain selection (0-4)

    output wire [15:0] diff_mv,    // signed 16-bit: differential voltage (mV)
    output wire [15:0] single_mv   // signed 16-bit: single-ended voltage (mV)
);

    // =========================================================================
    // Q20 fixed-point coefficients: COEFF = round(1835 * 2^20 / (32768 * gain))
    // All values fit in 16 bits.
    // =========================================================================
    localparam [15:0] COEFF_1   = 16'd58720;  // PGA ×1
    localparam [15:0] COEFF_1P5 = 16'd39147;  // PGA ×1.5  (default)
    localparam [15:0] COEFF_2   = 16'd29360;  // PGA ×2
    localparam [15:0] COEFF_4   = 16'd14680;  // PGA ×4
    localparam [15:0] COEFF_9   = 16'd6524;   // PGA ×9

    // VSET1P1 reference voltage in mV (1.11 V nominal)
    localparam signed [15:0] VSET1P1_MV = 16'sd1110;

    // =========================================================================
    // Intermediate computation
    // =========================================================================
    // tmp: adc_code zero-extended to 17 bits, then biased by -32768.
    //      Range: -32768 (code=0x0000) to +32767 (code=0xFFFF).
    //      17-bit signed so that both extremes are representable.
    // =========================================================================
    reg signed [16:0] tmp;         // adc_code − 32768  (17-bit signed)
    reg [15:0]        coeff;       // Q20 coefficient   (16-bit unsigned)
    reg signed [33:0] product;     // tmp × coeff       (34-bit signed)
    reg signed [15:0] diff_mv_r;   // scaled result     (16-bit signed, mV)
    reg signed [15:0] single_mv_r; // diff + 1110 mV    (16-bit signed, mV)

    always @(*) begin
        // Step 1 – remove offset-binary bias
        tmp = {1'b0, adc_code} - 17'sd32768;

        // Step 2 – select Q20 coefficient for the active PGA gain
        case (pga_sel)
            3'd0:    coeff = COEFF_1;
            3'd1:    coeff = COEFF_1P5;
            3'd2:    coeff = COEFF_2;
            3'd3:    coeff = COEFF_4;
            3'd4:    coeff = COEFF_9;
            default: coeff = COEFF_1P5;  // reserved: treat as ×1.5
        endcase

        // Step 3 – Q20 multiply: product is 34-bit signed.
        //   The assignment target (product) is 34 bits wide.  Verilog evaluates
        //   the multiplication in that 34-bit context, sign-extending both
        //   operands before the multiply.
        //   $signed(tmp): 17-bit signed value, sign-extended to 34 bits.
        //   $signed({1'b0, coeff}): {1'b0, coeff} is 17-bit with MSB=0
        //   (always positive), treated as a positive signed value and
        //   zero-extended to 34 bits.
        product = $signed(tmp) * $signed({1'b0, coeff});

        // Step 4 – arithmetic right-shift by 20 (extract integer mV part).
        //   product[33] is the sign bit; bits [33:20] are the 14-bit integer
        //   part. Sign-extend to 16 bits for the output.
        diff_mv_r = {{2{product[33]}}, product[33:20]};

        // Step 5 – add VSET1P1 reference for single-ended measurement.
        //   Meaningful when MuxN = VSET1P1 (measurement modes 0 and 1).
        single_mv_r = diff_mv_r + VSET1P1_MV;
    end

    assign diff_mv   = diff_mv_r;
    assign single_mv = single_mv_r;

endmodule
