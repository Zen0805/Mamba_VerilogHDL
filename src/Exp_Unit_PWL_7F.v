`include "_parameter.v"

module Exp_Unit_PWL_7F
(
    input clk,
    input signed[`DATA_WIDTH-1:0] in_data, // Q8.7
    output reg signed[`DATA_WIDTH-1:0] out_data
);

    (* rom_style = "distributed" *) reg[31:0] rom [0:63];

    initial begin
        $readmemh("exp_pwl_coeffs_7F.mem", rom);
    end

    // 1. Address Generation
    // L?y 6 bit [9:4] tıõng ğıõng ğ? phân gi?i 0.125
    // Trong m? bù 2 âm: -8.0 (F800) có bit[9:4] = 000000 (Index 0)
    // -0.125 (FFFF) có bit[9:4] = 111111 (Index 63)
    wire[5:0] addr;
    assign addr = in_data[9:4];

    // 2. Fetch Coefficients
    wire signed [15:0] slope_comb;
    wire signed[15:0] intercept_comb;
    assign {slope_comb, intercept_comb} = rom[addr];
    
    // 3. Calculation
    reg signed [31:0] prod;
    reg signed[31:0] res;
    
    // Ngı?ng k?p (Anti-aliasing)
    localparam signed [15:0] MIN_EXP_INPUT = -16'd1024; // -8.0 trong Q8.7
    localparam signed [15:0] ONE_FIXED     =  16'd128;  // 1.0 trong Q8.7

    always @(posedge clk) begin
        // Tính y = ax + b
        prod = slope_comb * in_data;
        res = ((prod + 64) >>> `FRAC_BITS) + intercept_comb; //Round-to-Nearest
        
        // --- SATURATION & ANTI-ALIASING LOGIC ---
        // 1. N?u x > 0 (Trı?ng h?p d? thı?ng c?a Mamba), e^x ép v? 1.0
        if (in_data > 0) begin
            out_data <= ONE_FIXED;
        end 
        // 2. N?u x <= -8.0, e^x quá nh?, ph?n c?ng coi nhı b?ng 0
        else if (in_data <= MIN_EXP_INPUT) begin
            out_data <= 16'd0;              
        end 
        // 3. N?m trong d?i[-8.0 ğ?n 0], s? d?ng k?t qu? tính toán
        else begin
            // Ğ?m b?o k?t qu? không bao gi? r?t xu?ng s? âm do sai s? làm tr?n
            if (res < 0) out_data <= 16'd0;
            else out_data <= res[15:0];          
        end
    end

endmodule