`include "_parameter.v"

module SiLU_Unit_PWL_7F
(
    input clk,
    input signed [`DATA_WIDTH-1:0] in_data, // Q8.7
    output reg signed [`DATA_WIDTH-1:0] out_data
);

    (* rom_style = "distributed" *) reg [31:0] rom[0:63];

    initial begin
        $readmemh("silu_pwl_coeffs_7F.mem", rom);
    end

    // 1. L?y ð?a ch? ROM (Address Generation)
    // Ð? phân gi?i 0.25 (týõng ðýõng bit 5 trong Q8.7)
    // L?y 6 bit: in_data[10:5] chia vùng[-8.0, 8.0] thành 64 ðo?n
    wire [5:0] addr;
    assign addr = in_data[10:5];

    // 2. L?y h? s?
    wire signed [15:0] slope_comb;
    wire signed [15:0] intercept_comb;
    assign {slope_comb, intercept_comb} = rom[addr];
    
    // 3. Thanh ghi tính toán
    reg signed[31:0] prod;
    reg signed [31:0] res;
    
    // H?ng s? biên (Bypass Thresholds)
    localparam signed [15:0] LIMIT_POS =  16'd1024; // +8.0 (8 * 128)
    localparam signed [15:0] LIMIT_NEG = -16'd1024; // -8.0 (-8 * 128)
    localparam signed [31:0] MAX_OUT   =  32'd32767;
    localparam signed[31:0] MIN_OUT   = -32'd32768;

    always @(posedge clk) begin
        // Tính y = ax + b
        prod = slope_comb * in_data;
        
        // D?ch ph?i FRAC_BITS, có c?ng thêm 64 (týõng ðýõng 0.5) ð? làm tr?n s? h?c (Round to nearest)
        res = ((prod + 64) >>> `FRAC_BITS) + intercept_comb;
        
        // --- BYPASS & SATURATION LOGIC ---
        
        // N?u x >= 8.0 -> SiLU(x) x?p x? x
        if (in_data >= LIMIT_POS) begin
            out_data <= in_data;
        end 
        // N?u x <= -8.0 -> SiLU(x) x?p x? 0
        else if (in_data <= LIMIT_NEG) begin
            out_data <= 16'd0;
        end 
        // Vùng cong [-8.0, 8.0] -> L?y k?t qu? PWL
        else begin
            // K?p ch?ng tràn an toàn (M?c dù k?t qu? SiLU không bao gi? vý?t ngý?ng này)
            if (res > MAX_OUT) 
                out_data <= MAX_OUT[15:0];
            else if (res < MIN_OUT) 
                out_data <= MIN_OUT[15:0];
            else 
                out_data <= res[15:0];
        end
    end

endmodule