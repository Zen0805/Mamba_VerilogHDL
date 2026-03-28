`include "_parameter.v"

module Softplus_Unit_PWL_7F
(
    input clk,
    input signed [`DATA_WIDTH-1:0] in_data, // Q8.7
    output reg signed [`DATA_WIDTH-1:0] out_data
);

    (* rom_style = "distributed" *) reg [31:0] rom[0:63];

    initial begin
        $readmemh("softplus_pwl_coeffs_7F.mem", rom);
    end

    // 1. Address Generation
    // Bý?c nh?y c?a ta là 0.25. 
    // Trong Q8.7, 0.25 týõng ðýõng giá tr? 32 (0000_0000_0010_0000)
    // T?c là bit[5] ð?i di?n cho 0.25.
    // D?i [-8.0, 8.0) s? ðý?c cover tr?n v?n b?i 6 bit[10:5]
    wire [5:0] addr;
    assign addr = in_data[10:5];

    // 2. Fetch Coefficients
    wire signed [15:0] slope_comb;
    wire signed [15:0] intercept_comb;
    assign {slope_comb, intercept_comb} = rom[addr];
    
    // 3. Calculation
    reg signed[31:0] prod;
    reg signed [31:0] res;
    
    // Ngý?ng k?p (Anti-aliasing / Bypass Logic)
    localparam signed [15:0] MIN_INPUT = -16'd1024; // -8.0 trong Q8.7
    localparam signed [15:0] MAX_INPUT =  16'd1024; //  8.0 trong Q8.7

    always @(posedge clk) begin
        // Tính y = ax + b
        prod = slope_comb * in_data;
        res = ((prod + 64) >>> `FRAC_BITS) + intercept_comb; // Round-to-Nearest
        
        // --- SATURATION & BYPASS LOGIC ---
        // 1. N?u x >= 8.0: Softplus(x) ? x. Ti?t ki?m tính toán và gi? ð? chính xác tuy?t ð?i.
        if (in_data >= MAX_INPUT) begin
            out_data <= in_data;
        end 
        // 2. N?u x <= -8.0: Softplus(x) ? 0. Ph?n c?ng coi nhý b?ng 0.
        else if (in_data <= MIN_INPUT) begin
            out_data <= 16'd0;              
        end 
        // 3. N?m trong d?i[-8.0 ð?n 8.0), s? d?ng k?t qu? tính toán PWL
        else begin
            // Ð?m b?o k?t qu? không bao gi? r?t xu?ng s? âm (Softplus luôn > 0)
            if (res < 0) out_data <= 16'd0;
            else out_data <= res[15:0];          
        end
    end

endmodule