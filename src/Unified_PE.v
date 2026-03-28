`include "_parameter.v"

module Unified_PE
(
    input clk,
    input reset,
    
    input [1:0] op_mode,   // MAC, MUL, ADD
    input clear_acc,       

    input signed[`DATA_WIDTH-1:0] in_A,
    input signed [`DATA_WIDTH-1:0] in_B,

    output reg signed [`DATA_WIDTH-1:0] out_val
);

    // Saturation Boundaries cho 16-bit Q8.7
    localparam signed [`DATA_WIDTH-1:0] MAX_POS = 16'sh7FFF; // +32767
    localparam signed[`DATA_WIDTH-1:0] MIN_NEG = 16'sh8000; // -32768
    
    // Tích thô 32-bit
    wire signed [31:0] mult_raw;
    assign mult_raw = in_A * in_B;
    
    // D?ch bit có làm tr?n (Round-to-nearest)
    wire signed [31:0] mult_shifted;
    assign mult_shifted = (mult_raw + 64) >>> `FRAC_BITS;

    // K?T QU? TRUNG GIAN 32-BIT
    reg signed[31:0] temp_result;

    // THANH GHI TÍCH L?Y 32-BIT (C?U TINH CH?NG TRÀN ? ÐÂY)
    reg signed [31:0] acc_reg;

    // MUX Logic (T? h?p)
    always @(*) begin
        case (op_mode)
            `MODE_MAC: temp_result = acc_reg + mult_shifted;
            `MODE_MUL: temp_result = mult_shifted;
            `MODE_ADD: temp_result = in_A + in_B;
            default:   temp_result = 0;
        endcase
    end

    // Sequential Logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            acc_reg <= 0;
            out_val <= 0;
        end else if (clear_acc) begin
            acc_reg <= 0;
            out_val <= 0;
        end else begin
            // LÝU ? 1: Thanh ghi tích l?y lýu tr?n v?n 32-bit, KHÔNG K?P BIÊN!
            acc_reg <= temp_result;
            
            // LÝU ? 2: Output ch? r?ng 16-bit -> Ép k?p biên ? c?a ra
            if (temp_result > 32767) begin
                out_val <= MAX_POS;
            end else if (temp_result < -32768) begin
                out_val <= MIN_NEG;
            end else begin
                out_val <= temp_result[15:0];
            end
        end
    end

endmodule