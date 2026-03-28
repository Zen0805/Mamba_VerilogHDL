`timescale 1ns / 1ps

module BRAM_256b
#(
    parameter ADDR_WIDTH = 15, // 2^15 = 32,768 d?ng
    parameter DATA_WIDTH = 256 // 16 ph?n t? * 16 bit
)
(
    input clk,
    
    // Port A: Dùng ð? GHI (Write)
    input we_a,                  // Write Enable
    input [ADDR_WIDTH-1:0] addr_a,
    input [DATA_WIDTH-1:0] din_a,
    
    // Port B: Dùng ð? Ð?C (Read)
    input [ADDR_WIDTH-1:0] addr_b,
    output reg [DATA_WIDTH-1:0] dout_b
);

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] ram [0 : (1<<ADDR_WIDTH)-1];

    // Logic Ghi (Port A)
    always @(posedge clk) begin
        if (we_a) begin
            ram[addr_a] <= din_a;
        end
    end

    // Logic Ð?c (Port B)
    always @(posedge clk) begin
        dout_b <= ram[addr_b];
    end

endmodule