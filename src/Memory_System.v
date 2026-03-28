`include "_parameter.v"

module Memory_System
(
    input clk,
    input reset,

    // ============================================================
    // 1. GLOBAL CONTROL
    // ============================================================
    // 0: A -> Read (Input), B -> Write (Output)
    // 1: B -> Read (Input), A -> Write (Output)
    input bank_sel, 

    // ============================================================
    // 2. CORE INTERFACE (K?t n?i v?i Mamba_Top)
    // ============================================================
    
    // --- Data Read Port (Ping) ---
    input  [14:0] core_read_addr,
    output [255:0] core_read_data, 

    // --- Data Write Port (Pong) ---
    input         core_write_en,
    input  [14:0] core_write_addr,
    input  [255:0] core_write_data,

    // --- Weight Read Port ---
    input  [14:0] weight_read_addr,
    output [255:0] weight_read_data,
    
    // --- Const Read Port (NEW!) ---
    input  [14:0] const_read_addr,
    output [255:0] const_read_data,

    // ============================================================
    // 3. DMA INTERFACE (N?p d? li?u t? PS/Testbench)
    // ============================================================
    input         dma_write_en,
    input  [1:0]  dma_target, // 0: RAM_A, 1: RAM_B, 2: WEIGHT, 3: CONST
    input  [14:0] dma_addr,
    input  [255:0] dma_wdata
);

    // --- Internal Wires ---
    wire [255:0] out_ram_a, out_ram_b;
    
    // Logic Write Enable cho t?ng RAM (Ýu tiên DMA n?u DMA ðang kích ho?t)
    // Th?c t?: Khi ch?y Core th? DMA nên t?t, khi n?p DMA th? Core t?t.
    
    wire we_a = (dma_write_en && (dma_target == 2'd0)) || 
                (core_write_en && (bank_sel == 1) && !dma_write_en);
                
    wire we_b = (dma_write_en && (dma_target == 2'd1)) || 
                (core_write_en && (bank_sel == 0) && !dma_write_en);
                
    wire we_w = (dma_write_en && (dma_target == 2'd2));
    
    wire we_c = (dma_write_en && (dma_target == 2'd3));

    // Logic Address & Data Input cho RAM A/B
    // N?u DMA ghi -> L?y ð?a ch?/data DMA
    // N?u Core ghi -> L?y ð?a ch?/data Core
    wire [14:0] addr_a_wr = (dma_write_en && dma_target==0) ? dma_addr : core_write_addr;
    wire [255:0] din_a    = (dma_write_en && dma_target==0) ? dma_wdata : core_write_data;
    
    wire [14:0] addr_b_wr = (dma_write_en && dma_target==1) ? dma_addr : core_write_addr;
    wire [255:0] din_b    = (dma_write_en && dma_target==1) ? dma_wdata : core_write_data;

    // --- INSTANCE 4 BRAMs ---
    
    // 1. DATA RAM A
    BRAM_256b ram_a (
        .clk(clk),
        .we_a(we_a), 
        .addr_a(addr_a_wr), 
        .din_a(din_a),
        // Port B (Read): N?u bank_sel=0 th? Core ð?c, ngý?c l?i ð? 0
        .addr_b((bank_sel == 0) ? core_read_addr : 13'd0), 
        .dout_b(out_ram_a)
    );
    
    // 2. DATA RAM B
    BRAM_256b ram_b (
        .clk(clk),
        .we_a(we_b), 
        .addr_a(addr_b_wr), 
        .din_a(din_b),
        // Port B (Read): N?u bank_sel=1 th? Core ð?c
        .addr_b((bank_sel == 1) ? core_read_addr : 13'd0), 
        .dout_b(out_ram_b)
    );
    
    // 3. WEIGHT RAM (Core ch? ð?c)
    BRAM_256b ram_weight (
        .clk(clk),
        .we_a(we_w),       // Ch? DMA ghi
        .addr_a(dma_addr),
        .din_a(dma_wdata),
        
        .addr_b(weight_read_addr), // Core ð?c
        .dout_b(weight_read_data)
    );
    
    // 4. CONST RAM (NEW! Core ch? ð?c A, D)
    BRAM_256b ram_const (
        .clk(clk),
        .we_a(we_c),       // Ch? DMA ghi
        .addr_a(dma_addr),
        .din_a(dma_wdata),
        
        .addr_b(const_read_addr), // Core ð?c
        .dout_b(const_read_data)
    );

    assign core_read_data = (bank_sel == 0) ? out_ram_a : out_ram_b;

endmodule