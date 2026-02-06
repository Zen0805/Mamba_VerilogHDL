`timescale 1ns / 1ps
`include "_parameter.v"

module tb_Debug_Phase;

    // --- Signals ---
    reg clk, reset;
    reg start_system;
    wire done_system;
    
    // Internal Wires
    wire bank_sel;
    wire [14:0] mem_read_addr, mem_write_addr, mem_weight_addr, mem_const_addr;
    wire [255:0] mem_read_data, mem_write_data, mem_weight_data, mem_const_data;
    wire mem_write_en;
    
    // Interface Wires
    wire [2:0] mode_select;
    wire lin_start, lin_done, lin_en; wire [15:0] lin_len;
    wire signed [15:0] lin_x_val; wire signed [255:0] lin_W_vals, lin_bias_vals, lin_y_out;
    
    // DMA
    reg dma_write_en;
    reg [1:0] dma_target;
    reg [14:0] dma_addr;
    reg [255:0] dma_wdata;

    // --- MEMORY MAP DEBUG ---
    localparam ADDR_DEBUG_IN  = 15'd0;     // RAM A
    localparam W_BASE_OUTPROJ = 15'd2432;  // Weight RAM
    localparam ADDR_DEBUG_OUT = 15'd20000; // RAM B (Output)

    // --- DATA ARRAYS ---
    reg signed [15:0] file_input_flat [0:127999];  
    reg signed [15:0] file_weight_flat [0:8191];
    reg signed [15:0] file_golden_flat [0:63999];

    // --- STATS VARIABLES ---
    integer i, j, k, idx_flat;
    reg signed [15:0] got, exp;
    integer abs_diff;
    integer errors;
    
    integer max_diff;
    real sum_diff;
    integer perf_match;
    real total_samples;
    
    // Tolerance cho mô ph?ng Fixed-point 16-bit
    localparam TOLERANCE = 100; 

    // --- INSTANTIATE ---
    Global_Controller_Full_Flow u_ctrl (
        .clk(clk), .reset(reset), .start_system(start_system), .done_system(done_system),
        .bank_sel(bank_sel),
        .core_read_addr(mem_read_addr), .core_read_data(mem_read_data),
        .weight_read_addr(mem_weight_addr), .weight_read_data(mem_weight_data),
        .const_read_addr(mem_const_addr), .const_read_data(mem_const_data),
        .core_write_en(mem_write_en), .core_write_addr(mem_write_addr), .core_write_data(mem_write_data),
        .mode_select(mode_select),
        .lin_start(lin_start), .lin_en(lin_en), .lin_len(lin_len), .lin_done(lin_done),
        .lin_x_val(lin_x_val), .lin_W_vals(lin_W_vals), .lin_bias_vals(lin_bias_vals), .lin_y_out_in(lin_y_out),
        // Dummy
        .conv_start(), .conv_en(), .conv_valid_in(), .conv_ready_in(1'b1), .conv_valid_out(1'b0),
        .conv_x_vec(), .conv_w_vec(), .conv_b_vec(), .conv_y_vec(256'd0),
        .scan_start(), .scan_en(), .scan_done(1'b1), .scan_clear_h(),
        .scan_delta_val(), .scan_x_val(), .scan_D_val(), .scan_gate_val(),
        .scan_A_vec(), .scan_B_vec(), .scan_C_vec(), .scan_y_out(16'd0),
        .softplus_in(), .softplus_out(16'd0)
    );

    Memory_System u_mem (
        .clk(clk), .reset(reset), .bank_sel(bank_sel),
        .core_read_addr(mem_read_addr), .core_read_data(mem_read_data),
        .core_write_en(mem_write_en), .core_write_addr(mem_write_addr), .core_write_data(mem_write_data),
        .weight_read_addr(mem_weight_addr), .weight_read_data(mem_weight_data),
        .const_read_addr(mem_const_addr), .const_read_data(mem_const_data),
        .dma_write_en(dma_write_en), .dma_target(dma_target), .dma_addr(dma_addr), .dma_wdata(dma_wdata)
    );

    Mamba_Top u_top (
        .clk(clk), .reset(reset), .mode_select(mode_select),
        .lin_start(lin_start), .lin_en(lin_en), .lin_len(lin_len), .lin_done(lin_done),
        .lin_x_val(lin_x_val), .lin_W_vals(lin_W_vals), .lin_bias_vals(lin_bias_vals), .lin_y_out(lin_y_out),
        .conv_start(), .conv_en(), .conv_valid_in(), .conv_ready_in(), .conv_valid_out(),
        .conv_x_vec(), .conv_w_vec(), .conv_b_vec(), .conv_y_vec(),
        .scan_start(), .scan_en(), .scan_done(), .scan_clear_h(),
        .scan_delta_val(), .scan_x_val(), .scan_D_val(), .scan_gate_val(),
        .scan_A_vec(), .scan_B_vec(), .scan_C_vec(), .scan_y_out(),
        .softplus_in_val(), .softplus_out_val()
    );

    initial begin clk=0; forever #5 clk=~clk; end

    initial begin
        $readmemh("debug_input_correct.txt", file_input_flat);
        $readmemh("debug_weight_correct.txt", file_weight_flat);
        $readmemh("debug_golden_correct.txt", file_golden_flat);
        
        reset = 1; dma_write_en = 0; 
        errors = 0;
        max_diff = 0; sum_diff = 0; perf_match = 0; total_samples = 0;
        
        #20; reset = 0; #10;

        $display("==================================================");
        $display("   TESTBENCH: DEBUG PHASE (LINEAR LOGIC CHECK)    ");
        $display("==================================================");

        // 1. DMA LOAD INPUT
        $display(">> [1] Loading Input (Token-First) to RAM A...");
        idx_flat = 0;
        for(i=0; i<8000; i=i+1) begin
            for(j=0; j<16; j=j+1) begin
                dma_wdata[j*16+:16] = file_input_flat[idx_flat];
                idx_flat = idx_flat + 1;
            end
            dma_write(0, ADDR_DEBUG_IN + i, dma_wdata);
        end

        // 2. DMA LOAD WEIGHT
        $display(">> [1] Loading Weights to Weight RAM...");
        idx_flat = 0;
        for(i=0; i<512; i=i+1) begin
            for(j=0; j<16; j=j+1) begin
                dma_wdata[j*16+:16] = file_weight_flat[idx_flat];
                idx_flat = idx_flat + 1;
            end
            dma_write(2, W_BASE_OUTPROJ + i, dma_wdata);
        end
        
        // 3. START SYSTEM
        $display("\n>> [2] Starting Debug Phase (State 90)...");
        @(posedge clk);
        start_system = 1; // Dummy
        u_ctrl.state = 90; // FORCE STATE
        
        wait(done_system);
        @(posedge clk);
        $display("   -> Execution Done.");
        
        // 4. VERIFY
        $display("\n>> [3] Verification & Statistics Gathering...");
        
        idx_flat = 0;
        for(i=0; i<4000; i=i+1) begin
            dma_wdata = u_mem.ram_b.ram[ADDR_DEBUG_OUT + i];
            
            for(k=0; k<16; k=k+1) begin
                got = dma_wdata[k*16+:16];
                exp = file_golden_flat[idx_flat];
                idx_flat = idx_flat + 1;
                
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                
                // Stats Update
                total_samples = total_samples + 1;
                sum_diff = sum_diff + abs_diff;
                
                if (abs_diff > max_diff) max_diff = abs_diff;
                if (abs_diff == 0) perf_match = perf_match + 1;
                
                // Error Reporting
                if(abs_diff > TOLERANCE) begin
                    // Ch? in 10 l?i ð?u tiên cho ð? r?i
                    if (errors < 10) 
                        $display("[ERR] Addr %0d Ch %0d | Got=%h Exp=%h | Diff=%0d", i, k, got, exp, abs_diff);
                    errors = errors + 1;
                end 
            end
        end
        
        // --- FINAL REPORT ---
        $display("\n=========================================================================================");
        $display("                             PHASE 5 ONLY SUMMARY REPORT                                 ");
        //$display("                        (Pure Linear Engine Verification)                                ");
        $display("=========================================================================================");
        $display("  METRIC            | VALUE                                                              ");
        $display("--------------------|--------------------------------------------------------------------");
        $display("  Total Samples     | %0d", total_samples);
        $display("  Errors (>Tol %0d)  | %0d", TOLERANCE, errors);
        $display("  Status            | %s", (errors == 0) ? "PASS (Logic Correct)" : "FAIL (Check Setup)");
        $display("--------------------|--------------------------------------------------------------------");
        $display("  Max Diff          | %0d", max_diff);
        $display("  Avg Diff          | %0.4f", (total_samples > 0) ? (sum_diff / total_samples) : 0);
        $display("  Perfect Matches   | %0d (%0.2f%%)", perf_match, (total_samples > 0) ? (perf_match * 100.0 / total_samples) : 0);
        $display("=========================================================================================");
        
        $stop;
    end
    
    // Monitor
    reg [15:0] last_tok;
    initial last_tok = 9999;
    always @(posedge clk) begin
        if (u_ctrl.token_cnt % 100 == 0 && u_ctrl.token_cnt != last_tok) begin
            last_tok = u_ctrl.token_cnt;
            // $display("[TIME %0t] Processing Token %0d...", $time, u_ctrl.token_cnt);
        end
    end

    task dma_write;
        input [1:0] tgt; input [14:0] addr; input [255:0] data;
        begin
            dma_write_en=1; dma_target=tgt; dma_addr=addr; dma_wdata=data;
            @(posedge clk); #1; dma_write_en=0;
        end
    endtask

endmodule