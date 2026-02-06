`timescale 1ns / 1ps
`include "_parameter.v"

module tb_Global_Controller_Phase1_2_3;

    // --- 1. SIGNALS (GI? NGUYÊN) ---
    reg clk, reset;
    reg start_system;
    wire done_system;

    // Internal Wires
    wire bank_sel;
    wire [14:0] mem_read_addr, mem_write_addr, mem_weight_addr, mem_const_addr;
    wire [255:0] mem_read_data, mem_write_data, mem_weight_data, mem_const_data;
    wire mem_write_en;
    
    // Top Wires
    wire [2:0] mode_select;
    // Linear
    wire lin_start, lin_done, lin_en; wire [15:0] lin_len;
    wire signed [15:0] lin_x_val; wire signed [255:0] lin_W_vals, lin_bias_vals, lin_y_out;
    // Conv
    wire conv_start, conv_en, conv_valid_in, conv_ready_in, conv_valid_out;
    wire [255:0] conv_x_vec, conv_b_vec, conv_y_vec; wire [1023:0] conv_w_vec;
    // Scan
    wire scan_start, scan_en, scan_done, scan_clear_h;
    wire signed [15:0] scan_delta, scan_x, scan_D, scan_gate;
    wire signed [255:0] scan_A, scan_B, scan_C;
    wire signed [15:0] scan_y;
    // Softplus
    wire signed [15:0] softplus_in, softplus_out;

    // DMA
    reg dma_write_en;
    reg [1:0] dma_target; 
    reg [14:0] dma_addr;
    reg [255:0] dma_wdata;

    // --- MEMORY MAP (GI? NGUYÊN) ---
    localparam ADDR_X_INPUT     = 15'd0;         
    localparam ADDR_X_PRIM      = 15'd16384;     
    localparam ADDR_CONV_OUT    = 15'd24576;      
    localparam ADDR_SCAN_Y_BASE = 15'd8192;      
    
    // RAM B 
    localparam ADDR_B_BASE      = 15'd0;         
    localparam ADDR_C_BASE      = 15'd1500;      
    localparam ADDR_DT_RAW_BASE = 15'd3000;      
    localparam ADDR_GATE        = 15'd8192;      
    localparam ADDR_DELTA_BASE  = 15'd17000;     
    localparam ADDR_FINAL_OUT   = 15'd20000;    

    // Weight RAM
    localparam W_BASE_INPROJ1 = 15'd0;
    localparam W_BASE_INPROJ2 = 15'd512;
    localparam W_BASE_CONV    = 15'd1024;
    localparam W_BASE_XPROJ   = 15'd1536;
    localparam W_BASE_DTPROJ  = 15'd1920;
    localparam W_BASE_OUTPROJ = 15'd2432;       
    
    // Const RAM
    localparam CONST_CONV_BIAS    = 15'd0;       
    localparam CONST_DT_BIAS_BASE = 15'd128;     
    localparam ADDR_A_BASE        = 15'd1024;    
    localparam ADDR_D_BASE        = 15'd1152;    

    // --- TEST DATA ARRAYS ---
    reg signed [15:0] file_x_input [0:63999]; 
    reg signed [15:0] file_w_proj1 [0:8191]; 
    reg signed [15:0] file_w_proj2 [0:8191]; 
    reg signed [15:0] file_w_conv  [0:2047]; 
    reg signed [15:0] file_b_conv  [0:127];
    reg signed [15:0] file_w_xproj [0:6143];   
    reg signed [15:0] file_w_dtproj [0:511]; 
    reg signed [15:0] file_b_dtproj [0:127];
    
    // GOLDEN OUTPUTS
    reg signed [15:0] gold_primary [0:127999];
    reg signed [15:0] gold_gate    [0:127999];
    reg signed [15:0] gold_conv    [0:127999]; 
    reg signed [15:0] gold_B [0:15999];     
    reg signed [15:0] gold_C [0:15999];     
    reg signed [15:0] gold_dt_raw [0:15999];
    reg signed [15:0] gold_delta [0:127999];

    // Variables for Verification
    integer i, k, c;
    integer err_prim, err_gate, err_conv, err_B, err_C, err_DT, err_Delta;
    reg signed [15:0] got, exp;
    integer abs_diff;
    reg [255:0] val_ram;
    integer chunk_idx, token_idx, gold_line_idx;
    
    // --- VARIABLES FOR REPORTING (THÊM M?I) ---
    integer max_diff_prim, max_diff_gate, max_diff_conv, max_diff_B, max_diff_C, max_diff_DT, max_diff_Delta;
    real sum_diff_prim, sum_diff_gate, sum_diff_conv, sum_diff_B, sum_diff_C, sum_diff_DT, sum_diff_Delta;
    integer perf_prim, perf_gate, perf_conv, perf_B, perf_C, perf_DT, perf_Delta; // Count perfect matches (Diff=0)
    
    localparam TOL_P1 = 150;
    localparam TOL_P2 = 150;
    localparam TOL_P3 = 150;
    localparam TOL_DT = 150; 

    // --- INSTANTIATE (GI? NGUYÊN) ---
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
        .conv_start(conv_start), .conv_en(conv_en), .conv_valid_in(conv_valid_in), .conv_ready_in(conv_ready_in), .conv_valid_out(conv_valid_out),
        .conv_x_vec(conv_x_vec), .conv_w_vec(conv_w_vec), .conv_b_vec(conv_b_vec), .conv_y_vec(conv_y_vec),
        .scan_start(scan_start), .scan_en(scan_en), .scan_done(scan_done), .scan_clear_h(scan_clear_h),
        .scan_delta_val(scan_delta), .scan_x_val(scan_x), .scan_D_val(scan_D), .scan_gate_val(scan_gate),
        .scan_A_vec(scan_A), .scan_B_vec(scan_B), .scan_C_vec(scan_C), .scan_y_out(scan_y),
        .softplus_in(softplus_in), .softplus_out(softplus_out)
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
        .conv_start(conv_start), .conv_en(conv_en), .conv_valid_in(conv_valid_in), .conv_ready_in(conv_ready_in), .conv_valid_out(conv_valid_out),
        .conv_x_vec(conv_x_vec), .conv_w_vec(conv_w_vec), .conv_b_vec(conv_b_vec), .conv_y_vec(conv_y_vec),
        .scan_start(scan_start), .scan_en(scan_en), .scan_done(scan_done), .scan_clear_h(scan_clear_h),
        .scan_delta_val(scan_delta), .scan_x_val(scan_x), .scan_D_val(scan_D), .scan_gate_val(scan_gate),
        .scan_A_vec(scan_A), .scan_B_vec(scan_B), .scan_C_vec(scan_C), .scan_y_out(scan_y),
        .softplus_in_val(softplus_in), .softplus_out_val(softplus_out)
    );

    initial begin clk=0; forever #5 clk=~clk; end
    
    // --- INIT ARRAYS ---
    initial begin
        for (i=0; i<8192; i=i+1) begin file_w_proj1[i]=0; file_w_proj2[i]=0; end
        for (i=0; i<2048; i=i+1) file_w_conv[i]=0;
        for (i=0; i<128; i=i+1)  file_b_conv[i]=0;
        for (i=0; i<6144; i=i+1) file_w_xproj[i]=0;
        for (i=0; i<512; i=i+1)  file_w_dtproj[i]=0;
        for (i=0; i<128; i=i+1)  file_b_dtproj[i]=0;
    end

    // --- MAIN PROCESS ---
    initial begin
        // 1. LOAD ALL FILES
        $readmemh("linear_x_input.txt", file_x_input);
        $readmemh("lin_real_w1_reordered.txt", file_w_proj1);
        $readmemh("lin_real_w2_reordered.txt", file_w_proj2);
        $readmemh("conv_w_input_real.txt", file_w_conv);
        $readmemh("conv_b_input_real.txt", file_b_conv);
        $readmemh("w_xproj_reordered.txt", file_w_xproj);
        $readmemh("w_dt_proj_reordered.txt", file_w_dtproj);
        $readmemh("b_dt_proj.txt", file_b_dtproj);
        $readmemh("linear1_golden.txt", gold_primary);
        $readmemh("linear2_golden.txt", gold_gate);
        $readmemh("conv_y_golden_real.txt", gold_conv);
        $readmemh("scan_real_B_shared.txt", gold_B);
        $readmemh("scan_real_C_shared.txt", gold_C);
        $readmemh("gold_dt_raw_hw.txt", gold_dt_raw);
        $readmemh("gold_delta_final.txt", gold_delta);

        reset = 1; start_system = 0; dma_write_en = 0;
        
        // Init Stats
        err_prim=0; max_diff_prim=0; sum_diff_prim=0; perf_prim=0;
        err_gate=0; max_diff_gate=0; sum_diff_gate=0; perf_gate=0;
        err_conv=0; max_diff_conv=0; sum_diff_conv=0; perf_conv=0;
        err_B=0;    max_diff_B=0;    sum_diff_B=0;    perf_B=0;
        err_C=0;    max_diff_C=0;    sum_diff_C=0;    perf_C=0;
        err_DT=0;   max_diff_DT=0;   sum_diff_DT=0;   perf_DT=0;
        err_Delta=0;max_diff_Delta=0;sum_diff_Delta=0;perf_Delta=0;

        #20; reset = 0; #10;

        $display("==================================================");
        $display("   TESTBENCH: MAMBA BLOCK PHASE 1-2-3 VERIFICATION ");
        $display("==================================================");

        // --- DMA LOADING ---
        $display(">> [1] Loading Memory (Input, Weights, Bias)...");
        // (Copy l?i ph?n DMA c?)
        for (i=0; i<4000; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_x_input[i*16+k];
            dma_write(0, ADDR_X_INPUT + i, dma_wdata);
        end
        for (i=0; i<512; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_w_proj1[i*16+k];
            dma_write(2, W_BASE_INPROJ1 + i, dma_wdata);
        end
        for (i=0; i<512; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_w_proj2[i*16+k];
            dma_write(2, W_BASE_INPROJ2 + i, dma_wdata);
        end
        for (i=0; i<512; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_w_conv[i*16+k];
            dma_write(2, W_BASE_CONV + i, dma_wdata);
        end
        for (i=0; i<8; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_b_conv[i*16+k];
            dma_write(3, CONST_CONV_BIAS + i, dma_wdata);
        end
        for (i=0; i<384; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_w_xproj[i*16+k];
            dma_write(2, W_BASE_XPROJ + i, dma_wdata);
        end
        for (i=0; i<32; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_w_dtproj[i*16+k];
            dma_write(2, W_BASE_DTPROJ + i, dma_wdata);
        end
        for (i=0; i<8; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = file_b_dtproj[i*16+k];
            dma_write(3, CONST_DT_BIAS_BASE + i, dma_wdata);
        end
        $display("   -> Data Loaded.");

        // --- START SYSTEM ---
        @(posedge clk);
        $display("\n>> [2] System Execution Started...");
        start_system = 1; 
        @(posedge clk); #1 start_system = 0;

        wait(done_system);
        @(posedge clk);
        $display("   -> Execution Done.");

        // --- VERIFY & GATHER STATISTICS ---
        $display("\n>> [3] Verification & Statistics Gathering...");
        
        // --- PHASE 1: Primary ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_a.ram[ADDR_X_PRIM + i];
            chunk_idx = i/1000; token_idx = i%1000;
            gold_line_idx = (token_idx*8) + chunk_idx;
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16];
                exp = gold_primary[gold_line_idx*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                
                // Stats
                if (abs_diff > max_diff_prim) max_diff_prim = abs_diff;
                sum_diff_prim = sum_diff_prim + abs_diff;
                if (abs_diff == 0) perf_prim = perf_prim + 1;
                
                if (abs_diff > TOL_P1) err_prim = err_prim + 1;
            end
        end
        
        // --- PHASE 1: Gate ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_b.ram[ADDR_GATE + i];
            chunk_idx = i/1000; token_idx = i%1000;
            gold_line_idx = (token_idx*8) + chunk_idx;
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16];
                exp = gold_gate[gold_line_idx*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                
                if (abs_diff > max_diff_gate) max_diff_gate = abs_diff;
                sum_diff_gate = sum_diff_gate + abs_diff;
                if (abs_diff == 0) perf_gate = perf_gate + 1;
                
                if (abs_diff > TOL_P1) err_gate = err_gate + 1;
            end
        end
        
        // --- PHASE 2: Conv ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_a.ram[ADDR_CONV_OUT + i];
            // Conv Golden Token First (Sim)
            chunk_idx = i/1000; token_idx = i%1000;
            gold_line_idx = (token_idx*8) + chunk_idx;
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16];
                exp = gold_conv[i*16 + k]; // Assuming golden conv matched
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                
                if (abs_diff > max_diff_conv) max_diff_conv = abs_diff;
                sum_diff_conv = sum_diff_conv + abs_diff;
                if (abs_diff == 0) perf_conv = perf_conv + 1;
                
                if (abs_diff > TOL_P2) err_conv = err_conv + 1;
            end
        end
        
        // --- PHASE 3: B, C, DT, Delta ---
        for (i=0; i<1000; i=i+1) begin
            // B
            val_ram = u_mem.ram_b.ram[ADDR_B_BASE + i];
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_B[i*16+k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_diff_B) max_diff_B = abs_diff;
                sum_diff_B = sum_diff_B + abs_diff;
                if (abs_diff == 0) perf_B = perf_B + 1;
                if (abs_diff > TOL_P3) err_B = err_B + 1;
            end
            // C
            val_ram = u_mem.ram_b.ram[ADDR_C_BASE + i];
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_C[i*16+k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_diff_C) max_diff_C = abs_diff;
                sum_diff_C = sum_diff_C + abs_diff;
                if (abs_diff == 0) perf_C = perf_C + 1;
                if (abs_diff > TOL_P3) err_C = err_C + 1;
            end
            // DT Raw
            val_ram = u_mem.ram_b.ram[ADDR_DT_RAW_BASE + i];
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_dt_raw[i*16+k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_diff_DT) max_diff_DT = abs_diff;
                sum_diff_DT = sum_diff_DT + abs_diff;
                if (abs_diff == 0) perf_DT = perf_DT + 1;
                if (abs_diff > TOL_P3) err_DT = err_DT + 1;
            end
        end
        
        // Delta
        for (i=0; i<1000; i=i+1) begin
            for (c=0; c<8; c=c+1) begin
                val_ram = u_mem.ram_b.ram[ADDR_DELTA_BASE + (i*8) + c];
                for (k=0; k<16; k=k+1) begin
                    got = val_ram[k*16+:16];
                    exp = gold_delta[(i*8 + c)*16 + k];
                    abs_diff = (got > exp) ? (got - exp) : (exp - got);
                    if (abs_diff > max_diff_Delta) max_diff_Delta = abs_diff;
                    sum_diff_Delta = sum_diff_Delta + abs_diff;
                    if (abs_diff == 0) perf_Delta = perf_Delta + 1;
                    if (abs_diff > TOL_DT) err_Delta = err_Delta + 1;
                end
            end
        end

        // --- FINAL REPORT ---
        $display("\n=========================================================================================");
        $display("                              VERIFICATION SUMMARY REPORT                                ");
        $display("=========================================================================================");
        $display("  PHASE             | STATUS | ERRORS | MAX DIFF | AVG DIFF | PERFECT MATCHES (%%)  ");
        $display("--------------------|--------|--------|----------|----------|----------------------");
        
        $display("  1. Linear Primary | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_prim == 0) ? "PASS" : "FAIL", err_prim, max_diff_prim, sum_diff_prim/128000.0, (perf_prim/128000.0)*100);
            
        $display("  1. Linear Gate    | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_gate == 0) ? "PASS" : "FAIL", err_gate, max_diff_gate, sum_diff_gate/128000.0, (perf_gate/128000.0)*100);
            
        $display("  2. Conv1D         | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_conv == 0) ? "PASS" : "FAIL", err_conv, max_diff_conv, sum_diff_conv/128000.0, (perf_conv/128000.0)*100);
            
        $display("  3.1. Proj B       | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_B == 0) ? "PASS" : "FAIL", err_B, max_diff_B, sum_diff_B/16000.0, (perf_B/16000.0)*100);
            
        $display("  3.1. Proj C       | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_C == 0) ? "PASS" : "FAIL", err_C, max_diff_C, sum_diff_C/16000.0, (perf_C/16000.0)*100);
            
        $display("  3.1. DT Raw       | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_DT == 0) ? "PASS" : "FAIL", err_DT, max_diff_DT, sum_diff_DT/16000.0, (perf_DT/16000.0)*100);
            
        $display("  3.2. Delta Final  | %s   | %6d | %8d | %8.4f | %18.2f%% ", 
            (err_Delta == 0) ? "PASS" : "FAIL", err_Delta, max_diff_Delta, sum_diff_Delta/128000.0, (perf_Delta/128000.0)*100);
            
        $display("=========================================================================================");
        
        if (err_prim+err_gate+err_conv+err_B+err_C+err_DT+err_Delta == 0)
            $display(">> RESULT: SYSTEM PASSED ALL CHECKS FOR PHASE 1-2-3");
        else
            $display(">> RESULT: SYSTEM FAILED. PLEASE CHECK LOGS.");
            
        $stop;
    end

    // Task DMA
    task dma_write;
        input [1:0] tgt; input [14:0] addr; input [255:0] data;
        begin
            dma_write_en=1; dma_target=tgt; dma_addr=addr; dma_wdata=data;
            @(posedge clk); #1; dma_write_en=0;
        end
    endtask
    
    // Monitor
    reg [15:0] last_tok;
    initial last_tok = 9999;
    always @(posedge clk) begin
        if (u_ctrl.token_cnt % 100 == 0 && u_ctrl.token_cnt != last_tok) begin
            last_tok = u_ctrl.token_cnt;
            $display("[TIME %0t] Processing Token %0d (State %0d)...", $time, u_ctrl.token_cnt, u_ctrl.state);
        end
    end

endmodule