`timescale 1ns / 1ps
`include "_parameter.v"

module tb_Full_Flow;
    reg clk, reset, start_system;
    wire done_system;

    // Giao tiep
    wire bank_sel, mem_write_en;
    wire[14:0] mem_read_addr, mem_write_addr, mem_weight_addr, mem_const_addr;
    wire [255:0] mem_read_data, mem_write_data, mem_weight_data, mem_const_data;
    
    // Dummy Wires
    wire [2:0] mode_select;
    wire lin_start, lin_done, lin_en; wire[15:0] lin_len;
    wire signed [15:0] lin_x_val; wire signed [255:0] lin_W_vals, lin_bias_vals, lin_y_out;
    wire conv_start, conv_en, conv_valid_in, conv_ready_in, conv_valid_out;
    wire [255:0] conv_x_vec, conv_b_vec, conv_y_vec; wire[1023:0] conv_w_vec;
    wire scan_start, scan_en, scan_done, scan_clear_h;
    wire signed [15:0] scan_delta, scan_x, scan_D, scan_gate, scan_y;
    wire signed [255:0] scan_A, scan_B, scan_C;
    wire signed [15:0] softplus_in, softplus_out;

    reg dma_write_en;
    reg [1:0] dma_target; 
    reg [14:0] dma_addr;
    reg[255:0] dma_wdata;

    // --- MEMORY MAP ---
    localparam ADDR_X_INPUT     = 15'd0;         
    localparam ADDR_X_PRIM      = 15'd16384;     
    localparam ADDR_CONV_OUT    = 15'd24576;      
    localparam ADDR_SCAN_Y_BASE = 15'd8192; // RAM A 
    
    localparam ADDR_B_BASE      = 15'd0;         
    localparam ADDR_C_BASE      = 15'd1500;      
    localparam ADDR_DT_RAW_BASE = 15'd3000;      
    localparam ADDR_GATE        = 15'd8192;      
    localparam ADDR_DELTA_BASE  = 15'd17000;     
    localparam ADDR_FINAL_OUT   = 15'd25000; 
    
    localparam W_BASE_INPROJ1   = 15'd0;
    localparam W_BASE_INPROJ2   = 15'd512;
    localparam W_BASE_CONV      = 15'd1024;
    localparam W_BASE_XPROJ     = 15'd1536;
    localparam W_BASE_DTPROJ    = 15'd1920;
    localparam W_BASE_OUTPROJ   = 15'd2432;
    
    localparam CONST_CONV_BIAS  = 15'd0;       
    localparam CONST_DT_BIAS    = 15'd128;     
    localparam ADDR_A_BASE      = 15'd1024;    
    localparam ADDR_D_BASE      = 15'd1152;    

    // --- ARRAYS NAP INPUT & TRONG SO ---
    reg signed [15:0] f_Xin [0:63999]; 
    reg signed [15:0] f_Win1 [0:8191]; reg signed [15:0] f_Win2[0:8191]; 
    reg signed [15:0] f_Wconv[0:2047]; reg signed [15:0] f_Bconv [0:127];
    reg signed [15:0] f_Wxproj [0:7679];   
    reg signed [15:0] f_Wdtproj [0:511]; reg signed[15:0] f_Bdtproj [0:127];
    reg signed[15:0] f_A[0:2047]; reg signed [15:0] f_D [0:127];
    reg signed [15:0] f_Woutproj[0:8191];
    
    // --- ARRAYS GOLDEN TU PYTHON ---
    reg signed [15:0] gold_x_prim [0:127999];
    reg signed [15:0] gold_gate   [0:127999];
    reg signed [15:0] gold_conv   [0:127999]; 
    reg signed [15:0] gold_B      [0:15999];     
    reg signed [15:0] gold_C      [0:15999];     
    reg signed [15:0] gold_dt_raw [0:15999];
    reg signed [15:0] gold_delta  [0:127999];
    reg signed [15:0] gold_P4_Y   [0:127999];
    reg signed [15:0] gold_final  [0:63999]; 

    integer i, k;
    reg[255:0] val_ram;
    reg signed [15:0] got, exp;
    integer abs_diff;
    
    // Tracking Variables
    integer err_p1, err_p2, err_p3, err_p4, err_p5;
    integer max_p1, max_p2, max_p3, max_p4, max_p5;
    
    integer TOTAL_P1 = 256000;
    integer TOTAL_P2 = 128000;
    integer TOTAL_P3 = 128000;
    integer TOTAL_P4 = 128000;
    integer TOTAL_P5 = 64000;
    integer TOTAL_ALL;
    integer ERR_ALL;
    
    real acc_p1, acc_p2, acc_p3, acc_p4, acc_p5, acc_all;

    localparam TOLERANCE = 150; 

    // --- INSTANTIATE BLOCK ---
    Global_Controller_Full_Flow u_ctrl (
        .clk(clk), .reset(reset), .start_system(start_system), .done_system(done_system),
        .bank_sel(bank_sel), .core_read_addr(mem_read_addr), .core_read_data(mem_read_data),
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

    initial begin
        // LOAD INPUT & WEIGHTS
        $readmemh("ptb_X_input_7F.txt", f_Xin);
        $readmemh("ptb_W_in1_7F.txt", f_Win1); $readmemh("ptb_W_in2_7F.txt", f_Win2);
        $readmemh("ptb_W_conv_7F.txt", f_Wconv); $readmemh("ptb_B_conv_7F.txt", f_Bconv);
        $readmemh("ptb_W_xproj_7F.txt", f_Wxproj);
        $readmemh("ptb_W_dtproj_7F.txt", f_Wdtproj); $readmemh("ptb_B_dtproj_7F.txt", f_Bdtproj);
        $readmemh("ptb_A_7F.txt", f_A); $readmemh("ptb_D_7F.txt", f_D);
        $readmemh("ptb_W_outproj_7F.txt", f_Woutproj);
        
        // LOAD GOLDEN VERIFY
        $readmemh("ptb_gold_P1_x_7F.txt", gold_x_prim);
        $readmemh("ptb_gold_P1_z_7F.txt", gold_gate);
        $readmemh("ptb_gold_P2_conv_7F.txt", gold_conv);
        $readmemh("ptb_gold_P3_dtraw_7F.txt", gold_dt_raw);
        $readmemh("ptb_gold_P3_B_7F.txt", gold_B);
        $readmemh("ptb_gold_P3_C_7F.txt", gold_C);
        $readmemh("ptb_gold_P32_delta_7F.txt", gold_delta);
        $readmemh("ptb_gold_P4_Y_7F.txt", gold_P4_Y);
        $readmemh("ptb_gold_P5_Out_7F.txt", gold_final);

        reset = 1; start_system = 0; dma_write_en = 0;
        err_p1=0; err_p2=0; err_p3=0; err_p4=0; err_p5=0;
        max_p1=0; max_p2=0; max_p3=0; max_p4=0; max_p5=0;
        #20; reset = 0; #10;

        $display(">> [1] LOAD DATA...");
        for (i=0; i<4000; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Xin[i*16+k];
            dma_write(0, ADDR_X_INPUT + i, dma_wdata);
        end
        for (i=0; i<512; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Win1[i*16+k]; dma_write(2, W_BASE_INPROJ1 + i, dma_wdata);
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Win2[i*16+k]; dma_write(2, W_BASE_INPROJ2 + i, dma_wdata);
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Wconv[i*16+k]; dma_write(2, W_BASE_CONV + i, dma_wdata);
        end
        for (i=0; i<480; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Wxproj[i*16+k]; dma_write(2, W_BASE_XPROJ + i, dma_wdata);
        end
        for (i=0; i<32; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Wdtproj[i*16+k]; dma_write(2, W_BASE_DTPROJ + i, dma_wdata);
        end
        for (i=0; i<512; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Woutproj[i*16+k]; dma_write(2, W_BASE_OUTPROJ + i, dma_wdata);
        end
        for (i=0; i<128; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_A[i*16+k]; dma_write(3, ADDR_A_BASE + i, dma_wdata);
        end
        for (i=0; i<8; i=i+1) begin
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Bconv[i*16+k]; dma_write(3, CONST_CONV_BIAS + i, dma_wdata);
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_Bdtproj[i*16+k]; dma_write(3, CONST_DT_BIAS + i, dma_wdata);
            for (k=0; k<16; k=k+1) dma_wdata[k*16+:16] = f_D[i*16+k]; dma_write(3, ADDR_D_BASE + i, dma_wdata);
        end

        $display("\n>> [2] SYSTEM START...");
        @(posedge clk); start_system = 1; 
        @(posedge clk); #1 start_system = 0;

        wait(done_system); 
        @(posedge clk);
        $display("\n>> [3] DONE ! VERIFYING ...\n");

        // --- CHECK PHASE 1 (X_PRIM & GATE) ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_a.ram[ADDR_X_PRIM + i]; // RAM A
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_x_prim[i*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_p1) max_p1 = abs_diff;
                if (abs_diff > TOLERANCE) err_p1 = err_p1 + 1;
            end
            
            val_ram = u_mem.ram_b.ram[ADDR_GATE + i]; // RAM B
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_gate[i*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_p1) max_p1 = abs_diff;
                if (abs_diff > TOLERANCE) err_p1 = err_p1 + 1;
            end
        end

        // --- CHECK PHASE 2 (CONV) ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_b.ram[ADDR_CONV_OUT + i]; // RAM B
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_conv[i*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_p2) max_p2 = abs_diff;
                if (abs_diff > TOLERANCE) err_p2 = err_p2 + 1;
            end
        end
        
        // --- CHECK PHASE 3 (DELTA) ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_b.ram[ADDR_DELTA_BASE + i]; // RAM B
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_delta[i*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_p3) max_p3 = abs_diff;
                if (abs_diff > TOLERANCE) err_p3 = err_p3 + 1;
            end
        end
        
        // --- CHECK PHASE 4 (Y_GATED) ---
        for (i=0; i<8000; i=i+1) begin
            val_ram = u_mem.ram_a.ram[ADDR_SCAN_Y_BASE + i]; // RAM A
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_P4_Y[i*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_p4) max_p4 = abs_diff;
                if (abs_diff > TOLERANCE) err_p4 = err_p4 + 1;
            end
        end
        
        // --- CHECK PHASE 5 (FINAL OUT) ---
        for (i=0; i<4000; i=i+1) begin
            val_ram = u_mem.ram_b.ram[ADDR_FINAL_OUT + i]; // RAM B
            for (k=0; k<16; k=k+1) begin
                got = val_ram[k*16+:16]; exp = gold_final[i*16 + k];
                abs_diff = (got > exp) ? (got - exp) : (exp - got);
                if (abs_diff > max_p5) max_p5 = abs_diff;
                if (abs_diff > TOLERANCE) err_p5 = err_p5 + 1;
            end
        end

        // --- STATISTICAL CALCULATIONS  ---

        begin
            TOTAL_ALL = TOTAL_P1 + TOTAL_P2 + TOTAL_P3 + TOTAL_P4 + TOTAL_P5;
            ERR_ALL   = err_p1 + err_p2 + err_p3 + err_p4 + err_p5;

            acc_p1 = ((TOTAL_P1 - err_p1) * 100.0) / TOTAL_P1;
            acc_p2 = ((TOTAL_P2 - err_p2) * 100.0) / TOTAL_P2;
            acc_p3 = ((TOTAL_P3 - err_p3) * 100.0) / TOTAL_P3;
            acc_p4 = ((TOTAL_P4 - err_p4) * 100.0) / TOTAL_P4;
            acc_p5 = ((TOTAL_P5 - err_p5) * 100.0) / TOTAL_P5;
            acc_all = ((TOTAL_ALL - ERR_ALL) * 100.0) / TOTAL_ALL;

            // --- FINAL REPORT  ---
            $display("\n=========================================================================================");
            $display("                     MAMBA ACCELERATOR - HARDWARE VERIFICATION REPORT                    ");
            $display("=========================================================================================");
            $display(" Format        : Fixed-Point Q8.7");
            $display(" Tolerance     : +/- %0d (Max allowed deviation from Golden Float32 model)", TOLERANCE);
            $display(" Flow Status   : FULL END-TO-END EXECUTION COMPLETE");
            $display("=========================================================================================");
            $display(" PHASE | MODULE NAME   | TOTAL POINTS | OUT OF BOUNDS | MAX DIFF | HARDWARE ACCURACY (%) ");
            $display("-------|---------------|--------------|---------------|----------|-----------------------");
            $display("  [1]  | Input Project | %12d | %13d | %8d | %18.2f %%", TOTAL_P1, err_p1, max_p1, acc_p1);
            $display("  [2]  | Conv1D        | %12d | %13d | %8d | %18.2f %%", TOTAL_P2, err_p2, max_p2, acc_p2);
            $display("  [3]  | Delta Param   | %12d | %13d | %8d | %18.2f %%", TOTAL_P3, err_p3, max_p3, acc_p3);
            $display("  [4]  | SSM Scan Core | %12d | %13d | %8d | %18.2f %%", TOTAL_P4, err_p4, max_p4, acc_p4);
            $display("  [5]  | Output Proj   | %12d | %13d | %8d | %18.2f %%", TOTAL_P5, err_p5, max_p5, acc_p5);
            $display("=========================================================================================");
            $display("       | SYSTEM AVERAGE| %12d | %13d |   ---    | %18.2f %%", TOTAL_ALL, ERR_ALL, acc_all);
            $display("       | SYSTEM FINAL  | %12s | %13s |   ---    | %18.2f %%", "----", "----", acc_p5);
            $display("=========================================================================================\n");

            if (acc_all > 90.0) begin
                $display(">>> [SUCCESS] OVERALL PIPELINE PASSED WITH HIGH FIDELITY (>90%%).");
                $display(">>> Data flow from P1 to P5 operates seamlessly. Error margins are within typical quantization bounds.");
            end else begin
                $display(">>> [WARNING] OVERALL PIPELINE ACCURACY IS BELOW 90%%.");
            end
        end
        
        // Bump
        $display(">> Xuat du lieu Verilog Phase 5 ra file de ve bieu do...");
        begin : WRITE_FILE
            integer fd;
            fd = $fopen("Mamba_verilog_final_out.txt", "w");
            for (i=0; i<4000; i=i+1) begin
                val_ram = u_mem.ram_b.ram[ADDR_FINAL_OUT + i];
                for (k=0; k<16; k=k+1) begin
                    $fdisplay(fd, "%0d", $signed(val_ram[k*16+:16])); // Ghi s? nguyên h? Dec
                end
            end
            $fclose(fd);
        end
        
        $stop;
    end

    // Task DMA 
    task dma_write;
        input [1:0] tgt; input [14:0] addr; input[255:0] data;
        begin
            dma_write_en=1; dma_target=tgt; dma_addr=addr; dma_wdata=data;
            @(posedge clk); #1; dma_write_en=0;
        end
    endtask

    // Monitor Tracking 
    always @(posedge clk) begin
        if (u_ctrl.state == 1)  $display("[TIME %0t] Phase 1...", $time);
        if (u_ctrl.state == 10) $display("[TIME %0t] Phase 2...", $time);
        if (u_ctrl.state == 30) $display("[TIME %0t] Phase 3...", $time);
        if (u_ctrl.state == 50) $display("[TIME %0t] Phase 4 (Scan Core)...", $time);
        if (u_ctrl.state == 80) $display("[TIME %0t] Phase 5 (Out Proj)...", $time);
    end

endmodule