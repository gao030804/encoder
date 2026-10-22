`timescale 1ns/1ps
// 使用端口读写与平坦参考数组比较，覆盖每一行、128bit接缝、bank切换。
module sram_bank_case #(parameter W=256, D=8192)(output reg done=0);
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0, req=0, wr=0;
    reg [$clog2(D)-1:0] addr=0;
    reg [W-1:0] data=0;
    reg [W/8-1:0] mask=0;
    wire [W-1:0] result;
    wire valid;
    reg [W-1:0] expected [0:D-1];
    reg [W-1:0] held;
    integer i,j,a;
    opentitan_sram_1p_adapter #(.WIDTH(W),.DEPTH(D)) dut (
        .clk_i(clk),.rst_ni(rst_n),.req_i(req),.write_i(wr),
        .addr_i(addr),.wdata_i(data),.wmask_i(mask),
        .rdata_o(result),.rvalid_o(valid)
    );
    initial begin
        repeat(2) @(negedge clk);
        rst_n=1;
        // 全量初始化；不同地址和不同Byte使用不同内容，防止位宽/地址交换漏检。
        for(i=0;i<D;i=i+1) begin
            @(negedge clk); req=1; wr=1; addr=i; mask='1;
            for(j=0;j<W/8;j=j+1) data[j*8+:8]=(i*37+j*11+(i>>8));
            expected[i]=data;
            @(posedge clk); #1;
            if(valid!==0) $fatal(1,"write asserted read valid");
        end
        // Byte mask交替覆盖两半，包括Byte15/16，未选中字节必须保持。
        for(i=0;i<D;i=i+1) begin
            @(negedge clk); req=1; wr=1; addr=i;
            for(j=0;j<W/8;j=j+1) begin
                mask[j]=((i+j)%3==0);
                data[j*8+:8]=(i*13+j*7);
                if(mask[j]) expected[i][j*8+:8]=data[j*8+:8];
            end
            @(posedge clk); #1;
            if(valid!==0) $fatal(1,"masked write asserted read valid");
        end
        // 连续读：从尾到头交错跨bank访问，下一拍立即可返回下一组数据。
        for(i=0;i<D;i=i+1) begin
            a=(i%2==0) ? i/2 : D-1-i/2;
            @(negedge clk); req=1; wr=0; addr=a;
            @(posedge clk); #1;
            if(valid!==1 || result!==expected[a])
                $fatal(1,"W=%0d D=%0d addr=%0d mismatch",W,D,a);
        end
        held=result;
        // 非2幂深度专项：首个非法地址和编码最大值均不得别名到有效行。
        if(D < (1 << $clog2(D))) begin
            for(i=0;i<2;i=i+1) begin
                a=(i==0) ? D : (1 << $clog2(D))-1;
                @(negedge clk);req=1;wr=1;addr=a;data='1;mask='1;
                @(posedge clk);#1;
                if(valid!==0 || result!==held) $fatal(1,"OOB write changed response D=%0d",D);
                @(negedge clk);wr=0;
                @(posedge clk);#1;
                if(valid!==0 || result!==held) $fatal(1,"OOB read produced response D=%0d",D);
            end
            for(i=0;i<2;i=i+1) begin
                a=(i==0) ? 0 : D-1;
                @(negedge clk);req=1;wr=0;addr=a;
                @(posedge clk);#1;
                if(valid!==1 || result!==expected[a]) $fatal(1,"OOB address aliased D=%0d",D);
            end
            held=result;
            $display("[OOB_PASS] WIDTH=%0d DEPTH=%0d invalid_first=%0d invalid_last=%0d",W,D,D,(1 << $clog2(D))-1);
        end
        @(negedge clk); req=0; addr=0;
        repeat(3) begin
            @(posedge clk); #1;
            if(valid!==0 || result!==held) $fatal(1,"idle response changed");
        end
        // 复位只清响应控制，不清SRAM；复位后读回最后一行。
        @(negedge clk); rst_n=0;
        @(negedge clk); rst_n=1; req=1; addr=D-1;
        @(posedge clk); #1;
        if(valid!==1 || result!==expected[D-1]) $fatal(1,"reset retention mismatch");
        @(negedge clk); req=0;
        $display("[CASE_PASS] WIDTH=%0d DEPTH=%0d",W,D);
        done=1;
    end
endmodule

module tb_sram_banking;
    wire [9:0] done;
    sram_bank_case #(256,8192) global_case(done[0]);
    sram_bank_case #(256,2048) parameter_case(done[1]);
    sram_bank_case #(256,256) local_case(done[2]);
    sram_bank_case #(16,2048) activation_case(done[3]);
    sram_bank_case #(64,2048) residual_case(done[4]);
    sram_bank_case #(256,4) resident_case(done[5]);
    // 新旧容量同时测，逐行初始化/掩码写/交错读，覆盖所有512行bank接缝。
    sram_bank_case #(256,7168) trimmed_global_case(done[6]);
    sram_bank_case #(16,1536) trimmed_activation_case(done[7]);
    sram_bank_case #(64,1536) trimmed_residual_case(done[8]);
    sram_bank_case #(256,1536) rvq_case(done[9]);
    initial begin
        wait(&done);
        $display("[TB_PASS] banked SRAM port checks passed");
        $finish;
    end
    initial begin
        #1000000;
        $fatal(1,"SRAM test timeout");
    end
endmodule
