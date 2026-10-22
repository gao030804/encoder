`timescale 1ns/1ps
// 模块端口级边界：Activation双Buffer最后一行/越界/反压；Residual越界/错误复位。
// 不force内部数据，全部通过读写端口建立并核对内容。
module tb_sram_trim_ports;
    reg clk=0,rst_n=0;
    always #5 clk=~clk;
    reg rb=0,wb=0,rv=0,rr=0,wv=0;
    reg [31:0] rbase=32'h1000,wbase=32'h1000;
    reg [31:0] ra0=0,ra1=0,ra2=0,ra3=0,wa=0;
    reg [3:0] mask=4'hf;
    reg [63:0] wd=0;
    reg [7:0] ws=8'hff;
    wire ready,valid,wready,act_error;
    wire [63:0] result;
    activation_buffer_pingpong u_act (
        .clk(clk),.rst_n(rst_n),.rd_buffer_select(rb),.rd_buffer_base(rbase),
        .rd_req_valid(rv),.rd_req_ready(ready),.rd_addr0(ra0),.rd_addr1(ra1),
        .rd_addr2(ra2),.rd_addr3(ra3),.rd_mask(mask),.rd_rsp_valid(valid),
        .rd_rsp_ready(rr),.rd_rsp_data(result),.wr_buffer_select(wb),.wr_buffer_base(wbase),
        .wr_valid(wv),.wr_ready(wready),.wr_addr(wa),.wr_data(wd),.wr_strb(ws),.access_error(act_error)
    );
    reg swv=0,srv=0;
    reg [10:0] swa=0,sra=0;
    reg [63:0] swd=0;
    wire swready,srready,svalid,serror;
    wire [63:0] sdata;
    residual_scratchpad u_res (
        .clk(clk),.rst_n(rst_n),.wr_valid(swv),.wr_ready(swready),.wr_addr(swa),
        .wr_data(swd),.wr_strb(8'hff),.rd_valid(srv),.rd_ready(srready),.rd_addr(sra),
        .rsp_valid(svalid),.rsp_data(sdata),.error(serror)
    );
    integer rows[0:7];
    integer b,i,n,checks=0;
    function [63:0] pattern;
        input integer bank,row;
        begin pattern={16'hd0e1,bank[15:0],row[15:0],(16'h75a3 ^ row[15:0])};end
    endfunction
    task act_write;
        input integer bank,row;
        begin
            @(negedge clk);wb=bank;wbase=bank ? 32'h5000:32'h1000;
            wa=wbase+row*8;wd=pattern(bank,row);wv=1;
            @(posedge clk);#1;
            if(!wready) $fatal(1,"activation write unexpectedly stalled");
            @(negedge clk);wv=0;
        end
    endtask
    task act_read;
        input integer bank,row;
        input [63:0] expected;
        begin
            @(negedge clk);rb=bank;rbase=bank ? 32'h5000:32'h1000;
            ra0=rbase+row*8;ra1=ra0+2;ra2=ra0+4;ra3=ra0+6;rv=1;rr=0;
            @(posedge clk);#1;
            if(!ready) $fatal(1,"activation read unexpectedly stalled");
            @(negedge clk);rv=0;n=0;
            while(!valid && n<10) begin @(negedge clk);n=n+1;end
            if(!valid || result!==expected) $fatal(1,"activation data mismatch bank=%0d row=%0d",bank,row);
            repeat(3) begin
                @(posedge clk);#1;
                if(!valid || result!==expected) $fatal(1,"activation response changed while stalled");
            end
            @(negedge clk);rr=1;
            @(posedge clk);#1;
            @(negedge clk);rr=0;checks=checks+1;
        end
    endtask
    task scratch_write;
        input integer row;
        begin
            @(negedge clk);swa=row;swd=pattern(2,row);swv=1;
            @(posedge clk);#1;
            if(!swready) $fatal(1,"residual write stalled");
            @(negedge clk);swv=0;
        end
    endtask
    task scratch_read;
        input integer row;
        begin
            @(negedge clk);sra=row;srv=1;
            @(posedge clk);#1;
            if(!svalid || sdata!==pattern(2,row)) $fatal(1,"residual mismatch row=%0d",row);
            @(negedge clk);srv=0;checks=checks+1;
        end
    endtask
    initial begin
        rows[0]=0;rows[1]=511;rows[2]=512;rows[3]=1023;
        rows[4]=1024;rows[5]=1279;rows[6]=1280;rows[7]=1535;
        repeat(3) @(negedge clk);rst_n=1;
        if(u_act.ROWS!=1536 || u_res.DEPTH!=1536) $fatal(1,"wrong trimmed defaults");
        for(b=0;b<2;b=b+1) begin
            for(i=0;i<8;i=i+1) act_write(b,rows[i]);
            for(i=7;i>=0;i=i-1) act_read(b,rows[i],pattern(b,rows[i]));
        end
        if(act_error) $fatal(1,"unexpected Activation error");
        // 1536是第一个非法行；写入必须被阻止，读请求返回零并报告错误。
        act_write(0,1536);
        if(!act_error) $fatal(1,"Activation OOB write not reported");
        act_read(0,1536,64'd0);
        act_read(0,0,pattern(0,0));
        act_read(0,1535,pattern(0,1535));
        for(i=0;i<8;i=i+1) scratch_write(rows[i]);
        for(i=7;i>=0;i=i-1) scratch_read(rows[i]);
        if(serror) $fatal(1,"unexpected Residual error");
        scratch_write(1536);
        if(!serror) $fatal(1,"Residual OOB write not reported");
        @(negedge clk);srv=1;sra=1536;
        @(posedge clk);#1;
        if(svalid) $fatal(1,"Residual OOB read produced a response");
        @(negedge clk);srv=0;rst_n=0;
        repeat(2) @(negedge clk);rst_n=1;
        @(posedge clk);#1;
        if(act_error || serror) $fatal(1,"sticky access error did not reset");
        // 复位不清SRAM，证明非法写没有覆盖首行或尾行。
        scratch_read(0);scratch_read(1535);
        act_read(0,1535,pattern(0,1535));act_read(1,1535,pattern(1,1535));
        $display("[TB_PASS] SRAM trimmed module ports checks=%0d boundaries=511/512,1023/1024,1279/1280,1535/1536 backpressure=PASS reset=PASS",checks);
        $finish;
    end
    initial begin #100000;$fatal(1,"trimmed SRAM ports timeout");end
endmodule
