`timescale 1ns/1ps
// PP2函数三方对拍：Guard/Sticky候选 vs 修改前73位公式 vs 独立旧80位绝对值算法。
// 此处只检查函数；真实三级握手/参数/Tag由tb_postprocess_pipeline另行验证。
module tb_postprocess_guard_sticky;
    wire ready, valid;
    wire [127:0] data;
    wire tag;
    postprocess_8lane dut (
        .clk(1'b0),.rst_n(1'b0),.in_valid(1'b0),.in_ready(ready),
        .acc_data(320'd0),.bias_data(512'd0),.mult_data(256'd0),
        .shift_data(64'd0),.zero_point_data(64'd0),
        .post_valid(valid),.post_ready(1'b0),.post_data(data),
        .in_tag(1'b0),.post_tag(tag)
    );
    localparam signed [127:0] LIMIT = 128'sd1 << 70;
    integer errors=0, checks=0, shift_id, zp_id, sample_id, q_id, rem_id;
    integer seed=32'h63429acd, i, z_value;
    reg signed [39:0] a;
    reg signed [31:0] m;
    reg signed [71:0] p;
    reg signed [127:0] base_q, candidate, remainder_value, divisor, half;
    reg [7:0] shift;
    reg signed [7:0] zp;
    reg [15:0] actual, expected73, expected80;
    reg guard, sticky;
    reg [71:0] mask;
    integer coverage[0:7]; // {sign,guard,sticky}的八种组合
    integer range_checks=0, high_shift_checks=0;

    function [15:0] ref73;
        input signed [71:0] product;
        input [7:0] s;
        input signed [7:0] z;
        reg signed [72:0] ext, offset, v, out;
        begin
            ext={product[71],product};
            if(s==0) v=ext;
            else if(s<72) begin
                offset=(73'sd1<<(s-1'b1))-(product[71]?73'sd1:73'sd0);
                v=(ext+offset)>>>s;
            end else if(s==80) v=product[71]?73'sd1:-73'sd1;
            else v=0;
            out=v+{{65{z[7]}},z};
            if(out>32767) ref73=16'h7fff;
            else if(out< -32768) ref73=16'h8000;
            else ref73=out[15:0];
        end
    endfunction

    function [15:0] ref80;
        input signed [71:0] product;
        input [7:0] s;
        input signed [7:0] z;
        reg signed [79:0] ext, mag, rounded, shifted, out;
        begin
            ext={{8{product[71]}},product};
            if(s==0) shifted=ext;
            else if(ext>=0) begin
                rounded=ext+(80'sd1<<(s-1'b1));shifted=rounded>>>s;
            end else begin
                mag=-ext;rounded=mag+(80'sd1<<(s-1'b1));shifted=-(rounded>>>s);
            end
            out=shifted+{{72{z[7]}},z};
            if(out>32767) ref80=16'h7fff;
            else if(out< -32768) ref80=16'h8000;
            else ref80=out[15:0];
        end
    endfunction

    task check;
        input signed [71:0] value;
        input [7:0] s;
        input signed [7:0] z;
        begin
            actual=dut.round_shift_sat16_opt(value,s,z);
            expected73=ref73(value,s,z);expected80=ref80(value,s,z);
            checks=checks+1;
            if(actual!==expected73 || actual!==expected80) begin
                if(errors<12) $display("[ERROR] GS p=%h shift=%0d zp=%0d actual=%h ref73=%h ref80=%h",value,s,z,actual,expected73,expected80);
                errors=errors+1;
            end
            if(s>0 && s<72) begin
                guard=value[s-1'b1];mask=(72'd1<<(s-1'b1))-1;
                sticky=|(value&mask);
                coverage[{value[71],guard,sticky}]=coverage[{value[71],guard,sticky}]+1;
            end
            if(s>=72) high_shift_checks=high_shift_checks+1;
        end
    endtask

    initial begin
        for(i=0;i<8;i=i+1) coverage[i]=0;
        // 完整UINT8 shift x 代表性ZP x p极值/正负小数/饱和附近。
        for(shift_id=0;shift_id<256;shift_id=shift_id+1)
            for(zp_id=0;zp_id<8;zp_id=zp_id+1)
                for(sample_id=0;sample_id<16;sample_id=sample_id+1) begin
                    case(zp_id)
                        0:zp=-128;1:zp=-127;2:zp=-1;3:zp=0;
                        4:zp=1;5:zp=2;6:zp=126;7:zp=127;
                    endcase
                    case(sample_id)
                        0:p=0;1:p=1;2:p=-1;3:p=2;4:p=-2;
                        5:p=5;6:p=-5;7:p=6;8:p=-6;
                        9:p=72'sd1<<70;10:p=-(72'sd1<<70);
                        11:p=(72'sd1<<70)-1;12:p=-(72'sd1<<70)+1;
                        13:p=32767;14:p=-32768;15:p=65536;
                    endcase
                    check(p,shift_id,zp);
                end

        // signed17范围边界/INT16边界，余数在half前、half、half后；
        // 穷举全部256个ZP，重点防止截断q后误饱和或漏掉ZP的“救回”效果。
        for(shift_id=0;shift_id<72;shift_id=shift_id+1)
            for(q_id=0;q_id<12;q_id=q_id+1)
                for(rem_id=0;rem_id<5;rem_id=rem_id+1) begin
                    case(q_id)
                        0:base_q=-65537;1:base_q=-65536;2:base_q=-65535;
                        3:base_q=-32769;4:base_q=-32768;5:base_q=-32767;
                        6:base_q=32766;7:base_q=32767;8:base_q=32768;
                        9:base_q=65534;10:base_q=65535;11:base_q=65536;
                    endcase
                    divisor=128'sd1<<shift_id;half=divisor>>>1;
                    case(rem_id)
                        0:remainder_value=0;1:remainder_value=half-1;
                        2:remainder_value=half;3:remainder_value=half+1;
                        4:remainder_value=divisor-1;
                    endcase
                    candidate=(base_q<<<shift_id)+remainder_value;
                    if(remainder_value>=0 && remainder_value<divisor &&
                       candidate>=-LIMIT && candidate<=LIMIT) begin
                        for(zp_id=0;zp_id<256;zp_id=zp_id+1) begin
                            z_value=zp_id-128;
                            check(candidate[71:0],shift_id,z_value);
                            range_checks=range_checks+1;
                        end
                    end
                end

        // 真实可达的INT40 x INT32随机乘积，避免只测试小值或常数。
        for(i=0;i<40000;i=i+1) begin
            a={$random(seed),$random(seed)};m=$random(seed);
            p=$signed(a)*$signed(m);shift=$random(seed);zp=$random(seed);
            check(p,shift,zp);
        end
        for(i=0;i<8;i=i+1) if(coverage[i]==0) errors=errors+1;
        if(range_checks==0 || high_shift_checks==0) errors=errors+1;
        $display("[GS_COVERAGE] checks=%0d range_zp_checks=%0d high_shift=%0d sign_guard_sticky=8/8 random_products=40000",checks,range_checks,high_shift_checks);
        if(errors==0) $display("[TB_PASS] Guard/Sticky + signed17 range + INT18 ZP addition match previous INT73 and legacy INT80");
        else $display("[TB_FAIL] Guard/Sticky errors=%0d",errors);
        $finish;
    end
endmodule
