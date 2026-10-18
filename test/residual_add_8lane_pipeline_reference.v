// 仅用于仿真：四状态协议模型，量化仍用独立64位偏置舍入，不使用DUT的Guard函数。
// 原始三状态冻结参考仍保留在residual_add_8lane_legacy_reference.v，不修改。
// 此文件是根据截图重建的对拍模型，不是从服务器下载的源码，禁止加入综合清单。
module residual_add_8lane_pipeline_reference #(
    parameter ADDR_WIDTH=32,
    parameter SCRATCH_ADDR_WIDTH=11,
    parameter SCALE_ALIGN_ENABLE=0
) (
    input clk, rst_n, in_valid,
    output in_ready, busy,
    input [ADDR_WIDTH-1:0] in_addr, output_base,
    input [63:0] in_data,
    input [7:0] in_strb,
    input [119:0] scale_params,
    output scratch_rd_valid,
    input scratch_rd_ready,
    output [SCRATCH_ADDR_WIDTH-1:0] scratch_rd_addr,
    input scratch_rsp_valid,
    input [63:0] scratch_rsp_data,
    output reg out_valid,
    input out_ready,
    output reg [ADDR_WIDTH-1:0] out_addr,
    output reg [63:0] out_data,
    output reg [7:0] out_strb,
    output reg error
);
    localparam [2:0] S_IDLE=0, S_READ=1, S_WAIT=2, S_WAIT2=3;
    reg [2:0] state;
    reg [ADDR_WIDTH-1:0] addr_q, base_q;
    reg [63:0] data_q;
    reg [7:0] strb_q;
    reg [119:0] params_q;
    reg signed [15:0] identity_q[0:3], main_q[0:3];
    reg signed [63:0] sum;
    reg [63:0] result;
    integer n,r;
    wire [ADDR_WIDTH-1:0] offset=addr_q-base_q;
    wire [ADDR_WIDTH-1:0] input_offset=in_addr-output_base;
    assign in_ready=(state==S_IDLE) && (!out_valid || out_ready);
    assign busy=(state!=S_IDLE) || out_valid;
    assign scratch_rd_valid=(state==S_READ);
    assign scratch_rd_addr=offset[SCRATCH_ADDR_WIDTH+2:3];

    function signed [63:0] quant64;
        input signed [15:0] value;
        input signed [31:0] multiplier;
        input [7:0] shift;
        reg signed [63:0] p,mag;
        begin
            p=value*multiplier;
            if(shift==0) quant64=p;
            else if(shift>=64) quant64=0;
            else begin
                mag=(p<0)?-p:p;
                mag=(mag+(64'sd1<<(shift-1)))>>>shift;
                quant64=(p<0)?-mag:mag;
            end
        end
    endfunction
    function signed [15:0] sat16;
        input signed [63:0] value;
        begin
            if(value>32767) sat16=16'h7fff;
            else if(value< -32768) sat16=16'h8000;
            else sat16=value[15:0];
        end
    endfunction

    always @(*) begin
        result=0;
        sum=0;
        for(n=0;n<4;n=n+1) begin
            if(SCALE_ALIGN_ENABLE)
                sum=$signed(identity_q[n])+quant64(main_q[n],params_q[111:80],params_q[119:112]);
            else sum=$signed(identity_q[n])+$signed(main_q[n]);
            if(strb_q[2*n] || strb_q[2*n+1]) result[16*n+:16]=sat16(sum);
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=S_IDLE;addr_q<=0;base_q<=0;data_q<=0;strb_q<=0;params_q<=0;
            out_valid<=0;out_addr<=0;out_data<=0;out_strb<=0;error<=0;
            for(r=0;r<4;r=r+1) begin identity_q[r]<=0;main_q[r]<=0;end
        end else begin
            if(out_valid && out_ready) out_valid<=0;
            case(state)
                S_IDLE: if(in_valid && in_ready) begin
                    addr_q<=in_addr;base_q<=output_base;data_q<=in_data;
                    strb_q<=in_strb;params_q<=scale_params;
                    if(in_addr<output_base || input_offset[2:0]!=0) error<=1;
                    state<=S_READ;
                end
                S_READ: if(scratch_rd_ready) state<=S_WAIT;
                S_WAIT: if(scratch_rsp_valid) begin
                    for(r=0;r<4;r=r+1) begin
                        if(SCALE_ALIGN_ENABLE) begin
                            identity_q[r]<=sat16(quant64(scratch_rsp_data[16*r+:16],params_q[31:0],params_q[39:32]));
                            main_q[r]<=sat16(quant64(data_q[16*r+:16],params_q[71:40],params_q[79:72]));
                        end else begin
                            identity_q[r]<=scratch_rsp_data[16*r+:16];
                            main_q[r]<=data_q[16*r+:16];
                        end
                    end
                    state<=S_WAIT2;
                end
                S_WAIT2: begin
                    out_valid<=1;out_addr<=addr_q;out_data<=result;out_strb<=strb_q;
                    state<=S_IDLE;
                end
                default: begin state<=S_IDLE;error<=1;end
            endcase
        end
    end
endmodule
