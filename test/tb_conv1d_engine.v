`timescale 1ns/1ps
//=============================================================================
// Conv1d Engine 自检 Testbench
//
// run_case 会根据同一套确定性函数同时生成：
//   Activation SRAM 内容、Weight SRAM 布局、Bias/Requant 参数和 Golden。
// DUT 完成后逐个比较 Y[m][channel]，打印具体 m/channel/期望值/实际值。
// 五个测试依次覆盖小卷积、完整 Initial Conv、dilation、stride 和随机
// backpressure。最终只输出 TEST PASS 或带 error_count 的 TEST FAIL。
//=============================================================================
module tb_conv1d_engine;
reg clk,rst_n,start,random_enable;
wire busy,done,error;
reg [31:0] cfg_input_base,cfg_output_base,cfg_weight_base;
reg [8:0] cfg_cin,cfg_cout;reg [4:0] cfg_kernel;reg [3:0] cfg_stride,cfg_dilation;
reg [7:0] cfg_left_pad;reg [8:0] cfg_input_length,cfg_output_length;
reg [11:0] cfg_k_total;reg [9:0] cfg_k_groups;reg [5:0] cfg_n_groups;
reg cfg_elu_enable;
reg elu_lut_cfg_valid;wire elu_lut_cfg_ready;reg [6:0] elu_lut_cfg_addr;reg [7:0] elu_lut_cfg_wdata;
wire act_rd_req_valid,act_rd_req_ready;wire [31:0] act_rd_addr0,act_rd_addr1,act_rd_addr2,act_rd_addr3;
wire [3:0] act_rd_mask;wire act_rd_rsp_valid,act_rd_rsp_ready;wire [31:0] act_rd_data;
wire wgt_rd_req_valid,wgt_rd_req_ready;wire [31:0] wgt_rd_addr;wire wgt_rd_rsp_valid,wgt_rd_rsp_ready;wire [255:0] wgt_rd_data;
wire param_rd_req_valid,param_rd_req_ready;wire [5:0] param_rd_output_group;wire param_rd_rsp_valid,param_rd_rsp_ready;
wire [255:0] param_bias_data,param_mult_data;wire [47:0] param_shift_data;
wire [63:0] param_zero_point_data;
wire out_wr_valid,out_wr_ready;wire [31:0] out_wr_addr;wire [63:0] out_wr_data;wire [7:0] out_wr_strb;
integer error_count,test_count,timeout_count,i,j,k,c,m,n,ti,acc,expected,actual;
integer kt,kg,ng,block,lane,nlane;
reg [7:0] expected_elu_lut[0:127];

conv1d_engine_top dut(
 .clk(clk),.rst_n(rst_n),.mesh_scan_test_en(1'b0),.start(start),.busy(busy),.done(done),.error(error),
 .cfg_input_base(cfg_input_base),.cfg_output_base(cfg_output_base),.cfg_weight_base(cfg_weight_base),
 .cfg_cin(cfg_cin),.cfg_cout(cfg_cout),.cfg_kernel(cfg_kernel),.cfg_stride(cfg_stride),
 .cfg_dilation(cfg_dilation),.cfg_left_pad(cfg_left_pad),.cfg_input_length(cfg_input_length),
 .cfg_output_length(cfg_output_length),.cfg_k_total(cfg_k_total),.cfg_k_groups(cfg_k_groups),.cfg_n_groups(cfg_n_groups),
 .cfg_elu_enable(cfg_elu_enable),.elu_lut_cfg_valid(elu_lut_cfg_valid),.elu_lut_cfg_ready(elu_lut_cfg_ready),
 .elu_lut_cfg_addr(elu_lut_cfg_addr),.elu_lut_cfg_wdata(elu_lut_cfg_wdata),
 .act_rd_req_valid(act_rd_req_valid),.act_rd_req_ready(act_rd_req_ready),.act_rd_addr0(act_rd_addr0),.act_rd_addr1(act_rd_addr1),
 .act_rd_addr2(act_rd_addr2),.act_rd_addr3(act_rd_addr3),.act_rd_mask(act_rd_mask),
 .act_rd_rsp_valid(act_rd_rsp_valid),.act_rd_rsp_ready(act_rd_rsp_ready),.act_rd_data(act_rd_data),
 .wgt_rd_req_valid(wgt_rd_req_valid),.wgt_rd_req_ready(wgt_rd_req_ready),.wgt_rd_addr(wgt_rd_addr),
 .wgt_rd_rsp_valid(wgt_rd_rsp_valid),.wgt_rd_rsp_ready(wgt_rd_rsp_ready),.wgt_rd_data(wgt_rd_data),
 .param_rd_req_valid(param_rd_req_valid),.param_rd_req_ready(param_rd_req_ready),.param_rd_output_group(param_rd_output_group),
 .param_rd_rsp_valid(param_rd_rsp_valid),.param_rd_rsp_ready(param_rd_rsp_ready),
 .param_bias_data(param_bias_data),.param_mult_data(param_mult_data),.param_shift_data(param_shift_data),
 .param_zero_point_data(param_zero_point_data),
 .out_wr_valid(out_wr_valid),.out_wr_ready(out_wr_ready),.out_wr_addr(out_wr_addr),.out_wr_data(out_wr_data),.out_wr_strb(out_wr_strb));
activation_sram_model u_act(.clk(clk),.rst_n(rst_n),.random_enable(random_enable),.req_valid(act_rd_req_valid),.req_ready(act_rd_req_ready),
 .addr0(act_rd_addr0),.addr1(act_rd_addr1),.addr2(act_rd_addr2),.addr3(act_rd_addr3),.mask(act_rd_mask),
 .rsp_valid(act_rd_rsp_valid),.rsp_ready(act_rd_rsp_ready),.rsp_data(act_rd_data));
weight_sram_model u_wgt(.clk(clk),.rst_n(rst_n),.random_enable(random_enable),.req_valid(wgt_rd_req_valid),.req_ready(wgt_rd_req_ready),
 .req_addr(wgt_rd_addr),.rsp_valid(wgt_rd_rsp_valid),.rsp_ready(wgt_rd_rsp_ready),.rsp_data(wgt_rd_data));
parameter_memory_model u_param(.clk(clk),.rst_n(rst_n),.random_enable(random_enable),.req_valid(param_rd_req_valid),.req_ready(param_rd_req_ready),
 .output_group(param_rd_output_group),.rsp_valid(param_rd_rsp_valid),.rsp_ready(param_rd_rsp_ready),
 .bias_data(param_bias_data),.mult_data(param_mult_data),.shift_data(param_shift_data),
 .zero_point_data(param_zero_point_data));
output_sram_model u_out(.clk(clk),.rst_n(rst_n),.random_enable(random_enable),.wr_valid(out_wr_valid),.wr_ready(out_wr_ready),
 .wr_addr(out_wr_addr),.wr_data(out_wr_data),.wr_strb(out_wr_strb));

always #5 clk=~clk;
function integer input_value;input integer t;input integer ch;begin input_value=((t*3+ch*5)%17)-8;end endfunction
function integer weight_value;input integer oc;input integer ch;input integer ki;begin weight_value=((oc*7+ch*3+ki*5)%7)-3;end endfunction
function integer bias_value;input integer oc;begin bias_value=oc-4;end endfunction
function integer mult_value;input integer oc;begin mult_value=(oc%3)+1;end endfunction
function integer shift_value;input integer oc;begin shift_value=(oc%3)+2;end endfunction
function integer zero_point_value;input integer oc;begin zero_point_value=(oc%5)-2;end endfunction
function integer sat8;input integer x;begin if(x>127)sat8=127;else if(x< -128)sat8=-128;else sat8=x;end endfunction
function integer requant_value;
 input integer acc_value;input integer oc;integer x;integer mag;begin
  x=(acc_value+bias_value(oc))*mult_value(oc);
  if(shift_value(oc)!=0)begin
   if(x>=0)x=(x+(1<<(shift_value(oc)-1)))>>>shift_value(oc);
   else begin mag=-x;x=-((mag+(1<<(shift_value(oc)-1)))>>>shift_value(oc));end
  end
  requant_value=sat8(x+zero_point_value(oc));
 end
endfunction

function integer elu_value;
 input integer q;reg [7:0] qb;reg [6:0] lut_index;begin
  qb=q[7:0];
  if(qb[7])begin lut_index=~qb[6:0];elu_value=$signed(expected_elu_lut[lut_index]);end
  else elu_value=$signed(qb);
 end
endfunction

task load_elu_lut;integer lut_i;begin
 for(lut_i=0;lut_i<128;lut_i=lut_i+1)begin
  @(negedge clk);elu_lut_cfg_addr=lut_i[6:0];elu_lut_cfg_wdata=expected_elu_lut[lut_i];elu_lut_cfg_valid=1;
  while(!elu_lut_cfg_ready)@(negedge clk);
  @(posedge clk);
 end
 @(negedge clk);elu_lut_cfg_valid=0;
end endtask

// 每组测试前复位控制流水；Accumulator 数据阵列不依赖 reset 清零。
task reset_dut;begin
 rst_n=0;start=0;repeat(4)@(posedge clk);rst_n=1;repeat(2)@(posedge clk);
end endtask

task run_case;
 input integer CIN,COUT,KERNEL,STRIDE,DILATION,LEFT_PAD,IN_LEN,OUT_LEN,RANDOM_BP,ELU_ENABLE;
 begin
  // 根据卷积配置计算软件侧派生参数。
  test_count=test_count+1;random_enable=RANDOM_BP;reset_dut;
  kt=CIN*KERNEL;kg=(kt+3)/4;ng=(COUT+7)/8;
  cfg_input_base=0;cfg_output_base=0;cfg_weight_base=0;
  cfg_cin=CIN;cfg_cout=COUT;cfg_kernel=KERNEL;cfg_stride=STRIDE;cfg_dilation=DILATION;
  cfg_left_pad=LEFT_PAD;cfg_input_length=IN_LEN;cfg_output_length=OUT_LEN;
  cfg_k_total=kt;cfg_k_groups=kg;cfg_n_groups=ng;
  cfg_elu_enable=ELU_ENABLE;
  load_elu_lut;
  for(i=0;i<IN_LEN;i=i+1)for(c=0;c<CIN;c=c+1)u_act.mem[i*CIN+c]=input_value(i,c);
  // 权重按照 [output_group][k_group][k_lane][n_lane] 预排布。
  for(i=0;i<ng*kg;i=i+1)u_wgt.mem[i]=256'd0;
  for(n=0;n<COUT;n=n+1)for(k=0;k<KERNEL;k=k+1)for(c=0;c<CIN;c=c+1)begin
   i=k*CIN+c;block=(n/8)*kg+i/4;lane=i%4;nlane=n%8;
   u_wgt.mem[block][8*(lane*8+nlane) +: 8]=weight_value(n,c,k);
  end
   for(i=0;i<ng;i=i+1)begin
    u_param.bias_mem[i]=0;u_param.mult_mem[i]=0;u_param.shift_mem[i]=0;u_param.zero_point_mem[i]=0;
    for(j=0;j<8;j=j+1)begin
     if((i*8+j)<COUT)begin
      u_param.bias_mem[i][32*j +: 32]=bias_value(i*8+j);
      u_param.mult_mem[i][32*j +: 32]=mult_value(i*8+j);
      u_param.shift_mem[i][6*j +: 6]=shift_value(i*8+j);
      u_param.zero_point_mem[i][8*j +: 8]=zero_point_value(i*8+j);
     end
   end
  end
  for(i=0;i<OUT_LEN*COUT;i=i+1)u_out.mem[i]=8'h5a;
  @(negedge clk);start=1;@(negedge clk);start=0;
  // 等待单周期 done/error，同时设置超时防止 FSM 死锁。
  timeout_count=0;
  while(!done&&!error&&(timeout_count<2000000))begin @(posedge clk);timeout_count=timeout_count+1;end
  if(error)begin $display("ERROR: legal configuration rejected in test %0d",test_count);error_count=error_count+1;end
  else if(timeout_count>=2000000)begin $display("ERROR: timeout in test %0d",test_count);error_count=error_count+1;end
  else begin
   // 直接卷积 Golden Model：包含 stride、dilation 和左 Padding。
   for(m=0;m<OUT_LEN;m=m+1)for(n=0;n<COUT;n=n+1)begin
    acc=0;
    for(k=0;k<KERNEL;k=k+1)begin ti=m*STRIDE-LEFT_PAD+k*DILATION;
     if((ti>=0)&&(ti<IN_LEN))for(c=0;c<CIN;c=c+1)acc=acc+input_value(ti,c)*weight_value(n,c,k);
    end
    expected=requant_value(acc,n);
    if(ELU_ENABLE!=0)expected=elu_value(expected);
    actual=$signed(u_out.mem[m*COUT+n]);
    if(actual!==expected)begin
     $display("MISMATCH test=%0d m=%0d channel=%0d expected=%0d actual=%0d output_group=%0d Cin=%0d Cout=%0d K=%0d",test_count,m,n,expected,actual,n/8,CIN,COUT,KERNEL);
     error_count=error_count+1;
    end
   end
  end
  $display("test %0d complete, cycles=%0d",test_count,timeout_count);
 end
endtask

initial begin
 clk=0;rst_n=0;start=0;random_enable=0;error_count=0;test_count=0;
 cfg_input_base=0;cfg_output_base=0;cfg_weight_base=0;cfg_cin=0;cfg_cout=0;cfg_kernel=0;
 cfg_stride=0;cfg_dilation=0;cfg_left_pad=0;cfg_input_length=0;cfg_output_length=0;cfg_k_total=0;cfg_k_groups=0;cfg_n_groups=0;
 cfg_elu_enable=0;elu_lut_cfg_valid=0;elu_lut_cfg_addr=0;elu_lut_cfg_wdata=0;
 $readmemh("tb/data/elu_lut_s1_32.hex",expected_elu_lut);
 // 1：快速 causal padding/K-tail 测试。
 run_case(1,8,3,1,1,2,8,8,0,0);
 // 2：需求指定的 320x16 Initial Conv。
 run_case(1,16,7,1,1,6,320,320,0,1);
 // 3：dilation=3 地址测试。
 run_case(16,8,3,1,3,6,16,16,0,0);
 // 4：stride=2 下采样测试。
 run_case(16,16,4,2,1,3,16,8,0,0);
 // 5：随机请求、响应和输出 backpressure。
 run_case(1,8,3,1,1,2,8,8,1,1);
 if(error_count==0)$display("TEST PASS");else $display("TEST FAIL, error_count = %0d",error_count);
 $finish;
end
endmodule
