// ============================================================================
// 中文阅读导引（当前实现）
// 4行×8列共32个PE。激活从左向右传播，部分和沿行号增大方向传递。
// 权重字节索引为8*(row*8+column)，每列对应一个输出通道。
// 输入需先错位，原始各列结果具有不同延迟，因此外部必须使用output_deskew对齐。
// ============================================================================
//=============================================================================
// 模块名称：Systolic_Array
// 功能：4行×8列权重固定脉动阵列。
//
// 数据方向：
//   * 4路激活分别从4行左侧进入，并沿列方向向右传播；
//   * 每一列对应一个输出通道；
//   * 部分和从第0行向第3行传播；
//   * 第3行输出该列4个乘积之和。
//
// 打包约定：
//   XIN_DATA[8*k_lane +: 8]
//   WEIGHT_DATA[8*(k_lane*8+n_lane) +: 8]
//   YOUT_DATA[20*n_lane +: 20]
//
// 输入激活必须在阵列外完成0/1/2/3拍错位，8列原始输出也必须在
// 阵列外完成7/6/.../0拍反向对齐。CE=0时所有PE统一冻结。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 4行8列共32个PE。权重字节编号为8*(row*8+column)，每列生成一个输出通道部分和。
// - 激活沿水平方向传播，部分和沿垂直方向传播；底行输出仍需deskew后才能并行对齐。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：4×8权重固定脉动阵列。
// 关键变量/接口：4行对应reduction lane，8列对应输出通道；32个PE并行完成4×8次INT8乘加。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module Systolic_Array #(
    parameter ACT_WIDTH = 16,
    parameter ACC_WIDTH = 26,
    parameter WGT_WIDTH = 8
) (
    input                                       CLK,
    input                                       RSTn,
    input                                       CE,
    input              [255:0]                  WEIGHT_DATA,
    input              [4*ACT_WIDTH-1:0]        XIN_DATA,
    output             [8*ACC_WIDTH-1:0]        YOUT_DATA
);

// 兼容下方PE位置参数：第一参数为激活宽度，第二参数为部分和宽度。
localparam DATA_WIDTH = ACT_WIDTH;

wire signed [ACT_WIDTH-1:0] x0 = XIN_DATA[ACT_WIDTH-1:0];
wire signed [ACT_WIDTH-1:0] x1 = XIN_DATA[2*ACT_WIDTH-1:ACT_WIDTH];
wire signed [ACT_WIDTH-1:0] x2 = XIN_DATA[3*ACT_WIDTH-1:2*ACT_WIDTH];
wire signed [ACT_WIDTH-1:0] x3 = XIN_DATA[4*ACT_WIDTH-1:3*ACT_WIDTH];

// 每一行的激活水平传播链。
wire signed [ACT_WIDTH-1:0] x00_01, x01_02, x02_03, x03_04;
wire signed [ACT_WIDTH-1:0] x04_05, x05_06, x06_07, x07_out;
wire signed [ACT_WIDTH-1:0] x10_11, x11_12, x12_13, x13_14;
wire signed [ACT_WIDTH-1:0] x14_15, x15_16, x16_17, x17_out;
wire signed [ACT_WIDTH-1:0] x20_21, x21_22, x22_23, x23_24;
wire signed [ACT_WIDTH-1:0] x24_25, x25_26, x26_27, x27_out;
wire signed [ACT_WIDTH-1:0] x30_31, x31_32, x32_33, x33_34;
wire signed [ACT_WIDTH-1:0] x34_35, x35_36, x36_37, x37_out;

// 每个PE的INT20部分和输出。
wire signed [ACC_WIDTH-1:0] p00, p01, p02, p03, p04, p05, p06, p07;
wire signed [ACC_WIDTH-1:0] p10, p11, p12, p13, p14, p15, p16, p17;
wire signed [ACC_WIDTH-1:0] p20, p21, p22, p23, p24, p25, p26, p27;
wire signed [ACC_WIDTH-1:0] p30, p31, p32, p33, p34, p35, p36, p37;

// 第0列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe00(CLK,RSTn,CE,WEIGHT_DATA[7:0],   x0,{{ACC_WIDTH{1'b0}}},x00_01,p00);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe10(CLK,RSTn,CE,WEIGHT_DATA[71:64], x1,p00,x10_11,p10);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe20(CLK,RSTn,CE,WEIGHT_DATA[135:128],x2,p10,x20_21,p20);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe30(CLK,RSTn,CE,WEIGHT_DATA[199:192],x3,p20,x30_31,p30);

// 第1列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe01(CLK,RSTn,CE,WEIGHT_DATA[15:8],   x00_01,{{ACC_WIDTH{1'b0}}},x01_02,p01);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe11(CLK,RSTn,CE,WEIGHT_DATA[79:72],  x10_11,p01,x11_12,p11);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe21(CLK,RSTn,CE,WEIGHT_DATA[143:136],x20_21,p11,x21_22,p21);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe31(CLK,RSTn,CE,WEIGHT_DATA[207:200],x30_31,p21,x31_32,p31);

// 第2列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe02(CLK,RSTn,CE,WEIGHT_DATA[23:16],  x01_02,{{ACC_WIDTH{1'b0}}},x02_03,p02);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe12(CLK,RSTn,CE,WEIGHT_DATA[87:80],  x11_12,p02,x12_13,p12);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe22(CLK,RSTn,CE,WEIGHT_DATA[151:144],x21_22,p12,x22_23,p22);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe32(CLK,RSTn,CE,WEIGHT_DATA[215:208],x31_32,p22,x32_33,p32);

// 第3列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe03(CLK,RSTn,CE,WEIGHT_DATA[31:24],  x02_03,{{ACC_WIDTH{1'b0}}},x03_04,p03);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe13(CLK,RSTn,CE,WEIGHT_DATA[95:88],  x12_13,p03,x13_14,p13);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe23(CLK,RSTn,CE,WEIGHT_DATA[159:152],x22_23,p13,x23_24,p23);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe33(CLK,RSTn,CE,WEIGHT_DATA[223:216],x32_33,p23,x33_34,p33);

// 第4列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe04(CLK,RSTn,CE,WEIGHT_DATA[39:32],  x03_04,{{ACC_WIDTH{1'b0}}},x04_05,p04);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe14(CLK,RSTn,CE,WEIGHT_DATA[103:96], x13_14,p04,x14_15,p14);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe24(CLK,RSTn,CE,WEIGHT_DATA[167:160],x23_24,p14,x24_25,p24);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe34(CLK,RSTn,CE,WEIGHT_DATA[231:224],x33_34,p24,x34_35,p34);

// 第5列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe05(CLK,RSTn,CE,WEIGHT_DATA[47:40],  x04_05,{{ACC_WIDTH{1'b0}}},x05_06,p05);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe15(CLK,RSTn,CE,WEIGHT_DATA[111:104],x14_15,p05,x15_16,p15);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe25(CLK,RSTn,CE,WEIGHT_DATA[175:168],x24_25,p15,x25_26,p25);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe35(CLK,RSTn,CE,WEIGHT_DATA[239:232],x34_35,p25,x35_36,p35);

// 第6列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe06(CLK,RSTn,CE,WEIGHT_DATA[55:48],  x05_06,{{ACC_WIDTH{1'b0}}},x06_07,p06);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe16(CLK,RSTn,CE,WEIGHT_DATA[119:112],x15_16,p06,x16_17,p16);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe26(CLK,RSTn,CE,WEIGHT_DATA[183:176],x25_26,p16,x26_27,p26);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe36(CLK,RSTn,CE,WEIGHT_DATA[247:240],x35_36,p26,x36_37,p36);

// 第7列。
PE #(DATA_WIDTH, ACC_WIDTH) u_pe07(CLK,RSTn,CE,WEIGHT_DATA[63:56],  x06_07,{{ACC_WIDTH{1'b0}}},x07_out,p07);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe17(CLK,RSTn,CE,WEIGHT_DATA[127:120],x16_17,p07,x17_out,p17);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe27(CLK,RSTn,CE,WEIGHT_DATA[191:184],x26_27,p17,x27_out,p27);
PE #(DATA_WIDTH, ACC_WIDTH) u_pe37(CLK,RSTn,CE,WEIGHT_DATA[255:248],x36_37,p27,x37_out,p37);

// 每一列的最终结果来自第3行。
// 连续赋值：组合生成YOUT_DATA及其相邻接口信号，表达握手、选择或地址关系。
assign YOUT_DATA[ACC_WIDTH-1:0] = p30;
assign YOUT_DATA[2*ACC_WIDTH-1:ACC_WIDTH] = p31;
// 连续赋值：组合生成YOUT_DATA及其相邻接口信号，表达握手、选择或地址关系。
assign YOUT_DATA[3*ACC_WIDTH-1:2*ACC_WIDTH] = p32;
assign YOUT_DATA[4*ACC_WIDTH-1:3*ACC_WIDTH] = p33;
// 连续赋值：组合生成YOUT_DATA及其相邻接口信号，表达握手、选择或地址关系。
assign YOUT_DATA[5*ACC_WIDTH-1:4*ACC_WIDTH] = p34;
assign YOUT_DATA[6*ACC_WIDTH-1:5*ACC_WIDTH] = p35;
// 连续赋值：组合生成YOUT_DATA及其相邻接口信号，表达握手、选择或地址关系。
assign YOUT_DATA[7*ACC_WIDTH-1:6*ACC_WIDTH] = p36;
assign YOUT_DATA[8*ACC_WIDTH-1:7*ACC_WIDTH] = p37;
endmodule
