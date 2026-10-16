`timescale 1ns/1ps
// ============================================================================
// 整条 Mesh 共用的低电平锁存型时钟门控接口。
// ASIC：定义 MESH_ICG_USE_TECH，并提供 mesh_icg_tech 工艺封装（相同四个端口）。
//       工艺封装中必须实例化真实库 ICG，不能把下面的行为模型当作物理单元。
// FPGA：默认直接传递 clk，保留 PE 的 CE，避免在普通逻辑中构造门控时钟。
// 仿真：SIM_MESH_ICG 可在 FPGA RAM 仿真配置下启用行为门控，以验证真实停钟。
// ============================================================================
module mesh_icg (
    input  wire clk_i,
    input  wire en_i,
    input  wire test_en_i,
    output wire gclk_o
);
`ifdef MESH_ICG_BYPASS
    // 仅供原始常开时钟对照组，不改变 Mesh 的原有 CE 行为。
    assign gclk_o = clk_i;
`else
`ifdef MESH_ICG_USE_TECH
    mesh_icg_tech u_tech (
        .clk_i(clk_i), .en_i(en_i), .test_en_i(test_en_i), .gclk_o(gclk_o)
    );
`else
`ifdef FPGA_TARGET
`ifndef SIM_MESH_ICG
    assign gclk_o = clk_i;
`else
    reg en_latched;
    always @(*) begin
        if (!clk_i) en_latched = en_i | test_en_i;
    end
    assign gclk_o = clk_i & en_latched;
`endif
`else
`ifdef SYNTHESIS
    // 有意要求真实工艺封装：缺少 mesh_icg_tech 时综合必须报未解析模块，
    // 避免误把普通 latch+AND 当成已经映射的工艺 ICG。
    mesh_icg_tech u_tech (
        .clk_i(clk_i), .en_i(en_i), .test_en_i(test_en_i), .gclk_o(gclk_o)
    );
`else
    reg en_latched;
    // 只在 CLK 低电平更新使能；高电平期间保持，使输出脉冲不被截断。
    // 这里不加复位：输入时钟从低电平启动，使能由复位后的控制器确定。
    always @(*) begin
        if (!clk_i) en_latched = en_i | test_en_i;
    end
    assign gclk_o = clk_i & en_latched;
`endif
`endif
`endif
`endif
endmodule
