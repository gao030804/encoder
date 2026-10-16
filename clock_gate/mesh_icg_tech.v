`timescale 1ns/1ps
// ============================================================================
// 与服务器 en/clock_gate/mesh_icg_tech.v 同步的临时门控封装。
// en_latched：在主时钟下降沿保存功能使能或扫描使能；下一高电平期间保持。
// gclk_o：Mesh 使用的门控时钟，关断时保持低电平。
//
// 注意：这是 negedge FF + AND，不是真实工艺库 ICG 实例，不加复位/initial。
// 上电初值仍可能为 X，必须先经历使能已知的下降沿，不能靠本模块保证初始化。
// 与低透明 latch 并非无条件等价：下降沿后才改变的使能要再等一个周期；
// 控制器须保证使能在下降沿前有效，并检查上升沿到下降沿的半周期时序。
// 默认 mesh_icg.v 的 latch 仿真分支不变；定义 MESH_ICG_USE_TECH 时选本封装。
// ASIC 正式实现仍需使用目标库 ICG，并重新验证 STA/DFT/门级仿真。
// ============================================================================
module mesh_icg_tech (
    input  wire clk_i,      // 未门控主时钟
    input  wire en_i,       // Mesh 功能运行使能
    input  wire test_en_i,  // 扫描测试强制开钟使能
    output wire gclk_o     // 门控后时钟
);
    reg en_latched;

    // 仅下降沿更新，非阻塞赋值；高电平时不允许使能变化截断时钟脉冲。
    always @(negedge clk_i) begin
        en_latched <= en_i | test_en_i;
    end

    assign gclk_o = clk_i & en_latched;
endmodule
