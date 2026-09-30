// ============================================================================
// 中文阅读导引（当前实现）
// 统一Mesh接口的薄封装。weight_load直接装Active；weight_shadow_load装下一组；weight_swap切换。
// 数据和标签随ce统一暂停；此处没有独立ready，反压由controller换算成ce。
// ============================================================================
//=============================================================================
// 模块名称：mesh_adapter
// 功能：向上层提供统一4×8 Weight-Stationary Mesh接口。
//
// 当前实现使用mesh_core_4x8_systolic，内部连接真实Systolic_Array.v，
// 不再依赖tb/mesh_core_4x8_model.v。weight_data的字节排列为：
//   weight_data[8*(k_lane*8+n_lane) +: 8]
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 封装输入错拍、4x8阵列和输出反错拍，并传递与数据同行的m_tag。
// - weight_load写Active，weight_shadow_load写Shadow，weight_swap在安全边界交换两组寄存器。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：Mesh接口适配器。
// 关键变量/接口：统一权重装载/交换、激活输入和8路部分和输出，并保持valid/ready反压边界。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module mesh_adapter #(
    parameter M_TAG_WIDTH = 9,
    parameter ACT_WIDTH = 16,
    parameter PSUM_WIDTH = 26
) (
    input                                       clk,
    input                                       rst_n,
    input                                       ce,
    input                                       weight_load,
    input                                       weight_shadow_load,
    input                                       weight_swap,
    input              [255:0]                  weight_data,
    input                                       act_valid,
    input              [4*ACT_WIDTH-1:0]        act_data,
    input              [M_TAG_WIDTH-1:0]        act_m_tag,
    output                                      psum_valid,
    output             [8*PSUM_WIDTH-1:0]       psum_data,
    output             [M_TAG_WIDTH-1:0]        psum_m_tag
);

mesh_core_4x8_systolic #(
    .M_TAG_WIDTH                 (M_TAG_WIDTH)
    ,.ACT_WIDTH                  (ACT_WIDTH)
    ,.PSUM_WIDTH                 (PSUM_WIDTH)
) u_mesh_core (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .ce                          (ce),
    .weight_load                 (weight_load),
    .weight_shadow_load          (weight_shadow_load),
    .weight_swap                 (weight_swap),
    .weight_data                 (weight_data),
    .act_valid                   (act_valid),
    .act_data                    (act_data),
    .act_m_tag                   (act_m_tag),
    .psum_valid                  (psum_valid),
    .psum_data                   (psum_data),
    .psum_m_tag                  (psum_m_tag)
);

endmodule
