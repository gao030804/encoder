# mesh_icg_tech 映射到 TSMC28 工艺 ICG 的建议

日期：2026-10-16。本文件是服务器实施建议，不是已完成的工艺映射或签核报告。
本次只修改后处理关键路径；没有改门控 RTL、综合/时序约束，也没有进行 hold 修复。

## 1. 当前实现与目标

当前 `rtl/clock/mesh_icg_tech.v` 和 `E:/111/en/clock_gate/mesh_icg_tech.v` 是
下降沿触发器 + AND：

```verilog
always @(negedge clk_i)
    en_latched <= en_i | test_en_i;
assign gclk_o = clk_i & en_latched;
```

它不是标准单元库 ICG。目标是在保持四个接口不变的前提下，只替换该封装内部为
目标库真正的 integrated clock gating cell。外层 `mesh_icg.v`、Mesh CE 逻辑和
扫描输入层级不需要因此改动。

`negedge FF` 只在下降沿采样；低透明锁存式 ICG 在低电平窗口透明，两者并非任意
输入时序下等价。不能只因 FF 模型回归通过就跳过真实 ICG 的使能时序验证。

## 2. 先确认服务器库，不能凭名字接线

附件的候选名为 `CLKNQD4BWP30P140HVT`，这里只把它作为待查的候选；尚未获得
服务器对应 Liberty/Verilog/LEF，不能确认其是否存在、门控形式和管脚极性。
同为 TSMC28 不同库版本也可能有不同单元/引脚，不能凭 CP/E/TE/Q 的示例猜测。

| 必查资料/项目 | 需要确认的内容 |
|---|---|
| `.lib` / `.db` | 正确工艺、VT、电压/温度角；目标单元存在且非 black box |
| ICG 属性 | `clock_gating_integrated_cell`；时钟、使能、测试使能、输出的 clock-gate pin 属性 |
| 功能与极性 | 活动时钟边沿、低/高透明窗口、E/TE是否高有效、输出是否反相、附加输出用途 |
| 时序弧 | 时钟传播、clock gating setup/hold、最小脉宽、输入/输出转换和负载限制 |
| Verilog模型 | 管脚与库一致；UDP/电源脚/时序notifier/编译选项符合库说明 |
| LEF/物理库 | 单元尺寸、供电、CTS允许使用、物理pin和逻辑pin匹配 |
| 使用限制 | 是否被dont_use屏蔽；DFT规则、库许可和项目工艺规则 |

Liberty 的 ICG 属性用于定义门控形式和相关引脚/检查；可参考
[Synopsys Liberty 2013.03 手册（公开镜像）](https://people.eecs.berkeley.edu/~alanmi/publications/other/liberty13_03.pdf)。
实际项目应使用服务器所配版本的库和工具手册。

## 3. 推荐实施：显式例化工艺 ICG

先用服务器 DC 查询已加载库，以下为检查起点而非完整综合脚本：

```tcl
set candidates [get_lib_cells -quiet */*CLKNQ*]
foreach_in_collection c $candidates { puts [get_object_name $c] }

set cell [get_lib_cells -quiet */CLKNQD4BWP30P140HVT]
if {[sizeof_collection $cell] != 1} {
    error "ICG candidate missing or ambiguous; select one exact loaded library"
}
get_lib_pins */CLKNQD4BWP30P140HVT/*
# 根据当前DC版本：report_lib / get_attribute进一步核对属性、function和timing arcs。
# 不要忽略get_attribute报不支持的属性；回到实际Liberty或工具帮助检查。
```

确认后仅修改服务器 `en/clock_gate/mesh_icg_tech.v`。下面是**占位模板，不能直接编译**：

```verilog
module mesh_icg_tech (
    input wire clk_i, en_i, test_en_i,
    output wire gclk_o
);
    CONFIRMED_TSMC28_ICG u_icg (
        .CONFIRMED_CLOCK_PIN       (clk_i),
        .CONFIRMED_ENABLE_PIN      (en_i),
        .CONFIRMED_TEST_ENABLE_PIN (test_en_i),
        .CONFIRMED_GATED_CLOCK_PIN (gclk_o)
        // 电源/地或附加输出按实际模型/工程电源连接规范处理。
    );
endmodule
```

只有确认管脚确实是 CP/E/TE/Q 且极性吻合后，才可以把模板写成这些名字。
QD等额外管脚不得猜测存在或随意悬空；按真实模型处理。低有效E/TE必须按库定义适配。
若单元没有独立TE，是否允许在锁存器前合并E和TE须由库功能和DFT流程确认。

### 编译与选择

- 主RTL清单包含 `mesh_icg.v` 与**唯一一份**工艺版 `mesh_icg_tech.v`；不能同时编译
  临时FF模型和工艺版同名模块。
- RTL+真实库功能仿真定义 `MESH_ICG_USE_TECH`；综合的 `SYNTHESIS` 分支也会例化封装。
- Link使用正确 `.db`；仿真使用对应单元Verilog/UDP模型；P&R使用对应LEF/物理库。
- 先不盲目对整个封装层级加dont_touch；如需保留ICG实例，遵循DC/CTS流程的实例约束。
- 不把内部 gated clock 导出为异步、独立时钟；核实工具能由主时钟穿过ICG传播到Mesh。
  未传播时先修正库识别和约束，不要随意建立第二个互不相关的 `create_clock`。

## 4. 使能时序、DFT与驱动选择

功能使能源在 `conv1d_controller.v`：

```text
state、mesh_psum_valid、acc_in_ready
       → mesh_phase_active && mesh_ce
       → ICG Enable
       → ICG output → CTS buffer tree → Mesh寄存器
```

- 单独检查Enable和Test Enable的门控 setup/hold，不只看普通数据路径WNS。
  门控检查属于独立时序检查类别，见
  [OpenSTA实现中的 clock-gating report/check 选项](https://github.com/The-OpenROAD-Project/OpenSTA/blob/master/search/Search.tcl)。
  Synopsys服务器使用本机PrimeTime版本相应命令，不照搬OpenSTA选项。
- 正常功能STA可将顶层scan端口做 `set_case_analysis 0`；这是分析模式约束，
  不是在生产RTL里永久接0。扫描模式由DFT流程控制TE，验证功能和扫描都能安全开钟。
- 初始优先评估已确认的D4 HVT，再与D8 HVT对照；D4不是“最多驱动4个FF”。
  ICG输出经CTS缓冲树驱动Mesh，按实际slew/capacitance/skew决定驱动和布局。
- 记录D4/D8的门控检查、输出转换、钟树插入延迟/skew、buffer数量、时钟能耗。
  不要为使能晚到问题仅换大驱动，更不承诺更大驱动一定省功耗。

## 5. 最小验证闭环与验收

1. **库级小测**：主时钟低电平启动、E/TE已知，验证开关钟、完整脉宽、功能/扫描切换；
   对低电平晚到使能采用真实latch语义，不强求与临时FF模型逐周期等价。
2. **RTL+真实ICG**：单元库模型参与仿真，跑10帧真实PCM、630层、90级RVQ、DMA对拍。
   输出计数/最终码流必须检查，不能只看done或仿真退出码。
3. **综合网表检查**：`u_tech/u_icg`引用真实库单元；没有普通FF+AND替代物、
   unresolved cell或black box；门控识别报告及库/角/脚本信息留档。
4. **门级功能仿真**：先无SDF，再按流程加SDF。若gclk出现X，追踪clk/E/TE、电源、
   notifier和第一处未知值，不以全局 `initreg+0` 的PASS替代原因分析。
   库规定的初始化/UDP选项可以使用，但应记录并说明证明边界。
5. **CTS/布线后STA与功耗**：确认传播时钟、gating checks、pulse width、
   slew/cap、setup/hold和DFT模式；基于真实活动比较每帧能量。

本次不执行hold修复，也不修改1 ns uncertainty。正式映射后仍需重新检查
hold/clock-gating hold；本文并不建议跳过最终签核。

推荐的映射验收表：

| 项目 | 验收条件 |
|---|---|
| 功能 | 10帧逐层、逐级RVQ、最终码流/DMA与Golden一致 |
| 映射 | 网表实例真实库ICG，版本/角/VT及pin功能可追踪 |
| 门控 | functional/scan切换正确，无窄脉冲、未知钟或异常漏拍 |
| 时序 | 库门控检查、普通setup/hold及CTS/布线后时钟质量按项目要求通过 |
| PPA | 相同输入/活动窗口、库角和约束下测量，不从RTL名字或FF模型推定收益 |

## 6. 本地修改边界

当前未提供服务器库，因此本次不直接把候选单元写入活动RTL；继续保留已验证的
`mesh_icg_tech` 临时实现。待库资料确认后，应作为独立变更进行真实ICG映射与回归。
