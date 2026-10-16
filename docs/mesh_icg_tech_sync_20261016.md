# 服务器 Mesh 门控封装同步

## 本次范围

仅同步截图中的负边沿门控封装，不同步截图中提及但未提供实现的 P1 后处理三级流水。

- 主工程新增：`E:/111/encoder/rtl/clock/mesh_icg_tech.v`。
- 本地服务器上传副本新增：`E:/111/en/clock_gate/mesh_icg_tech.v`，两者内容相同。
- `E:/111/en/clock_gate/mesh_icg.v` 保持不变：普通 RTL 仿真仍用低透明 latch。
- 没有修改 Mesh、卷积、RVQ、SRAM、复位、hold 或时钟约束。

核心实现与服务器截图一致：

```verilog
reg en_latched;
always @(negedge clk_i) begin
    en_latched <= en_i | test_en_i;
end
assign gclk_o = clk_i & en_latched;
```

## 选择与验证

`mesh_icg.v` 在 `MESH_ICG_USE_TECH` 宏开启时实例化 `mesh_icg_tech`；现有 ASIC
`SYNTHESIS` 分支也实例化该封装。编译清单需包含此文件，不能只定义宏却漏编译封装。
默认不定义宏的 RTL latch 分支未改变。FPGA 不应因此改为逻辑门控时钟。

回归脚本新增可选开关，默认路径不变：

```powershell
cd E:\111\encoder
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -UseTechClockGate -Phase All -ClockMHz 50 -Frames 10 -LogTag mesh_negedge_sync
```

测试包括：

1. `tb_mesh_icg_tech.v`：通过真实 wrapper 的 TECH 分支测试功能使能、扫描使能、
   空闲停钟、完整脉冲，并专门检查低电平后半段更新使能时与 latch 的采样差异。
2. 现有投影边界测试及真实 Conv1d 输出反压测试。
3. 真实 PCM 10 帧独立编码、逐层 Golden、逐级 RVQ、DMA 记录及软件重建比较。

日志目录：`E:/111/encoder/logs/mesh_negedge_sync_50mhz/`。

### 2026-10-16 实测结果

- ModelSim：完整清单编译，以及门控相关文件 Verilog-2001 编译，均 0 错误、0 警告。
- XSIM：TECH 门控单测、投影边界测试、真实 Conv1d 反压测试均出现 `[TB_PASS]`。
- TECH 分支 50 MHz / QSPI 25 MHz 的真实 PCM 10 帧动态回归通过：
  630 个 `[LAYER_PASS]`、90 个 `[RVQ_STAGE_PASS]`、10 个 `[FRAME_PASS]`、
  10 个 `[DMA_PASS]`，日志末尾 `[TB_PASS]`；软件从 10 条实际 DMA 记录重建向量也匹配 Golden。
- 每帧 531524 周期，即 10.630480 ms，包含帧内搬运、Encoder、投影和 RVQ；
  不包含一次性的 Flash boot（21.628700 ms），不包含人为施加的输出排队等待。
- XSIM 本次实际编译了 `en/clock_gate/mesh_icg_tech.v`，其内容与主工程副本完全一致；
  最终 ModelSim 编译亦核实可选择主工程的 `rtl/clock/mesh_icg_tech.v`。
- 两份新封装 SHA256：`A6ADBC4AB6035D62E4D0F2622D9876CCFDCE0522DEE1B4073E55404B47D405F7`。
- 未改动的 `mesh_icg.v` SHA256：`0EE00EEE86E34EC12497E463C4DA54CC91E4AFAB7BFE202447CB1E96F743D813`。

以上是本地功能仿真/编译证据，没有运行服务器 DC、库 UDP/SDF 或最终物理时序验证。

## 适用边界

这是服务器兼容的 **negedge FF + AND 临时实现**，不是目标库 ICG 实例。
旧 wrapper 中要求绑定真实 ICG 的说明仍是正式 ASIC 实现要求，不能因封装名字含
`tech` 就声称已完成工艺映射。

- FF 只在下降沿采样，而 latch 在整个低电平透明，二者并非无条件等价。
- 当前控制器由主时钟上升沿更新使能；必须保证其满足下降沿采样的半周期时序。
- 模块没有 reset/initial。上电初值仍可能为 X；测试先经历已知使能的下降沿再检查。
- 本地功能回归不能证明服务器 UDP 初始化问题的根因，也不能替代库模型、SDF、
  时钟门控检查、DFT、CTS 和最终 STA。
