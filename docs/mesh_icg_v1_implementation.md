# 4×8 Mesh V1 时钟门控实现与验证

日期：2026-10-14。依据用户提供的《Encoder 4×8 Mesh 整体时钟门控（ICG）方案总结》实施 V1。

## 1. 门控边界

在 `rtl/conv1d_controller.v` 的 `u_mesh` 时钟入口插入 `mesh_icg`：

```text
clk ─────────────────── Controller / Activation Reader / Weight Loader
  ├──────────────────── Accumulator / 后处理 / SRAM / Flash
  └─ mesh_icg ─ mesh_gclk ─ Weight Active/Shadow
                         ├ Input Skew
                         ├ Systolic Array / 32 PE
                         ├ Valid / Tag Pipe
                         └ Output Deskew
```

```verilog
mesh_phase_active = (state == S_WEIGHT_LOAD) ||
                    (state == S_STREAM) ||
                    (state == S_DRAIN) ||
                    (state == S_WAIT_ACC);
mesh_clk_en = mesh_phase_active && mesh_ce;
mesh_ce = !mesh_psum_valid || acc_in_ready;
```

保留原有 CE、输入握手、acc_fire 计数、Shadow 装载与 Swap 条件。无新增 FSM 状态，不修改 Skew/Deskew 延迟。WAIT_ACC 整段允许开钟，尚未实施 V1.1/V2。

## 2. RTL 改动

| 文件 | 改动 |
|---|---|
| `rtl/clock/mesh_icg.v` | 新增 ICG 接口、低电平锁存行为模型、工艺绑定分支及 FPGA 时钟旁路。 |
| `rtl/conv1d_controller.v` | 新增 V1 开钟窗口；仅将 u_mesh 时钟改接 mesh_gclk；参数 MESH_CLOCK_GATE_ENABLE=0 可回到常开时钟。 |
| `rtl/conv1d_engine_top.v` | 向 Dense 控制器传递 mesh_scan_test_en。 |
| `rtl/conv1d_engine_flash_top.v` | 向卷积引擎传递 mesh_scan_test_en。 |
| `rtl/soundstream_encoder_top.v` | 向计算引擎传递 mesh_scan_test_en。 |
| `rtl/soundstream_encoder_rvq_w8a16_top.v` | 编码系统引出并传递 mesh_scan_test_en。 |
| `rtl/fpga/fpga_system_top.v` | FPGA 功能模式将扫描旁路固定为 0。 |
| `scripts/fpga_system_files.f` | 新增 mesh_icg.v，共 50 个 RTL 文件。 |
| 6 个已有 Controller 编译脚本 | 补充 mesh_icg.v，防止旧的 Encoder 回归遗漏依赖。 |

`mesh_scan_test_en` 是默认低电平的可选输入。ASIC DFT 应从真实测试模式连接该输入；测试使能仅旁路 ICG，不修改功能 CE 和 valid/ready。

## 3. 三种实现配置

| 配置 | 行为 |
|---|---|
| ASIC 功能仿真，不定义 FPGA_TARGET | 使用低电平透明 latch + AND 的功能模型。 |
| 定义 FPGA_TARGET，未定义 SIM_MESH_ICG | 时钟直通；原有 CE 保留，避免 FPGA 普通逻辑门控时钟。 |
| 定义 FPGA_TARGET 和 SIM_MESH_ICG | 使用 FPGA RAM 模型，但 Mesh 时钟实际停止，用于门控功能验证。SIM_MESH_ICG 仅用于仿真。 |
| 定义 MESH_ICG_USE_TECH | 实例化 mesh_icg_tech 工艺封装，需另提供真实工艺绑定。 |
| 定义 MESH_ICG_BYPASS | 常开时钟对照组，仍保留原有 CE。仅用于基线比较。 |

当前工程没有可确认的 TSMC ICG 单元名和引脚定义，不能声称已经完成工艺 ICG 映射。ASIC 综合必须显式定义 `MESH_ICG_USE_TECH` 并提供如下同接口封装：

```verilog
module mesh_icg_tech (
    input  clk_i,
    input  en_i,
    input  test_en_i,
    output gclk_o
);
    // 在此实例化 Liberty 与 Verilog 模型中一致的真实 ICG。
    // clk_i/en_i/test_en_i/gclk_o 按库中的名称和极性连接。
endmodule
```

上面的空模块仅是接口说明，不能用于综合。需要真正的实例和引脚连接。定义 SYNTHESIS 的非 FPGA 分支同样要求 mesh_icg_tech，缺失绑定应作为综合错误处理。

服务器完成绑定后，应核对 ICG 实际实例、Mesh 寄存器时钟扇出、clock-gating setup/hold、正常 setup/hold 与 CTS 后检查，随后再比较 PrimeTime PX 每帧能量。功能仿真的时钟边沿统计不等于实际节电比例。

## 4. 新增验证文件

| 文件 | 用途 |
|---|---|
| `tb/tb_mesh_clock_gate.v` | 门控 Mesh 与常开 Mesh 同时运行，使用独立整数点积比较全部 8 路部分和和 m_tag。 |
| `tb/tb_mesh_icg_encoder_regression.v` | 复用真实 PCM、QSPI Flash、Encoder63、投影、RVQ9 和 UART 的 Golden scoreboard，额外观察实际 GCLK 和门控窗口。 |
| `scripts/run_mesh_icg_xsim.ps1` | Xsim 编译和动态仿真入口，日志必须有 TB_PASS，集成还必须满足覆盖计数。 |
| `scripts/run_mesh_icg_modelsim.ps1` | ModelSim 入口，支持 CompileOnly。 |
| `scripts/wave_mesh_icg.do` | ModelSim 单元测试关键波形。 |

单元测试覆盖连续和气泡输入、长反压、Direct Load、Shadow Load、Swap、Drain、空闲停钟、停钟异步复位和高电平期间测试使能改变。空闲阶段的无效数据不要求与常开版相同。

集成回归来自 `pcm/` 的真实 10 帧输入，使用已生成的 `tb/data/encoder_rvq_multiframe` Golden。每帧历史清零；这不是跨帧历史流式功能验证。

## 5. 运行命令

从 `E:\111\encoder` 执行：

```powershell
# Xsim：阵列门控单元验证
powershell -ExecutionPolicy Bypass -File .\scripts\run_mesh_icg_xsim.ps1

# Xsim：启用真实停钟，10 帧 PC -> FPGA 编码 -> PC 回归
powershell -ExecutionPolicy Bypass -File .\scripts\run_mesh_icg_xsim.ps1 -Integration -Frames 10 -Mode Realtime

# Xsim：常开时钟一帧对照，独立日志与 Offline 输出文件
powershell -ExecutionPolicy Bypass -File .\scripts\run_mesh_icg_xsim.ps1 -Integration -Baseline -Frames 1 -Mode Offline

# ModelSim：同样的单元测试（需要有效 vsim 许可证）
powershell -ExecutionPolicy Bypass -File .\scripts\run_mesh_icg_modelsim.ps1

# ModelSim：完整 10 帧集成测试
powershell -ExecutionPolicy Bypass -File .\scripts\run_mesh_icg_modelsim.ps1 -Integration -Frames 10
```

不要同时运行相同 Mode 的两个集成测试：原 UART testbench 按 Mode 写结果文件，会发生输出文件冲突。

## 6. 波形判读

- IDLE/参数等待/权重等待/后处理：en=0，下一低电平捕获后 GCLK 停止。
- WEIGHT_LOAD：GCLK 在预期的主时钟上升沿出现，Active 权重装入。
- STREAM/DRAIN：ce=1 时产生完整 GCLK，数据与 valid/tag 同步推进。
- 输出 valid=1 且 Accumulator ready=0：ce=0，下一周期 GCLK 不出现，整条 Mesh 保持。
- 反压解除：下一允许的主时钟上升沿恢复推进，输入与输出顺序不变。
- Shadow Load：只更新 Shadow；Swap：当前输出已经被接收、累加写回结束后切换 Active。
- 测试使能在 CLK 高电平改变：不能产生中途上升沿或截短高电平脉冲。

ModelSim 单元测试查看方式：

```tcl
vsim -voptargs=+acc -lib build/mesh_icg_modelsim_work tb_mesh_clock_gate
do scripts/wave_mesh_icg.do
```

## 7. 本次实测结果

动态仿真使用 Xsim 2018.3；ModelSim 10.6c 编译无错误、无警告，本机 vsim 因许可证不可用未运行成功。

- 单元测试已实际 PASS：96 个向量、117 个反压周期、1 次 Direct Load、2 次 Shadow Load、2 次 Swap。
- 单元统计：主时钟 392 个上升沿、Mesh 门控时钟 241 个上升沿。
- 完整门控回归已实际 PASS：10 帧真实 PCM，630 次逐层检查、90 次 RVQ 级检查、10 条 UART/DMA 输出记录，软件由实际记录重建的结果也匹配 Golden。
- 常开时钟一帧对照已实际 PASS：63 层、9 级；第 0 帧完整 128-bit 输出记录与门控版本完全相同，为 `0000716000000000f93d798bb147b942`。
- 常开对照与门控的全部 10 帧服务时间一致：531155 个 50 MHz 周期，即 10.62310 ms。第 0 帧 PC→FPGA→PC 延迟均为 899427 周期，即 17.98854 ms。

| 指标 | 10 帧门控回归 |
|---|---:|
| 主时钟上升沿 | 10997201 |
| Mesh 门控时钟上升沿 | 2084950 |
| GCLK/CLK | 18.9589% |
| 非 Mesh 活动阶段周期 | 8912251 |
| Direct Load / Shadow Load / Swap | 3130 / 45070 / 45070 |
| Accumulator 接受的部分和向量 | 1217280 |
| BOOT Global / Parameter / RVQ 写入块 | 6912 / 1792 / 1536 |

统计窗口从系统复位释放开始，包含 Flash BOOT、UART 收发和帧间等待。因此 18.9589% 是这个测试窗口的时钟边沿比例，不是仅卷积阶段占比，也不是整机功耗比例。集成运行中自然 Accumulator 反压为 0；反压下的门控/结果一致性由单元测试的 117 个阻塞周期覆盖。

输入 manifest：`raw_pcm_independent_frames`，来源 `E:\111\encoder\pcm`，SHA256 为 `cc3bf75ac82e60058fc2cab30159770935f3151ccc8acc20d230f69628541011`。

已核对新 FPGA 清单 50 个文件均存在且均列入硬件清单文档。Xsim 门控与基线编译/展开日志没有 WARNING/ERROR。

日志：

- `logs/mesh_icg_unit.log`
- `logs/mesh_icg_encoder_realtime.log`
- `logs/mesh_icg_encoder_offline_baseline.log`
- `logs/mesh_icg_modelsim_encoder_realtime_compile.log`

本次验证不包含实际工艺 ICG、SDF 门级仿真、DC/STA 或 PT PX 测量。
