# 扫描端口tri0的ASIC综合兼容性修复

日期：2026-10-16。

## 问题与改法

服务器截图显示所用DC前端在读取`input tri0 mesh_scan_test_en`时报告VER-181。
此前ModelSim/Xsim接受该写法，不代表服务器DC前端接受。
本次将活动硬件层级的五处声明统一改为`input wire`，不依赖隐式弱下拉。

```verilog
// 原来：input tri0 mesh_scan_test_en;
input wire mesh_scan_test_en;

// 功能模式的顶层实例必须显式连接，不能继续悬空：
.mesh_scan_test_en(1'b0)
```

各级内部仍传递真实scan输入，最终连接ICG的test_en_i。
不能为解决语法问题在控制器内部永久绑0，因为DFT需要旁路门控。
功能模式由顶层/testbench明确驱动0，扫描模式由测试控制明确驱动。

## 文件与同步范围

| 本地文件 | en/服务器对应位置 | 修改 |
|---|---|---|
| `rtl/conv1d_controller.v` | `en/adress/conv1d_controller.v` | 普通wire端口，说明功能模式必须接0 |
| `rtl/conv1d_engine_top.v` | `en/adress/conv1d_engine_top.v` | 普通wire端口 |
| `rtl/conv1d_engine_flash_top.v` | `en/flash/conv1d_engine_flash_top.v` | 普通wire端口 |
| `rtl/soundstream_encoder_top.v` | `en/system top/soundstream_encoder_top.v` | 普通wire端口 |
| `rtl/soundstream_encoder_rvq_w8a16_top.v` | `en/system top/soundstream_encoder_rvq_w8a16_top.v` | 普通wire端口 |
| `rtl/soundstream_encoder_rvq_top.v` | 旧版兼容封装，不属于当前W8A16核心filelist | 内部Encoder实例明确接scan=0 |

修改前五对本地/en文件SHA256完全一致，已备份到`build/dc_scan_port_baseline_20261016/`后同步五个修复文件。
本地en的完整帧TB保留原有版本，只增加scan=0一条连接，没有用本地较新整份TB强行覆盖。

本地以下四个TB的实例增加scan=0：

- `tb/tb_flash_encoder_rvq_full_frame.v`
- `tb/tb_flash_encoder_full_frame.v`
- `tb/tb_soundstream_encoder_full_frame.v`
- `tb/tb_conv1d_engine.v`

旧TB只维护连接，不宣称其旧位宽/Golden数据已完成独立回归。
FPGA板级顶层与P0后处理专项原先已明确接0，保持不变。

## 测试与防回归

新增`tb/tb_mesh_scan_port_compat.v`：实际实例化控制器与行为ICG，检查功能模式空闲停钟、scan=1透传7+5个完整脉冲、关闭后恢复停钟。不force DUT。

更新`scripts/run_qor_p0_regression.ps1`：

- 明确使用五个本地端口修复模块，避免误编译旧en副本。
- 编译前拒绝活动源文件中的`input tri0 mesh_scan_test_en`，防止重新引入。
- Units阶段加入扫描专项。
- 增加LogTag参数，保留旧P0日志。

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Phase All -ClockMHz 50 -Frames 10 -LogTag dc_scan_fix
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Phase Full -ClockMHz 30 -Frames 10 -LogTag dc_scan_fix
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Simulator ModelSim -CompileOnly -LogTag dc_scan_fix
```

已完成：扫描专项动态PASS（28个root边沿、12个完整Mesh透传脉冲）；投影专项和后处理反压专项动态PASS；ModelSim全源以及六个相关RTL的Verilog-2001编译均0错误、0警告。
50/30 MHz分别实际运行完整真实10帧，脚本退出码均为0，最终TB_PASS与QOR_P0_PASS均出现。

| 检查 | 50 MHz / QSPI 25 MHz | 30 MHz / QSPI 15 MHz |
|---|---:|---:|
| 逐层PASS | 630 | 630 |
| 逐级RVQ PASS | 90 | 90 |
| 完整帧 / DMA记录 | 各10 | 各10 |
| 错误/FAIL标记 | 0 | 0 |
| 每帧周期 | 531524 | 531524 |
| PCM到结果入输出环 | 10.630480 ms | 17.717467 ms |
| 20 ms剩余预算 | 9.369520 ms | 2.282533 ms |
| 实际DMA码流的软件重建 | 10条全部匹配Golden | 10条全部匹配Golden |

计时包含帧内数据搬运，不含一次BOOT和DMA消费者等待。帧周期与端口修复前P0一致。
动态日志为`logs/dc_scan_fix_50mhz/tb_qor_p0_full_frame.log`与`logs/dc_scan_fix_30mhz/tb_qor_p0_full_frame.log`。
源码SHA256记录在各目录的source_hashes.csv。动态结果使用本地SRAM/ICG/Flash功能模型，不含服务器SDF或目标工艺时序。

## 交付与边界

五个ASIC端口修复文件整理到`build/dc_scan_port_fix_upload_20261016/`，并更新原P0上传目录，避免旧上传副本再次带回tri0。
原filelist中的模块声明变化，没有新增综合硬件模块。
旧ICG说明`docs/mesh_icg_v1_implementation.md`同步改成必须显式接0，避免继续照旧说明悬空端口。
服务器若有其他未同步改动，应只合并端口声明与功能模式连接，不盲目整份覆盖。

本次不改P0算术流水、量化公式、Golden、模型参数、ICG实现、SDC和Hold处理。
没有执行截图中的目录删除操作，也未启动服务器DC任务。
本机没有服务器DC/工艺库，不宣称DC综合或VER-181复测已实际通过。实际分析、链接和综合仍需服务器重跑。
