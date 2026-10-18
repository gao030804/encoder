# Residual Add同步服务器截图：四状态流水版本

日期：2026-10-18。

## 1. 本次范围与版本依据

依据用户提供的服务器截图，将本地三状态Residual Add改为：

```text
S_IDLE --输入握手--> S_READ --SRAM请求握手--> S_WAIT
S_WAIT --scratch_rsp_valid/锁存首次量化结果--> S_WAIT2
S_WAIT2 --下一上升沿寄存最终结果--> S_IDLE
```

截图不是完整服务器源码，因此本次对齐的是可见的状态编码与流水分段，
不宣称本地文件与服务器整个文件逐字节相同。
本地接口为64-bit一拍、4个INT16 lane；历史8lane模块名不变，不扩成8个INT16。
保留前一次48-bit乘积、Guard舍入、sat16/clip17、signed18加法优化。
没有修改Golden、量化参数、ICG、时钟约束、SRAM结构或Encoder/RVQ运算公式。

## 2. 硬件修改

仅修改 `rtl/residual_add_8lane.v`。

### 2.1 四状态及第一段寄存器

```verilog
localparam [2:0] S_IDLE=3'd0, S_READ=3'd1, S_WAIT=3'd2, S_WAIT2=3'd3;
reg signed [15:0] identity_aligned_w[0:3], main_aligned_w[0:3];
reg signed [15:0] identity_aligned_q[0:3], main_aligned_q[0:3];
```

`_w`是第一段组合结果；`_q`只在消费SRAM响应时写入，并在复位时清零。
两路各4x16 bit，共增加128 bit数据寄存器。

### 2.2 S_WAIT只完成第一段

Scale开启时，identity和main各自做首次requant及INT16饱和。
Scale关闭时，两路原始INT16也必须经过相同寄存级，不能在下一状态继续读取实时SRAM输出。

```verilog
S_WAIT: if(scratch_rsp_valid) begin
    for(reset_lane=0;reset_lane<4;reset_lane=reset_lane+1) begin
        identity_aligned_q[reset_lane] <= identity_aligned_w[reset_lane];
        main_aligned_q[reset_lane] <= main_aligned_w[reset_lane];
    end
    state <= S_WAIT2;
end
```

### 2.3 S_WAIT2完成第二段及输出

`added_data`只由锁存的`_q`计算，Scale开启时：

```text
main_aligned_q -> requant_clip17(m3,s3)
             + identity_aligned_q
             -> signed18 sum -> sat16 -> out_data
```

S_WAIT2仅停留一拍，无条件寄存输出并回IDLE。
返回IDLE不等于输出已消费：`busy=(state!=S_IDLE)||out_valid`；
`out_valid&&!out_ready`时禁止新输入覆盖，输出数据/地址/strb保持不变。

相对于原本地三状态版，每笔Residual处理增加一个系统周期；
这不是每拍接收新输入的全吞吐流水，仍沿用一次只处理一笔SRAM事务的状态机。
50MHz增加的单笔处理延迟为20ns，30MHz为约33.33ns；
全帧增加多少周期取决于Residual写拍数及上游反压，不能简单说每帧只加一拍。

## 3. 测试修改

- 修改 `tb/tb_residual_guard_optimization.v`：仍保留380268组算术与1901340组最终饱和检查。
- 新增 `tb/residual_add_8lane_pipeline_reference.v`：使用旧64位偏置舍入算法，
  但采用截图四状态协议，独立对比新RTL的逐周期握手及输出。
- 原三状态 `tb/residual_add_8lane_legacy_reference.v` 继续冻结，未修改；
  新增的四状态参考是测试模型，不是假称服务器原件，两者均不参与综合。
- SRAM无效拍扰动返回数据，保证WAIT2依赖锁存值；响应后必须先进入WAIT2且不输出，
  再下一沿输出；覆盖等SRAM响应及WAIT2内部两处中途复位。
- 保留1032笔事务、全部256种strb、Scale开启/关闭、随机请求延迟/输入停顿/
  输出反压、参数接收后变化及8笔错误地址检查。
- 修改 `scripts/run_qor_p0_regression.ps1` 加入四状态参考模块。

## 4. 仿真记录

XSIM专项测试已经通过：1032笔事务；1032次WAIT2输出时序检查；两处中途复位。
算术检查380268组、最终求和饱和1901340组，已有手算Residual测试通过。
ModelSim完整清单及修改模块/测试的Verilog-2001编译0错误、0警告。
50MHz/25MHz QSPI和30MHz/15MHz QSPI的真实PCM十帧独立编码均完成动态PASS，
每组630个LAYER_PASS、90个RVQ_STAGE_PASS、10个LATENT_PASS、10个FRAME_PASS、
10个DMA_PASS、10个QOR_DEADLINE和最终TB_PASS，未发现失败/错误标记。
两组都由实际DMA记录软件重建10帧，并匹配Golden。

| 系统/QSPI时钟 | 三状态旧版单帧 | 四状态新版单帧 | 新版距20ms的余量 |
| --- | ---: | ---: | ---: |
| 50/25MHz | 539144周期 / 10.782880ms | 549434周期 / 10.988680ms | 9.011320ms |
| 30/15MHz | 539144周期 / 17.971467ms | 549434周期 / 18.314467ms | 1.685533ms |

新版每帧增加10290周期；在50MHz下增加0.205800ms，在30MHz下增加0.343000ms。
帧0的增加分布于物理层2/4/6（各1279周期）、12/17/22（各1277周期）、
30/35/40（各633周期）、48/53/58（各241周期），合计10290周期。
其他层引擎周期未变化；受上游流量重叠影响，不要求所有Residual写拍都增加完整可见周期。

单帧时间从PCM输入启动统计到编码结果入输出环形缓冲，包含帧内输入/权重搬运、
Encoder、投影和RVQ；不含一次性Flash启动与测试人为DMA排队。
boot分别约21.628700ms与36.047833ms，不能算进每帧稳态20ms预算。
这是每帧历史清零的独立十帧回归，不是跨帧历史流式/FPGA板上/门级SDF测试。

逐条比较新旧实际DMA记录：10帧的frame_id和packed64码流完全相同；
因完成时间变晚，timestamp允许改变，不再宣称整个128-bit记录逐字节相同。
50MHz使用技术门控封装，30MHz使用原行为门控模型，ICG本身未修改。

完整日志：

- `logs/residual_wait2_50mhz/tb_qor_p0_full_frame.log`
- `logs/residual_wait2_30mhz/tb_qor_p0_full_frame.log`
- 专项：`logs/residual_wait2_50mhz/tb_residual_guard_optimization.log`

相关Postprocess Guard/Sticky、流水反压、Dense/Depthwise、投影和技术门控单测也通过。

```powershell
cd E:\111\encoder
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase All -UseTechClockGate -ClockMHz 50 -Frames 10 -LogTag residual_wait2
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Full -ClockMHz 30 -Frames 10 -LogTag residual_wait2
```

结果目录为`logs/residual_wait2_50mhz`、`logs/residual_wait2_30mhz`。
本轮没有运行服务器DC/VCS/SDF、STA或功耗分析，不给出新的PPA数值。

## 5. en位置

| 本地工程文件 | 本地en位置 |
| --- | --- |
| `rtl/residual_add_8lane.v` | `E:/111/en/lane/residual_add_8lane.v` |
| `tb/tb_residual_guard_optimization.v` | `E:/111/en/test/tb_residual_guard_optimization.v` |
| `tb/residual_add_8lane_pipeline_reference.v` | `E:/111/en/test/residual_add_8lane_pipeline_reference.v` |
| 本说明 | `E:/111/en/docs/residual_wait2_pipeline_sync_20261018.md` |

旧说明 `docs/residual_guard_optimization_20261018.md` 保留为上一阶段记录，
顶部新增版本提示；其中三状态/周期不变结果不适用于本次流水版本。
修改前的三状态硬件、测试与脚本备份在 `build/residual_wait2_baseline_20261018/`。
