# 后处理关键路径优化与三级弹性流水

日期：2026-10-16；依据用户提供的《Encoder_RVQ_最新QoR_ICG工艺映射与PPA优化总结》。
目标为切断Depthwise Acc到后处理输出之间的长组合路径，不做资源共享、SRAM缩容或hold修复。

## 1. 改了什么，没改什么

| 文件 | 修改内容 | en中对应位置 |
|---|---|---|
| `rtl/postprocess_8lane.v` | 72-bit精确乘积、等价舍入、三级Elastic Pipeline、Tag随数据流动 | `lane/postprocess_8lane.v` |
| `rtl/conv1d_controller.v` | Dense接入后处理内部Tag，删除外侧旧的单拍post Tag延迟 | `adress/conv1d_controller.v` |
| `rtl/depthwise_conv1d_controller.v` | 显式连接无用Tag为0，说明并验证握手等待可适配三级延迟 | `adress/depthwise_conv1d_controller.v` |
| `scripts/run_qor_p0_regression.ps1` | 使用本地新RTL覆盖en旧副本，增加两项专项动态测试 | 本地测试脚本，不加入综合 |

新增测试：`tb/tb_postprocess_pipeline.v`、`tb/tb_postprocess_depthwise.v`。
新增说明：本文及独立的 `docs/mesh_icg_tsmc28_mapping_recommendations.md`。

以下内容保持不变：两套独立Dense/Depthwise后处理实例、所有SRAM容量及宏组织、Mesh、
Accumulator/RVQ计算算法、Golden文件和真实量化参数、门控RTL、hold约束和1 ns uncertainty。
72-bit乘积是后处理INT40 x INT32的精确结果宽度，不是改变Mesh的INT16 x INT8乘法。
没有承诺面积下降；流水寄存器会增加，目标是缩短每拍组合路径。

## 2. 原路径与新路径

原来同一拍包含：

```text
Depthwise Acc INT40
 → Bias加法/INT40饱和 → 乘INT32 → 80位舍入/移位 → ZP → INT16饱和
 → post_data寄存器
```

现在Dense和Depthwise都使用以下三级（仍各有一套，不共享）：

| 级 | 运算 | 存储的数据及随路信息 |
|---|---|---|
| PP0 | 41位精确Acc+Bias，饱和回INT40 | 8xINT40；8xINT32 Mult、8xUINT8 Shift、8xINT8 ZP、Tag |
| PP1 | signed INT40 x signed INT32 | 8xINT72；对应的Shift、ZP、Tag |
| PP2 | 舍入、右移、加ZP、INT16饱和 | 8xINT16、Tag |

输入在第N个上升沿握手后，PP0在该沿更新、PP1在N+1更新、PP2在N+2更新，
下游可在N+3沿握手。原来N+1即可握手，因此首结果增加两拍，连续吞吐仍可达II=1。
这不是将三个级并回一条组合路径；使能信号和valid/ready不携带算术计算。

## 3. 算术等价优化

### 3.1 Bias与乘法

保持原有合同：Bias外部64位容器仅低40位有效，Acc+Bias先用41位精确求和，
再饱和到[-2^39,2^39-1]，然后才乘INT32。

```verilog
sum = {acc[39],acc} + {bias[39],bias};
// INT40饱和后进入PP0寄存器
pp1_product[72*lane +: 72] <=
    $signed(pp0_biased[40*lane +: 40]) *
    $signed(pp0_mult[32*lane +: 32]);
```

40x32的完整乘积为72位，不丢失有效位。舍入前扩展到73位，避免舍入加法越界。
不能为了减少乘法宽度而先把Acc或Bias截断成INT32。

### 3.2 Half-away-from-zero

正常位移s>0时使用：

\[
q=\left(p+2^{s-1}-[p<0]\right)\mathbin{\text{>>>}}s
\]

```verilog
extended_product = {product[71],product};
round_offset = (73'sd1 << (shift-1'b1)) -
               (product[71] ? 73'sd1 : 73'sd0);
shifted = (extended_product + round_offset) >>> shift;
```

例如p=-3,s=1，得到-2；p=-4,s=1，仍得到-2。
不是对负数直接“减half再算术右移”，后者会使部分负数多减1。
shift=0旁路舍入；之后按旧顺序加signed INT8 Zero Point，再饱和到INT16。

### 3.3 完整UINT8输入域的兼容说明

检查 `tb/data/w8a16_encoder/parameter_shift.hex`，当前参数为0、36..42。
本次测试仍覆盖了0..255，避免变位宽时无意改变旧RTL的扩展行为：

- 1..71：使用上述73位等价公式；0：直接用乘积。
- 72..79及81..255：旧80位实现的移位结果为0，新实现也为0。
- **80：旧RTL的80位舍入常数落在符号位，产生特殊的有限位宽行为**：
  乘积正或零时移位结果为-1，乘积负时为+1。新代码显式保留这一行为以做到位级兼容。

最后一项不是理想的任意精度量化公式，也不推荐导出shift=80。本次没有悄悄修复该异常
输入政策；若今后允许该参数，应作为单独变更统一导出器/Golden/RTL，并重新验证。
部署范围内与数学half-away-from-zero、旧RTL和现有Golden一致。

## 4. 反压和Tag为什么必须一起改

每级采用标准空槽/腾空接收规则：

```verilog
wire pp2_ready = !post_valid || post_ready;
wire pp1_ready = !pp1_valid || pp2_ready;
wire pp0_ready = !pp0_valid || pp1_ready;
```

例如PP2被阻塞时，PP1/PP0仍可利用空槽吸收输入；三槽全部占用后才反压上游。
每一级数据、参数、Tag和valid必须同时保持。不能只把valid延迟三拍，也不能在
乘法那拍读取已换成下一笔的外部multiplier或shift。

Dense原来在Controller中单独把m_tag延迟一拍。现在这样会与三级数据错位，因此：

```verilog
postprocess_8lane #(.TAG_WIDTH(M_TAG_WIDTH)) u_post (
    // 原来的数据/参数接口保持
    .in_tag(post_skid_tag),
    .post_tag(post_tag_q)
);
```

ELU单级的Tag镜像仍保留，最终写回地址由与输出数据同笔的m_tag计算。
`post_tag_valid`变为`post_valid`的可读别名，不再有独立的一拍延迟寄存器。

Depthwise本来就按握手等待而不是固定延迟，**无需改状态编号或强加计数器**：

```verilog
S_POST_SEND: if(post_in_ready) state <= S_POST_WAIT;
S_POST_WAIT: if(post_valid && post_ready) state <= S_WRITE_WAIT;
S_WRITE_WAIT: if(writer_vector_done) begin
    // 完整8xINT16写回后才更新m/og，最后一组仍按strobe屏蔽尾通道
end
```

本路径只允许一个向量在途，m/og在完整写回前不变，Tag接0即可。
后处理每向量多2拍会直接增加Depthwise延迟；Dense则主要在每个输出组流水起止增加延迟。

## 5. 测试与当前结果

### 专项验证（实际XSIM动态执行）

| 测试 | 覆盖与结果 |
|---|---|
| `tb_postprocess_pipeline` | 1024向量/8192 lane，与独立旧80位参考逐位一致；256种shift；INT40 Bias溢出、INT32极值、半值、INT16饱和、逐笔换参/Tag、复位清流水 |
| Elastic Pipeline覆盖 | 最大连续输出78组；三槽满2871拍；三级分别反压2182/2347/3011拍；无阻塞时输入到输出握手3周期 |
| `tb_postprocess_depthwise` | 四层不复位连续启动，K/Cout尾、单通道末项、stride=2/dilation=2、ReLU、随机SRAM延迟及长输出反压；72向量全部逐Byte匹配 |
| 既有Dense反压测试 | 72向量逐Byte匹配；Skid/Accumulator FIFO/后处理/ELU/Writer反压与输出地址计分板通过 |
| 既有投影/门控单测 | 32个投影边界、1280 score、9级平局选择、扫描/功能门控均通过 |
| ModelSim编译 | 完整源清单以及修改文件Verilog-2001编译：0错误、0警告；不是将编译结果当作动态PASS |

专项日志：`logs/postprocess_p1_units_50mhz/`。
ModelSim编译日志：`logs/postprocess_p1_compile_50mhz/`。

### 50 MHz完整10帧回归（实际XSIM动态执行）

系统50 MHz、QSPI25 MHz，选择临时TECH门控；真实PCM独立10帧：
630个LAYER_PASS、90个RVQ_STAGE_PASS、10个FRAME_PASS、10个DMA_PASS，
最终TB_PASS；软件从实际10条DMA记录重建量化向量也匹配Golden。

| 时间边界 | 优化前 | 三级后处理后 |
|---|---:|---:|
| 单帧服务周期 | 531524 | 539144 |
| PCM输入到latent | 10.381540 ms | 10.533940 ms |
| 投影+RVQ到结果入队 | 0.248940 ms | 0.248940 ms |
| 帧内搬运+Encoder+投影/RVQ | 10.630480 ms | 10.782880 ms |
| 距20 ms预算余量 | 9.369520 ms | 9.217120 ms |
| 一次性Flash boot | 21.628700 ms | 21.628700 ms |

增加7620周期，即0.152400 ms（约1.43%），与63层engine周期差值之和一致。
其中Depthwise增加7008周期，Dense增加612周期。流水目标是Setup裕量，不是降低
固定频率下的首结果延迟。帧服务时间不含一次性boot及测试人为制造的DMA排队等待。

完整日志：`logs/postprocess_p1_50mhz/tb_qor_p0_full_frame.log`。

### 30 MHz完整10帧回归（实际XSIM动态执行）

系统30 MHz、QSPI15 MHz，使用未修改的行为latch门控分支。630层、90级RVQ、
10帧/DMA及软件重建全部匹配Golden，最终TB_PASS，单帧同为539144周期。

| 时间边界 | 三级后处理后 |
|---|---:|
| PCM输入到latent | 17.556567 ms |
| 投影+RVQ到结果入队 | 0.414900 ms |
| 帧内搬运+Encoder+投影/RVQ | 17.971467 ms |
| 距20 ms预算余量 | 2.028533 ms |
| 一次性Flash boot（不计入稳态单帧） | 36.047833 ms |

完整日志：`logs/postprocess_p1_30mhz/tb_qor_p0_full_frame.log`。
这里测试的是10帧独立编码，不是跨帧历史的真实连续流算法，也不是PC/UART板级证明。

## 6. 复现命令

```powershell
cd E:\111\encoder
# 专项动态测试
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Units -UseTechClockGate -ClockMHz 50 -LogTag postprocess_p1_units
# 50 MHz/25 MHz，10帧完整动态测试
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Full -UseTechClockGate -ClockMHz 50 -Frames 10 -LogTag postprocess_p1
# 30 MHz/15 MHz，原行为latch门控分支，10帧完整动态测试
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Full -ClockMHz 30 -Frames 10 -LogTag postprocess_p1
```

脚本支持ModelSim编译/运行入口；本次动态结果来自XSIM，不是服务器DC/VCS/SDF验证。
回归继续使用原Golden和真实参数，没有为适应新RTL修改预期数据。

## 7. 上传与证明边界

三份硬件文件必须作为同一变更上传，不能只替换postprocess而留下旧Dense单拍Tag：

```text
en/lane/postprocess_8lane.v
en/adress/conv1d_controller.v
en/adress/depthwise_conv1d_controller.v
```

三份本地en副本已同步并用SHA256验证与主工程一致；覆盖前先检查en仍与修改前原件一致。

新模块没有新增硬件源依赖，综合清单原文件路径保持。新增测试放en/test（若需服务器
运行它们应加入测试清单），不放硬件综合清单。修改前原件在
`build/postprocess_p1_baseline_20261016/`。

当前只证明功能/位级测试和实时预算；**没有重新运行服务器综合，因此没有新的
WNS/TNS、关键路径ns、面积或功耗结果**。特别不能承诺“16.4 ns平均分成三等份”。
服务器需在相同库角/50 MHz/原uncertainty下重新综合，检查PP0、PP1和PP2的新路径、
clock gating checks及寄存器数量。hold不做本轮修复，但流水后的新短路径仍需报告。
真实TSMC28 ICG实施单独见 `mesh_icg_tsmc28_mapping_recommendations.md`。
