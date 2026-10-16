# Encoder + RVQ：P0 RTL分拍优化与回归

日期：2026-10-16。

后续DC端口兼容性修复见`docs/dc_scan_port_compat_fix_20261016.md`：
五级`mesh_scan_test_en`改为普通wire，并在功能模式显式接0，修正服务器报告的tri0/VER-181问题。
下述P0算术流水保持不变；重新上传还应包含兼容性文档列出的端口文件，不要再使用旧tri0副本。

## 1. 实施范围

按照《Encoder_RVQ_综合QoR与RTL优化总结》先实施两个P0：

1. RVQ投影量化拆成Bias饱和、宽乘法、舍入饱和三拍。
2. Accumulator响应FIFO至Postprocess之间加入Data + Tag + Valid弹性寄存器。

遵照用户要求，不改Hold处理、SDC、1 ns时钟不确定性、工艺库、SRAM结构和Mesh ICG。
1 ns不确定性是否足以解释服务器上的全部Hold违例，需要详细时序报告确认，本次不据此修改约束。

文档明确建议P0后重综合，再决定P1三级Postprocess是否必要。
因此本次不同时实施P1，也不开展P2 Dense/Depthwise共享后处理、FIFO缩深和功耗架构实验。
没有改Golden、模型参数、量化顺序、输出码流布局和算术位宽。

## 2. 改动一：RVQ投影量化

文件：`E:\111\encoder\RVQ\rvq_backend_w8a16.v`。

旧路径：

```text
projection_acc -> INT40 Bias饱和 -> INT40*INT32 -> round80 -> sat16 -> residual
```

新路径：

```text
PBIAS : projection_acc + bias -> INT40饱和 -> biased_q[39:0]
PMUL  : biased_q * multiplier             -> product_q[79:0]
PQUANT: round80(product_q,shift) -> sat16  -> residual[channel]
```

新增`PBIAS=23`、`PMUL=24`，原0～22状态编码不变。
新增有复位、有状态更新使能的40-bit `biased_q`与80-bit `product_q`。
最后一拍PMAC非阻塞更新累加器，下一拍PBIAS再读取完整结果。
每通道的Multiplier/Shift在量化结束前保持不变。

示例：

```verilog
PBIAS: begin
    biased_q <= biased40;
    state    <= PMUL;
end
PMUL: begin
    product_q <= $signed(biased_q) * $signed(multiplier);
    state     <= PQUANT;
end
PQUANT: begin
    residual[channel] <= projection_result;
    // projection观察口与residual使用同一个最终结果。
end
```

保持INT40加Bias饱和、INT32有符号Multiplier、UINT8 Shift、half-away-from-zero及INT16饱和顺序。
每通道新增2拍，32通道共新增64拍。

## 3. 改动二：Accumulator输出弹性级

文件：`E:\111\encoder\rtl\conv1d_controller.v`。

新增寄存器：

- `post_skid_data[319:0]`：8路INT40累加结果。
- `post_skid_tag[M_TAG_WIDTH-1:0]`：与数据绑定的m地址标签。
- `post_skid_valid`：该级是否保存有效事务。

这里是一级弹性寄存器，不宣称是具有两个存储槽的完整Skid Buffer。

关键握手：

```verilog
wire post_skid_ready = !post_skid_valid || post_in_ready;
assign acc_rd_rsp_ready = post_stream_active && post_skid_ready;
```

该级空闲时可接收上游数据；满且下游阻塞时保持Data/Tag/Valid；
本拍下游消费旧数据时可同时接收新数据，连续流吞吐仍可达II=1。
只有有效事务到达才更新宽数据与Tag，不在空拍改写这些寄存器。

Postprocess输入改接`post_skid_data`，其valid改接`post_skid_valid`。
`post_tag_q`同时由原来的`acc_rd_rsp_addr`改为`post_skid_tag`，并在同一握手条件下推进。
如果只改数据不改Tag，输出的数值可能正确但写回m地址会错位。

控制器继续保持`S_POST_READ`，直到最后一个128-bit向量的两次64-bit写回全部完成。
离开该阶段才清除弹性级valid，避免在途结果被提前丢弃。

该级新增320+9+1=330位寄存状态，与投影的120位合计约450位，不含综合映射差异。
没有改变Accumulator SRAM、8深响应FIFO、Depthwise模块或Postprocess模块接口。

## 4. 测试与运行

新增测试：

| 文件 | 检查内容 |
|---|---|
| `tb/tb_qor_p0_projection.v` | 通过SRAM写口装载1536块合成镜像；32通道INT40上下界、Multiplier符号、Shift 0/1/63、正负半值舍入、INT16饱和；1280个Score、288次残差更新、9级平分取最小Index及结果反压 |
| `tb/tb_qor_p0_post_stream.v` | 真正启动Conv1d；4种短层、K/Cout尾部、单元素、ReLU、可变SRAM延迟、LFSR反压和70周期连续写回阻塞；逐Byte Golden、Data/Tag稳定和FIFO满/同时push-pop |
| `tb/tb_qor_p0_full_frame.v` | 包装现有完整Flash/真实PCM/逐层及RVQ Golden测试，增加新状态覆盖、弹性级稳定性及20 ms截止时间 |
| `scripts/run_qor_p0_regression.ps1` | Xsim动态回归与ModelSim编译/仿真入口，核对PASS、覆盖率及实际DMA记录的软件重建 |

完整回归读取en共享核心，但明确将两个同名模块替换成本次修改的本地RTL。
避免误用`E:\111\en`中尚未同步的旧文件；`source_hashes.csv`记录实际编译文件及SHA256。
本地使用OpenTitan SRAM行为模型、功能ICG模型与P25Q32H Flash模型。
不实例化UART，不定义FPGA_TARGET，不替代服务器TSMC SRAM/ICG库的验证。

```powershell
# 专项单元测试
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Phase Units

# 完整真实10帧，50 MHz / QSPI 25 MHz
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Phase Full -ClockMHz 50 -Frames 10

# 完整真实10帧，30 MHz / QSPI 15 MHz
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Phase Full -ClockMHz 30 -Frames 10

# ModelSim只编译，不等同于动态通过
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 -Simulator ModelSim -CompileOnly
```

日志目录分别为`logs/qor_p0_50mhz/`与`logs/qor_p0_30mhz/`。
完整主日志为`tb_qor_p0_full_frame.log`。

10帧完整动态PASS要求：630个LAYER_PASS、90个RVQ_STAGE_PASS、10个LATENT_PASS、
10个FRAME_PASS、10个DMA_PASS、10个QOR_DEADLINE、QOR_P0_PASS与最终TB_PASS同时成立，无ERROR/FAIL。
内部还逐帧比较32个投影结果、1280个Score与288次残差/码字/减法/重量化。
软件读取真实DMA导出记录，与原Golden重建向量核对，不能用软件参考记录替代实际DMA数据。

实际输入仍为`pcm`目录的真实PCM16，源SHA256：
`cc3bf75ac82e60058fc2cab30159770935f3151ccc8acc20d230f69628541011`。
10帧各320采样，16 kHz，独立帧计算历史清零，BOOT只执行一次。
这是独立帧回归，不是跨帧状态保留的流式音频质量验证。

## 5. 动态结果

专项动态运行已完成：

- 投影：32通道、1280个Score、288次更新、9级平分选择、13拍结果反压全部通过。
- 后处理：4个短层全部通过，写回442/304/2/162字节。
- 后处理覆盖：弹性级阻塞357拍、同时消费/替换66次、FIFO满124拍、同时push/pop 25次。
- 后处理/ELU/写回分别观察到379/396/342拍反压，未丢失、重复或错位。

### 完整50 MHz动态结果

Xsim 2018.3实际执行10帧完毕，脚本退出码0，最终TB_PASS与QOR_P0_PASS均出现。
630个LAYER_PASS、90个RVQ_STAGE_PASS、10个LATENT_PASS、10个FRAME_PASS、10个DMA_PASS及10个截止时间检查全部满足。
投影PBIAS和PMUL各执行320次，弹性级阻塞407840拍、同时消费/替换135820次，Data/Tag均保持正确。
实际10条DMA记录的软件重建与Golden全部一致。

| 边界 | 原版周期 | P0优化周期 | 增加周期 | 50 MHz优化后时间 |
|---|---:|---:|---:|---:|
| PCM启动到latent有效 | 518771 | 519077 | 306 | 10.381540 ms |
| latent到结果入输出环 | 12383 | 12447 | 64 | 0.248940 ms |
| PCM启动到结果入输出环 | 531154 | 531524 | 370 | 10.630480 ms |

10帧完整编码周期相同，距离20 ms预算9.369520 ms。总新增7.40 us，约0.0697%的原版帧延迟。
逐层CSV比较表明51层Dense/Pointwise计算受弹性级影响，合计增加306拍；其余12层Depthwise保持原周期。
一次BOOT仍为21.628700 ms，不计入稳态每帧编码时间。DMA消费者人为反压等待单独统计。
这些数字比较的是周期/服务时间；新增流水用于切断组合路径，不是减少运算周期。

### 编译兼容性

ModelSim 10.6c完整源文件编译为0错误、0警告。
另外对两个修改RTL单独运行`-vlog01compat`，确认Verilog-2001兼容，0错误、0警告。
这里不宣称ModelSim动态PASS；动态结果来自上述Xsim实际仿真。
Xsim整帧展开有一个既存未使用观察端口`output_full`未连接提示，未修改该无关接口。

### 完整30 MHz动态结果

Xsim 2018.3另行实际执行10帧，脚本退出码0，TB_PASS与QOR_P0_PASS均出现。
不是只按50 MHz周期换算。覆盖率与50 MHz完全相同，实际10条DMA记录的软件重建也全部匹配Golden。

| 项目 | 50 MHz / QSPI 25 MHz | 30 MHz / QSPI 15 MHz |
|---|---:|---:|
| 每帧周期 | 531524 | 531524 |
| PCM启动到latent | 10.381540 ms | 17.302567 ms |
| latent到结果入输出环 | 0.248940 ms | 0.414900 ms |
| 完整编码时间，含帧内搬运 | 10.630480 ms | 17.717467 ms |
| 距离20 ms预算 | 9.369520 ms | 2.282533 ms |
| 一次BOOT | 21.628700 ms | 36.047833 ms |
| 逐层/逐级/完整帧/DMA通过数 | 630/90/10/10 | 630/90/10/10 |

两次回归均为BOOT一次、10帧独立计算历史、多帧之间不复位整个电路。
BOOT和故意施加的DMA队列等待不计入稳态编码服务时间。

原Golden/参数未修改，抽查SHA256用于复现：

| 文件 | SHA256 |
|---|---|
| `tb/data/flash_encoder_rvq/flash_image.hex` | `554CE3F36C865B3301C65D79CF9FB64CF4F8065346AB102E655FD850DEA1C09D` |
| `tb/data/encoder_rvq_multiframe/frame_000/golden_layers.mem` | `2D039C56BD98F4155C54CE1669943EAA573A962435A4A000F0290FED7A1168B1` |
| `tb/data/encoder_rvq_multiframe/packed_all.hex` | `55FB069D4B1414A8471F4ACF9F1951F0A6C663D3DDF5C501D1BF719BE19A5483` |

## 6. 服务器需要更新的文件

只有两个硬件RTL文件需要替换，不新增硬件模块，不需要增加硬件filelist条目：

| 本地修改文件 | 服务器en目录目标 |
|---|---|
| `E:\111\encoder\RVQ\rvq_backend_w8a16.v` | `en/rvq_rtl/rvq_backend_w8a16.v` |
| `E:\111\encoder\rtl\conv1d_controller.v` | `en/adress/conv1d_controller.v` |

两个RTL修改前的原始副本位于`build/qor_p0_baseline_20261016/`。
上传副本整理到`build/qor_p0_upload_20261016/`，按`adress/rvq_rtl/test/scripts/docs`分类。
本次没有自动覆盖外部`E:\111\en`副本。
旧的、只读取en目录的仿真脚本仍会读到旧RTL，应先同步这两个文件，或使用本次明确覆盖源文件的新入口。

如在服务器复现专项测试，再上传3个新增TB到`en/test/`并加入测试filelist，选择对应顶层。
完整封装依赖已存在的基础TB与3个`.vh`，include/data/log路径按服务器原有布局调整。
PowerShell脚本供本地使用，Linux沿用服务器VCS入口。
继续使用服务器原Flash模型，不上传或覆盖服务器Flash模型与SRAM/ICG宏。

## 7. 证据边界

本次只验证RTL整数功能、握手与周期预算。
没有服务器综合器、目标工艺库与完整SDC，因此不宣称新的ns延迟、WNS、逻辑级数、面积或功耗已改善。
下一步在相同SDC/工艺角下重综合两个修改文件，检查关键路径迁移，再决定是否实施P1后处理三级流水。
新增寄存器可能减少组合毛刺，也会增加时钟负载，净功耗变化需同条件门级活动分析。
