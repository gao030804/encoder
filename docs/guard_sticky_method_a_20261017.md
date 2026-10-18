# PP2 Guard/Sticky 方法A：实现与验证

日期：2026-10-17。依据《Encoder_RVQ_GuardSticky_73bit舍入加法优化方法A.md》实施。

## 1. 修改边界

唯一硬件修改为 `rtl/postprocess_8lane.v` 中PP2的舍入函数及其调用名。
原函数 `round_shift_sat16` 替换为 `round_shift_sat16_opt`。

- PP0 Bias加法/INT40饱和、PP1 INT40 x INT32乘法及72-bit乘积不变。
- 三级流水级数、ready/valid、参数/Tag锁存、复位和输出接口不变。
- 不增加流水级，不修改Dense/Depthwise控制器、RVQ、SRAM容量或ICG。
- Golden、量化参数、Flash镜像和原始PCM测试数据不变。
- 不修改综合约束、不做hold修复，不合并方法B的Shift集合专用化。

修改前73-bit版及原测试脚本保存在 `build/guard_sticky_baseline_20261017/`。
脚本检查确认：`integer lane;`之后的全部流水时序代码，归一化函数调用名后与原件完全一致。

## 2. 算术如何改变

原算法：

```text
72-bit product -> sign extend73 -> 加半量化偏置/负数修正
 -> 可变算术右移 -> 宽Zero Point加法 -> INT16饱和
```

方法A：

```text
72-bit product -> 可变算术右移q
              -> Guard/Sticky判断round_up
 -> q的signed17范围判断
 -> 18-bit的q + round_up + Zero Point -> INT16饱和
```

### Guard/Sticky与负数半值

对1<=s<=71：

```verilog
q = product >>> shift;
guard_bit = product[shift-1'b1];
sticky_mask = (72'd1 << (shift-1'b1)) - 72'd1;
sticky_bit = |(product & sticky_mask);
round_up = guard_bit & (~product[71] | sticky_bit);
```

算术右移对负数向负无穷取整：负半值保留q，只有guard和sticky都为1时才q+1。
例如-2/4得到-1，-5/4得到-1，-6/4得到-2，仍是half-away-from-zero。

附件示意表中`p=+6,s=2`的sticky应为0而非1（二进制110的最低位是0）；结果仍为+2。
实现依据正确公式和旧参考函数，不依据该表中错误的sticky展示值。

### 为什么不能直接截断q再饱和

先用宽位检查q是否是17位数的符号扩展：

```verilog
if (q[71:17] != {55{q[16]}})
    result = q[71] ? 16'h8000 : 16'h7fff;
```

不在signed17范围时，round_up+ZP的[-128,128]校正不可能把结果救回INT16范围。
在范围内时才使用18位加法：

```verilog
result18 = $signed({q[16],q[16:0]}) +
           $signed({17'd0,round_up}) +
           $signed({{10{zero_point[7]}},zero_point});
```

之后比较32767/-32768饱和。必须先完成校正再饱和：例如q=32768、ZP=-1可以回到32767。

### Shift完整兼容

- 0：q=product，round_up=0。
- 1..71：Guard/Sticky。
- 72..79及81..255：与原实现一样先得到0，再加ZP。
- 80：保留旧80-bit容器的特殊结果：product>=0时-1，product<0时+1，再加ZP。

shift=80分支仅为旧版有限位宽兼容，不是推荐的理想数学舍入。
不改变异常参数政策，也不重新生成Golden来迁就候选版。

## 3. 真实参数审计

重新读取完整 `encoder_parameter_groups.bin`：46816 Byte / 112 Byte = 418组。
每组8个Shift位于偏移96..103，有效值集合为 `{36,37,38,39,40,41,42}`。
`tb/data/w8a16_encoder/parameter_shift.hex`额外包含未使用槽的0。

`E:/lyra/value`和`E:/111/en/value`二进制一致，SHA256为：
`97566a55951cac368e38754cf0e2a93188743042a74a592e35a4be1324b08216`。
另回读实际 `tb/data/flash_integration/flash_image.bin` 从0x28000开始的46816 Byte，
确认与该完整参数二进制逐Byte一致。
本实现仍保留完整UINT8接口，不针对这七个Shift做专用结构。

## 4. 仿真证据

### 专项测试（XSIM动态执行）

新增 `tb/tb_postprocess_guard_sticky.v`，直接调用候选函数，与两套独立参考三方比较：

1. 修改前73-bit舍入偏置公式。
2. 旧80-bit绝对值/恢复符号公式，保留其有限位宽行为。

结果：911424组逐位比较通过，其中838656组为范围/余数/ZP定向检查，
另含40000组真实INT40 x INT32随机乘积；sign/guard/sticky八种组合均命中。
所有Shift0..255、ZP全部256种取值、正负2^70、0/80、71/72/79/80/81等边界被覆盖。

函数三方测试在仿真时间0完成纯算术循环；它不证明流水时序。
真实时钟/反压另由以下动态测试验证：

- `tb_postprocess_pipeline`：1024向量/8192 lane，最大连续输出78组，三槽满2871拍，
  三级分别反压2182/2347/3011拍；Tag、参数和数据保持，II=1、三周期握手延迟及复位冲刷通过。
- `tb_postprocess_depthwise`：四层不复位连续启动、K/Cout尾、stride/dilation、ReLU、长反压，72向量通过。
- `tb_qor_p0_post_stream`：真实Dense+Accumulator+Skid+ELU+Writer反压及地址/Tag对拍通过。
- 既有投影边界/平局选择与门控单测通过。

这属于定向/随机仿真覆盖，不是对全部72位输入的形式穷举证明。

### 编译

ModelSim完整源清单编译及新RTL/专项测试Verilog-2001编译：0错误、0警告。
动态PASS来自XSIM；编译成功不当作动态PASS。

### 完整编码

50 MHz/25 MHz QSPI（临时TECH门控）与30 MHz/15 MHz QSPI（行为latch门控）
的真实PCM十帧独立编码回归均已完成，最终日志均有TB_PASS：

| 检查 | 每组结果 |
|---|---|
| Encoder逐层 | 630个LAYER_PASS |
| RVQ逐级 | 90个RVQ_STAGE_PASS，score/residual/codeword/index匹配 |
| 帧/最终结果 | 10个FRAME_PASS、10个DMA_PASS，软件重建匹配Golden |
| 单帧周期 | 539144，与修改前P1一致 |
| 50 MHz单帧服务时间 | 10.782880 ms，余量9.217120 ms |
| 30 MHz单帧服务时间 | 17.971467 ms，余量2.028533 ms |

逐帧比较修改前P1日志：全部630层engine周期、全部10帧服务周期均保持一致；
实际DMA输出记录文件（含帧ID和时间戳）与同频率P1基线逐Byte一致。
统计包含PCM输入、帧内Global/Local/阵列搬运、卷积、投影/RVQ到结果入队；
不包含一次性Flash boot及人为DMA排队等待。两组各做10帧独立编码，非跨帧历史流式验证。

## 5. 文件与运行方式

修改：

- `rtl/postprocess_8lane.v`：仅方法A函数与调用。
- `scripts/run_qor_p0_regression.ps1`：新增方法A专项测试的编译和运行入口。

新增：

- `tb/tb_postprocess_guard_sticky.v`。
- 本说明文档。

复现：

```powershell
cd E:\111\encoder
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase All -UseTechClockGate -ClockMHz 50 -Frames 10 -LogTag guard_sticky_a
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Full -ClockMHz 30 -Frames 10 -LogTag guard_sticky_a
```

日志：`logs/guard_sticky_a_50mhz/`、`logs/guard_sticky_a_30mhz/`。
编译证据：`logs/guard_sticky_compile_50mhz/`。

硬件上传只需替换 `en/lane/postprocess_8lane.v`，不新增硬件源文件。
专项测试可放 `en/test/tb_postprocess_guard_sticky.v`，不加入综合清单。
旧73-bit版和候选版同名模块不能一起加入同一硬件filelist。

候选RTL SHA256：`2479bc288d50bb14c61d5d8dba14934c8bd150116f3e8c73aeb2d88c1e40b0ec`。
基线RTL SHA256：`daa74aec7b9140196089a31d66f69f9993c4cacee8e000f2881f8f5395f0e917`。

## 6. PPA证明边界

目前没有运行服务器DC/PrimeTime/CTS/P&R，所以不声称关键路径缩短了多少ns、
面积/功耗下降了多少。Sticky动态掩码和OR归约、位选择、72-bit移位器与范围检查
仍然有成本，方法A也可能把瓶颈转移到这些逻辑。

服务器A/B须固定工艺角、库、ICG、50 MHz、SDC、RTL其余文件和活动窗口。
比较PP2路径及全设计WNS/TNS、标准单元面积、寄存器数量、hold和每帧能量。
若无收益，可以恢复备份版；不要把改变uncertainty、Shift专用化和方法A混为一次实验。
