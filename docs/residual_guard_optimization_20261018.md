# Residual Add：48-bit Guard舍入、clip17与sum18优化

> 历史阶段记录：下述三状态/539144周期结果对应首次算术优化。
> 此后已按服务器截图增加S_WAIT2流水级，当前版本及新仿真结果见
> `residual_wait2_pipeline_sync_20261018.md`；不要将本文旧周期用于当前四状态RTL。

日期：2026-10-18。依据用户提供的
`residual_add_requant16_48bit_guard_saturation_lane_sum18_optimization.md`实施。

## 1. 实际版本与修改边界

本地 `rtl/residual_add_8lane.v` 及修改前的 `en/lane` 副本一致，当前都是三状态：
`S_IDLE/S_READ/S_WAIT`，不是附件截图中的四状态 `S_WAIT2` 版本。
本次只替换组合算术，FSM、状态编码、Scratch请求/响应、地址、参数锁存、输出反压、
busy、error、strb协议均不变。源代码对比已确认状态机和接口组合控制部分与原件完全一致。

**若服务器实际为四状态版，不要直接用本地三状态整文件覆盖；应保留服务器FSM，
只移植对应函数和组合算术。需要四状态原文件才能生成其精确补丁。**

保留历史模块名 `residual_add_8lane`，但当前64-bit写拍实际只有4个INT16元素，
原来的lane循环为4次，本次不改成8次。8个INT16的向量由上游Writer拆成两个64-bit写拍。

没有修改Postprocess Guard/Sticky、Dense/Depthwise、RVQ、SRAM、ICG、SDC或Golden。
仅修正本文件原先INT8/9-bit的过期注释，使其与现有W8A16实现一致。

## 2. 运算合同保持不变

Scale对齐开启时，定义原宽位requant为R、INT16饱和为sat16：

```text
I = sat16(R(identity,m1,s1))
M = sat16(R(main,m2,s2))
Y = sat16(I + R(M,m3,s3))
```

新实现：

```text
I = requant_sat16(identity,m1,s1)
M = requant_sat16(main,m2,s2)
X = requant_clip17(M,m3,s3)  // 限制到[-65535,65535]，不是sat16
Y = sat16(signed18(I) + signed18(X))
```

关闭Scale对齐时仍为两路INT16直接相加后饱和。

## 3. 具体优化

### 3.1 完整乘积只需48位

```verilog
reg signed [47:0] product;
reg [47:0] mag, q;
product = $signed(value) * $signed(multiplier);
```

signed INT16 x signed INT32完整乘积需48位，最大绝对值<=2^46；
不会碰到signed48最小值-2^47，所以在48位中取绝对值安全。

### 3.2 绝对值上只需Guard，不需Sticky

```verilog
mag = product[47] ? -product : product;
q = mag >> shift;
round_up = mag[shift-1'b1];
```

这是非负绝对值的half-up，随后恢复符号，即负半值远离零。
它与Postprocess对signed算术右移q使用的Guard/Sticky方法不同，不能照搬其负数判定。

- shift=0：直接对原乘积做本阶段饱和/限幅，不舍入。
- 1..47：按上式处理。
- >=48：返回0，与旧64位算法对INT16 x INT32可达乘积的结果一致。
- **47不能跳过**：`(-32768)*(-2147483648)=2^46`，除以2^47是正半值，结果应为1。

### 3.3 第一次直接输出饱和INT16

`requant_sat16`先在48位q上判断是否已确定饱和，只有未饱和路径才做16位进位：

```verilog
rounded16 = {1'b0,q[14:0]} + {15'd0,round_up};
```

正负阈值分别处理，允许INT16的负端点-32768；不在高位做无条件的`q+1`。

### 3.4 第二次只clip17，不能提前sat16

`requant_clip17`限制到对称范围[-65535,65535]。因为I在[-32768,32767]，
X超过该范围时，无论I如何取值，最终sat16都已确定在对应端点。
在限幅区间内，使用17位窄增量后恢复符号：

```verilog
rounded17 = {1'b0,q[15:0]} + {16'd0,round_up};
```

例如I=-32768、第二次量化结果65534，正确输出是32766；
若将第二次结果提前sat16，会错误变成-1。专项测试包含此类相消边界。

### 3.5 lane_sum改为signed18

```verilog
reg signed [17:0] lane_sum;
reg signed [16:0] main_requant_clip;
lane_sum = {{2{identity_aligned[15]}},identity_aligned} +
           {main_requant_clip[16],main_requant_clip};
```

范围[-98303,98302]，18位足够。最终INT16饱和比较也改成18位常数。
Scale关闭的分支同样显式符号扩展两路INT16后相加，数值语义不变。

## 4. 测试与实际结果

### 独立算术与原件逐周期比较（XSIM动态）

新增 `tb/tb_residual_guard_optimization.v`：

- 380268组算术，使用旧64位乘法/绝对值/偏置舍入作为独立Golden。
- 1901340组identity与clip17结果相加的最终饱和等价检查。
- 完整Shift0..255交叉INT16/INT32极值；所有65536种INT16值在5组代表性乘数/Shift上穷举；
  40000组随机输入；65534/65535/65536正负边界与半值。
- 同时实例化新/旧、Scale关闭/开启共4个模块，逐拍比较ready/busy/请求地址/valid/
  error及有效输出数据/地址/strb，不只比较最后结果。
- 1032笔事务：全部256种strb、SRAM延迟、输入停顿17871拍、读请求停顿1095拍、
  输出反压11258拍、旧输出消费与新输入接收重叠1031次。
- 8笔错误地址（未对齐或低于base），保留原版sticky error；等待响应中复位及最终错误清除通过。
- 独立事务计分板校验两种模式的算术、地址和strb，不仅依赖旧模块。

冻结旧原件：`tb/residual_add_8lane_legacy_reference.v`；除模块名与标识注释外，
与备份原件相同。**仅作测试参考，不加入综合清单。**

最终专项日志：`logs/residual_guard_50mhz/tb_residual_guard_final.log`。
另有此前一次专项运行日志，其覆盖统计在最终复位后显示beats=0；
这只是统计显示位置问题，已将计数保存到复位前，并重新执行最终测试得到beats=1032。

已有 `tb_residual_scale_align` 手算/参数锁存/反压测试也通过。
Postprocess Guard/Sticky、流水反压、Dense、Depthwise、RVQ投影与门控单测通过。

### 编译

ModelSim完整清单编译与修改硬件/新测试的Verilog-2001编译均0错误、0警告。
动态PASS来自XSIM，不把编译成功当作动态通过；没有宣称服务器DC/VCS/SDF已运行。

### 完整编码

50 MHz/25 MHz QSPI及30 MHz/15 MHz QSPI的真实PCM十帧独立编码均已通过；
每次回归包含630层、90级RVQ、10帧DMA及软件重建匹配Golden。

| 系统/QSPI频率 | 单帧周期 | 单帧编码时间 | 距20 ms截止时间的余量 |
| --- | ---: | ---: | ---: |
| 50/25 MHz | 539144 | 10.782880 ms | 9.217120 ms |
| 30/15 MHz | 539144 | 17.971467 ms | 2.028533 ms |

两组日志均严格核验630个LAYER_PASS、90个RVQ_STAGE_PASS、10个FRAME_PASS、
10个DMA_PASS、10个QOR_DEADLINE及最终TB_PASS，无失败/错误标记。
与优化前同频率GuardSticky回归比较：630层的周期数、10帧的周期数全部一致，
`encoded_records_multiframe.hex`逐字节相同（含时间戳），没有增加延迟。
50 MHz使用技术门控封装，30 MHz使用原行为门控模型；本轮没有改变ICG实现。

完整日志：

- `logs/residual_guard_50mhz/tb_qor_p0_full_frame.log`
- `logs/residual_guard_30mhz/tb_qor_p0_full_frame.log`

统计含帧内搬运、Encoder、投影/RVQ到结果入队，不含一次性boot和人为DMA排队。
这些是每帧历史清零的独立帧，不是跨帧历史流式测试或板上实测。

## 5. 变更文件与复现

硬件仅修改 `rtl/residual_add_8lane.v`，本地en位置为 `en/lane/residual_add_8lane.v`。
修改 `scripts/run_qor_p0_regression.ps1`：强制选本地新Residual RTL并加入新旧对拍和已有手算测试。
新增上面两个test文件及本说明；已有Golden/量化参数未改。

本地en同步位置（测试参考文件不放入综合filelist）：

| 工程文件 | en副本 |
| --- | --- |
| `rtl/residual_add_8lane.v` | `E:/111/en/lane/residual_add_8lane.v` |
| `tb/tb_residual_guard_optimization.v` | `E:/111/en/test/tb_residual_guard_optimization.v` |
| `tb/residual_add_8lane_legacy_reference.v` | `E:/111/en/test/residual_add_8lane_legacy_reference.v` |
| 本说明 | `E:/111/en/docs/residual_guard_optimization_20261018.md` |

PowerShell回归入口仍位于本地encoder/scripts，不直接拷贝到服务器执行。
上传服务器前务必核对第1节的三状态/四状态差异。

```powershell
cd E:\111\encoder
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase All -UseTechClockGate -ClockMHz 50 -Frames 10 -LogTag residual_guard
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Full -ClockMHz 30 -Frames 10 -LogTag residual_guard
```

原件备份：`build/residual_guard_baseline_20261018/`。
仅还原硬件但仍运行新增函数测试会因函数名变化不匹配，回退时应同时选择旧脚本或对应测试。

## 6. 综合/时序证明边界

本轮没有运行DC/PrimeTime/CTS/P&R，没有新的WNS/TNS、关键路径ns、面积或功耗测量。
48位绝对值、移位器、阈值比较仍有成本，不能从RTL缩位直接推算PPA收益。
服务器应固定工艺角、约束、ICG、RTL其余文件及活动窗口做A/B。
若服务器为四状态流水版，应在其FSM上仅合并算术，重新执行逐周期/全帧验证后综合。
