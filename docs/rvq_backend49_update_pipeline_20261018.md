# RVQ Backend：49-bit残差量化、UPDATE两拍与投影舍入优化

日期：2026-10-18。依据 `E:/111/Encoder_RVQ_Backend_关键路径优化总结.md`。

## 1. 实施范围

硬件只修改 `E:/111/encoder/RVQ/rvq_backend_w8a16.v`，对应本地en位置：
`E:/111/en/rvq_rtl/rvq_backend_w8a16.v`。
用户本次明确要求三项都实施，因此未停在文档建议的“先缩位、重综合后再决定流水”阶段。
没有修改PCM量化、Residual Add四状态实现、Mesh/ICG、Flash地址、SRAM规模、
参数文件、Python Golden、候选搜索顺序及平分取最小索引规则。

## 2. rq64残差路径：完整49-bit乘积，直接输出INT16

原公式：

```text
sub_value(INT17) x next_m(INT32)
    -> rq64宽位结果 -> sat16 -> residual/trace_next
```

新公式：

```text
INT17 x INT32 -> signed49 product_q
    -> abs/右移/Guard/饱和感知窄增量 -> signed INT16
```

17+32=49，不能按Residual Add的16x32误写成48位。
例如(-65536)x(-2147483648)=+2^47，超出signed48正数范围。
新函数 `round_sat16_49(product,shift)` 不生成完整64位舍入值。
绝对值上half-up只需Guard，不需要Sticky；恢复符号后保持half-away-from-zero。

```verilog
mag = p[48] ? -p : p;
q = mag >> s;
guard_bit = mag[s-1'b1]; // 仅在非零且索引合法的分支执行
// 先在q上判断饱和，未饱和才做16-bit加一
rounded16 = {1'b0,q[14:0]} + {15'd0,guard_bit};
```

正负饱和阈值分别处理，保留INT16的-32768端点。
shift=0不舍入；合法参数0..63中的大移位正确归零。
极值乘积2^47在shift=48时恰为半值，仍应得到+1，不能从48开始直接清零。

兼容性说明：原RPWAIT对shift>63置error，本次保留。
原rq64掩码写法在非法shift>=65时返回+1或-1（零乘积为+1）；
新实现显式保留该异常数值行为，不把未授权的异常合同修复混入优化。
完整UINT8 shift域都与旧公式对拍；这不意味着非法参数可以用于部署。

## 3. UPDATE拆为乘法和舍入两拍

保留现有状态0..24的编号；原19改名UPDATE_MUL，新增UPDATE_RND=25。

| 状态 | 操作 | dimension/trace |
| --- | --- | --- |
| UPDATE_MUL=19 | INT17减法、17x32乘法，锁存49位乘积与shift | 锁存stage/dimension/旧残差/码字/差值，dimension不变 |
| UPDATE_RND=25 | 舍入、INT16饱和、写回及可选output_sum累加 | 发update_valid及对齐的trace，再递增维度或进STAGE_END |

```verilog
UPDATE_MUL: begin
    update_product_q <= $signed(sub_value)*$signed(next_m);
    update_dimension_q <= dimension;
    // 其余参数/trace字段也同步锁存
    state <= UPDATE_RND;
end
UPDATE_RND: begin
    trace_dimension <= update_dimension_q;
    trace_next <= next_value;
    update_valid <= 1;
    if(update_stage_q!=8) residual[update_dimension_q] <= next_value;
    if(update_dimension_q==31) state <= STAGE_END;
    else begin dimension <= update_dimension_q+1'b1;state <= UPDATE_MUL;end
end
```

stage8仍只产生trace_next，不覆盖residual；保留原最后一级约定。
每帧9级x32维，增加288周期；不会改变1280个候选的搜索与评分次数。

## 4. round80投影路径：72-bit乘积，Guard+直接INT16饱和

INT40 biased_q x INT32 multiplier只需signed72完整乘积。
将product_q从80缩为72，原`sat16(round80(product_q,shift))`替换成
`round_sat16_72(product_q,shift)`，消除80位舍入偏置加法。
正负边界处理与49位版本相同；投影仍沿用PBIAS/PMUL/PQUANT三拍，不增加投影周期。
INT40加Bias饱和及其顺序不变。

## 5. COMPUTE_VECTOR=1不能提前sat16

旧rq64还用于可选重建向量，不可将所有调用统一替换为INT16，否则跨级相消会出错。
此路径改为UPDATE_MUL寄存完整INT8xINT32的40位乘积，UPDATE_RND按Guard舍入成
足够宽的signed49结果，然后与INT32 output_sum相加。

- 每个stage/dimension仅在UPDATE_RND累加一次。
- 超过INT32范围仍置error并保存低32位，与旧实现一致。
- 最后OUTPUT才统一sat16，禁止每级贡献提前sat16。
- 完整系统COMPUTE_VECTOR=0不变；向量仍由软件从码流重建。

## 6. 仿真

新增 `tb/tb_rvq_backend_arithmetic.v`：独立复制旧64/80位公式对拍。

- 残差49位量化721488组：全17-bit输入在5组代表参数上穷举、极值交叉、50000随机组。
- 投影72位舍入67448组：40x32乘积极值、正负半值及32767/32768/32769阈值。
- 重建向量66128组：确认宽位值不提前饱和。
- UINT8 shift 0..255均有覆盖，包括63/64/65等异常边界的旧行为。

扩展 `tb/tb_qor_p0_projection.v`，新增两个顶层包装：

| 测试顶层 | 场景 |
| --- | --- |
| tb_qor_p0_projection | 原投影32个边界向量，零码本/平分选最小索引 |
| tb_rvq_backend_update | 非零码本/正负Scale，独立遍历评分与残差Golden；两流水级中途复位 |
| tb_rvq_backend_vector_overflow | 大码字缩放触发INT32累加溢出；校验error、回绕及最终饱和 |

三个场景每个都检查32个投影、1280个score、288次trace更新、9个最佳索引、
最终向量与码流，以及结果反压保持；每场景UPDATE_MUL/RND各288次。
不force DUT内部状态，使用真实SRAM接口BOOT装载合成测试镜像。
这些专项已动态PASS，ModelSim完整清单/Verilog-2001编译0错误、0警告。

### 完整真实PCM十帧回归：50MHz及30MHz均动态PASS

每组完成630层Encoder、320个投影通道、12800个候选score、2880次残差更新、
90级RVQ、10帧DMA；日志核验630个LAYER_PASS、90个RVQ_STAGE_PASS、10个LATENT_PASS、
10个FRAME_PASS、10个DMA_PASS、10个QOR_DEADLINE及最终TB_PASS，无失败/错误标记。
UPDATE_MUL与UPDATE_RND每组各2880拍，并由 `[RVQ_UPDATE_PIPE_PASS]` 明确检查。
实际DMA记录经软件重建10帧，匹配Golden。

| 系统/QSPI时钟 | 优化前（四状态Residual版） | 优化后 | 距20ms余量 |
| --- | ---: | ---: | ---: |
| 50/25MHz | 549434周期 / 10.988680ms | 549722周期 / 10.994440ms | 9.005560ms |
| 30/15MHz | 549434周期 / 18.314467ms | 549722周期 / 18.324067ms | 1.675933ms |

每帧恰增加288周期：50MHz增加5.760us，30MHz增加9.600us。
新旧630层Encoder引擎周期逐项一致；十帧frame_id与packed64逐条相同。
timestamp随完成时间变化，不宣称整个128-bit记录逐字节一致。

统计从PCM输入启动到编码结果入环形缓冲，含帧内搬运、卷积、投影及RVQ，
不含一次性Flash启动或测试人为DMA排队。Flash启动约21.628700ms（50MHz）/
36.047833ms（30MHz），没有改变。
输入为真实PCM连续音频截出的10帧，但各帧历史清零独立编码，不是跨帧state流式测试。
50MHz采用技术门控封装，30MHz采用原行为门控；没有重新测试FPGA板上UART通信。

最终完整日志：

- `E:/111/encoder/logs/rvq_backend49_50mhz/tb_qor_p0_full_frame.log`
- `E:/111/encoder/logs/rvq_backend49_30mhz/tb_qor_p0_full_frame.log`
- 专项日志在 `E:/111/encoder/logs/rvq_backend49_50mhz/`。

同批既有Residual、Postprocess Guard/Sticky/流水、Dense/Depthwise和ICG单测均通过。

## 7. 修改/新增文件及en同步位置

| 操作 | encoder工程文件 | en对应位置 |
| --- | --- | --- |
| 修改硬件 | RVQ/rvq_backend_w8a16.v | E:/111/en/rvq_rtl/rvq_backend_w8a16.v |
| 修改测试 | tb/tb_qor_p0_projection.v | E:/111/en/test/tb_qor_p0_projection.v |
| 修改测试 | tb/tb_qor_p0_full_frame.v | E:/111/en/test/tb_qor_p0_full_frame.v |
| 新增测试 | tb/tb_rvq_backend_arithmetic.v | E:/111/en/test/tb_rvq_backend_arithmetic.v |
| 新增说明 | docs/rvq_backend49_update_pipeline_20261018.md | E:/111/en/docs/rvq_backend49_update_pipeline_20261018.md |
| 修改入口 | scripts/run_qor_p0_regression.ps1 | 本地入口，不直接拷贝到服务器运行 |

测试文件不加入综合filelist；服务器重新综合只需替换硬件文件。
修改前4个文件备份在 `build/rvq_backend_baseline_20261018/`。

复现命令：

```powershell
cd E:\111\encoder
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase All -UseTechClockGate -ClockMHz 50 -Frames 10 -LogTag rvq_backend49
powershell -ExecutionPolicy Bypass -File .\scripts\run_qor_p0_regression.ps1 `
  -Phase Full -ClockMHz 30 -Frames 10 -LogTag rvq_backend49
```

动态测试使用XSIM；ModelSim此轮只报告编译，不冒充ModelSim动态PASS。
本轮未运行DC/PrimeTime/VCS/SDF/布局布线，不推算新的slack、面积、功耗。
服务器请固定PVT/SDC/ICG及其余RTL做A/B综合，查看关键路径是否迁移。
