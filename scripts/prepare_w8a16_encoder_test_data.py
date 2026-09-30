"""把训练导出的value/golden转换成ModelSim可直接readmemh的W8A16测试数据。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import numpy as np

# revision-3的ReLU边界来自训练网络：LowRank中间无激活，仅包装器输出激活。
RELU_LAYERS = {1,2,3,4,5,6,10,12,15,17,20,22,28,30,33,35,38,40,46,48,51,53,56,58}
RESIDUAL_PAIRS = [(1,2),(3,4),(5,6),(8,12),(13,17),(18,22),
                  (26,30),(31,35),(36,40),(44,48),(49,53),(54,58)]


def fixed_ratio(ratio):
    if not math.isfinite(ratio) or ratio < 0:
        raise ValueError(f"invalid scale ratio {ratio}")
    shift = 48
    while shift and math.floor(ratio * (1 << shift) + 0.5) > 0x7fffffff:
        shift -= 1
    multiplier = math.floor(ratio * (1 << shift) + 0.5)
    if multiplier > 0x7fffffff:
        raise ValueError("scale ratio exceeds INT32 multiplier")
    return multiplier, shift


def requant_scalar(value, multiplier, shift):
    # Python int为任意精度，避免ACC40×INT32在numpy/torch INT64里溢出。
    product = int(value) * int(multiplier)
    magnitude = (abs(product) + ((1 << (shift-1)) if shift else 0)) >> shift
    return -magnitude if product < 0 else magnitude


def residual_requant(identity, branch, params):
    result = np.empty_like(branch)
    for pos in np.ndindex(branch.shape):
        qi = max(-32768, min(32767, requant_scalar(identity[pos], *params[0:2])))
        qr = max(-32768, min(32767, requant_scalar(branch[pos], *params[2:4])))
        qr = requant_scalar(qr, *params[4:6])
        result[pos] = max(-32768, min(32767, qi + qr))
    return result


def write_hex(path: Path, values, digits: int) -> None:
    path.write_text("".join(f"{int(v) & ((1 << (4*digits))-1):0{digits}x}\n" for v in values), encoding="ascii")


def integer_layer(x, layer, weight_blob, parameter_blob):
    """使用部署参数执行一层W8A16/ACC40卷积，返回[Cout,Tout] INT16。"""
    cin, cout = int(layer["cin"]), int(layer["cout"])
    kernel, stride = int(layer["kernel"]), int(layer["stride"])
    dilation, left_pad = int(layer["dilation"]), int(layer["left_pad"])
    depthwise = bool(layer["is_depthwise"])
    weight_cin = int(layer["weight_cin_per_group"])
    k_total = weight_cin * kernel
    k_groups, n_groups = (k_total + 3) // 4, (cout + 7) // 8
    packed = np.frombuffer(
        weight_blob[int(layer["weight_offset"]):int(layer["weight_offset"])+int(layer["weight_bytes"])],
        dtype=np.int8,
    ).reshape(n_groups, k_groups, 4, 8)
    weight = np.zeros((cout, weight_cin, kernel), dtype=np.int64)
    for oc in range(cout):
        for tap in range(kernel):
            for ic in range(weight_cin):
                r = tap * weight_cin + ic
                weight[oc, ic, tap] = packed[oc//8, r//4, r%4, oc%8]

    biases, multipliers, shifts = [], [], []
    base = int(layer["parameter_group_offset"])
    for group in range(n_groups):
        record = parameter_blob[base+112*group:base+112*(group+1)]
        biases.extend(int.from_bytes(record[8*i:8*i+8], "little", signed=True) for i in range(8))
        multipliers.extend(int.from_bytes(record[64+4*i:68+4*i], "little", signed=True) for i in range(8))
        shifts.extend(record[96:104])

    tout = (x.shape[1] + left_pad - dilation*(kernel-1) - 1)//stride + 1
    acc = np.zeros((cout, tout), dtype=np.int64)
    for oc in range(cout):
        source_channel_base = oc if depthwise else 0
        for m in range(tout):
            total = 0
            for tap in range(kernel):
                time_index = m*stride-left_pad+tap*dilation
                if 0 <= time_index < x.shape[1]:
                    for ic in range(weight_cin):
                        total += int(x[source_channel_base+ic, time_index]) * int(weight[oc, ic, tap])
            if total < -(1 << 39) or total > (1 << 39)-1:
                raise OverflowError(f"layer {layer['name']} INT40 accumulator overflow")
            acc[oc, m] = total

    out = np.zeros((cout, tout), dtype=np.int16)
    for oc in range(cout):
        for m in range(tout):
            biased = max(-(1 << 39), min((1 << 39)-1, int(acc[oc, m]) + int(biases[oc])))
            product = biased * int(multipliers[oc])
            shift = int(shifts[oc])
            magnitude = (abs(product) + ((1 << (shift-1)) if shift else 0)) >> shift
            requant = -magnitude if product < 0 else magnitude
            out[oc, m] = max(-32768, min(32767, requant))
    if layer.get("relu_enabled", False):
        out = np.maximum(out, 0).astype(np.int16)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--value-dir", type=Path, default=Path(r"E:\lyra\value"))
    ap.add_argument("--golden-dir", type=Path, default=Path(r"E:\lyra\golden"))
    ap.add_argument("--out-dir", type=Path, default=Path(r"E:\111\encoder\tb\data\w8a16_encoder"))
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    model = json.loads((args.value_dir / "encoder_manifest.json").read_text(encoding="utf-8"))
    golden = json.loads((args.golden_dir / "golden_manifest.json").read_text(encoding="utf-8"))
    if (model["activation_bits"], model["accumulator_bits"], model["weight_bits"]) != (16, 40, 8):
        raise SystemExit("encoder_manifest is not W8A16/INT40")
    if (golden["activation_bits"], golden["accumulator_bits"], golden["weight_bits"]) != (16, 40, 8):
        raise SystemExit("golden_manifest is not W8A16/INT40")
    if golden.get("rounding") != "half_away_from_zero":
        raise SystemExit("golden_manifest rounding is not half_away_from_zero")
    if golden["layer_count"] != 63 or len(model["layers"]) != 63:
        raise SystemExit("expected exactly 63 physical layers")
    if model.get("source_checkpoint_sha256") != golden.get("source_checkpoint_sha256"):
        raise SystemExit("value/golden checkpoint SHA256 mismatch")

    # 修复旧manifest漏标嵌套包装器的ReLU；原始训练导出包保留。
    for index, layer in enumerate(model["layers"]):
        layer["relu_enabled"] = index in RELU_LAYERS
    # 这些层号只适用于本工程revision-3；拒绝把其它网络静默套入此表。
    if (model["layers"][10]["name"] != "2.0.fn.0.net.1.net.1" or
        model["layers"][58]["name"] != "4.2.fn.2.net.1"):
        raise SystemExit("unexpected Encoder topology for revision-3 activation table")
    residual_parameters = {}
    residual_words = [0] * 64
    residual_contract = []
    for start, end in RESIDUAL_PAIRS:
        prefix = model["layers"][start]["name"].split(".fn.")[0]
        entry = next(r for r in model["residuals"] if r["name"] == prefix)
        si = float(entry.get("identity_scale", model["layers"][start]["input_scale"]))
        sr = float(entry.get("branch_scale", model["layers"][end]["output_scale"]))
        # 旧包未导出Add输出Scale；当前相邻消费者拥有同一条量化边。
        so = float(entry.get("output_scale", model["layers"][end+1]["input_scale"]))
        params = (*fixed_ratio(si/so), *fixed_ratio(sr/so), *fixed_ratio(float(entry["residual_scale"])))
        residual_parameters[end] = params
        residual_contract.append({"capture_layer": start, "add_layer": end,
            "identity_scale": si, "branch_scale": sr, "output_scale": so,
            "output_scale_source": "residual_manifest" if "output_scale" in entry else "next_conv_input_scale_legacy_inference",
            "params": params})
        residual_words[end] = sum((int(params[2*j]) | (int(params[2*j+1]) << 32)) << (40*j) for j in range(3))
    write_hex(args.out_dir / "residual_parameters.hex", residual_words, 30)
    (args.out_dir / "residual_contract.json").write_text(
        json.dumps(residual_contract, indent=2), encoding="utf-8")

    # 权重文件已经按output_group/k_group/4x8排列；一行一个Byte供快速Weight SRAM模型读取。
    weights = (args.value_dir / "encoder_weights_int8.bin").read_bytes()
    write_hex(args.out_dir / "weights.mem", weights, 2)

    # 每个参数组：8xint64 Bias，8xint32 Mult，8xuint8 Shift，8xint8 ZP。
    raw = (args.value_dir / "encoder_parameter_groups.bin").read_bytes()
    bias = [0] * 2048
    mult = [0] * 2048
    shift = [0] * 2048
    zp = [0] * 2048
    for layer_index, layer in enumerate(model["layers"]):
        groups = (int(layer["cout"]) + 7) // 8
        base = int(layer["parameter_group_offset"])
        for group in range(groups):
            record = raw[base + 112*group:base + 112*(group+1)]
            if len(record) != 112:
                raise SystemExit(f"short parameter record layer={layer_index} group={group}")
            address = layer_index * 32 + group
            bias[address] = int.from_bytes(record[0:64], "little", signed=False)
            mult[address] = int.from_bytes(record[64:96], "little", signed=False)
            shift[address] = int.from_bytes(record[96:104], "little", signed=False)
            zp[address] = int.from_bytes(record[104:112], "little", signed=False)
    write_hex(args.out_dir / "parameter_bias.hex", bias, 128)
    write_hex(args.out_dir / "parameter_multiplier.hex", mult, 64)
    write_hex(args.out_dir / "parameter_shift.hex", shift, 16)
    write_hex(args.out_dir / "parameter_zero_point.hex", zp, 16)

    # 63层采用固定8192个INT16槽，@地址使RTL可用layer*8192+element直接索引。
    integer_golden_dir = args.golden_dir / "integer_rtl"
    integer_golden_dir.mkdir(parents=True, exist_ok=True)
    layer0_in = (args.golden_dir / "layer_00_input_bct_int16.bin").read_bytes()
    current = np.frombuffer(layer0_in, dtype="<i2").astype(np.int16).reshape(
        int(model["layers"][0]["cin"]), -1
    )
    residual = None
    capture_layers = {1, 3, 5, 8, 13, 18, 26, 31, 36, 44, 49, 54}
    add_layers = {2, 4, 6, 12, 17, 22, 30, 35, 40, 48, 53, 58}
    integer_outputs = []
    integer_manifest_layers = []
    for index, layer in enumerate(model["layers"]):
        if index in capture_layers:
            residual = current.copy()
        output = integer_layer(current, layer, weights, raw)
        if index in add_layers:
            if residual is None or residual.shape != output.shape:
                raise SystemExit(f"residual shape mismatch at layer {index}")
            output = residual_requant(residual, output, residual_parameters[index])
            residual = None
        time_major = output.T.reshape(-1)
        bin_name = f"layer_{index:02d}_output_time_major_int16.bin"
        hex_name = f"layer_{index:02d}_output_time_major_int16.hex"
        (integer_golden_dir / bin_name).write_bytes(time_major.astype("<i2").tobytes())
        write_hex(integer_golden_dir / hex_name, time_major, 4)
        integer_outputs.append(time_major)
        integer_manifest_layers.append({
            "index": index, "name": layer["name"],
            "output_shape_bct": [1, int(output.shape[0]), int(output.shape[1])],
            "output_time_major_file": bin_name,
            "output_time_major_hex": hex_name,
        })
        current = output

    (integer_golden_dir / "golden_manifest.json").write_text(json.dumps({
        "format_version": 2,
        "description": "Bit-exact W8A16/ACC40 recursive integer RTL golden",
        "generation_mode": "recursive_integer_rtl_contract",
        "weight_bits": 8, "activation_bits": 16, "accumulator_bits": 40,
        "rounding": "half_away_from_zero", "layer_count": 63,
        "layers": integer_manifest_layers,
        "relu_layers": sorted(RELU_LAYERS),
        "residual_parameters": residual_parameters,
        "residual_contract": residual_contract,
        "residual_order": "sat16(align(identity)) + gain(sat16(align(branch))), then sat16",
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    with (args.out_dir / "golden_layers.mem").open("w", encoding="ascii") as out:
        for index, values in enumerate(integer_outputs):
            out.write(f"@{index*8192:x}\n")
            out.writelines(f"{int(v)&0xffff:04x}\n" for v in values)

    pcm = (args.golden_dir / "input_pcm_s16le.hex").read_text(encoding="ascii").split()
    write_hex(args.out_dir / "pcm16.hex", (int(v, 16) for v in pcm), 4)
    layer0_values = [int.from_bytes(layer0_in[i:i+2], "little", signed=False) for i in range(0, len(layer0_in), 2)]
    write_hex(args.out_dir / "pcm_activation_int16.hex", layer0_values, 4)

    # q = round_half_away(pcm * multiplier / 2^shift)，选择不溢出signed INT32的最大shift。
    input_scale = float(model["layers"][0]["input_scale"])
    ratio = 1.0 / (32768.0 * input_scale)
    pcm_shift = 30
    while round(ratio * (1 << pcm_shift)) > 0x7fffffff:
        pcm_shift -= 1
    pcm_multiplier = int(math.floor(ratio * (1 << pcm_shift) + 0.5))
    write_hex(args.out_dir / "pcm_multiplier.hex", [pcm_multiplier], 8)
    write_hex(args.out_dir / "pcm_shift.hex", [pcm_shift], 2)

    expected_elements = []
    for layer in golden["layers"]:
        shape = layer["output_shape_bct"]
        expected_elements.append(int(shape[1]) * int(shape[2]))
    write_hex(args.out_dir / "expected_layer_elements.hex", expected_elements, 8)
    print(f"Prepared W8A16 data: weights={len(weights)} bytes, layers=63, out={args.out_dir}")


if __name__ == "__main__":
    main()
