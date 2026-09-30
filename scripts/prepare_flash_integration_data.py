"""将真实W8A16部署数据打包为Flash镜像；不运行或修改整数Golden。"""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--value-dir', type=Path, default=Path('E:/lyra/value'))
    args = parser.parse_args()
    out = ROOT / 'tb/data/flash_integration'
    out.mkdir(parents=True, exist_ok=True)
    # 64KiB前缀留给其他镜像/启动信息；片内偏移与现有权重AGU兼容。
    flash_base, image_size = 0x10000, 0x35000
    regions = [
        ('weights', 'encoder_weights_int8.bin', 0x00000, 0x28000),
        ('parameters', 'encoder_parameter_groups.bin', 0x28000, 0x34000),
        ('residual', 'encoder_residual_params.bin', 0x34000, 0x35000),
    ]
    image = bytearray([0xff]) * image_size
    report = {'flash_base': flash_base, 'image_bytes': image_size,
              'global_capacity_bytes': 262144, 'regions': [], 'layers': []}
    blobs = {}
    for name, filename, start, end in regions:
        blob = (args.value_dir / filename).read_bytes()
        if not blob or len(blob) > end-start:
            raise ValueError(f'{filename}: empty or exceeds assigned region')
        image[start:start+len(blob)] = blob
        blobs[name] = blob
        report['regions'].append(dict(name=name, source=filename,
            global_offset=start, flash_address=flash_base+start,
            bytes=len(blob), reserved_bytes=end-start,
            sha256=hashlib.sha256(blob).hexdigest()))
    manifest = json.loads((args.value_dir / 'encoder_manifest.json').read_text(encoding='utf-8'))
    for layer in manifest['layers']:
        wo, wn = int(layer['weight_offset']), int(layer['weight_bytes'])
        po, pn = int(layer['parameter_group_offset']), int(layer['parameter_group_bytes'])
        if wo % 32 or wo+wn > len(blobs['weights']) or po+pn > len(blobs['parameters']):
            raise ValueError(f"invalid offsets for layer {layer['name']}")
        report['layers'].append(dict(name=layer['name'], weight_bytes=wn,
            weight_global=wo, weight_flash=flash_base+wo,
            parameter_global=0x28000+po, parameter_flash=flash_base+0x28000+po,
            parameter_bytes=pn))
    # Flash模型以Byte初始化；SRAM和scoreboard都按低地址Byte放低位。
    (out/'flash_image.hex').write_text('@010000\n'+''.join(f'{b:02x}\n' for b in image), encoding='ascii')
    (out/'flash_image.bin').write_bytes(image)
    (out/'expected_blocks.hex').write_text(''.join(
        f'{int.from_bytes(image[i:i+32], "little"):064x}\n'
        for i in range(0, image_size, 32)), encoding='ascii')
    (out/'address_map.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    # 独立回读打包结果，防止地址或字节序错误被测试数据生成所掩盖。
    decoded = bytes.fromhex(''.join((out/'flash_image.hex').read_text().splitlines()[1:]))
    assert decoded == image
    for name, filename, start, end in regions:
        assert decoded[start:start+len(blobs[name])] == (args.value_dir/filename).read_bytes()
    print(f'[IMAGE_PASS] {len(manifest["layers"])} layers, {image_size} bytes, {image_size//32} blocks')
    print(f'Flash 0x{flash_base:06x} -> Global 0x000000; map: {out / "address_map.json"}')


if __name__ == '__main__':
    main()
