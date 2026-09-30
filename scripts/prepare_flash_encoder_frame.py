"""在Flash存储回归镜像上增加完整帧所需固定点控制参数。"""
import json
from pathlib import Path
from prepare_flash_integration_data import main as make_base_image

ROOT = Path(__file__).resolve().parents[1]

if __name__ == '__main__':
    make_base_image()
    base = ROOT / 'tb/data/flash_integration'
    golden = ROOT / 'tb/data/w8a16_encoder'
    out = ROOT / 'tb/data/flash_encoder_frame'
    out.mkdir(parents=True, exist_ok=True)
    image = bytearray((base/'flash_image.bin').read_bytes())
    image.extend(b'\xff' * (0x36000-len(image)))
    # 每层16Byte：120-bit固定点Residual参数+1Byte填充。
    # 原始Residual导出记录保留在0x34000，本区为RTL实际消费格式。
    residual = [int(v, 16) for v in (golden/'residual_parameters.hex').read_text().split()]
    assert len(residual) == 64
    for layer, word in enumerate(residual):
        image[0x35000+16*layer:0x35000+16*(layer+1)] = word.to_bytes(16, 'little')
    multiplier = int((golden/'pcm_multiplier.hex').read_text().strip(), 16)
    shift = int((golden/'pcm_shift.hex').read_text().strip(), 16)
    image[0x35400:0x35405] = multiplier.to_bytes(4, 'little') + bytes([shift])
    address_map = json.loads((base/'address_map.json').read_text())
    offsets = [entry['parameter_global'] for entry in address_map['layers']]
    (out/'layer_parameter_offsets.hex').write_text(''.join(f'{v:08x}\n' for v in offsets), encoding='ascii')
    (out/'flash_image.hex').write_text('@010000\n'+''.join(f'{v:02x}\n' for v in image), encoding='ascii')
    (out/'expected_blocks.hex').write_text(''.join(
        f'{int.from_bytes(image[i:i+32], "little"):064x}\n' for i in range(0,len(image),32)), encoding='ascii')
    (out/'flash_image.bin').write_bytes(image)
    address_map.update(image_bytes=len(image), residual_fixed_global=0x35000, pcm_quant_global=0x35400)
    (out/'address_map.json').write_text(json.dumps(address_map, indent=2), encoding='utf-8')
    print(f'[FRAME_IMAGE] bytes={len(image)} blocks={len(image)//32}')
