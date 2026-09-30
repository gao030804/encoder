"""将真实RVQ包和投影打包到Flash尾部，并独立生成逐候选/逐级整数Golden。

正常模式必须提供真实projection导出。--smoke仅用于RTL连通性，明确使用合成投影。
不修改Encoder Golden；以已验证的第62层整数输出递归计算后端Golden。
"""
import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def rq(value, m, s):
    assert 0 <= m <= 0x7fffffff and 0 <= s <= 63
    p = int(value) * int(m)
    a = (abs(p) + ((1 << (s-1)) if s else 0)) >> s
    return -a if p < 0 else a


def clip16(v):
    return max(-32768, min(32767, v))


def write_hex(path, values, bits):
    path.write_text(''.join(f'{int(v)&((1<<bits)-1):0{(bits+3)//4}x}\n' for v in values), encoding='ascii')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rvq-dir', type=Path, default=Path('E:/lyra/rvq'))
    ap.add_argument('--projection-dir', type=Path, default=Path('E:/lyra/rvq'))
    ap.add_argument('--smoke', action='store_true')
    ap.add_argument('--encoder-data', type=Path, default=ROOT/'tb/data/w8a16_encoder')
    ap.add_argument('--out-dir', type=Path, default=ROOT/'tb/data/flash_encoder_rvq')
    ap.add_argument('--golden-only', action='store_true')
    args = ap.parse_args()
    out = args.out_dir
    manifest = json.loads((args.rvq_dir/'rvq_manifest.json').read_text(encoding='utf-8'))
    assert manifest['format_version'] == 1, 'Only V1 equal residual/codebook scales supported'
    assert manifest['num_quantizers'] == 9 and manifest['rounding'] == 'half_away_from_zero'
    assert manifest['zero_point'] == 0 and manifest['tie_break'] == 'lowest_index'
    for q, st in enumerate(manifest['stages']):
        assert st['codebook_dim'] == 32 and st['codebook_size'] == (256 if q == 0 else 128)
        assert st.get('residual_scale', st['scale']) == st['scale']
    for name, digest in manifest['sha256'].items():
        assert hashlib.sha256((args.rvq_dir/name).read_bytes()).hexdigest() == digest, name
    encoder = json.loads((Path('E:/lyra/value')/'encoder_manifest.json').read_text())
    if args.smoke:
        weights = [[(o*3+i*5)%7-3 for i in range(64)] for o in range(32)]
        bias = [0]*32
        mult = [1 << 20]*32
        shift = [20+o%4 for o in range(32)]
    else:
        pdir = args.projection_dir
        required = ['projection_manifest.json', 'projection_weights_4x8_int8.bin',
                    'projection_bias_int40_in_int64.bin','projection_multiplier_int32.bin','projection_shift_uint8.bin']
        missing = [n for n in required if not (pdir/n).exists()]
        if missing:
            raise SystemExit('Missing REAL 64->32 projection export: '+', '.join(missing)+
                             '. Supply --projection-dir; --smoke is synthetic and not deployment validation.')
        pm = json.loads((pdir/required[0]).read_text())
        assert (pm['cin'],pm['cout'],pm['weight_bits'],pm['activation_bits'],pm['accumulator_bits']) == (64,32,8,16,40)
        assert pm['rounding']=='half_away_from_zero' and pm['zero_point']==0
        assert math.isclose(pm['input_scale'],encoder['layers'][-1]['output_scale'],rel_tol=1e-6)
        assert math.isclose(pm['output_scale'],manifest['input_scale'],rel_tol=1e-6)
        packed = (pdir/required[1]).read_bytes()
        assert len(packed)==2048
        def signed8(v): return v-256 if v>=128 else v
        weights = [[signed8(packed[((o//8)*16+i//4)*32+(i%4)*8+o%8]) for i in range(64)] for o in range(32)]
        bias = list(struct.unpack('<32q',(pdir/required[2]).read_bytes()))
        mult = list(struct.unpack('<32i',(pdir/required[3]).read_bytes()))
        shift = list((pdir/required[4]).read_bytes()); assert len(shift)==32
    assert all(-2**39 <= b < 2**39 for b in bias)
    assert all(0 <= m < 2**31 for m in mult) and all(0 <= s <= 63 for s in shift)
    image = bytearray(0xc000)
    image[:0x800] = bytes(v&255 for row in weights for v in row)
    for o in range(32): image[0x800+o*16:0x810+o*16] = struct.pack('<qiB3x',bias[o],mult[o],shift[o])
    books = (args.rvq_dir/'rvq_codebook_int8.bin').read_bytes()
    norms_blob = (args.rvq_dir/'rvq_codebook_norm_int32.bin').read_bytes()
    params = (args.rvq_dir/'rvq_stage_requant_params.bin').read_bytes()
    assert (len(books),len(norms_blob),len(params)) == (40960,5120,90)
    image[0xa00:0xaa00]=books; image[0xaa00:0xbe00]=norms_blob
    for q in range(9): image[0xbe00+q*32:0xbe00+q*32+10]=params[q*10:q*10+10]
    # Encoder Golden的@地址以INT16元素计数；第62层是1x64向量。
    mem={}; address=0
    for token in (args.encoder_data/'golden_layers.mem').read_text().split():
        if token.startswith('@'): address=int(token[1:],16)
        else:
            v=int(token,16);mem[address]=v-65536 if v>=32768 else v;address+=1
    latent=[mem[62*8192+j] for j in range(64)]
    acc=[sum(a*b for a,b in zip(latent,w))+bias[o] for o,w in enumerate(weights)]
    assert all(-2**39 <= a < 2**39 for a in acc)
    query=[clip16(rq(a,m,s)) for a,m,s in zip(acc,mult,shift)]
    residual=query[:]; sums=[0]*32
    scores=[0]*(9*256); inputs=[]; selected=[]; subtract=[]; following=[]; indices=[]
    entry=0
    for q in range(9):
        st=manifest['stages'][q]; k=st['codebook_size']; inputs.extend(residual)
        nm,ns,om,os=struct.unpack_from('<iBiB',params,q*10)
        assert (nm,ns,om,os)==(st['next_multiplier'],st['next_shift'],st['output_multiplier'],st['output_shift'])
        rows=[list(struct.unpack_from('<32b',books,(entry+c)*32)) for c in range(k)]
        # 独立平方差距离找index，与省略R平方项的score交叉核对。
        distances=[sum((r-e)**2 for r,e in zip(residual,row)) for row in rows]
        index=min(range(k), key=lambda c:distances[c]); indices.append(index)
        for c,row in enumerate(rows):
            norm=sum(e*e for e in row)
            assert norm==struct.unpack_from('<i',norms_blob,(entry+c)*4)[0]
            value=norm-2*sum(r*e for r,e in zip(residual,row))
            assert value==distances[c]-sum(r*r for r in residual) and -2**31<=value<2**31
            scores[q*256+c]=value
        chosen=rows[index]; selected.extend(chosen)
        diff=[r-e for r,e in zip(residual,chosen)]; subtract.extend(diff)
        assert all(-65536 <= v <= 65535 for v in diff)
        nxt=[clip16(rq(v,nm,ns)) for v in diff];following.extend(nxt)
        sums=[v+rq(e,om,os) for v,e in zip(sums,chosen)]
        assert all(-2**31<=v<2**31 for v in sums)
        if q<8: residual=nxt
        entry+=k
    packed_indices=indices[0]+sum(indices[q]<<(8+7*(q-1)) for q in range(1,9))
    out.mkdir(parents=True,exist_ok=True)
    for name,values,bits in [('latent',latent,16),('projection',query,16),('scores',scores,32),
            ('stage_input',inputs,16),('selected',selected,8),('subtract',subtract,17),
            ('stage_next',following,16),('indices',indices,8),('output',list(map(clip16,sums)),16),
            ('packed',[packed_indices],64),('test_mode',[int(args.smoke)],8)]:
        write_hex(out/(name+'.hex'),values,bits)
    write_hex(out/'backend_blocks.hex',[int.from_bytes(image[i:i+32],'little') for i in range(0,len(image),32)],256)
    full=(ROOT/'tb/data/flash_encoder_frame/flash_image.bin').read_bytes()+image
    assert len(full)==270336
    if not args.golden_only:
        (out/'flash_image.bin').write_bytes(full)
        (out/'flash_image.hex').write_text('@010000\n'+''.join(f'{v:02x}\n' for v in full),encoding='ascii')
    report=dict(synthetic_projection=args.smoke,rvq_diagnostic_only=manifest.get('diagnostic_only',False),
                projection_dir=str(args.projection_dir),rvq_dir=str(args.rvq_dir),
                flash_base=0x10000,backend_flash_base=0x46000,backend_bytes=len(image),
                flash_bytes=len(full),indices=indices,packed_indices=f'{packed_indices:016x}',
                source_hashes=manifest['sha256'])
    (out/'manifest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('[GOLDEN_READY] synthetic_projection='+str(args.smoke)+f' indices={indices} packed={packed_indices:016x}')
    if manifest.get('diagnostic_only'): print('[NOTICE] RVQ candidate is diagnostic_only; RTL match does not approve audio quality.')


if __name__=='__main__': main()
