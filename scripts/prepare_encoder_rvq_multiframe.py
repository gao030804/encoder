"""10帧独立零历史Golden；--wav为16kHz单声道PCM16，否则为单帧派生测试向量。"""
import argparse
import hashlib
import json
import subprocess
import sys
import wave
from pathlib import Path
import numpy as np
from prepare_w8a16_encoder_test_data import (
    RELU_LAYERS, RESIDUAL_PAIRS, integer_layer, residual_requant,
    requant_scalar, write_hex,
)
ROOT=Path(__file__).resolve().parents[1]


def main():
    ap=argparse.ArgumentParser()
    inputs=ap.add_mutually_exclusive_group()
    inputs.add_argument('--wav',type=Path)
    inputs.add_argument('--pcm-dir',type=Path,help='frame_00.pcm..frame_09.pcm, raw signed INT16 little endian')
    ap.add_argument('--start-sample',type=int,default=0)
    ap.add_argument('--rvq-dir',type=Path,default=Path('E:/lyra/rvq'))
    ap.add_argument('--projection-dir',type=Path,default=Path('E:/lyra/rvq'))
    args=ap.parse_args()
    assert args.start_sample>=0
    out=ROOT/'tb/data/encoder_rvq_multiframe'
    common=ROOT/'tb/data/w8a16_encoder'
    out.mkdir(parents=True,exist_ok=True)
    if args.pcm_dir:
        assert args.start_sample==0, '--start-sample applies to WAV, not pre-split PCM'
        chunks=[(args.pcm_dir/f'frame_{i:02d}.pcm').read_bytes() for i in range(10)]
        assert all(len(chunk)==640 for chunk in chunks), 'Each PCM frame must contain exactly 320 INT16 samples (640 bytes)'
        combined=b''.join(chunks)
        combined_file=args.pcm_dir/'frames_00_to_09.pcm'
        if combined_file.exists():
            assert combined_file.read_bytes()==combined, 'Combined PCM differs from frame_00..09 concatenation'
        frames=np.frombuffer(combined,dtype='<i2').reshape(10,320)
        source=dict(mode='raw_pcm_independent_frames',path=str(args.pcm_dir.resolve()),
                    sha256=hashlib.sha256(combined).hexdigest(),pcm_format='signed_int16_little_endian',
                    sample_rate_assumption=16000)
    elif args.wav:
        with wave.open(str(args.wav),'rb') as f:
            assert (f.getnchannels(),f.getsampwidth(),f.getframerate(),f.getcomptype())==(1,2,16000,'NONE'), 'Require mono PCM16 16kHz WAV'
            assert f.getnframes()>=args.start_sample+3200, 'Need 3200 samples; no silent padding'
            f.setpos(args.start_sample)
            frames=np.frombuffer(f.readframes(3200),dtype='<i2').reshape(10,320)
        source=dict(mode='wav_independent_frames',path=str(args.wav.resolve()),
                    sha256=hashlib.sha256(args.wav.read_bytes()).hexdigest(),start_sample=args.start_sample)
    else:
        original=np.array([int(v,16) for v in (common/'pcm16.hex').read_text().split()],dtype=np.int64)
        original=np.where(original>=32768,original-65536,original)
        assert len(original)==320
        frames=np.array([np.clip(np.roll(original,17*f)*(-1 if f%2 else 1)*(10-f)//10,-32768,32767)
                         for f in range(10)],dtype=np.int16)
        source=dict(mode='derived_single_frame_test_vectors',source=str(common/'pcm16.hex'),
                    sha256=hashlib.sha256(frames.astype('<i2').tobytes()).hexdigest(),
                    description='shift, inversion, attenuation; NOT original consecutive audio')
    model=json.loads((Path('E:/lyra/value')/'encoder_manifest.json').read_text())
    weights=(Path('E:/lyra/value')/'encoder_weights_int8.bin').read_bytes()
    parameters=(Path('E:/lyra/value')/'encoder_parameter_groups.bin').read_bytes()
    contract=json.loads((common/'residual_contract.json').read_text())
    residual_params={r['add_layer']:r['params'] for r in contract}
    captures={a for a,b in RESIDUAL_PAIRS}
    multiplier=int((common/'pcm_multiplier.hex').read_text(),16)
    shift=int((common/'pcm_shift.hex').read_text(),16)
    all_packed=[];all_output=[];reports=[]
    expected_sizes=[int(v,16) for v in (common/'expected_layer_elements.hex').read_text().split()]
    for fid,pcm in enumerate(frames):
        folder=out/f'frame_{fid:03d}';folder.mkdir(exist_ok=True)
        write_hex(folder/'pcm16.hex',pcm,4)
        current=np.array([max(-32768,min(32767,requant_scalar(v,multiplier,shift))) for v in pcm],dtype=np.int16).reshape(1,320)
        write_hex(folder/'pcm_activation_int16.hex',current.reshape(-1),4)
        residual=None
        with (folder/'golden_layers.mem').open('w',encoding='ascii') as stream:
            for index,layer in enumerate(model['layers']):
                layer['relu_enabled']=index in RELU_LAYERS
                if index in captures: residual=current.copy()
                current=integer_layer(current,layer,weights,parameters)
                if index in residual_params:
                    assert residual is not None and residual.shape==current.shape
                    current=residual_requant(residual,current,residual_params[index]);residual=None
                assert current.size==expected_sizes[index]
                stream.write(f'@{index*8192:x}\n')
                stream.writelines(f'{int(v)&65535:04x}\n' for v in current.T.reshape(-1))
        subprocess.run([sys.executable,str(ROOT/'scripts/prepare_encoder_rvq_frame.py'),
                        '--encoder-data',str(folder),'--out-dir',str(folder),'--golden-only',
                        '--rvq-dir',str(args.rvq_dir),'--projection-dir',str(args.projection_dir)],check=True)
        subprocess.run([sys.executable,str(ROOT/'scripts/test_encoder_rvq_golden.py'),
                        '--data-dir',str(folder)],check=True)
        all_packed.append(int((folder/'packed.hex').read_text(),16))
        all_output.extend(int(v,16) for v in (folder/'output.hex').read_text().split())
        reports.append(dict(frame=fid,pcm_sha256=hashlib.sha256(pcm.astype('<i2').tobytes()).hexdigest(),
                            packed64=f'{all_packed[-1]:016x}'))
        print(f'[FRAME_GOLDEN_READY] frame={fid} packed={all_packed[-1]:016x}',flush=True)
    assert len({r['pcm_sha256'] for r in reports})>1, 'Inputs all identical; choose varied audio'
    write_hex(out/'packed_all.hex',all_packed,16)
    write_hex(out/'output_all.hex',all_output,4)
    # testbench读取并写入ModelSim日志，避免只有TB_PASS却看不出使用了哪组音频。
    source_mode={'derived_single_frame_test_vectors':0,
                 'raw_pcm_independent_frames':1,
                 'wav_independent_frames':2}[source['mode']]
    (out/'input_mode.hex').write_text(f'{source_mode:02x}\n',encoding='ascii')
    (out/'input_sha256.hex').write_text(source['sha256']+'\n',encoding='ascii')
    # 仅供软件解包/重建自检，不是RTL产生的DMA记录；时间戳为人工测试值。
    write_hex(out/'software_reference_records.hex',
              [(fid*20000<<96)|(fid<<64)|v for fid,v in enumerate(all_packed)],32)
    (out/'manifest.json').write_text(json.dumps(dict(frame_count=10,frame_samples=320,sample_rate=16000,
        history='zero at each frame; not streaming',source=source,frames=reports),indent=2),encoding='utf-8')
    print(f'[MULTIFRAME_GOLDEN_PASS] 10 independent frames; source={source["mode"]}')


if __name__=='__main__': main()
