#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
import argparse,hashlib,json,subprocess
from pathlib import Path
r=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--ndk',required=True,type=Path);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--stock-cpp',type=Path);g.add_argument('--vendor-cache',type=Path);p.add_argument('--device',type=Path,default=r.parent/'xr-device-io');p.add_argument('--device-include',type=Path);p.add_argument('--hexagonrpc-source',type=Path,default=Path('/usr/share/xr-hexagonrpc/source'));p.add_argument('--output',required=True,type=Path);a=p.parse_args();device_include=a.device_include or (Path('/usr/include') if Path('/usr/include/xr/device').exists() else a.device/'include')
if a.vendor_cache:a.stock_cpp=a.vendor_cache/'controller/root/system/lib/libc++_shared.so'
if a.output.exists():p.error('Choose a new output directory')
tools=a.ndk/'toolchains/llvm/prebuilt/linux-x86_64/bin';cc=tools/'armv7a-linux-androideabi28-clang++'
if not cc.is_file() or not a.stock_cpp.is_file():p.error('Need the NDK compiler and stock ARM32 libc++_shared.so')
a.output.mkdir(parents=True);commands=[]
for src,name in [('controller_sdk_pose.cpp','controller-sdk-pose'),('controller_sdk_imu.c','controller-sdk-imu'),('controller_sdk_probe.c','controller-sdk-probe')]:
 compiler=cc if src.endswith('.cpp') else tools/'armv7a-linux-androideabi28-clang'
 prefixes=[(Path.cwd(),'/build'),(r.parent,'/usr/src'),(a.ndk,'/opt/android-ndk'),(r,'/usr/src/xr-controller-tracking'),(a.device,'/usr/src/xr-device-io')]
 args=[str(compiler),'-fdebug-compilation-dir','/build',*[flag+str(src)+'='+dest for src,dest in prefixes for flag in ['-fdebug-prefix-map=']],'-O2','-g0','-Wall','-Wextra','-fPIE','-pie','-I'+str(r/'include'),'-I'+str(device_include),str(r/'src'/src)]
 if src.endswith('.cpp'):args+=['-std=c++17','-nostdlib++',str(a.stock_cpp)]
 args+=['-ldl','-o',str(a.output/name)];subprocess.run(args,check=True);commands.append(args)
rpc=a.hexagonrpc_source
client=sorted((rpc/'libhexagonrpc').rglob('*.c'))
daemon=[rpc/'hexagonrpcd'/name for name in ['apps_mem.c','apps_std.c','interface/adsp_default_listener.c','interface/adsp_listener.c','interface/apps_mem.c','interface/apps_std.c','hexagonfs.c','hexagonfs_mapped.c','hexagonfs_plat_subtype_name.c','hexagonfs_virt_dir.c','iobuffer.c','listener.c','localctl.c','rpcd_builder.c']]
if not (rpc/'include/libhexagonrpc/error.h').is_file():p.error('Install xr-hexagonrpc-source or provide --hexagonrpc-source')
for src,name in [('cdsprpc-mainline.c','libcdsprpc.so'),('snpe-runtime-override.c','liba081-snpe-runtime.so')]:
 args=[str(tools/'armv7a-linux-androideabi28-clang'),*[flag+str(src)+'='+dest for src,dest in prefixes+[(rpc,'/usr/src/xr-hexagonrpc')] for flag in ['-fdebug-prefix-map=']],'-O2','-fPIC','-shared','-Wl,-z,defs','-I'+str(r/'include'),'-I'+str(rpc/'include'),'-I'+str(rpc/'hexagonrpcd'),str(r/'src'/src)]
 if src=='cdsprpc-mainline.c':args += [str(q) for q in client+daemon]
 args+=['-ldl','-o',str(a.output/name)]
 subprocess.run(args,check=True);commands.append(args)
files={q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in a.output.iterdir() if q.is_file()}
(a.output/'build-manifest.json').write_text(json.dumps({'commands':commands,'files_sha256':files,'stock_cpp_sha256':hashlib.sha256(a.stock_cpp.read_bytes()).hexdigest(),'vendor_cache_receipt_sha256':hashlib.sha256((a.vendor_cache/'receipt.json').read_bytes()).hexdigest() if a.vendor_cache else None,'device_tested':False},indent=2)+'\n')
