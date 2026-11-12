#!/usr/bin/env python3
import os,subprocess,sys
from pathlib import Path
r=Path(__file__).resolve().parents[1]
ndk=os.environ.get('ANDROID_NDK_HOME');cache=os.environ.get('XR_VENDOR_CACHE')
if not ndk or not cache:sys.exit('Set ANDROID_NDK_HOME and XR_VENDOR_CACHE to prepared, offline inputs')
subprocess.run([sys.executable,str(r/'tools/build.py'),'--ndk',ndk,'--vendor-cache',cache,'--device-include',os.environ.get('XR_DEVICE_INCLUDE_DIR','/usr/include'),'--hexagonrpc-source',os.environ.get('XR_HEXAGONRPC_SOURCE','/usr/share/xr-hexagonrpc/source'),'--kernel-uapi',os.environ.get('XR_KERNEL_UAPI_DIR','/usr/include/xr-kernel'),'--output',str(r/'build/worker')],check=True)
