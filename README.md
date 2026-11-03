# xr-controller-tracking

ARM32 SDK/ABI 手柄输入与定位 helper。


构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [openkylin-xr-docs](https://gitee.com/bigbits/openkylin-xr-docs)。

## 构建

`python3 tools/build.py --ndk /path/to/android-ndk --stock-cpp /path/to/stock/ARM32/libc++_shared.so --output /new/build`

NDK、原厂 SDK、SNPE 与固件由使用者提供；仓库仅包含桥接代码和来源记录。相邻 xr-device-io 提供共享输入合同。输出仍是 Bionic ARM32 helper，通过 IPC 接入 ARM64/glibc 消费者。

已归档的原厂依赖可由 xr-image-tools 的 `tools/vendor.py fetch` 获取；构建时可改用 `python3 tools/build.py --ndk /path/to/android-ndk --vendor-cache /path/to/vendor-cache --output /new/build`，直接消费固定版本 controller 包里的 libc++_shared.so。SDK/Bionic 闭包另存于公开 xr-vendor-assets Release，正常构建无需提取原厂镜像；每台设备的 persist 校准单独提供。

## openKylin 打包

本仓库包含独立的 `debian/` 配方。使用目标发行线的干净构建环境运行 `dpkg-buildpackage -us -uc -b`，预先准备 Build-Depends 和离线构建输入。包构建、目标系统安装和设备验收分别记录。

当前迁移不自动启用竞争采集、切换活动运行时或写设备启动分区；已有验证基线继续保留。
