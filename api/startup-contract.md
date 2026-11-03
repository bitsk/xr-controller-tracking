# A081 Nordic 启动与 type-2 状态补充

本轮结论：**当前原厂 native 启动路径没有“收到 type-2 后发 Enable/stream 命令”的分支**。可确认的最小接收启动流程不含 TX：打开 SPI/通知节点，设置本地 META 解码模式，注册时间/数据回调，启动接收循环。只收到 type-2，不能据此认定需要发送未知启用命令。本轮没有发包、访问设备、改动已冻结 helper/root，也没有改参考解析 cpp。

实机原厂 SDK 的 `exec/load/create` 已由主任务验收；这是库复用边界通过。以下依据主任务保存的15秒 Nordic 抓包和原厂静态调用链，不是手柄定位已恢复。

## r1 当前实机状态机接入

下文旧库地址和36字节接收是r1历史证据。当前实机 serial SHA `855dabd3eaf5a1be9ae9ba9d1450670cb1905d18cd61bc0d9d6e2731dc7f9e1f`、JNI SHA `410da7fe91820628ae6d033121583c5560c2b119f60d08f02f30933ef9036c9c` 使用一次72字节交换。新 `controller_sdk_pose.cpp` 按当前库回调分开保存 link、enable、ctrl state 和实时位姿有效性。

- type2 的 head power3/4均使 Android 第二连接标志为1，只有3使 enable=1。GZLL高半字节单独保留，不命名为配对成功。完整状态字节等于`04`才触发ctrl state0及控制字段清空；`34`不触发ctrl0。
- type7 非零四元数回调ctrl state5，全零四元数回调state7；JNI回调直接保存状态。type7还使用缓存GZLL更新head power：版本<25为3；新版本GZLL0/1/2→1，3→3，4→4，其他不改变。新worker补上这条enable更新路径。
- Android getConnectState仍将link与enable分开。新worker保持head4的link_present；只要真实radio源持续发原始SPI片段，休眠/禁用跟踪不等于无线链路消失。OpenVR可显示connected但pose invalid；未推测Java/UI对其余状态的命名。
- 睡眠、enable/version变化、每手IMU超过100ms及源EOF清理本轮跟踪epoch和控制字段；恢复后必须重新收到有效时钟/IMU和图像。100ms是本移植的实时消费门槛，不是原厂无线断连定时器；当前`HeartBeatThread`仍直接return0。
- 原厂state7复用缓存位置只保留为诊断事实，不将缓存发布为实时有效位置。光学有效还要求本epoch两帧图像、真实新鲜来源、SDK置信度恰好1，并等待epoch开始后100ms，排除休眠前的视觉置信窗口。
- 只有通过时钟检查和SDK接收的IMU才推进`last_clock`，避免异常未来时间戳永久阻塞恢复。未伪造SOF、头姿或手柄时钟。radio的PatternID版本缓存按原厂持续到被更新/进程重启，移除无原厂依据的3秒过期；实际相机SOF的100ms门槛不变。

实机首轮接入发现SDK初始化前已暴露input socket：radio首包452617842213ns，SDK首query452782084633ns，相差164ms，超过100ms输入门槛。已将factory/mode/start和首次pose query移到bind/listen之前，再发布`controller_input_ready`；增加拒绝包头/长度/FD/时间差诊断，不放宽实时门。原始记录在`reports/device/steamos-controller-position-r1/first-candidate/raw/`。

当前静态证据：`build/controller-position-r1/sdk-producer/live-serial-state-rea.json`（当前ParseImuPreInt `0xf694`、ParsePatternID `0x1008c`、HeartBeat `0xf684`），`live-jni-state-rea.json`（getConnectState、initialize），`live-jni-power-rea.json`（head power回调）及`live-jni-power-store.asm`（ctrl power存储，LLVM指令证据）。REA部分function-identity查询不支持file-offset，但对应伪代码项返回ok；保留原始错误与实际覆盖，不视为全函数分析通过。代码已ARM32 NDK严格编译；实际休眠→唤醒的人工运动验证另记。


## 当前抓包的准确解释

`reports/device/xr-tracking-r1/nordic-counts.log` 记录7494次交换：7479全零、4条严格 `A5 5A` 的有效 Pattern ID 包、11条相同 payload 但头为 `A4 5B / A7 58 / A6 59` 的记录；没有 type-7。

有效36字节帧：

```text
a55aa000ffffffff442b07000000000000000000442b07002000003400000000000000dc
```

去掉头部后 `p[34]` 的 key `p[1]=0`，因此 META 解码不改变该样本，sum8=`0xdc`正确。原厂 `GetPatternIDPackageInfo` 和注册 callback 的结果是：

| 字段 | 值 / 原厂消费 |
| --- | --- |
| type / hand | type=2，hand=1（右手链路） |
| Pattern version | `{p[24],p[23],p[22]}={0,0,32}`；交给 **head version** callback，是头显端无线侧版本，不是已证实的手柄本体版本 |
| 状态字节 | `p[25]=0x34`；版本第三项>=25时分解为 low=4、high=3 |
| low=4 | `head_pwr` callback 收到4；JNI右回调将 `enable=0`、第二连接标志=1；`getConnectState(1)` 返回 `[0,1]` |
| high=3 | `head_gzll_pwr` callback 收到3；`getPairState(1)` 原样返回3 |
| active 对照 | `head_pwr=3` 才把相应 `enable` 置1；3和4均使第二连接标志为1 |

因此当前右手链路**没有进入原厂允许提交手柄图像的 enable 状态**。仍缺 Java/UI 对 GZLL 值3的命名，不能把3擅自解释为“已配对”或“未配对”，也不能仅靠这包区分手柄关机、离线或无线状态未完成。

`ParsePatternIDPackageImpl` 中清空控制器字段的特殊分支先比较完整状态字节 `==4`，随后才对新版本拆半字节。当前原值为 `0x34`，不会走该清空分支；“low=4”等于“执行了清空”是不成立的。

原厂 SPI 接收函数 `0x109c0` **严格要求 `A5 5A`**，在 payload 解码前完成该判断；META 只处理 payload，不能解释或修复11条异形头。应保留这些原始记录，不把它们默认为合法帧或擅自改成 A55A。

## 实际启动调用链

1. `QiyiController` 构造器 `0xed8c` 创建 `SerialPortIO`。后者的 Impl 构造器 `0xbeb8` 创建左右 hand record，打开 SPI/通知节点；`0xc1d4` 没有发初始化命令。
2. JNI `0xee46/0xee48` 明确传 `SetEncryptWay(2)`。该设置选择本地 `DecodePackage_META` 路径，不是向无线固件发指令。旧 Ghidra 伪代码的 `SetEncryptWay(0x94)` 丢失了 this/参数，不能照抄。
3. `initialize` 注册 `CLOCK_BOOTTIME` 获取函数与各 hand 数据/状态 callback。末尾可见 `Write(0x15,time)`，但绑定重载 `SerialPortIO::Write(uchar const&,long long const&)` 的 ELF `0x8f90` 仅 `return 0`，**不发包**。
4. `Run` `0x87e0` 确保 SPI 打开并创建 `SerialPortLoop` `0xc464`；它调用 `ReceiveThread` `0x10898`，后者等待通知、做36字节交换、记录接收时刻，再调 `ParseImpl` `0x106a0`。没有发送队列时 TX 为36个 `0xee`。
5. `ParseImpl` 先解码，再按校验后的包类型分发。type-2进入 `ParsePatternIDPackageImpl` `0xff50`，只更新字段/回调；本轮检查的完整该函数中没有 `Write/WriteCmd/WritePattern`。
6. `QiyiController::start` `0x10824` 再调用 `Run`、SDK虚表 `+4(self,0)` 与 `+0(self,0)`，没有直接发无线启用命令。

`HeartBeatThread` `0xf69e` 在本构建也是空实现。`TimeSync_META/TimeSync_XR3/WritePattern` 等导出存在，并不能证明上面的活动接收循环自动调用它们。不能把其他机型或旧库的心跳状态机搬过来。

## META 解码必须位于校验和手位解析之前

针对本机 `SetEncryptWay(2)`，原厂 `DecodePackage_META` 的精确等价操作如下，`p` 是去掉 A55A 后的34字节副本：

```cpp
const uint8_t key = p[1];
p[0] ^= key;
for (unsigned i = 2; i < 34; ++i)
    p[i] ^= key;
// p[1] 保留原值，也仍参与校验。
const uint8_t checksum = sum8(p, 33);  // p[0]..p[32]
if (checksum != p[33])
    reject();
const unsigned type = (p[0] >> 4) & 7;
const unsigned hand = p[0] >> 7;
```

地址证据：serial `0x121da` 读 key；`0x121f8` 写 p0；`0x121fe..0x12254` 写 p2..16；`0x12256..0x12262` NEON处理 p17..32；`0x12266..0x1226c` 写 p33。没有写 p1。`CheckCRC` `0x1229c` 在 `0x122a2/0x122a6` 将 p1 加入总和，在 `0x12318..0x1232c` 与 p33比较；实际包识别调用的 `CheckImuPreIntPackage` `0x12344` 有相同求和。`GetImuPreIntPackageHandleId` `0x1233e` 读取**解码后**p0右移7。

这个顺序修正了原参考与线上原始包之间的缺层。参考 `decode_preintegrated_transfer` 保持处理已解码载荷；实际采集程序由主任务在副本上增加 META 层，同时保留 raw。不能在调用参考解析器后才 XOR，也不能重复 XOR。当前 key=0 的样本不会因这项修正变成 IMU，但未来非零 key 的 type-7 需要此层。

## 命令范围与未闭合部分

| 入口 | 实际行为 | 本轮结论 |
| --- | --- | --- |
| JNI `sendCmd` `0x117dc` | 支持 DFU、start pair=5、unbond=10、cancel pair=12；其余走 ignore | 不用于启动接收，不自动发送 |
| serial `WriteCmd` `0x949c` | type3控制载荷；cmd8重复写入DFU命令，cmd10额外清理本地配对状态 | 不能泛化为无副作用接口 |
| `WritePattern` `0x91d8` | type1，五字节载荷为 `{pattern,0,0,0,0}` | 未恢复 pattern 枚举/活动调用处，不把任意 pattern 当 Enable |
| `TimeSync_XR3` `0xf5a8` | 未就绪时先 `CMD_GENERATE_STATIC_SECRET_KEY`（cmd0x10）；另一分支 `WriteSyncWithIMU` | 不是已证明无持久副作用的通用启动握手，本轮不调用 |
| `WriteSyncWithIMU` `0x93a8` | 构造8字节同步请求：左 `A5 08 51 00 00 00 00 59`；右 `A5 08 D2 00 00 00 00 DA` | 未在当前启动链找到必要调用，不将查询包作为启动动作发送 |
| `WriteTimeStampCmd` `0x9ba0` | 来自真实 ctrl-tracking 帧 SOF 的时间回传，见 `input-contract.md` | 保留原厂相机分相调用条件；没有相机帧时不伪造它 |

本轮没有增加可自动执行的 TX。可继续自动推进 raw/META采集、原厂SDK生命周期、相机/标定接入；若这些明确入口之外仍需要无线模式切换，需要实际调用方或接收固件证据来确认含义，再由主任务决定发送。当前缺口是活动无线/相机时序与手柄状态，不能由一个猜测的 Enable 命令代替。

## 证据

目标 SHA-256：serial `cdedc3c7e18819c53fd70d841a2e9705271c996b4b15f851aa52fe88efbff4ae`；JNI `3b05f44cbdf84415fa303472d29e0bf433fd8e83372823276ee8447420302461`。地址为 ELF VA；REA地址加 `0x10000`。

新增证据位于 `build/xr-tracking-r1/controller/startup-contract/`：

- `startup-functions.json` / `ev_36b644511b75a62df8e1b206b205355b9faaaec00bd783f231531049405e4834`：SerialPortLoop、WriteCmd、WritePattern、TimeSync_XR3、HandleTrig、ParsePatternID，全部 resolved/ok。
- `serial-runtime-functions.json` / `ev_10e0ac7af853bbb34cf0458299935dc47d328313b8e580da7be7e8a5360d897f`：ReceiveThread、WriteSyncWithIMU、Impl/HandleState构造、空Write重载、空HeartBeatThread，全部 resolved/ok。
- `jni-state-start-command.json` / `ev_385f54b150a9fb86e94ecb01ee34132ddde8917afc9fe9e9ffdb2433cf5ea408`：start、sendCmd、getConnectState、getPairState，全部 resolved/ok。
- `jni-head-state-callbacks.json` / `ev_81265397ed88f821945bbf002f4d667826a45a9348c8e24d303887e2362747bc`：左右 head/gzll callback，全部 resolved/ok。
- `serial-decode-meta-disassembly.txt`、`checksum-disassembly.txt`、`head-version-setter-disassembly.txt`：LLVM指令证据。`head version` setter `0xb0a0` 的function对象位置与ParsePatternID回调位置一致。
- 复用 `build/xr-rea-analysis/controller/serial-{flow,transport,packets-verified}.json` 与 `jni-entrypoints.json`；`build/xr-tracking-r1/controller/jni-constructor-disassembly.txt` 核对 SetEncryptWay 的实参2。

没有重新导入 SDK 大库；没有部署或执行目标程序。源码片段是对原厂实际调用的参考表达，不是完整无线协议实现。
