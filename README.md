# Wuji Hand 2 · 手套遥操作

用 **Wuji Glove（手套）** 实时控制 **Wuji Hand 2（二代灵巧手）**。

本仓库基于官方 [wuji-sdk](https://github.com/wuji-technology/wuji-sdk) 示例改造：保留官方 `RetargetSession`，增加调参、脚踏门控、回零，以及 **二代以太网直驱**（不再依赖一代 `wujihandros2` / `wujihandcpp`）。

---

## 架构

```text
手套 Wuji Glove
    │  hand_skeleton (21×3 MediaPipe 关键点)
    ▼
预处理（可选）
    · 手指缩放 / 对掌拉近
    · SDK 命名用户标定 URDF（或默认人手 URDF）
    ▼
官方 RetargetSession
    · HandModel.WujiHand2 + 左右手
    · step() → qpos[20]（固件关节序）
    ▼
后处理 + 门控
    · 小指增益、捏合加成
    · Footkey（F7）/ go_home
    · EMA 平滑（二代）
    ▼
执行层（二选一）
    ├─ --drive sdk  →  JointCommand 直发二代手（推荐）
    └─ --drive ros  →  ROS topic → wujihandros2 → 一代 USB 手
```

| 层 | 作用 |
|---|---|
| 输入 | `wuji_sdk` 订阅手套骨架 |
| 重定向 | `RetargetSession.for_hand(WujiHand2)` |
| 调参 | `2.teleop_tuned.py` 应用层 |
| 执行 | 二代：SDK 以太网；一代：ROS + USB 驱动 |

---

## 环境要求

| 项 | 说明 |
|---|---|
| 系统 | Linux aarch64 / x86_64（Jetson Orin 已验证） |
| Python | ≥ 3.10 |
| SDK | `wuji-sdk`（含 retarget） |
| 网络 | 电脑与二代手同一网段（本仓库默认 `192.168.10.x`） |
| 硬件 | Wuji Glove + Wuji Hand 2（左右可选） |

```bash
# 推荐 conda 环境
conda create -n wuji python=3.10 -y
conda activate wuji
pip install "wuji-sdk" numpy pynput

# 若要用 ROS 脚踏话题 / go_home 服务（可选）
source /opt/ros/humble/setup.bash
```

---

## 网络：二代手 IP

二代手出厂默认：

| 手 | 出厂 IP | 本仓库常用 IP |
|---|---|---|
| 左 `WH2J…` | `192.168.1.110` | `192.168.10.110` |
| 右 `WH2K…` | `192.168.1.111` | `192.168.10.111` |

电脑有线网卡需与手同网段（例如 `192.168.10.149/24`）。

若要把出厂 `192.168.1.x` 改成 `192.168.10.x`：

```bash
# 1) 电脑临时改到 192.168.1.100，确认 ping 通 110/111
# 2) 改 IP 并重启手
python examples/python/wuji_hand_2/change_hand_ip_to_10.py
# 3) 电脑改回 192.168.10.149，ping 192.168.10.110 / .111
```

验证发现：

```bash
python - <<'PY'
from wuji_sdk import SdkManager
m = SdkManager.instance()
for d in m.scan():
    print(d.device_type, d.sn, d.address)
PY
```

应能看到 `WujiHand2` 与 `WujiGlove`。

---

## 快速开始（二代 · 推荐）

**不要**启动 `wujihandros2`，**先关掉** Wuji Studio（避免抢连接）。

```bash
cd examples/python/retargeting

# 双手手套 → 双手二代（默认 --drive sdk --hand-model wujihand2）
python 2.teleop_tuned.py --no-footkey

# 只要一侧
python 2.teleop_tuned.py --drive sdk --hand-model wujihand2 --side right --no-footkey

# 需要脚踏门控：按住 F7 才发指令（与 Apex Teleop 一致）
python 2.teleop_tuned.py --drive sdk --hand-model wujihand2
```

正常时终端会出现类似：

```text
Drive: sdk (Wuji Hand 2 direct — wujihandros2 NOT used)
Hand2 direct: left SN=WH2J… → hand_left
Hand2 direct: right SN=WH2K… → hand_right
Teleoperating 2 hand(s) (Ctrl+C to stop)...
```

`Ctrl+C` 停止；脚本会 `disable` 电机并断开设备。

---

## 脚本说明

| 文件 | 用途 |
|---|---|
| [`2.teleop_tuned.py`](examples/python/retargeting/2.teleop_tuned.py) | **主入口**：手套 → 重定向 → 二代直驱 / 一代 ROS |
| [`1.teleop_real.py`](examples/python/retargeting/1.teleop_real.py) | 官方风格直驱（少调参），已兼容 Hand2 `handedness()` |
| [`0.retarget_session.py`](examples/python/retargeting/0.retarget_session.py) | 无硬件，测 `RetargetSession` |
| [`3.save_home.py`](examples/python/retargeting/3.save_home.py) | 录当前手位姿 → `home_pose.json` |
| [`home_pose_service.py`](examples/python/retargeting/home_pose_service.py) | 回零服务（teleop 内 / HTTP） |
| [`change_hand_ip_to_10.py`](examples/python/wuji_hand_2/change_hand_ip_to_10.py) | 二代手 IP 改到 `192.168.10.x` |
| [`wuji_hand_2/0.subscribe_callback.py`](examples/python/wuji_hand_2/0.subscribe_callback.py) | 官方：订阅关节状态 |
| [`wuji_hand_2/2.publish.py`](examples/python/wuji_hand_2/2.publish.py) | 官方：使能并零位保持 |

---

## `2.teleop_tuned.py` 常用参数

```bash
python 2.teleop_tuned.py --help
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--drive {sdk,ros}` | `sdk`（当 hand-model=wujihand2） | `sdk`=二代直驱；`ros`=一代 wujihandros2 |
| `--hand-model` | `wujihand2` | 重定向目标手型 |
| `--side` | `both` | `left` / `right` / `both` |
| `--no-footkey` | off | 关闭 F7 门控，始终发指令 |
| `--user-name` | 首个命名用户 | 手套 IK 用标定 URDF |
| `--default-user` | — | 强制默认用户 + 内置 URDF |
| `--finger-scaling` | `1 1 1 1 1.1` | 五指关键点缩放 |
| `--pinky-flex-gain` | `0.9` | 小指屈曲增益 |
| `--pinky-open-bias` | `0.05` | 小指张开偏置（rad） |
| `--opposition-*` / `--pinch-*` | 见 `--help` | 对掌 / 捏合 |
| `--no-home` | — | 不加载回零 |
| `--go-home-service` | `/tj/control/go_home` | ROS Trigger 回零（需 ROS） |

二代 MIT / 平滑（改源码顶部常量）：

| 常量 | 默认 | 说明 |
|---|---|---|
| `HAND2_KP` | `5.0` | 刚度，更大更跟手 |
| `HAND2_KD` | `0.15` | 阻尼 |
| `HAND2_QPOS_EMA` | `0.35` | 指令平滑，更大更稳、更钝 |
| `HAND2_EFFORT_LIMIT` | `1.5` | 电流限制（A） |

---

## 回零（Home）

```bash
# 手已连接时，录制当前姿态
python 3.save_home.py

# teleop 运行中（若启用了 ROS go_home）
ros2 service call /tj/control/go_home std_srvs/srv/Trigger
```

`home_pose.json` 按本机手位姿生成，**不建议**把个人标定文件强行提交；仓库可只保留示例流程。

---

## 一代手（可选 · ROS2）

仅当仍使用 **USB 一代手** + `wujihandros2`：

```bash
# 终端 A
source /opt/ros/humble/setup.bash
source /path/to/wujihandros2/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch wujihand_bringup wujihand.launch.py \
  hand_name:=hand_right serial_number:=YOUR_USB_SN

# 终端 B
python 2.teleop_tuned.py --drive ros --hand-model wujihand --no-footkey
```

> 二代手 **不能** 用 `wujihandros2`（其底层是 `wujihandcpp` USB）。二代请用 `--drive sdk`。

---

## 官方二代单机示例（不含手套）

```bash
cd examples/python/wuji_hand_2
python 0.subscribe_callback.py   # 读关节
python 2.publish.py              # 使能 + 零位保持 5s（注意安全）
```

---

## 故障排查

| 现象 | 处理 |
|---|---|
| Studio 能看见设备但 Connect 失败 | 电脑与手不同网段；先 `ping` 通再连 |
| `SdkManager()` 报错 | 用 `SdkManager.instance()` |
| `handedness_name` 不存在 | Hand2 用 `hand.handedness().get()`（本仓库已修） |
| 动作一卡一卡 | 关掉 Studio；确认 `--drive sdk`；略增 `HAND2_KD` / `HAND2_QPOS_EMA` |
| 扫不到手 | 检查网线/交换机、静态 IP、防火墙 |
| 电机不动 | 是否 `--no-footkey` 或已按住 F7；是否 `enable` 成功 |
| 想换回一代 | `--drive ros --hand-model wujihand` + 启动 wujihandros2 |

---

## 目录结构

```text
├── README.md                 # 本说明
├── LICENSE
├── CHANGELOG.md              # 上游 SDK 变更（参考）
└── examples/python/
    ├── retargeting/          # 手套 → 手 遥操作
    │   ├── 2.teleop_tuned.py # 主程序（二代 sdk / 一代 ros）
    │   ├── 1.teleop_real.py
    │   ├── 0.retarget_session.py
    │   ├── 3.save_home.py
    │   └── home_pose_service.py
    └── wuji_hand_2/          # 二代官方示例 + 改 IP 工具
        ├── change_hand_ip_to_10.py
        ├── 0.subscribe_callback.py
        ├── 1.subscribe_async.py
        ├── 2.publish.py
        └── 3.fingertip_typed.py
```

---

## 安全提示

- 首次使能前确认手指周围无干涉。
- `2.publish.py` / teleop 会使能电机；急停可断电或 `Ctrl+C`（脚本会 `disable`）。
- 多人同时连同一只手时，写指令会互相覆盖（SDK bridge 默认开启）。

---

## 参考

- [Wuji Docs · Hand 2 SDK](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)
- [Wuji Docs · Retargeting](https://docs.wuji.tech/docs/en/wuji-sdk/latest/retargeting/)
- 上游 SDK：[wuji-technology/wuji-sdk](https://github.com/wuji-technology/wuji-sdk)

## License

MIT（与上游 wuji-sdk 示例一致）。
