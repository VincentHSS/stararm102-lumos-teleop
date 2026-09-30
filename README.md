# StarArm 102 → Lumos Python 遥操作

使用 StarArm 102 主臂实时控制 Lumos 单臂的 Python 示例。程序通过 StarArm UART 和 Lumos CAN 通信，不依赖 LeRobot。客户操作步骤见 [`docs/StarArm102_Lumos_Python_Teleoperation_Guide.docx`](docs/StarArm102_Lumos_Python_Teleoperation_Guide.docx)。

## 硬件与环境

- Ubuntu 20.04、22.04 或 24.04 主机
- Python 3.10（推荐）
- StarArm 102 主臂及 UART/USB 连接
- Lumos 单臂及已配置为 1 Mbps 的 CAN 接口（默认 `can0`）
- StarArm Python SDK、Lumos `startouch_sdk` 安装在运行脚本的同一个 Python 环境

安装 SDK 前，请阅读 Lumos [安装说明](https://github.com/lumos-open/startouch_sdk/blob/main/README_INSTALL.md) 和 StarArm [Python SDK 指南](https://github.com/servodevelop/Star-Arm-102/blob/main/Python_SDK/PYTHON_SDK_GUIDE.md)。Lumos API 参考：[README_API.md](https://github.com/lumos-open/startouch_sdk/blob/main/README_API.md)。

安装 StarArm Python 依赖：

```bash
python -m pip install pyserial fashionstar-uart-sdk
```

按照 Lumos SDK 安装说明安装其依赖，并在 SDK 源码目录执行：

```bash
python -m pip install -r requirements.txt
python -m pip install -e .
```

## 首次校准

确认急停可用、工作区域畅通，并让两台机械臂处于安全且相互对应的参考姿态。校准会连接 Lumos SDK（SDK 连接会使能电机），采集 StarArm 关节范围并保存参考位置；不会下发运动目标。

```bash
python stararm102_lumos_teleop.py --calibrate --can can0 --leader-port /dev/ttyUSB0 --gripper
```

校准文件默认保存在 `~/.config/stararm102_lumos/calibration.json`。若不使用夹爪，从校准和后续命令中一并删除 `--gripper`。

## 先预览，再执行

不带 `--execute` 时程序只显示输入，不向 Lumos 下发运动目标：

```bash
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper
```

确认关节映射和方向正确后才运行执行模式：

```bash
python stararm102_lumos_teleop.py --can can0 --leader-port /dev/ttyUSB0 --gripper --execute
```

如果串口不是 `/dev/ttyUSB0`，替换为实际设备名。夹爪方向相反时，在预览和执行命令中都增加 `--invert-gripper`。使用 `Ctrl+C` 正常停止；发生卡滞、碰撞或持续力矩报警时，立即停止并使用硬件急停。

## 调整关节映射与速度

脚本顶部的常量控制对应关系和目标变化速度：

- `JOINT_SIGNS`：六个 Lumos 关节方向，`1` 同向、`-1` 反向。
- `FOLLOW_GAIN`：主臂相对参考姿态的角度变化映射比例，范围 `(0, 1]`。
- `MAX_STEP_DEG`：每个控制周期允许的最大目标角度变化。
- `CONTROL_RATE_HZ`：控制循环频率。

修改后先使用预览模式确认，再谨慎执行。不要通过增大参数或修改 SDK 保护阈值来掩盖力矩报警。

## 安全说明

本程序控制真实机械臂。执行前须确认关节顺序、方向、活动范围和急停功能；首次执行时只小幅移动一个关节并观察。保持人员远离运动范围。出现异常立即停止并使用硬件急停。该程序不会替代机械臂自身限位、急停和安全操作规程。

## 仓库内容

```text
.
├── stararm102_lumos_teleop.py
├── README.md
└── docs/
    └── StarArm102_Lumos_Python_Teleoperation_Guide.docx
```

## License

本仓库当前未授予开源许可证；公开可见不代表获得复制、修改或再分发许可。若需为客户授予使用权，请在发布前添加经权利人确认的许可证或授权条款。
