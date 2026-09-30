# StarArm 102 → Lumos Python 遥操作

使用 StarArm 102 主臂实时控制 Lumos 单臂。程序通过 StarArm UART 和 Lumos CAN 通信，不依赖 LeRobot。客户操作步骤见 [`docs/StarArm102_Lumos_Python_Teleoperation_Guide.docx`](docs/StarArm102_Lumos_Python_Teleoperation_Guide.docx)。

## 硬件与环境

- Ubuntu 20.04、22.04 或 24.04 主机
- Python 3.10（推荐）
- StarArm 102 主臂及 UART/USB 连接
- Lumos 单臂及已配置为 1 Mbps 的 CAN 接口（默认 `can0`）
- `git`、`sudo` 权限，以及可访问 GitHub 和 PyPI 的网络

## 安装

先准备并激活 Python 3.10 环境。若使用 Conda：

```bash
conda create -n stararm102-lumos python=3.10 -y
conda activate stararm102-lumos
```

下载本项目并运行一键安装脚本：

```bash
git clone https://github.com/VincentHSS/stararm102-lumos-teleop.git
cd stararm102-lumos-teleop
bash scripts/install_ubuntu.sh
```

脚本会安装 Lumos SDK 所需的系统构建依赖、StarArm Python 包和 Lumos Python 依赖；然后自动从 Lumos 官方仓库下载 StarTouch SDK 到项目目录 `.dependencies/startouch_sdk` 并完成安装。客户不必手动打开 Lumos 安装文档或进入 SDK 源码目录。脚本使用当前激活的 Python 环境；首次运行需要联网及 `sudo` 权限。

### 手动安装（排错时使用）

在项目根目录、已激活的 Python 3.10 环境中依次执行。以下命令和一键脚本执行相同的依赖安装：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config libeigen3-dev libyaml-cpp-dev liborocos-kdl-dev libtinyxml2-dev pybind11-dev

python -m pip install --upgrade pip setuptools wheel
python -m pip install pyserial fashionstar-uart-sdk

mkdir -p .dependencies
if [ ! -d .dependencies/startouch_sdk ]; then
  git clone --depth 1 https://github.com/lumos-open/startouch_sdk.git .dependencies/startouch_sdk
fi
python -m pip install -r .dependencies/startouch_sdk/requirements.txt
python -m pip install .dependencies/startouch_sdk
```

安装完成后验证接口：

```bash
python -c "import serial, fashionstar_uart_sdk; from startouchclass import SingleArm; print('SDK OK')"
```

> Lumos SDK 源码由安装脚本从 [Lumos 官方仓库](https://github.com/lumos-open/startouch_sdk) 获取。上游仓库未声明再分发许可证，因此本项目不复制或打包其 SDK 源码。

## CAN 接口

检查接口：

```bash
ip -details link show can0
```

如果接口尚未启用，再配置为 1 Mbps：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

若提示 `Device or resource busy`，先再次查看接口状态；如果 `can0` 已经是 `UP`，不需要重复设置。串口名称不确定时运行 `ls -l /dev/ttyUSB*`，并在后续命令中替换默认的 `/dev/ttyUSB0`。

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

## 仓库内容

```text
.
├── stararm102_lumos_teleop.py
├── scripts/
│   └── install_ubuntu.sh
├── README.md
└── docs/
    └── StarArm102_Lumos_Python_Teleoperation_Guide.docx
```

## License

本仓库当前未授予开源许可证；公开可见不代表获得复制、修改或再分发许可。若需为客户授予使用权，请在发布前添加经权利人确认的许可证或授权条款。
