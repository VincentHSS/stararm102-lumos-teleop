"""本仓库的 StarTouch 单臂接口，对应 SDK 0.1.9。

这个模块是 SDK ``startouchclass.SingleArm`` 的薄封装：统一连接配置、资源释放、
接口命名和少量组合查询。运动规划、IK、力矩控制和硬件保护仍在原 SDK 内实现。
导入本模块不会连接设备；调用 ``LumosArm.connect()`` 才会创建控制器并使能电机。

单位约定：六轴角度 rad、角速度 rad/s、关节力矩 Nm；末端位置 m、姿态 rad；
夹爪归一化开度为 0（闭合）到 1（打开）。本模块不做主从关节映射。
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path
from typing import Sequence


# 原 SDK 初始化时按相对路径读取该文件。只安装 Python 模块而没有这份配置时，
# SingleArm 仍可能初始化失败，因此连接前先定位完整的 SDK 源码根目录。
CONFIG = Path("src/config/robot_kinematics.yaml")


def find_sdk_root(explicit: str | Path | None = None) -> Path:
    """返回含有 CONFIG 的 SDK 根目录；找不到时抛出 FileNotFoundError。

    explicit 指定后只检查该目录，不回退到其他候选目录，避免误用另一套配置。
    未指定时先查本仓库安装脚本使用的 .dependencies/startouch_sdk。
    """
    here = Path(__file__).resolve().parent
    candidates = ([Path(explicit)] if explicit is not None else [
        here / ".dependencies/startouch_sdk", here / "startouch_sdk",
        Path.cwd(), Path.cwd() / "startouch_sdk",
    ])
    for candidate in candidates:
        # expanduser 支持用户传入的 ~；resolve 使后续临时切换工作目录不影响路径。
        root = candidate.expanduser().resolve()
        if (root / CONFIG).is_file():
            return root
    raise FileNotFoundError(
        f"找不到 StarTouch 配置 {CONFIG}；请安装 SDK 或指定 sdk_root"
    )


@contextmanager
def _sdk_directory(root: Path):
    """仅在构造 SDK 控制器期间切换目录，异常时也恢复原工作目录。

    os.chdir 是进程全局状态，因此不应与其他依赖当前目录的线程并发初始化。
    构造完成后不在此上下文中执行控制命令。
    """
    previous = Path.cwd()
    try:
        os.chdir(root)
        yield
    finally:
        os.chdir(previous)


class LumosArm:
    """持有一个 StarTouch ``SingleArm`` 控制器。

    推荐使用 ``with LumosArm.connect(...) as arm``，退出时自动调用 cleanup。
    ``sdk`` 保留原始 SingleArm，供尚未封装的接口使用；直接使用时仍须遵守
    原 SDK 的参数、单位和硬件约束。
    """

    def __init__(self, sdk):
        # 单独保留构造器，方便测试时传入模拟 SDK，不需要连接真机。
        self.sdk = sdk
        self._closed = False

    @classmethod
    def connect(cls, can: str = "can0", gripper: bool = True,
                enable_fd: bool = False, sdk_root: str | Path | None = None,
                dry_run: bool = False) -> "LumosArm":
        """连接单臂并返回封装对象。

        can 是 Linux CAN 接口名；gripper 控制是否初始化夹爪；enable_fd 和
        dry_run 原样交给 SDK。sdk_root 是含 CONFIG 的源码根目录。
        即使随后只读取状态，原 SDK 构造过程也会使能电机。
        """
        root = find_sdk_root(sdk_root)
        try:
            # 延迟导入：仅查阅文档或导入本模块时，不要求当前环境已安装硬件 SDK。
            from startouchclass import SingleArm
        except ImportError as exc:
            raise RuntimeError("缺少 StarTouch Python SDK；请运行 scripts/install_ubuntu.sh") from exc
        # SDK 的配置路径相对当前目录；初始化结束立即恢复调用者的目录。
        with _sdk_directory(root):
            sdk = SingleArm(can_interface_=can, gripper=gripper,
                            enable_fd_=enable_fd, dry_run=dry_run)
        return cls(sdk)

    def __enter__(self) -> "LumosArm":
        """支持 with 语句，并将机械臂本身绑定到 as 后面的变量。"""
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        """正常退出或抛出异常时都释放 SDK 资源。"""
        self.close()

    def close(self) -> None:
        """只调用一次原 SDK cleanup；可在 with 之外显式调用。"""
        if not self._closed:
            # 先标记再 cleanup，避免 cleanup 抛出异常后又重复释放。
            self._closed = True
            self.sdk.cleanup()

    def joint_state(self) -> dict[str, list[float]]:
        """返回六轴位置、速度和力矩字典。

        三项是 SDK 的三次独立读取，不保证来自同一个严格同步的采样时刻。
        当前公开 Python 接口没有六轴反馈有效位，此处不据全零值判断通信成功。
        """
        return {
            "position_rad": list(self.sdk.get_joint_positions()),
            "velocity_rad_s": list(self.sdk.get_joint_velocities()),
            "torque_nm": list(self.sdk.get_joint_torques()),
        }

    def following_error(self, target_rad: Sequence[float]) -> list[float]:
        """读取一次关节位置，返回每轴 ``目标 - 实测`` 的误差（rad）。

        这是调用瞬间的误差，不会后台连续监视，也不保证目标曾被下发。
        """
        if len(target_rad) != 6:
            raise ValueError("target_rad 必须包含六个关节")
        actual = self.sdk.get_joint_positions()
        return [float(target) - float(current)
                for target, current in zip(target_rad, actual)]

    def ee_pose_euler(self):
        """读取当前末端位姿，返回 ``(xyz_m, roll_pitch_yaw_rad)``。

        这是 SDK 根据当前关节状态求出的末端位姿，不是任意输入关节角的 FK 接口。
        """
        return self.sdk.get_ee_pose_euler()

    def ee_pose_quat(self):
        """读取当前末端位姿，返回 ``(xyz_m, quat_wxyz)``。"""
        return self.sdk.get_ee_pose_quat()

    def solve_ik(self, xyz_m, quat_wxyz, q_seed=None):
        """求目标末端位姿对应的关节角，返回 ``(q_rad, ok)``。

        xyz_m 长度 3，quat_wxyz 长度 4；q_seed 是可选的六轴初始猜测。
        是否允许近似 IK 回退由 SDK YAML 配置决定；ok 只代表 SDK 接受了解。
        """
        return self.sdk.solve_ik(xyz_m, quat_wxyz, q_seed=q_seed)

    def servo_joints(self, position_rad, velocity_rad_s):
        """直接下发六轴位置和速度目标，对应 SDK ``set_joint_raw``。

        此接口不为上层输入序列生成完整轨迹。调用方须自行管理循环频率、
        单周期步长和异常反馈；底层在 MIT 模式仍有自身的保护性平滑。
        """
        return self.sdk.set_joint_raw(position_rad, velocity_rad_s)

    def move_joints(self, position_rad, tf: float = 2.0):
        """请求在 tf 秒内运动到一个六轴目标（rad）。

        对应 SDK ``set_joint``；它在接受指令后返回，不保证返回时已到位。
        要等待整段路径结束，使用 ``joint_waypoints``。
        """
        return self.sdk.set_joint(position_rad, tf=tf)

    def joint_waypoints(self, waypoints, *, time_sec=None, speed_percent=None):
        """执行 N×6 关节路点，阻塞至路径结束并返回规划时长（s）。

        time_sec 是整条路径请求时长；speed_percent 是 (0,1] 的速度比例。
        两者互斥；都不传时 SDK 默认 speed_percent=0.1。短路径即使使用
        speed_percent=0.1 也可能很快结束，demo 因此显式传 time_sec。
        """
        return self.sdk.set_joint_waypoints(waypoints, time_sec=time_sec,
                                            speed_percent=speed_percent)

    def joint_waypoints_with_gripper(self, waypoints, gripper_positions, *,
                                     time_sec=None, speed_percent=None):
        """执行 N×6 关节路点，并为每个路点提供一个 0..1 夹爪开度。

        路点数和夹爪值个数须相同。时间模式与速度比例模式的规则同
        ``joint_waypoints``；具体夹爪同步能力取决于安装的 SDK 后端。
        """
        return self.sdk.move_joint_waypoints_with_gripper(
            waypoints, gripper_positions, time_sec=time_sec,
            speed_percent=speed_percent)

    def update_joint_waypoint_chunk(self, waypoints, *, time_sec=None,
                                    speed_percent=None, switch_delay_sec=0.05):
        """在线提交下一段关节路点，并指定切换延迟（秒）。

        该功能要求原生 pybind 提供同名方法；本层不实现队列或轨迹拼接。
        """
        return self.sdk.update_joint_waypoint_chunk(
            waypoints, time_sec=time_sec, speed_percent=speed_percent,
            switch_delay_sec=switch_delay_sec)

    def plan_joint_waypoints_with_gripper(self, q_start, waypoints,
                                         gripper_positions, *, time_sec=None,
                                         speed_percent=None):
        """从 q_start 规划关节及夹爪路点，原样返回 SDK 的规划结果。

        q_start 是六轴角度 rad；waypoints 为 N×6；gripper_positions 有 N 项。
        规划与执行是不同调用，本方法不会额外下发运动目标。
        """
        return self.sdk.plan_joint_waypoints_with_gripper(
            q_start, waypoints, gripper_positions, time_sec=time_sec,
            speed_percent=speed_percent)

    def move_p(self, poses, *, time_sec=None, speed_percent=None,
               blend_radius_m=0.002, position_tolerance_m=0.003,
               orientation_tolerance_rad=0.05):
        """阻塞式多点末端路径，返回 SDK 规划时长（秒）。

        poses 为 N×6，每行为 [x,y,z,roll,pitch,yaw]，前三项 m、后三项 rad。
        SDK 使用 IK 生成关节路点；blend_radius_m 控制路径拐角过渡。
        容差参数分别是位置 m 和姿态 rad；时长/速度参数规则同关节路点。
        """
        return self.sdk.move_p(
            poses, time_sec=time_sec, speed_percent=speed_percent,
            blend_radius_m=blend_radius_m,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad)

    def move_l(self, poses, *, time_sec=None, speed_percent=None,
               blend_radius_m=0.0, position_tolerance_m=0.003,
               orientation_tolerance_rad=0.05):
        """阻塞式末端直线段路径；输入、单位和返回值与 move_p 相同。

        默认 blend_radius_m=0，要求路径按直线段处理；目标位姿仍须可达。
        本方法只是执行器接口，不提供主臂到从臂的末端空间映射。
        """
        return self.sdk.move_l(
            poses, time_sec=time_sec, speed_percent=speed_percent,
            blend_radius_m=blend_radius_m,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad)

    def move_p_with_gripper(self, frames, *, time_sec=None, speed_percent=None,
                            blend_radius_m=0.002, position_tolerance_m=0.003,
                            orientation_tolerance_rad=0.05):
        """执行多点末端路径及同步夹爪开度，阻塞至路径结束。

        frames 可为 N×7 的 [x,y,z,roll,pitch,yaw,gripper]，也可为 SDK
        支持的 cmdpos/cmdeulerrad/cmdgripper 字典。严格时间对齐应使用
        time_sec；speed_percent 模式不提供逐帧物理时间保证。
        """
        return self.sdk.move_p_with_gripper(
            frames, time_sec=time_sec, speed_percent=speed_percent,
            blend_radius_m=blend_radius_m,
            position_tolerance_m=position_tolerance_m,
            orientation_tolerance_rad=orientation_tolerance_rad)

    def run_motion_program(self, program):
        """阻塞执行 SDK MotionProgram，返回累计规划运动时长（不含 sleep）。

        可以用 make_motion_program() 创建，再添加 movej/movep/movel/sleep 项。
        相邻同类型运动可能在 SDK 内合并，sleep 则形成执行断点。
        """
        return self.sdk.run_motion_program(program)

    @staticmethod
    def make_motion_program():
        """创建原 SDK 的空 MotionProgram；此处不连接或运动。"""
        from startouchclass import MotionProgram
        return MotionProgram()

    def gravity_compensation(self):
        """调用 SDK 的重力补偿接口。

        SDK YAML 另有重力、哥氏及惯量补偿开关；本调用不会修改这些开关，
        也不是可逐周期设置的六轴前馈力矩接口。
        """
        return self.sdk.gravity_compensation()

    def set_gripper_position(self, ratio):
        """将夹爪设为归一化开度：0 闭合，1 完全打开。

        原 SDK 会按 0..1 裁剪，并根据所配夹爪型号换算实际距离。
        """
        return self.sdk.setGripperPosition(ratio)

    def set_gripper_distance(self, distance_m, kp=None, kd=None):
        """按米设置夹爪开口；kp/kd 是夹爪增益，不是六轴电机增益。

        不传 kp/kd 时由 SDK 使用夹爪配置中的默认值；具体距离范围随型号变化。
        """
        return self.sdk.setGripperDistance(distance_m, kp=kp, kd=kd)

    def set_gripper_angle(self, angle_rad):
        """按两指总张开角控制 TypeNex 夹爪；其他型号由 SDK 报错。"""
        return self.sdk.setGripperAngle(angle_rad)

    def set_gripper_effort(self, *, distance_m=None, position=None,
                           angle_rad=None, effort_nm):
        """TypeNex 夹爪力位混控：目标位置和电机力矩限制一起下发。

        distance_m、position、angle_rad 恰好提供一种，分别表示距离 m、
        归一化开度和张开角 rad。effort_nm 为电机输出轴力矩目标/限制，
        并不是六轴力矩前馈，也不会向 StarArm 主臂回传触觉力。
        """
        # 先验证目标表达方式，避免模糊地选择其中一个并意外下发命令。
        selected = sum(value is not None for value in
                       (distance_m, position, angle_rad))
        if selected != 1:
            raise ValueError("需且仅需提供 distance_m、position 或 angle_rad 中的一项")
        if distance_m is not None:
            return self.sdk.setGripperDistanceEffort(distance_m, effort_nm)
        if position is not None:
            return self.sdk.setGripperPositionEffort(position, effort_nm)
        return self.sdk.setGripperAngleEffort(angle_rad, effort_nm)

    def gripper_state(self):
        """返回原生 GripperState，含开度、力矩、温度和反馈有效性等字段。

        该对象的 feedback_valid/feedback_age_ms 只描述夹爪反馈，
        不能用作六轴关节反馈是否有效的判断。
        """
        return self.sdk.get_gripper_state()

