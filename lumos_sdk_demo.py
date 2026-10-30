#!/usr/bin/env python3
"""演示本仓库的 LumosArm 封装，而非 StarArm 主从遥操作程序。

默认 action=status：连接控制器、等待反馈、读取六轴状态和末端位姿。
ik 只计算当前位姿的逆解；joint、linear、gripper 默认只打印目标，
只有加 --execute 才向 Lumos 下发这些目标。

注意：预览模式仍会构造 StarTouch SingleArm，因此电机仍会被使能。
本 demo 不实现主臂输入映射、持续监控或完整的应用层保护。
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

from lumos_arm import LumosArm


def parse_args(argv=None):
    """解析演示动作和参数；在连接硬件前拒绝明显越界的输入。

    argv=None 时读取真实命令行；测试时可传入字符串列表。
    限制角度/位移是 demo 的操作范围，不等于机械臂真实关节/空间限位。
    """
    parser = argparse.ArgumentParser(description="LumosArm 封装接口演示")
    # 连接参数直接传给 LumosArm.connect。--dry-run 的具体行为由原 SDK 决定。
    parser.add_argument("--can", default="can0", help="Lumos CAN 接口")
    parser.add_argument("--sdk-root", type=Path, help="StarTouch SDK 配置根目录")
    parser.add_argument("--gripper", action="store_true", help="连接时启用夹爪")
    parser.add_argument("--dry-run", action="store_true", help="传递 SDK dry_run 参数")
    parser.add_argument("--startup-wait", type=float, default=2.0,
                        help="连接后等待反馈更新的秒数，默认 2")
    # status/ik 是读数与计算；其他 action 有目标，但默认也只预览。
    # 运行运动命令时必须显式给出 --action，避免 --execute 落入默认 status 分支。
    parser.add_argument("--action", choices=("status", "ik", "joint", "linear", "gripper"),
                        default="status", help="演示的接口，默认仅查看状态")
    parser.add_argument("--execute", action="store_true", help="对 joint/linear/gripper 实际下发命令")
    parser.add_argument("--allow-zero-state", action="store_true",
                        help="确认实物确在全零位后，允许从全零反馈执行运动")
    # 关节编号对操作者使用 1..6；内部 Python 列表索引需要减一。
    parser.add_argument("--joint", type=int, choices=range(1, 7),
                        metavar="1..6", help="joint 动作修改的关节，默认 1")
    parser.add_argument("--delta-deg", type=float,
                        help="joint 动作的相对角度，范围 ±20 度，默认 +2 度")
    parser.add_argument("--delta-z-mm", type=float,
                        help="linear 动作的相对 Z 位移，默认 +5 mm")
    parser.add_argument("--gripper-position", type=float,
                        help="gripper 动作的归一化开度，默认 0.5")
    parser.add_argument("--gripper-timeout", type=float, default=5.0,
                        help="等待夹爪反馈到达目标的最长秒数，默认 5")
    args = parser.parse_args(argv)
    # argparse 的类型检查只保证能转换为 float；还要排除 nan/inf。
    # 这些范围属于 demo 的初始测试约束，SDK 的 URDF/YAML 另有真实保护参数。
    if not math.isfinite(args.startup_wait) or not 0 <= args.startup_wait <= 10:
        parser.error("--startup-wait 必须在 0..10 秒范围内")
    if not math.isfinite(args.gripper_timeout) or not 0 < args.gripper_timeout <= 30:
        parser.error("--gripper-timeout 必须在 (0,30] 秒范围内")
    if args.execute and args.action in ("status", "ik"):
        parser.error("--execute 需要指定 --action joint、linear 或 gripper")
    if (args.joint is not None or args.delta_deg is not None) and args.action != "joint":
        parser.error("--joint 和 --delta-deg 只能用于 --action joint")
    if args.delta_z_mm is not None and args.action != "linear":
        parser.error("--delta-z-mm 只能用于 --action linear")
    if args.gripper_position is not None and args.action != "gripper":
        parser.error("--gripper-position 只能用于 --action gripper")
    if args.allow_zero_state and (not args.execute or args.action not in ("joint", "linear")):
        parser.error("--allow-zero-state 只用于执行 joint 或 linear 动作")
    if args.delta_deg is not None and (not math.isfinite(args.delta_deg) or abs(args.delta_deg) > 20):
        parser.error("--delta-deg 必须在 -20..20 度范围内")
    if args.delta_z_mm is not None and (not math.isfinite(args.delta_z_mm) or abs(args.delta_z_mm) > 10):
        parser.error("--delta-z-mm 必须在 -10..10 mm 范围内")
    if args.gripper_position is not None and (
            not math.isfinite(args.gripper_position) or not 0 <= args.gripper_position <= 1):
        parser.error("--gripper-position 必须在 0..1 范围内")
    if args.action == "gripper" and not args.gripper:
        # 不启用夹爪时，连接后的控制器未必能安全接受夹爪命令。
        parser.error("gripper 动作需要同时传入 --gripper")
    # 用 None 区分“用户没传”与“用户显式传参”，完成动作相关校验后再填默认值。
    args.joint = 1 if args.joint is None else args.joint
    args.delta_deg = 2.0 if args.delta_deg is None else args.delta_deg
    args.delta_z_mm = 5.0 if args.delta_z_mm is None else args.delta_z_mm
    args.gripper_position = 0.5 if args.gripper_position is None else args.gripper_position
    return args


def describe_gripper_state(state) -> str:
    """把 pybind GripperState 的关键字段展开为可读文本。"""
    return (f"position={state.position:.3f}, distance_m={state.distance_m:.4f}, "
            f"motor_rad={state.motor_position_rad:.3f}, "
            f"feedback_valid={state.feedback_valid}, age_ms={state.feedback_age_ms:.1f}, "
            f"enabled={state.enabled}, error_code={state.error_code}")


def wait_for_gripper(arm: LumosArm, target: float, timeout: float):
    """保持连接并等待夹爪反馈接近目标，超时后报告最后一次实测状态。

    set_gripper_position 只提交目标，不等待物理夹爪走完。采用 SDK 的
    feedback_valid、反馈年龄和实测开度来判断是否到达；这里的 0.05 是
    demo 显示用的归一化开度容差，不是 SDK 内部位置保护阈值。
    """
    deadline = time.monotonic() + timeout
    while True:
        state = arm.gripper_state()
        if (state.feedback_valid and state.enabled and state.feedback_age_ms <= 100.0
                and abs(state.position - target) <= 0.05):
            return state
        if time.monotonic() >= deadline:
            raise RuntimeError("夹爪在等待时间内未到达目标；最后反馈："
                               + describe_gripper_state(state))
        time.sleep(0.1)


def run_demo(arm: LumosArm, args) -> None:
    """使用已经连接的封装对象执行一次演示动作。

    此函数本身不建立连接，方便使用模拟对象测试预览和执行分支。
    状态读取是一次性的；执行动作后仅再次读取关节位置计算瞬时误差。
    """
    # joint_state 在封装层组合三次 SDK 读取，三组值不是严格同步采样。
    state = arm.joint_state()
    q = state["position_rad"]
    if len(q) != 6:
        raise RuntimeError(f"预期六轴反馈，实际得到 {len(q)} 个值")
    # SDK 通过当前关节反馈计算末端位姿；若反馈无效，位姿也不可信。
    xyz, rpy = arm.ee_pose_euler()
    print("关节位置 rad:", [round(float(value), 4) for value in q])
    print("关节速度 rad/s:", [round(float(value), 4) for value in state["velocity_rad_s"]])
    print("关节力矩 Nm:", [round(float(value), 4) for value in state["torque_nm"]])
    print("末端位置 m:", [round(float(value), 4) for value in xyz])
    print("末端姿态 RPY rad:", [round(float(value), 4) for value in rpy])
    # 公开接口没有六轴反馈有效位。全零不能证明断线，但值得停止进一步运动。
    # 只对 joint/linear 执行分支设置阻断；手工确认真实零位后才允许显式覆盖。
    all_zero = all(abs(float(value)) < 1e-9 for values in state.values()
                   for value in values)
    if all_zero:
        print("注意：六轴位置、速度和力矩均为零。可能是实际零位，也可能尚未收到有效反馈；"
              "请对照实物姿态核对。当前末端位姿和 IK 也可能只是零关节角的计算结果。")
        if args.execute and args.action in ("joint", "linear") and not args.allow_zero_state:
            raise RuntimeError("全零反馈下停止运动；确认实物确在零位后再加 --allow-zero-state")

    if args.action == "status":
        # 只读一次并展示；没有后台循环或持续误差显示。
        return
    if args.action == "ik":
        # 使用当前末端位姿作为 IK 目标、当前关节角作为初值。
        # 成功代表这一组数据自洽，不能独立验证绝对定位精度。
        pos, quat = arm.ee_pose_quat()
        solution, ok = arm.solve_ik(pos, quat, q_seed=q)
        print("当前末端位姿 IK:", "成功" if ok else "失败",
              [round(float(value), 4) for value in solution] if ok else "")
        return

    if args.action == "joint":
        # 以当前实测姿态为起点，只改用户选择的一轴；度转换为 SDK 使用的弧度。
        target = list(q)
        target[args.joint - 1] += math.radians(args.delta_deg)
        # SDK speed_percent=0.1 仍可能为短路径规划出几十毫秒的时长。
        # demo 使用时间模式，按不超过 5 度/秒的平均变化量留出至少 2 秒。
        requested_time = max(2.0, abs(args.delta_deg) / 5.0)
        print("关节目标 rad:", [round(float(value), 4) for value in target])
        print(f"请求关节轨迹时长: {requested_time:.2f} s")
        if args.execute:
            # 路点接口阻塞至轨迹结束；time_sec 是整条路径请求时长。
            duration = arm.joint_waypoints([target], time_sec=requested_time)
            print(f"关节路径完成，规划时长 {duration:.2f} s")
            # 这是结束后单次采样的误差，不是整个运动过程的最大跟随误差。
            print("跟随误差 rad:", [round(value, 4) for value in arm.following_error(target)])
    elif args.action == "linear":
        # 在当前末端位姿基础上仅修改 Z，保持当前 RPY 姿态。
        # Python 列表前 3 项是 xyz（米），因此先将 mm 转为 m。
        target = list(xyz) + list(rpy)
        target[2] += args.delta_z_mm / 1000.0
        requested_time = max(2.0, abs(args.delta_z_mm) / 5.0)
        print("末端目标 [m,rad]:", [round(float(value), 4) for value in target])
        print(f"请求直线路径时长: {requested_time:.2f} s")
        if args.execute:
            # move_l 由 SDK 进行 IK、直线插补与阻塞式执行；目标必须可达。
            duration = arm.move_l([target], time_sec=requested_time)
            print(f"直线路径完成，规划时长 {duration:.2f} s")
    elif args.action == "gripper":
        # 归一化开度：0 为闭合，1 为打开；命令只影响已启用的夹爪。
        print("夹爪目标开度:", args.gripper_position)
        if args.execute:
            # 先记录实测起点；目标可能恰好已到位，便于解释“没有可见运动”。
            print("夹爪起始反馈:", describe_gripper_state(arm.gripper_state()))
            arm.set_gripper_position(args.gripper_position)
            # 保持连接直至夹爪到位或超时，避免下发后立刻 cleanup 使电机失能。
            state = wait_for_gripper(arm, args.gripper_position, args.gripper_timeout)
            print("夹爪到位反馈:", describe_gripper_state(state))

    if not args.execute:
        # status/ik 在前面已返回，此提示只对应可能产生运动命令的动作。
        print("预览完成；加 --execute 才会下发运动目标。")


def main(argv=None) -> int:
    """连接、等待第一批反馈、执行动作，并在所有退出路径释放控制器。"""
    args = parse_args(argv)
    print("连接 StarTouch SDK 将使能电机；预览模式也会连接并使能。")
    # with 在正常完成或 run_demo 抛出异常时都会调用 SDK cleanup。
    with LumosArm.connect(can=args.can, gripper=args.gripper,
                          sdk_root=args.sdk_root, dry_run=args.dry_run) as arm:
        # 原 SDK 初始化后反馈可能尚未更新；此前立即读取曾出现全零。
        if args.startup_wait:
            time.sleep(args.startup_wait)
        run_demo(arm, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
