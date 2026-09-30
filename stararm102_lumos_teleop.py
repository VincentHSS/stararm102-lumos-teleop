#!/usr/bin/env python3
"""StarArm 102 leader -> Lumos StarTouch single-arm Python teleoperation.

Run --calibrate first. Calibration records a neutral pose and the leader's
intended joint ranges. It never commands robot motion. Teleoperation is
read-only unless --execute is supplied.

IMPORTANT: This is a starting bridge, not a verified joint mapping. Confirm
joint order and direction with the arms clear and hardware E-stop reachable.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Sequence

LEADER_JOINT_IDS = (0, 1, 2, 3, 4, 5)
CALIBRATION_VERSION = 1

# ===================== 用户可调整的遥操作参数 =====================
# 顺序对应 Lumos 关节 1..6；1=同向，-1=反向。
JOINT_SIGNS = (-1.0, 1.0, 1.0, 1.0, -1.0, -1.0)
FOLLOW_GAIN = 1.0          # 主臂角度变化映射比例，范围 (0, 1]
MAX_STEP_DEG = 1.0         # 每次循环最多追踪的角度，增大后跟随更快
CONTROL_RATE_HZ = 50.0     # 读主臂/更新 Lumos 目标的循环频率
# ================================================================


def parse_joint_map(value: str) -> tuple[int, ...]:
    try:
        ids = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("expected six StarArm servo IDs, e.g. 0,1,2,3,4,5") from exc
    if len(ids) != 6 or len(set(ids)) != 6 or any(item not in LEADER_JOINT_IDS for item in ids):
        raise argparse.ArgumentTypeError("joint map must be six distinct IDs from 0 through 5")
    return ids


def read_leader(leader, joint_map: Sequence[int], include_gripper: bool = False) -> list[float]:
    ids = list(joint_map)
    if include_gripper:
        ids.append(6)
    leader.send_sync_servo_monitor(ids)
    return [float(leader.servos[servo_id].angle_monitor) for servo_id in ids]


def load_sdk():
    try:
        import serial
        import fashionstar_uart_sdk as uservo
        from startouchclass import SingleArm
    except ImportError as exc:
        raise RuntimeError(
            f"缺少 Python SDK：{exc}。请在同一个 Python 环境安装 pyserial、fashionstar-uart-sdk 和 Lumos startouch_sdk。"
        ) from exc
    return serial, uservo, SingleArm


def print_joint_ranges(range_min: Sequence[float], range_max: Sequence[float]) -> None:
    print("\n┌────────┬────────────┬────────────┬────────────┐")
    print("│ 关节   │ 最小角度° │ 最大角度° │ 范围°     │")
    print("├────────┼────────────┼────────────┼────────────┤")
    for index, (low, high) in enumerate(zip(range_min, range_max), start=1):
        print(f"│ J{index:<5} │ {low:>10.2f} │ {high:>10.2f} │ {high-low:>10.2f} │")
    print("└────────┴────────────┴────────────┴────────────┘")


def main() -> int:
    parser = argparse.ArgumentParser(description="StarArm 102 leader -> Lumos arm teleoperation")
    parser.add_argument("--leader-port", default="/dev/ttyUSB0", help="StarArm 102 leader UART port")
    parser.add_argument("--can", default="can0", help="Lumos CAN interface")
    parser.add_argument("--joint-map", type=parse_joint_map, default=(0, 1, 2, 3, 4, 5),
                        help="StarArm servo IDs corresponding to Lumos joints 1..6")
    parser.add_argument("--calibration-file", type=Path,
                        default=Path.home() / ".config" / "stararm102_lumos" / "calibration.json")
    parser.add_argument("--calibrate", action="store_true",
                        help="Record neutral pose and leader ranges, then save calibration")
    parser.add_argument("--gripper", action="store_true",
                        help="Calibrate and control StarArm servo 6 -> Lumos gripper")
    parser.add_argument("--invert-gripper", action="store_true",
                        help="Reverse the calibrated gripper mapping if open/close moves the wrong way")
    parser.add_argument("--execute", action="store_true",
                        help="Send commands to Lumos; without this flag only print proposed targets")
    args = parser.parse_args()

    if len(JOINT_SIGNS) != 6 or any(sign not in (-1.0, 1.0) for sign in JOINT_SIGNS):
        parser.error("JOINT_SIGNS must contain six values, each 1.0 or -1.0")
    if not 0 < FOLLOW_GAIN <= 1 or MAX_STEP_DEG <= 0 or not 0 < CONTROL_RATE_HZ <= 100:
        parser.error("Check FOLLOW_GAIN, MAX_STEP_DEG, and CONTROL_RATE_HZ at the top of the script")

    try:
        serial, uservo, SingleArm = load_sdk()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    leader_uart = None
    lumos = None
    try:
        leader_uart = serial.Serial(
            port=args.leader_port,
            baudrate=1_000_000,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0,
        )
        leader = uservo.UartServoManager(leader_uart)
        if args.calibrate:
            print("\n╔══════════════════════════════════════════════════════╗")
            print("║       StarArm 102 → Lumos  校准向导                 ║")
            print("╚══════════════════════════════════════════════════════╝")
            print("请先将 StarArm102 与 Lumos 摆到对应的遥操作参考姿态。")
            print("Lumos 连接时会使能电机；确认急停可用、工作区域畅通。")
            input("准备好后按 Enter 连接并记录参考姿态……")
        lumos = SingleArm(can_interface_=args.can, gripper=args.gripper, enable_fd_=False)
        time.sleep(2.0)

        if args.calibrate:
            print("\n[1/3] 参考姿态已记录")
            leader_home_and_gripper = read_leader(leader, args.joint_map, args.gripper)
            leader_home_deg = leader_home_and_gripper[:6]
            lumos_home_rad = [float(value) for value in lumos.get_joint_positions()]
            if len(lumos_home_rad) != 6:
                raise RuntimeError(f"Lumos 应返回 6 个关节位置，实际返回 {len(lumos_home_rad)} 个")

            print("\n[2/3] 记录主臂关节范围")
            print("缓慢移动六个关节，覆盖实际遥操作会用到的范围；完成后按 Enter 停止。")
            range_min = [float("inf")] * 6
            range_max = [float("-inf")] * 6

            stop_recording = threading.Event()

            def wait_for_enter() -> None:
                input("按 Enter 停止范围记录……")
                stop_recording.set()

            threading.Thread(target=wait_for_enter, daemon=True).start()
            samples = 0
            last_report = time.monotonic()
            while not stop_recording.is_set():
                values = read_leader(leader, args.joint_map)
                range_min = [min(old, value) for old, value in zip(range_min, values)]
                range_max = [max(old, value) for old, value in zip(range_max, values)]
                samples += 1
                if time.monotonic() - last_report >= 1.0:
                    summary = "  ".join(f"J{i+1}:{lo:.0f}..{hi:.0f}°" for i, (lo, hi) in enumerate(zip(range_min, range_max)))
                    print(f"\r采集中 {samples:>5} 帧  {summary:<90}", end="", flush=True)
                    last_report = time.monotonic()
                time.sleep(1.0 / CONTROL_RATE_HZ)
            if any(hi - lo < 2.0 for lo, hi in zip(range_min, range_max)):
                raise RuntimeError("至少一个主臂关节记录范围不足 2 度；请重新校准并移动六个关节。")
            print("\n\n[3/3] 记录结果")
            print_joint_ranges(range_min, range_max)

            gripper_range = None
            if args.gripper:
                print("\n夹爪范围")
                input("把 StarArm 夹爪摆到完全闭合位置，按 Enter 记录……")
                closed = read_leader(leader, args.joint_map, True)[-1]
                input("把 StarArm 夹爪摆到完全打开位置，按 Enter 记录……")
                opened = read_leader(leader, args.joint_map, True)[-1]
                if abs(opened - closed) < 2.0:
                    raise RuntimeError("夹爪记录范围不足 2 度；请重新校准。")
                gripper_range = {"closed_deg": closed, "open_deg": opened}

            calibration = {
                "format_version": CALIBRATION_VERSION,
                "leader_port": args.leader_port,
                "can_interface": args.can,
                "joint_map": list(args.joint_map),
                "signs": list(JOINT_SIGNS),
                "gain": FOLLOW_GAIN,
                "max_step_deg": MAX_STEP_DEG,
                "leader_home_deg": leader_home_deg,
                "leader_range_min_deg": range_min,
                "leader_range_max_deg": range_max,
                "lumos_home_rad": lumos_home_rad,
                "gripper_enabled": args.gripper,
                "gripper_range": gripper_range,
            }
            args.calibration_file.parent.mkdir(parents=True, exist_ok=True)
            args.calibration_file.write_text(json.dumps(calibration, indent=2), encoding="utf-8")
            print(f"\n校准完成，已保存到：{args.calibration_file}")
            print("校准完成。先预览检查方向，确认后再使用 --execute。")
            return 0

        if not args.calibration_file.is_file():
            raise RuntimeError(f"没有校准文件：{args.calibration_file}。先加 --calibrate 运行校准。")
        calibration = json.loads(args.calibration_file.read_text(encoding="utf-8"))
        if calibration.get("format_version") != CALIBRATION_VERSION:
            raise RuntimeError("校准文件版本不兼容，请重新运行 --calibrate。")
        if calibration["leader_port"] != args.leader_port or calibration["can_interface"] != args.can:
            raise RuntimeError("端口与校准文件不一致，请使用原端口或重新校准。")
        if calibration["joint_map"] != list(args.joint_map):
            raise RuntimeError("--joint-map 与校准文件不一致，请重新校准。")
        if bool(calibration["gripper_enabled"]) != args.gripper:
            raise RuntimeError("夹爪设置与校准文件不一致；请使用相同的 --gripper 设置。")

        leader_home_deg = calibration["leader_home_deg"]
        leader_min_deg = calibration["leader_range_min_deg"]
        leader_max_deg = calibration["leader_range_max_deg"]
        lumos_home_rad = calibration["lumos_home_rad"]
        # Motion parameters are intentionally taken from the code constants above,
        # not the old calibration JSON, so edits here always take effect.
        signs = JOINT_SIGNS
        gain = FOLLOW_GAIN
        max_step_rad = math.radians(MAX_STEP_DEG)
        gripper_range = calibration["gripper_range"]
        previous_target = [float(value) for value in lumos.get_joint_positions()]
        if len(previous_target) != 6:
            raise RuntimeError(f"Lumos 应返回 6 个关节位置，实际返回 {len(previous_target)} 个")
        home_error = [abs(now - calibrated) for now, calibrated in zip(previous_target, lumos_home_rad)]
        if args.execute and any(error > 0.15 for error in home_error):
            raise RuntimeError(
                "Lumos 当前姿态与校准参考姿态相差超过 0.15 rad。请摆回参考姿态或重新校准。"
            )

        print("EXECUTE: 正在控制 Lumos" if args.execute else "预览模式：只显示目标，不驱动 Lumos")
        print(f"校准文件：{args.calibration_file}")
        print(f"固定参数：signs={list(JOINT_SIGNS)}, gain={FOLLOW_GAIN}, max_step={MAX_STEP_DEG}°/周期, rate={CONTROL_RATE_HZ}Hz")
        print("Lumos 参考姿态 (rad)：", [round(v, 3) for v in lumos_home_rad])
        print("工作空间保持畅通；Ctrl+C 停止。")

        period = 1.0 / CONTROL_RATE_HZ
        next_tick = time.monotonic()
        while True:
            feedback = read_leader(leader, args.joint_map, args.gripper)
            leader_deg = feedback[:6]
            clipped = [max(lo, min(hi, value))
                       for value, lo, hi in zip(leader_deg, leader_min_deg, leader_max_deg)]
            delta_rad = [math.radians(now - home) for now, home in zip(clipped, leader_home_deg)]
            desired = [home + sign * gain * delta
                       for home, sign, delta in zip(lumos_home_rad, signs, delta_rad)]
            target = [old + max(-max_step_rad, min(max_step_rad, new - old))
                      for old, new in zip(previous_target, desired)]

            if args.execute:
                lumos.set_joint_raw(target, [0.0] * 6)
            previous_target = target

            line = f"leader_deg={[round(v, 1) for v in leader_deg]}"
            if args.gripper:
                g = feedback[-1]
                closed = float(gripper_range["closed_deg"])
                opened = float(gripper_range["open_deg"])
                grip_ratio = max(0.0, min(1.0, (g - closed) / (opened - closed)))
                if args.invert_gripper:
                    grip_ratio = 1.0 - grip_ratio
                line += f" gripper={grip_ratio:.2f}"
                if args.execute:
                    lumos.setGripperPosition(grip_ratio)
            print(line, end="\r", flush=True)

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\n遥操作已停止。")
        return 0
    except Exception as exc:
        print(f"\n运行失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if lumos is not None:
            try:
                lumos.cleanup()
            except Exception as exc:
                print(f"Lumos cleanup warning: {exc}", file=sys.stderr)
        if leader_uart is not None and leader_uart.is_open:
            leader_uart.close()


if __name__ == "__main__":
    raise SystemExit(main())
