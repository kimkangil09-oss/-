#!/usr/bin/env python3
"""사용법:
  python3 move.py fwd <mm>       전진(양수)/후진(음수) mm
  python3 move.py right <mm>     오른쪽(양수)/왼쪽(음수) mm
  python3 move.py rotate <deg>   시계방향(양수)/반시계(음수) 회전 각도
  python3 move.py seq            아래 SEQUENCE 순서대로 전부 실행
"""
import sys
import time
import math
from turbopi import RobotConfig, TurboPi

MM_PER_DUTY_PER_SEC = 460 / 2 / 40   # 실측: duty40, 2초 -> 460mm
DEG_PER_SEC_AT_DUTY40 = 90.0          # ※ 아직 미실측 추정치! rotate로 먼저 보정할 것
MAX_DUTY = 60


def translate(robot, dx_mm, dy_mm, duty=40):
    dist = math.hypot(dx_mm, dy_mm)
    if dist < 1:
        return
    ux, uy = dx_mm / dist, dy_mm / dist
    vx, vy = duty * ux, duty * uy
    d1, d2, d3, d4 = -vy - vx, vy - vx, -vy + vx, vy + vx
    m = max(abs(d1), abs(d2), abs(d3), abs(d4), 1)
    scale = min(1.0, MAX_DUTY / m)
    duties = [d1 * scale, d2 * scale, d3 * scale, d4 * scale]
    speed_mms = duty * scale * MM_PER_DUTY_PER_SEC
    t = dist / speed_mms
    print(f"이동: dx={dx_mm:+.0f}mm dy={dy_mm:+.0f}mm dist={dist:.0f}mm t={t:.2f}s")
    robot.motor.set_duty([(i + 1, d) for i, d in enumerate(duties)])
    time.sleep(t)
    stop(robot)


def rotate(robot, degrees, duty=40):
    """degrees>0: 시계방향, degrees<0: 반시계방향. 전부 같은 duty -> 제자리 회전."""
    t = abs(degrees) / DEG_PER_SEC_AT_DUTY40
    d = duty if degrees > 0 else -duty
    print(f"회전: {degrees:+.0f}도 t={t:.2f}s (보정값 미확정 - 실측 필요)")
    robot.motor.set_duty([(1, d), (2, d), (3, d), (4, d)])
    time.sleep(t)
    stop(robot)


def stop(robot):
    robot.motor.set_duty([(1, 0), (2, 0), (3, 0), (4, 0)])


SEQUENCE = [
    ("fwd", 3 * 40 * MM_PER_DUTY_PER_SEC),      # 전진 3초 분량
    ("right", 1 * 40 * MM_PER_DUTY_PER_SEC),    # 오른쪽 1초 분량
    ("rotate", 180),
    ("fwd", 3 * 40 * MM_PER_DUTY_PER_SEC),
]


def run_sequence(robot):
    for kind, val in SEQUENCE:
        if kind == "fwd":
            translate(robot, 0, val)
        elif kind == "right":
            translate(robot, val, 0)
        elif kind == "rotate":
            rotate(robot, val)
        time.sleep(0.3)


if __name__ == "__main__":
    config = RobotConfig(device="/dev/ttyAMA0", i2c_bus=1)
    robot = TurboPi.connect(config)
    try:
        cmd = sys.argv[1]
        if cmd == "fwd":
            translate(robot, 0, float(sys.argv[2]))
        elif cmd == "right":
            translate(robot, float(sys.argv[2]), 0)
        elif cmd == "rotate":
            rotate(robot, float(sys.argv[2]))
        elif cmd == "seq":
            run_sequence(robot)
    except KeyboardInterrupt:
        print("긴급 정지")
    finally:
        stop(robot)
        robot.close()
