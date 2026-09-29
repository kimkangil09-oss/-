#!/usr/bin/env python3
"""사용법: python3 move.py <dx_mm> <dy_mm>
dx: 오른쪽(+)/왼쪽(-), dy: 전진(+)/후진(-)
"""
import sys
import time
import math
from turbopi import RobotConfig, TurboPi

MM_PER_DUTY_PER_SEC = 460 / 2 / 40
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


def stop(robot):
    robot.motor.set_duty([(1, 0), (2, 0), (3, 0), (4, 0)])


if __name__ == "__main__":
    dx = float(sys.argv[1])
    dy = float(sys.argv[2])
    config = RobotConfig(device="/dev/ttyAMA0", i2c_bus=1)
    robot = TurboPi.connect(config)
    try:
        translate(robot, dx, dy)
    except KeyboardInterrupt:
        print("긴급 정지")
    finally:
        stop(robot)
        robot.close()
