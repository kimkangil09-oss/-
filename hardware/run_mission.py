#!/usr/bin/env python3
"""2026 로보틱스 챌린지 - 실전 미션 스크립트 (TurboPi 실제 SDK + 실측 보정값 사용).

라즈베리파이에서 직접 실행. 회전 없이 병진 이동만으로 밀기(push) 미션 수행.
모터 믹싱 공식은 실물 개별모터 테스트로 검증됨(직진/횡이동 확인 완료).
"""
import time
import math
from turbopi import RobotConfig, TurboPi

# ══════════════════════════════════════════════════════════════════
# 실측 보정값 - 이 로봇에서 직접 측정한 값
# ══════════════════════════════════════════════════════════════════
MM_PER_DUTY_PER_SEC = 460 / 2 / 40   # duty 40으로 2초 이동 -> 460mm 실측
MAX_DUTY = 60                          # 안전 상한

# 검증된 모터 믹싱 (모터ID 1~4, 물리 위치: 1=좌앞,2=우앞,3=좌뒤,4=우뒤)
# duty[1]=-vy-vx  duty[2]=vy-vx  duty[3]=-vy+vx  duty[4]=vy+vx
# (vx=우측횡이동 성분, vy=전진 성분, 둘 다 이 로봇에서 실측 검증됨)


def translate(robot, dx_mm, dy_mm, duty=40):
    """dx_mm: 오른쪽(+)/왼쪽(-), dy_mm: 전진(+)/후진(-). 회전 없이 순수 이동."""
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
    print(f"  이동: dx={dx_mm:+.0f}mm dy={dy_mm:+.0f}mm dist={dist:.0f}mm t={t:.2f}s")
    robot.motor.set_duty([(i + 1, d) for i, d in enumerate(duties)])
    time.sleep(t)
    stop(robot)


def stop(robot):
    robot.motor.set_duty([(1, 0), (2, 0), (3, 0), (4, 0)])


# ══════════════════════════════════════════════════════════════════
# CONFIG - 실행 전 반드시 실제 보드 색 배치로 확인/수정
# ══════════════════════════════════════════════════════════════════
TEST_MODE = False  # 이미 개별 모터/직진/횡이동 테스트 끝났으면 False

ZONES_MM = {  # (center_x, center_y, width, depth) - 공식 룰북 실측치, mm
    "pcc_left":  (-401.5,  500.5, 300, 180),
    "hospital":  (   0.0,  500.5, 503, 180),
    "pcc_right": ( 401.5,  500.5, 300, 180),
    "lab":       ( -99.0, -495.5, 345, 150),
    "starting":  ( 313.5, -430.5, 480, 280),
}

# 실린더 6개: (x,y)[mm], 목적지 - *** 실제 보드 색 배치로 꼭 교체 ***
CYLINDER_JOBS = [
    ((-150, 210.5), "hospital"),
    ((-250, 210.5), "hospital"),
    ((-350, 210.5), "hospital"),
    (( 150, 210.5), "pcc_right"),
    (( 250, 210.5), "pcc_left"),
    (( 350, 210.5), "starting"),
]

KIT_JOBS = [
    (( 150, -500), "hospital"),
    (( 50,  -500), "hospital"),
    ((-50,  -500), "pcc_left"),
    ((-150, -500), "pcc_right"),
]

SAMPLE_JOBS = [
    ((-480, -520),),
    ((-410, -430),),
    ((-340, -350),),
]
LAB_TARGET = "lab"
START_POS = (0.0, -430.5)

APPROACH_MARGIN = 120   # 물체 뒤 접근 여유 거리(mm)
PUSH_OVERSHOOT = 0.55   # 목적지 경계까지 거리의 몇 %만큼 더 밀지


def push_job(robot, pos, slot_xy, zone_key):
    cx, cy, w, d = ZONES_MM[zone_key]
    tx, ty = slot_xy
    dirx, diry = cx - tx, cy - ty
    n = math.hypot(dirx, diry)
    dirx, diry = dirx / n, diry / n
    bx, by = tx - dirx * APPROACH_MARGIN, ty - diry * APPROACH_MARGIN
    translate(robot, bx - pos[0], by - pos[1])
    push_dist = APPROACH_MARGIN + n * PUSH_OVERSHOOT
    fx, fy = bx + dirx * push_dist, by + diry * push_dist
    translate(robot, fx - bx, fy - by)
    return (fx, fy)


def run_test_mode(robot):
    print("TEST_MODE: 전진 -> 정지 -> 우측이동")
    translate(robot, 0, 200)
    time.sleep(1)
    translate(robot, 200, 0)
    print("테스트 끝. 문제없으면 TEST_MODE=False로 바꿔서 재실행하세요.")


def run_mission(robot):
    pos = START_POS
    score = 0

    print("\n=== 1순위: 실린더 (90점) ===")
    for slot, zone in CYLINDER_JOBS:
        pos = push_job(robot, pos, slot, zone)
        score += 10
        print(f"  완료, 누적 {score}점")

    print("\n=== 2순위: 의료 키트 (40점) ===")
    for slot, zone in KIT_JOBS:
        pos = push_job(robot, pos, slot, zone)
        score += 10
        print(f"  완료, 누적 {score}점")

    print("\n=== 3순위: 샘플 (시간 남으면, 30점) ===")
    for (slot,) in SAMPLE_JOBS:
        pos = push_job(robot, pos, slot, LAB_TARGET)
        score += 10
        print(f"  완료, 누적 {score}점")

    print(f"\n총 예상 점수: {score} / 160")


if __name__ == "__main__":
    config = RobotConfig(device="/dev/ttyAMA0", i2c_bus=1)
    robot = TurboPi.connect(config)
    try:
        if TEST_MODE:
            run_test_mode(robot)
        else:
            run_mission(robot)
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 긴급 정지")
    finally:
        stop(robot)
        robot.close()
