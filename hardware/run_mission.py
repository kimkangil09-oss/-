"""2026 로보틱스 챌린지 - 실전 미션 스크립트 (단일 파일, 그대로 복붙해서 실행).

라즈베리파이에서 직접 실행. 메카넘 휠로 회전 없이 밀어서(push) 미션 수행.
실행 전 CONFIG 섹션을 반드시 실제 보드에 맞게 확인/수정할 것.
"""
import time
import sys

try:
    from smbus2 import SMBus
except ImportError:
    print("smbus2가 없습니다. 터미널에서 먼저: pip3 install smbus2")
    sys.exit(1)

# ══════════════════════════════════════════════════════════════════
# CONFIG - 실행 전 반드시 확인/수정
# ══════════════════════════════════════════════════════════════════

TEST_MODE = True
# ↑ 처음엔 True로 실행! 로봇을 바닥에 살짝 띄우거나 빈 공간에 놓고
#   돌려서 4바퀴가 올바르게(전진 시 다 같은 방향으로) 도는지만 확인.
#   확인되면 False로 바꿔서 실제 미션 실행.

# 실린더 6개: (필드 중심 기준 x,y[m], 목적지) - *** 실제 보드 색 배치로 꼭 교체 ***
# 목적지: "hospital"(빨강), "pcc_left"/"pcc_right"(노랑), "starting"(초록=회복구역)
CYLINDER_JOBS = [
    ((-0.15, 0.2105), "hospital"),
    ((-0.25, 0.2105), "hospital"),
    ((-0.35, 0.2105), "hospital"),
    ((0.15, 0.2105), "pcc_right"),
    ((0.25, 0.2105), "pcc_left"),
    ((0.35, 0.2105), "starting"),
]

# 키트 4개: 스타팅존 안 대략 위치 -> 목적지 (병원 2개, PCC 각 1개)
KIT_JOBS = [
    ((0.15, -0.50), "hospital"),
    ((0.05, -0.50), "hospital"),
    ((-0.05, -0.50), "pcc_left"),
    ((-0.15, -0.50), "pcc_right"),
]

# 샘플 3개 (격리구역->실험실): 우선순위 최하, 시간 없으면 이 리스트를 비워도 됨([])
SAMPLE_JOBS = [
    ((-0.48, -0.52),),
    ((-0.41, -0.43),),
    ((-0.34, -0.35),),
]

ZONES = {  # (center_x, center_y, width, depth) [m] - 공식 룰북 실측치, 수정 불필요
    "pcc_left":  (-0.4015,  0.5005, 0.300, 0.180),
    "hospital":  ( 0.0000,  0.5005, 0.503, 0.180),
    "pcc_right": ( 0.4015,  0.5005, 0.300, 0.180),
    "lab":       (-0.0990, -0.4955, 0.345, 0.150),
    "starting":  ( 0.3135, -0.4305, 0.480, 0.280),
}
LAB_TARGET = "lab"
START_POS = (0.0, -0.4305)  # 로봇을 여기 놓고 시작한다고 가정 (스타팅존 내부 기준점)

PUSH_SPEED_MPS = 0.15    # 밀 때 속도 추정치(이론값) - 실측하면 더 정확해짐
MOVE_SPEED_MPS = 0.24    # 빈 공간 이동 속도 추정치
APPROACH_MARGIN = 0.12   # 물체 뒤에서 접근 시작하는 여유거리(m)
PUSH_OVERSHOOT = 0.55    # 목적지까지 거리의 몇 %만큼 더 밀지 (여유있게 경계 안쪽까지)

# ══════════════════════════════════════════════════════════════════
# 모터 드라이버 (car_move_demo.ino 레지스터맵 그대로 포팅, 수정 불필요)
# ══════════════════════════════════════════════════════════════════

I2C_ADDR = 0x34
MOTOR_TYPE_ADDR = 20
MOTOR_ENCODER_POLARITY_ADDR = 21
MOTOR_FIXED_SPEED_ADDR = 51
MOTOR_TYPE_JGB = 3
MIX_A = (1, -1, -1, 1)
MIX_B = (1, 1, 1, 1)
MAX_SPEED_UNIT = 50


class MecanumDriver:
    def __init__(self, bus_num=1):
        self.bus = SMBus(bus_num)
        time.sleep(0.05)
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_TYPE_ADDR, [MOTOR_TYPE_JGB])
        time.sleep(0.005)
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_ENCODER_POLARITY_ADDR, [0])
        time.sleep(0.1)

    def _to_i8(self, v):
        return max(-128, min(127, int(round(v)))) & 0xFF

    def set_body_velocity(self, vx, vy):
        speeds = [self._to_i8((a * vx + b * vy) * MAX_SPEED_UNIT) for a, b in zip(MIX_A, MIX_B)]
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, speeds)

    def stop(self):
        self.bus.write_i2c_block_data(I2C_ADDR, MOTOR_FIXED_SPEED_ADDR, [0, 0, 0, 0])

    def drive_time(self, vx, vy, seconds):
        self.set_body_velocity(vx, vy)
        time.sleep(seconds)
        self.stop()

    def close(self):
        self.stop()
        self.bus.close()


# ══════════════════════════════════════════════════════════════════
# 밀기 시퀀스 로직 (수정 불필요)
# ══════════════════════════════════════════════════════════════════

def move(driver, from_xy, to_xy, speed_mps):
    dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
    dist = (dx ** 2 + dy ** 2) ** 0.5
    if dist < 1e-6:
        return
    ux, uy = dx / dist, dy / dist
    t = dist / speed_mps
    print(f"  이동: ({from_xy[0]:.2f},{from_xy[1]:.2f}) -> ({to_xy[0]:.2f},{to_xy[1]:.2f})  {t:.1f}s")
    # 로봇 '전진'(vx)=필드 +y, '우횡이동'(vy)=필드 +x 로 가정 (헤딩 고정, 회전 없음)
    driver.drive_time(vx=uy, vy=ux, seconds=t)


def push_job(driver, pos, slot_xy, zone_key):
    cx, cy, w, d = ZONES[zone_key]
    tx, ty = slot_xy
    dirx, diry = cx - tx, cy - ty
    n = (dirx ** 2 + diry ** 2) ** 0.5
    dirx, diry = dirx / n, diry / n
    bx, by = tx - dirx * APPROACH_MARGIN, ty - diry * APPROACH_MARGIN
    move(driver, pos, (bx, by), MOVE_SPEED_MPS)
    push_dist = APPROACH_MARGIN + n * PUSH_OVERSHOOT
    fx, fy = bx + dirx * push_dist, by + diry * push_dist
    move(driver, (bx, by), (fx, fy), PUSH_SPEED_MPS)
    return (fx, fy)


def run_test_mode(driver):
    print("TEST_MODE: 4바퀴 확인용 짧은 이동 (전진 1s -> 정지 2s -> 우측이동 1s -> 정지)")
    driver.drive_time(vx=1, vy=0, seconds=1.0)
    time.sleep(2.0)
    driver.drive_time(vx=0, vy=1, seconds=1.0)
    print("테스트 끝. 로봇이 먼저 앞으로, 그 다음 오른쪽으로 갔으면 정상.")
    print("방향이 다르면 CONFIG 아래 move()의 vx/vy 부호를 바꿔서 다시 테스트하세요.")


def run_mission(driver):
    pos = START_POS
    score = 0

    print("\n=== 1순위: 실린더 (90점) ===")
    for slot, zone in CYLINDER_JOBS:
        pos = push_job(driver, pos, slot, zone)
        score += 10
        print(f"  완료, 누적 {score}점")

    print("\n=== 2순위: 의료 키트 (40점) ===")
    for slot, zone in KIT_JOBS:
        pos = push_job(driver, pos, slot, zone)
        score += 10
        print(f"  완료, 누적 {score}점")

    print("\n=== 3순위: 샘플 (시간 남으면, 30점) ===")
    for (slot,) in SAMPLE_JOBS:
        pos = push_job(driver, pos, slot, LAB_TARGET)
        score += 10
        print(f"  완료, 누적 {score}점")

    print(f"\n총 예상 점수: {score} / 160")


if __name__ == "__main__":
    driver = MecanumDriver()
    try:
        if TEST_MODE:
            run_test_mode(driver)
        else:
            run_mission(driver)
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 긴급 정지")
    finally:
        driver.close()
