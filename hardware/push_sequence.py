"""오픈루프 '밀기' 미션 시퀀스. 팔 없이 로봇 차체로 실린더/키트를 밀어서 구역에 넣는다.

전략: 메카넘 휠은 회전 없이 병진만으로 전방향 이동 가능 -> 로봇 헤딩을 경기 내내
고정하고, 순수 (dx,dy) 병진 이동만으로 시퀀스를 구성한다(헤딩 드리프트 원천 차단).

우선순위: 실린더(90점) > 키트(40점) > 샘플(30점, 시간 남으면만).

※ CONFIG 항목 중 "확인 필요" 표시는 실제 보드를 보고 채워야 하는 값입니다.
   (어느 슬롯이 무슨 색인지는 도면 텍스트만으로 확정 못 했습니다.)
"""
import time
import sys
sys.path.insert(0, "../sim")
import field
from motor_driver import MecanumDriver

PUSH_SPEED = 0.5       # 정규화 속도(0~1), 밀 때는 살짝 느리게 - 미끄러짐 방지
APPROACH_SPEED = 0.8   # 빈 공간 이동 시 빠르게
EST_SPEED_MPS = {       # MecanumDriver.estimate_speed_mps 참고용 대략치. 실측 권장.
    PUSH_SPEED: 0.15,
    APPROACH_SPEED: 0.24,
}

# ── CONFIG: 확인 필요 ─────────────────────────────────────────────
# 각 실린더 슬롯의 (x,y)는 sim/field.py의 근사치. 실제 색상 배치는 보드 실측 필요.
# 아래는 "임시 배정" 예시이니, 본인 보드 사진 보고 색상별로 좌표를 다시 넣으세요.
CYLINDER_JOBS = [
    # (slot_xy, target_zone_key, 밀 거리 여유(m))
    (field.CYLINDER_POSITIONS["left"][0], "hospital", 0.05),
    (field.CYLINDER_POSITIONS["left"][1], "hospital", 0.05),
    (field.CYLINDER_POSITIONS["left"][2], "hospital", 0.05),
    (field.CYLINDER_POSITIONS["right"][0], "pcc_right", 0.05),
    (field.CYLINDER_POSITIONS["right"][1], "pcc_left", 0.05),
    (field.CYLINDER_POSITIONS["right"][2], "starting", 0.05),
]
# ─────────────────────────────────────────────────────────────────

START_POS = (0.0, -0.4305)  # 스타팅존 중앙 근사


def push_job(driver, from_xy, to_zone_key, robot_pos, margin):
    """from_xy에 있는 물체를 to_zone_key 구역 안쪽까지 미는 시퀀스.

    로봇은 물체 뒤(목표 반대쪽)로 접근 -> 목표 방향으로 밀기 -> 그 자리에서 정지.
    (뒤로 빼는 동작은 넣지 않음: 규정상 경기 종료 시점만 분리돼 있으면 되므로,
    남은 시간에 다음 작업으로 이동하는 것 자체가 자연스러운 분리가 됨.
    단, 마지막 물체는 반드시 후진해서 분리시켜야 함 - 아래 참고)
    """
    cx, cy, w, d = field.ZONES[to_zone_key]
    tx, ty = from_xy
    # 목표 구역 '중심 방향'으로 밀 벡터 계산
    dirx, diry = cx - tx, cy - ty
    n = (dirx ** 2 + diry ** 2) ** 0.5
    dirx, diry = dirx / n, diry / n

    approach_dist = 0.12  # 접근 여유 (물체 바로 뒤 12cm에서 시작)
    behind_x = tx - dirx * approach_dist
    behind_y = ty - diry * approach_dist

    # 1) 현재 위치 -> 물체 뒤로 이동 (빈 공간 이동, 빠르게)
    move(driver, robot_pos, (behind_x, behind_y), APPROACH_SPEED)

    # 2) 물체를 밀어서 구역 안쪽까지: 접근거리 + 목표 경계까지 거리 + margin
    push_dist = approach_dist + n + margin
    push_vec = (dirx, diry)
    dist_move(driver, push_vec, push_dist, PUSH_SPEED)

    return (behind_x + dirx * push_dist, behind_y + diry * push_dist)


def move(driver, from_xy, to_xy, speed_frac):
    dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
    dist = (dx ** 2 + dy ** 2) ** 0.5
    if dist < 1e-6:
        return
    dist_move(driver, (dx / dist, dy / dist), dist, speed_frac)


def dist_move(driver, unit_vec, dist, speed_frac):
    speed = EST_SPEED_MPS.get(speed_frac, 0.15)
    t = dist / speed
    print(f"  drive vec=({unit_vec[0]:+.2f},{unit_vec[1]:+.2f}) dist={dist:.3f}m speed={speed_frac} t={t:.2f}s")
    driver.drive_time(vx=unit_vec[1], vy=unit_vec[0], omega=0, seconds=t)
    # 주의: 로봇 body-frame(vx=전진,vy=우측횡이동)과 필드 frame(x,y) 매핑은
    #       로봇의 고정 헤딩에 따라 달라짐. 여기서는 로봇 '전진'=필드 +y, '우횡이동'=필드 +x로 가정.
    #       실제 로봇을 필드에 올렸을 때 어느 쪽이 전진인지 반드시 확인 후 부호 맞출 것.


def run(dry_run=True):
    driver = None if dry_run else MecanumDriver()
    pos = START_POS
    total_score = 0

    print("=== 실린더 밀기 (최우선) ===")
    for slot_xy, zone_key, margin in CYLINDER_JOBS:
        print(f"{slot_xy} -> {zone_key}")
        if not dry_run:
            pos = push_job(driver, slot_xy, zone_key, pos, margin)
        total_score += 10

    print(f"\n실린더 완료 예상 점수: {total_score}")
    print("남은 시간 있으면 샘플(격리->실험실) 진행, 없으면 여기서 '종료' 선언")

    if not dry_run:
        driver.close()


if __name__ == "__main__":
    run(dry_run=True)
