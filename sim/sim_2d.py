"""게임보드 2D 기구학 시뮬레이션. Gazebo/ROS2 없이, 필드 좌표 + 팔 IK만으로
'한자리 처리' 루트가 실제로 도달 가능한지 검증하고 SVG로 렌더링한다.
"""
import math
import field
import arm_model as am

WHEEL_R = 0.03895
DRIVE_SPEED = 0.35   # m/s, 보수적 가정치(실물 미검증)
ARM_CYCLE_S = 3.5    # 물체 1개 pick 또는 place에 걸리는 시간 가정치(인수인계 문서 추정)


def field_to_robot(px, py, rx, ry, theta):
    dx, dy = px - rx, py - ry
    c, s = math.cos(theta), math.sin(theta)
    return dx * c + dy * s, -dx * s + dy * c


def best_parking_spot(targets, z):
    """targets 좌표 리스트를 최대한 많이 IK로 도달 가능하게 하는 (rx,ry,theta) 완전탐색.

    로봇이 향할 수 있는 방향에 특별한 가정을 두지 않고, 군집 주변 x,y 격자 x 360도
    회전을 전수 탐색한다(경기장 밖으로 나가는 위치도 일단 포함, 이후 별도 필터링 가능).
    """
    cx = sum(t[0] for t in targets) / len(targets)
    cy = sum(t[1] for t in targets) / len(targets)

    best = None
    xs = [round(cx - 0.5 + 0.04 * i, 3) for i in range(26)]
    ys = [round(cy - 0.5 + 0.04 * i, 3) for i in range(26)]
    for rx in xs:
        for ry in ys:
            for deg in range(0, 360, 15):
                theta = math.radians(deg)
                hits = []
                for (tx, ty) in targets:
                    lx, ly = field_to_robot(tx, ty, rx, ry, theta)
                    ok, sol = am.reachable(lx, ly, z)
                    hits.append(sol if ok else None)
                n_ok = sum(1 for h in hits if h is not None)
                if best is None or n_ok > best[0]:
                    best = (n_ok, rx, ry, theta, hits)
                if n_ok == len(targets):
                    return best
    return best


def run():
    z_pick = 0.022     # 실린더/샘플: 바닥에서 집기
    z_place = 0.05      # 목적지 구역에 내려놓기 (여유 높이)

    stops = []

    # 1) 격리구역 샘플 3개 (좌표 미상 - 존 중앙 근사, 랜덤배치이므로 대략치)
    iso = field.ZONES["isolation"]
    sample_targets = [(iso[0] - 0.08, iso[1] + 0.06), (iso[0], iso[1]), (iso[0] + 0.08, iso[1] - 0.06)]
    stops.append(("격리구역(샘플 픽업)", sample_targets, z_pick))

    # 2) 실험실 드롭 (3슬롯, 60mm 간격 가정)
    lab = field.ZONES["lab"]
    lab_targets = [(lab[0] - 0.06, lab[1]), (lab[0], lab[1]), (lab[0] + 0.06, lab[1])]
    stops.append(("실험실(샘플 드롭)", lab_targets, z_place))

    # 3) 좌측 실린더 군집 - 필요한 5개만 픽업(9개 중, 색상 위치 불명이라 근사)
    stops.append(("좌측 실린더 군집 픽업", field.CYLINDER_POSITIONS["left"][:5], z_pick))
    # 4) 우측 실린더 군집 - 필요한 4개만 픽업
    stops.append(("우측 실린더 군집 픽업", field.CYLINDER_POSITIONS["right"][:4], z_pick))

    # 5~8) 드롭 존 (병원/PCC/RZ) - 실제 필요 개수만큼(빨강3+키트2, PCC 노랑+키트, 초록3)
    for name, key, n in [("병원 드롭(빨강3+키트2)", "hospital", 5),
                          ("PCC-좌 드롭(노랑2+키트1)", "pcc_left", 3),
                          ("PCC-우 드롭(노랑1+키트1)", "pcc_right", 2),
                          ("회복구역 드롭(초록3)", "starting", 3)]:
        cx, cy, w, d = field.ZONES[key]
        pts = [(cx + (i - (n - 1) / 2) * (w * 0.15), cy) for i in range(n)]
        stops.append((name, pts, z_place))

    print(f"{'구간':28s} {'대상':>4s} {'도달':>4s} {'파킹(x,y,deg)':>28s}")
    total_targets = 0
    total_reachable = 0
    prev_pos = (0.0, -0.4305)  # 스타팅존 근처에서 출발
    total_dist = 0.0
    n_items = 0
    plan = []

    for name, targets, z in stops:
        n_ok, rx, ry, theta, hits = best_parking_spot(targets, z)
        total_targets += len(targets)
        total_reachable += n_ok
        dist = math.hypot(rx - prev_pos[0], ry - prev_pos[1])
        total_dist += dist
        prev_pos = (rx, ry)
        n_items += len(targets)
        deg = math.degrees(theta)
        print(f"{name:28s} {len(targets):>4d} {n_ok:>4d} ({rx:+.3f},{ry:+.3f},{deg:+.0f}°)")
        plan.append((name, rx, ry, theta, targets, hits))

    drive_time = total_dist / DRIVE_SPEED
    arm_time = n_items * ARM_CYCLE_S
    print()
    print(f"도달 가능: {total_reachable}/{total_targets}")
    print(f"파킹 지점 간 이동거리 합: {total_dist:.2f} m  (주행속도 {DRIVE_SPEED} m/s 가정 -> {drive_time:.1f}s)")
    print(f"팔 작업 {n_items}회 x {ARM_CYCLE_S}s = {arm_time:.1f}s")
    print(f"예상 총 소요시간: {drive_time + arm_time:.1f}s / 제한시간 {field.MATCH_TIME_S}s")

    return plan


if __name__ == "__main__":
    plan = run()
    import render_svg
    render_svg.render(plan, "/tmp/claude-0/-home-user--/05dc4e9e-159d-5b47-ab82-0b063d5361cb/scratchpad/field_sim.svg")
