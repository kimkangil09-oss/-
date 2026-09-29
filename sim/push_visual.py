"""밀기 전략 애니메이션용 keyframe 생성. 우선순위: 실린더 > 키트 > 샘플(시간 남으면).
결과를 JSON으로 저장해서 HTML 애니메이션에서 재생한다.
"""
import json
import math
import field

PUSH_SPEED = 0.15   # m/s, 미는 중(보수적)
MOVE_SPEED = 0.24   # m/s, 빈 이동
START_POS = (0.0, -0.4305)

# 실린더 6개 임시 배정 (실제 색상은 보드 실측 후 교체 필요 - CONFIG 표시)
CYLINDER_JOBS = [
    (field.CYLINDER_POSITIONS["left"][0], "hospital", "red"),
    (field.CYLINDER_POSITIONS["left"][1], "hospital", "red"),
    (field.CYLINDER_POSITIONS["left"][2], "hospital", "red"),
    (field.CYLINDER_POSITIONS["right"][0], "pcc_right", "yellow"),
    (field.CYLINDER_POSITIONS["right"][1], "pcc_left", "yellow"),
    (field.CYLINDER_POSITIONS["right"][2], "starting", "green"),
]
# 키트 4개 (스타팅존에서 출발한다고 가정, 실제 시작좌표는 근사)
KIT_JOBS = [
    ((0.15, -0.50), "hospital", "kit"),
    ((0.05, -0.50), "hospital", "kit"),
    ((-0.05, -0.50), "pcc_left", "kit"),
    ((-0.15, -0.50), "pcc_right", "kit"),
]
# 샘플 3개 (격리구역, 우선순위 최하 - 시간 남을 때만)
SAMPLE_JOBS = [
    ((-0.48, -0.52), "lab", "sample"),
    ((-0.41, -0.43), "lab", "sample"),
    ((-0.34, -0.35), "lab", "sample"),
]

events = []  # {t, x, y, label, state}
items = []   # 정적 아이템 표시용 {x,y,color,id}
t = 0.0
pos = START_POS


def add_event(x, y, label):
    global t
    dx, dy = x - pos[0], y - pos[1]
    dist = math.hypot(dx, dy)
    speed = PUSH_SPEED if label == "push" else MOVE_SPEED
    dur = dist / speed if dist > 1e-6 else 0.3
    events.append({"t0": t, "t1": t + dur, "x0": pos[0], "y0": pos[1], "x1": x, "y1": y, "label": label})
    t += dur
    return x, y


def do_job(slot_xy, zone_key, color, item_id):
    global pos
    cx, cy, w, d = field.ZONES[zone_key]
    tx, ty = slot_xy
    dirx, diry = cx - tx, cy - ty
    n = math.hypot(dirx, diry)
    dirx, diry = dirx / n, diry / n
    approach = 0.12
    bx, by = tx - dirx * approach, ty - diry * approach
    t_arrive = t
    pos = add_event(bx, by, "approach")
    push_dist = approach + n * 0.55  # 구역 중심까지 갈 필요 없이 경계 살짝 안쪽까지만
    fx, fy = bx + dirx * push_dist, by + diry * push_dist
    t_push_start = t
    pos = add_event(fx, fy, "push")
    t_push_end = t
    for it in all_items:
        if it["id"] == item_id:
            it["push_t0"] = t_push_start
            it["push_t1"] = t_push_end
            it["final_x"] = fx
            it["final_y"] = fy


all_items = []
for i, (slot, zone, color) in enumerate(CYLINDER_JOBS):
    all_items.append({"id": f"cyl{i}", "x": slot[0], "y": slot[1], "color": color})
    do_job(slot, zone, color, f"cyl{i}")

CYLINDER_DONE_T = t

for i, (slot, zone, color) in enumerate(KIT_JOBS):
    all_items.append({"id": f"kit{i}", "x": slot[0], "y": slot[1], "color": "#c0392b"})
    do_job(slot, zone, color, f"kit{i}")

KIT_DONE_T = t

for i, (slot, zone, color) in enumerate(SAMPLE_JOBS):
    all_items.append({"id": f"sample{i}", "x": slot[0], "y": slot[1], "color": "#333"})
    do_job(slot, zone, color, f"sample{i}")

TOTAL_T = t

print(f"실린더 완료: {CYLINDER_DONE_T:.1f}s (60점)")
print(f"키트까지 완료: {KIT_DONE_T:.1f}s (+40점=100점)")
print(f"샘플까지 전부: {TOTAL_T:.1f}s (+30점=130점)")
print(f"경기 제한시간: {field.MATCH_TIME_S}s")

data = {
    "events": events,
    "items": all_items,
    "milestones": [
        {"t": CYLINDER_DONE_T, "label": "실린더 완료 (60점)"},
        {"t": KIT_DONE_T, "label": "키트 완료 (100점)"},
        {"t": TOTAL_T, "label": "샘플 완료 (130점)"},
    ],
    "match_time": field.MATCH_TIME_S,
    "zones": {k: list(v) for k, v in field.ZONES.items()},
    "field_w": field.FIELD_W,
    "field_d": field.FIELD_D,
}

with open("push_plan.json", "w") as f:
    json.dump(data, f, ensure_ascii=False, indent=1)
print("written push_plan.json")
