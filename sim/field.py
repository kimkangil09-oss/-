"""2026 로보틱스 챌린지 예선 필드 정의 (한국과학창의재단 공식 룰북 기준, 시니어 카테고리).

좌표계: 필드 중심이 원점. x는 오른쪽(+), y는 병원/PCC 쪽(+), 격리/실험실/스타팅존 쪽(-).
단위: m. 출처: 2026_로보틱스챌린지_문제해결트랙_안내.pdf p.10-19 (필드 실측 도면 A/B/C/D).
"""

FIELD_W = 1.143
FIELD_D = 1.181

# (center_x, center_y, width, depth) - 전부 공식 도면 치수로 검증됨
ZONES = {
    "pcc_left":  (-0.4015,  0.5005, 0.300, 0.180),
    "hospital":  ( 0.0000,  0.5005, 0.503, 0.180),
    "pcc_right": ( 0.4015,  0.5005, 0.300, 0.180),
    "isolation": (-0.4115, -0.4305, 0.280, 0.280),
    "lab":       (-0.0990, -0.4955, 0.345, 0.150),
    "starting":  ( 0.3135, -0.4305, 0.480, 0.280),  # 회복구역(RZ)과 동일 구역
}

# 실린더 좌표: 룰북 도면(150/100/100mm 열 간격, 450/200mm 행 간격) 기반.
# 열 간격(COL_X)은 확정, 행 위치(ROW_Y)는 기준벽 불명확 - 근사치.
COL_X = (0.150, 0.250, 0.350)   # 격리 H월 중심선에서 각 열까지 거리
ROW_Y = (0.2105, -0.0395)       # 두 행의 y좌표 (근사)

def _cluster(sign):
    return [(sign * cx, ry) for ry in ROW_Y for cx in COL_X]

CYLINDER_POSITIONS = {
    "left": _cluster(-1),   # 6개 (병원/PCC-left, 격리 쪽 절반)
    "right": _cluster(+1),  # 6개
}

# 시니어 예선 과제 (안내서 p.12) - 색상별 옮길 개수는 팀 자유 선택(4개 중 3개)
SCORING = {
    "sample_to_lab": {"count": 3, "points_each": 10},
    "kit_hospital": {"count": 2, "points_each": 10},
    "kit_pcc": {"count": 2, "points_each": 10},   # PCC 좌/우 각 1
    "red_to_hospital": {"count": 3, "points_each": 10},
    "yellow_to_pcc": {"count": 3, "points_each": 10},   # 각 PCC 최소 1
    "green_to_recovery": {"count": 3, "points_each": 10},
}
MAX_SCORE = sum(v["count"] * v["points_each"] for v in SCORING.values())  # 160

OUT_OF_FIELD_PENALTY = -10
MATCH_TIME_S = 120
