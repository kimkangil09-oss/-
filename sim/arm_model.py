"""로봇팔 순수 기구학 모델 (IK/FK). base_link = 바퀴중심·지면 기준, 단위 m, rad.

관절/링크 치수는 조립 STEP 실측값(인수인계 문서 기준).
집게는 항상 수직 하향 자세를 유지한다고 가정(수직 파지 전용 IK).
"""
import math

SHOULDER_X, SHOULDER_Y, SHOULDER_Z = -0.121, 0.010, 0.1219  # J2 위치
L2 = 0.120   # 상완 (J2->J3)
L3 = 0.191   # 전완 (J3->J4)
L4 = 0.089   # 손목->파지점 (J4->grasp)

LIM_YAW = math.radians(30)     # j1: 3:1 감속기, 실측 확인됨
LIM_SHOULDER = math.radians(90)  # j2
LIM_ELBOW = math.radians(135)    # j3
LIM_WRIST = math.radians(90)     # j4

GRIP_OPEN, GRIP_CLOSED = 0.016, -0.004


def ik(x, y, z, grip=GRIP_OPEN):
    """(x,y,z)에 집게를 수직 하향으로 위치시키는 [t1,t2,t3,t4,t5,grip] 반환. 불가능하면 None."""
    dx, dy = x - SHOULDER_X, y - SHOULDER_Y
    for yaw, sign in ((math.atan2(dy, dx), 1.0), (math.atan2(-dy, -dx), -1.0)):
        if abs(yaw) > LIM_YAW:
            continue
        r = sign * math.hypot(dx, dy)
        h = (z - SHOULDER_Z) + L4  # 손목점 높이(상완 기준). 손목은 파지점보다 L4 위.
        d = math.hypot(r, h)
        if d > L2 + L3 or d < abs(L2 - L3):
            continue
        base = math.atan2(h, r)
        cos_off = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * d)
        cos_elbow = (L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3)
        cos_off = max(-1.0, min(1.0, cos_off))
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        off = math.acos(cos_off)
        elbow_interior = math.acos(cos_elbow)
        elbow_bend = math.pi - elbow_interior  # 상완에 대한 전완의 꺾임각

        for t2, t3 in ((base - off, elbow_bend), (base + off, -elbow_bend)):
            if abs(t2) > LIM_SHOULDER or abs(t3) > LIM_ELBOW:
                continue
            # 손목 관절: 마지막 링크(L4)가 항상 -h 방향(수직 아래)을 향하도록.
            t4 = -math.pi / 2 - (t2 + t3)
            t4 = math.atan2(math.sin(t4), math.cos(t4))  # [-pi, pi] 정규화
            if abs(t4) > LIM_WRIST:
                continue
            return [yaw, t2, t3, t4, 0.0, grip]
    return None


def fk(joints):
    """검증용 순기구학. ik()가 반환한 관절각으로 파지점 (x,y,z)를 복원."""
    yaw, t2, t3, t4, _t5, _grip = joints
    r = L2 * math.cos(t2) + L3 * math.cos(t2 + t3)
    h = SHOULDER_Z + L2 * math.sin(t2) + L3 * math.sin(t2 + t3)
    # 마지막 링크: 절대각 (t2+t3+t4)
    r_g = r + L4 * math.cos(t2 + t3 + t4)
    h_g = h + L4 * math.sin(t2 + t3 + t4)
    x = SHOULDER_X + r_g * math.cos(yaw)
    y = SHOULDER_Y + r_g * math.sin(yaw)
    return x, y, h_g


def reachable(x, y, z):
    sol = ik(x, y, z)
    if sol is None:
        return False, None
    fx, fy, fz = fk(sol)
    err = math.dist((x, y, z), (fx, fy, fz))
    return err < 1e-6, sol
