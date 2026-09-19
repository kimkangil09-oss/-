"""필드 + 계획된 로봇 파킹 지점을 위에서 본 SVG로 렌더링 (수직캠 모사)."""
import math
import field

SCALE = 600  # px per meter
PAD = 40


def fx(x):
    return PAD + (x + field.FIELD_W / 2) * SCALE


def fy(y):
    return PAD + (field.FIELD_D / 2 - y) * SCALE  # y+ 위쪽 -> svg는 아래가 +


ZONE_COLOR = {
    "pcc_left": "#cfe8ff", "hospital": "#ffd6d6", "pcc_right": "#cfe8ff",
    "isolation": "#e0e0e0", "lab": "#d8f5d8", "starting": "#fff2c2",
}
CYL_STOP_COLOR = {"픽업": "#666", "드롭": "#222"}


def render(plan, out_path):
    W = int(field.FIELD_W * SCALE + 2 * PAD)
    H = int(field.FIELD_D * SCALE + 2 * PAD)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">']
    svg.append(f'<rect x="0" y="0" width="{W}" height="{H}" fill="white"/>')

    # 필드 외곽
    svg.append(f'<rect x="{fx(-field.FIELD_W/2)}" y="{fy(field.FIELD_D/2)}" '
                f'width="{field.FIELD_W*SCALE}" height="{field.FIELD_D*SCALE}" '
                f'fill="none" stroke="black" stroke-width="3"/>')

    # 구역
    for name, (cx, cy, w, d) in field.ZONES.items():
        x0, y0 = cx - w / 2, cy + d / 2
        svg.append(f'<rect x="{fx(x0)}" y="{fy(y0)}" width="{w*SCALE}" height="{d*SCALE}" '
                    f'fill="{ZONE_COLOR.get(name,"#eee")}" stroke="black" stroke-width="1.5"/>')
        svg.append(f'<text x="{fx(cx)}" y="{fy(cy)}" font-size="13" text-anchor="middle" '
                    f'fill="#333">{name}</text>')

    # 실린더 초기 위치(참고용 음영 표시)
    for side, pts in field.CYLINDER_POSITIONS.items():
        sign = -1 if side == "left" else 1
        for (px, py) in pts:
            svg.append(f'<circle cx="{fx(sign*px if False else px)}" cy="{fy(py)}" r="6" '
                        f'fill="none" stroke="#999" stroke-width="1" stroke-dasharray="2,2"/>')

    colors = ["#e74c3c", "#27ae60", "#2980b9", "#8e44ad", "#f39c12", "#16a085", "#c0392b", "#2c3e50"]
    prevx, prevy = fx(0.0), fy(-0.4305)
    for i, (name, rx, ry, theta, targets, hits) in enumerate(plan):
        color = colors[i % len(colors)]
        x, y = fx(rx), fy(ry)
        svg.append(f'<line x1="{prevx}" y1="{prevy}" x2="{x}" y2="{y}" stroke="{color}" '
                    f'stroke-width="2" stroke-dasharray="5,3"/>')
        prevx, prevy = x, y
        # 로봇 위치 + 헤딩
        hx = x + 22 * math.cos(-theta)
        hy = y + 22 * math.sin(-theta)
        svg.append(f'<rect x="{x-13}" y="{y-9}" width="26" height="18" fill="{color}" '
                    f'fill-opacity="0.35" stroke="{color}" stroke-width="2" '
                    f'transform="rotate({-math.degrees(theta)} {x} {y})"/>')
        svg.append(f'<line x1="{x}" y1="{y}" x2="{hx}" y2="{hy}" stroke="{color}" stroke-width="2"/>')
        svg.append(f'<text x="{x}" y="{y-14}" font-size="10" text-anchor="middle" fill="{color}">'
                    f'{i+1}.{name}</text>')
        for (tx, ty), sol in zip(targets, hits):
            tcol = "#2ecc71" if sol is not None else "#e74c3c"
            svg.append(f'<circle cx="{fx(tx)}" cy="{fy(ty)}" r="5" fill="{tcol}" stroke="black" '
                        f'stroke-width="0.5"/>')

    svg.append('</svg>')
    with open(out_path, "w") as f:
        f.write("\n".join(svg))
    print(f"SVG written: {out_path}")
