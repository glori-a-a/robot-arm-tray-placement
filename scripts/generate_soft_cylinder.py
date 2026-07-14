from __future__ import annotations

import math
from pathlib import Path

CYLINDER_DIAMETER = 0.02
CYLINDER_HEIGHT = 0.18
REST_CLEARANCE = 0.004
TABLE_TOP_Z = 0.40
OUTPUT = Path(__file__).resolve().parents[1] / "assets" / "cylinder" / "soft_cylinder.xml"

SOFT_FRICTION = 'friction="1.8 0.08 0.02"'
SOFT_SOLIMP = 'solimp="0.85 0.92 0.001"'
SOFT_SOLREF = 'solref="0.04 1"'
CORE_RGBA = "0.92 0.42 0.12 1"


def build_xml() -> str:
    radius = CYLINDER_DIAMETER / 2
    half_shaft = CYLINDER_HEIGHT / 2 - radius
    center_z = TABLE_TOP_Z + radius + REST_CLEARANCE

    lines = [
        "<mujocoinclude>",
        f'  <body name="soft_cylinder" pos="0.5 0.05 {center_z:.4f}" quat="1 0 0 0">',
        "    <freejoint/>",
        '    <inertial mass="0.06" pos="0 0 0" diaginertia="0.000015 0.00008 0.00008"/>',
        f'    <geom name="cylinder_core" type="capsule" '
        f'fromto="0 -{half_shaft:.4f} 0 0 {half_shaft:.4f} 0" size="{radius:.4f}" '
        f'material="cylinder_material" {SOFT_FRICTION} {SOFT_SOLIMP} {SOFT_SOLREF} '
        f'rgba="{CORE_RGBA}"/>',
        # Overhead localisation beacon (body XY origin, slightly above shaft).
        '    <geom name="cylinder_detect" type="sphere" size="0.016" pos="0 0 0.035" '
        f'rgba="0.92 0.42 0.12 0.35" mass="1e-5" friction="0.05 0.005 0.0005"/>',
    ]

    # bumps
    rows = 10
    cols = 8
    bump_idx = 0
    for row in range(rows):
        y = -half_shaft + (row + 0.5) * (2 * half_shaft / rows)
        for col in range(cols):
            angle = (col / cols) * 2 * math.pi + row * 0.22
            local_z = math.sin(angle)
            if local_z < -0.15:
                continue
            bump_r = radius + 0.0006 + 0.0003 * abs(math.sin(row * 1.7 + col))
            x = bump_r * math.cos(angle)
            z = bump_r * local_z
            size = 0.0018 + 0.0005 * abs(math.sin(row + col * 1.3))
            lines.append(
                f'    <geom name="bump_{bump_idx}" type="sphere" '
                f'pos="{x:.5f} {y:.5f} {z:.5f}" size="{size:.4f}" '
                f'material="cylinder_material" {SOFT_FRICTION} {SOFT_SOLIMP} {SOFT_SOLREF} '
                f'rgba="{CORE_RGBA}"/>'
            )
            bump_idx += 1

    lines.extend(["  </body>", "</mujocoinclude>", ""])
    return "\n".join(lines)


if __name__ == "__main__":
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    xml = build_xml()
    OUTPUT.write_text(xml)
    bump_count = xml.count('type="sphere"') - 1
    print(f"Wrote {OUTPUT}")
    print(f"  diameter={CYLINDER_DIAMETER*100:.0f} cm, length={CYLINDER_HEIGHT*100:.0f} cm")
    print(f"  non-smooth bumps={bump_count}, detect beacon enabled")
