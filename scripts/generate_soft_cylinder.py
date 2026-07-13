"""Generate a bumpy soft-cylinder MJCF include file."""

from pathlib import Path
import math

CYLINDER_DIAMETER = 0.02
CYLINDER_HEIGHT = 0.18
BUMP_COUNT = 48
OUTPUT = Path(__file__).resolve().parents[1] / "assets" / "cylinder" / "soft_cylinder.xml"


def build_xml() -> str:
    radius = CYLINDER_DIAMETER / 2
    lines = [
        '<!-- Soft cylinder: 2 cm diameter, 18 cm height, bumpy surface -->',
        "<mujocoinclude>",
        '  <body name="soft_cylinder" pos="0.5 0.05 0.411" quat="0.7071 0 0.7071 0">',
        "    <freejoint/>",
        '    <inertial mass="0.08" pos="0 0 0" diaginertia="0.00002 0.0001 0.0001"/>',
        '    <geom name="cylinder_core" type="capsule" fromto="-0.09 0 0 0.09 0 0" size="0.01" '
        'material="cylinder_material" friction="1.5 0.05 0.01" solimp="0.9 0.95 0.001" solref="0.02 1" '
        'rgba="0.95 0.45 0.10 1"/>',
    ]

    rows = 12
    cols = 4
    for row in range(rows):
        z = -CYLINDER_HEIGHT / 2 + (row + 0.5) * (CYLINDER_HEIGHT / rows)
        for col in range(cols):
            angle = (col / cols) * 2 * math.pi + (row * 0.35)
            bump = 0.0015 * math.sin(row * 1.7 + col * 2.3)
            r = radius + bump
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            size = 0.0035 + 0.001 * abs(math.sin(row + col))
            lines.append(
                f'    <geom type="sphere" pos="{x:.5f} {y:.5f} {z:.5f}" size="{size:.4f}" '
                f'material="cylinder_material" friction="1.5 0.05 0.01" '
                f'solimp="0.9 0.95 0.001" solref="0.02 1" rgba="0.85 0.35 0.15 1"/>'
            )

    lines.extend(
        [
            '    <site name="cylinder_top" pos="0 0 0.09" size="0.008" rgba="1 0.5 0 0.4"/>',
            "  </body>",
            "</mujocoinclude>",
            "",
        ]
    )
    return "\n".join(lines)


if __name__ == "__main__":
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(build_xml())
    print(f"Wrote {OUTPUT}")
