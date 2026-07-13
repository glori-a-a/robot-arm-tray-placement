"""Soft cylinder object metadata."""

CYLINDER_DIAMETER = 0.02
CYLINDER_HEIGHT = 0.18
BODY_NAME = "soft_cylinder"
TOP_SITE_NAME = "cylinder_top"


class SoftCylinder:
    """Describes the soft cylinder used in the scene."""

    diameter = CYLINDER_DIAMETER
    height = CYLINDER_HEIGHT
    body_name = BODY_NAME
    top_site_name = TOP_SITE_NAME

    @classmethod
    def grasp_height_offset(cls) -> float:
        """Grip height above the table for a horizontal cylinder."""
        return cls.diameter / 2 + 0.004
