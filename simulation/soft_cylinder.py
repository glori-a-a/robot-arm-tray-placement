"""Soft cylinder object metadata."""

CYLINDER_DIAMETER = 0.02
CYLINDER_HEIGHT = 0.18
# Extra clearance so surface bumps do not visually clip into the support plane
REST_CLEARANCE = 0.004
BODY_NAME = "soft_cylinder"
TOP_SITE_NAME = "cylinder_top"


class SoftCylinder:
    """Describes the soft cylinder used in the scene."""

    diameter = CYLINDER_DIAMETER
    height = CYLINDER_HEIGHT
    rest_clearance = REST_CLEARANCE
    body_name = BODY_NAME
    top_site_name = TOP_SITE_NAME

    @classmethod
    def center_z_on_surface(cls, surface_z: float) -> float:
        """Body-center height when the rod rests on a flat surface."""
        return surface_z + cls.diameter / 2 + cls.rest_clearance

    @classmethod
    def grasp_height_offset(cls) -> float:
        """Grip height above the table for a horizontal cylinder."""
        return cls.diameter / 2 + cls.rest_clearance
