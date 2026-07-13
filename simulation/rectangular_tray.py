"""Rectangular tray object metadata."""

TRAY_LENGTH = 0.30
TRAY_WIDTH = 0.15
TRAY_HEIGHT = 0.005
BODY_NAME = "rectangular_tray"
CENTER_SITE_NAME = "tray_center"


class RectangularTray:
    """Describes the rectangular tray used in the scene."""

    length = TRAY_LENGTH
    width = TRAY_WIDTH
    height = TRAY_HEIGHT
    body_name = BODY_NAME
    center_site_name = CENTER_SITE_NAME

    @classmethod
    def placement_height(cls, table_top_z: float = 0.40) -> float:
        return table_top_z + cls.height / 2 + 0.002
