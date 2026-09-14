"""GDY多视角彩色图数据采集工具。"""

from .config import CollectionConfig, load_collection_config
from .geometry import ReferenceGeometry, RoutePlan, Waypoint, build_routes

__all__ = [
    "CollectionConfig", "ReferenceGeometry", "RoutePlan", "Waypoint",
    "build_routes", "load_collection_config",
]
