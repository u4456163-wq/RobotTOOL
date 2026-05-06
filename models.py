from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import xml.etree.ElementTree as ET
import numpy as np 

# 1. Definición de Estructuras de Datos
@dataclass
class URDFElement:
    """Base class for any URDF component"""
    name: str
    origin_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

@dataclass
class Link(URDFElement):
    """Represents a physical link with inertia and visual properties"""
    name: str
    mass: float = 0.0
    inertia: dict = field(default_factory=dict)
    mesh_path: str = ""
    collision_mesh_path: str = ""
    visual_mesh_path: str = ""
    center_of_mass: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    material: dict = field(default_factory=dict)

@dataclass
class Joint(URDFElement):
    """Represents a joint connecting two links."""
    type: str = "revolute"
    parent: str = ""
    child: str = ""
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    limit_lower: Optional[float] = None
    limit_upper: Optional[float] = None
    effort_limit: Optional[float] = None
    velocity_limit: Optional[float] = None

@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)

