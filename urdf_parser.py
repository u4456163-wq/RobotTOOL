import xml.etree.ElementTree as ET 
from typing import Tuple
from models import Robot, Link, Joint 
from pathlib import Path

class URDFParser:
    @staticmethod
    def string_to_tuple(s: str) -> Tuple[float, float, float]:
        """Converts a space-separated string of numbers into a float tuple."""
        return tuple(map(float, s.split())) if s else (0.0, 0.0, 0.0)
    
    def parse(self, file_path: str) -> Robot:
        """Parses a URDF file and return a Robot object model."""
        
        # 1. Build the path by searching in 'models_robot'
        base_path = Path(__file__).parent / "models_robot"
        full_path = base_path / file_path

        # 2. Verify if it exists, if not, try to use the path as received
        if not full_path.exists():
            full_path = Path(file_path)
            if not full_path.exists():
                raise FileNotFoundError(f"The URDF file was not found: {full_path}")

        # 3. Now parse the file only once
        tree = ET.parse(str(full_path))
        root = tree.getroot()

        robot = Robot(name=root.get("name", "Robot"))

        # Parse Links
        for link_tag in root.findall("link"):
            robot.links.append(
                Link(name=link_tag.get("name"))
            )

        # Parse Joints
        for joint_tag in root.findall("joint"):
            origin = joint_tag.find("origin")
            xyz = self.string_to_tuple(origin.get("xyz")) if origin is not None else (0.0, 0.0, 0.0)
            rpy = self.string_to_tuple(origin.get("rpy")) if origin is not None else (0.0, 0.0, 0.0)

            axis_tag = joint_tag.find("axis")
            axis = self.string_to_tuple(axis_tag.get("xyz")) if axis_tag is not None else (0.0, 0.0, 1.0)

            # Here you get the names of father and child
            parent_tag = joint_tag.find("parent")
            child_tag = joint_tag.find("child")
            
            robot.joints.append(
                Joint(
                    name = joint_tag.get("name"),
                    joint_type = joint_tag.get("type"),
                    parent = parent_tag.get("link") if parent_tag is not None else "world",
                    child = child_tag.get("link") if child_tag is not None else "none",
                    origin_xyz = xyz,
                    origin_rpy = rpy,
                    axis = axis
                )
            )
        return robot