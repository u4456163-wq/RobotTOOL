import xml.etree.ElementTree as ET 
from typing import Tuple
from models import Robot, Link, Joint 

class URDFParser:
    @staticmethod
    def string_to_tuple(s: str) -> Tuple [float, float, float]:
        """Converts a space-separated string of numbers into a float tuple."""
        return tuple(map(float, s.split())) if s else (0.0, 0.0, 0.0)
    
    def parse(self, file_path: str) -> Robot:
        """Parses a URDF file and return a Robot object model."""
        tree = ET.parse(file_path)
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

            robot.joints.append(
                Joint(
                    name = joint_tag.get("name"),
                    type = joint_tag.get("type"),
                    parent = joint_tag.find("parent").get("link"),
                    child = joint_tag.find("child").get("link"),
                    origin_xyz = xyz,
                    origin_rpy = rpy,
                    axis = axis
                )
            )
        return robot