import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np

from models import Robot, Link, Joint


@dataclass
class Joint():
    name: str
    type: str
    parent: str
    child: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float]

@staticmethod
def joint_to_matrix(joint):
    x, y, z = joint.origin_xyz
    r, p, y_ang = joint.origin_rpy

    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y_ang), -np.sin(y_ang), 0], [np.sin(y_ang), np.cos(y_ang), 0], [0, 0, 1]])

    R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T
    
def forward_kinematics(robot, base="BASE"):
    T = np.eye(4)

    for joint in robot.joints:
        T = T @ joint_to_matrix(joint)

    return T

@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)

    def get_chain(self, start_link: str) -> List[str]:
        """Retorna el orden de los eslabones desde la base hasta el efector final"""
        chain = [start_link]
        current = start_link
        while True:
            next_joint = next((j for j in self.joints if j.parent == current), None)
            if next_joint:
                chain.append(next_joint.child)
                current = next_joint.child
            else:
                break
        return chain
