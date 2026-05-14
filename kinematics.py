import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np

from models import Robot, Link, Joint

def joint_to_matrix(joint: Joint) -> np.ndarray:
    """
    Converts a joint origin (x,y,z and rpy) into a 4x4 homogeneous transformation matrix.
    Uses Extrinsic XYZ Euler angles convention.
    """
    x, y, z = joint.origin_xyz
    r, p, y_ang = joint.origin_rpy

    # Rotation matrices for each axis
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y_ang), -np.sin(y_ang), 0], [np.sin(y_ang), np.cos(y_ang), 0], [0, 0, 1]])

    # Combined rotation matrix R = Rz * Ry * Rx 
    R = Rz @ Ry @ Rx

    # Construct 4x4 homogeneous tranformation matrix 
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T
    
def forward_kinematics_full(robot: Robot, q: np.ndarray) -> List[np.ndarray]:
    """
    Computes all transformation matrices along the kinematic chain for a given joint configuration.
    """
    all_transforms = [np.eye(4)]  # Start with the base transformation

    for i, joint in enumerate(robot.joints):
        T_joint = joint_to_matrix(joint)
        # Apply the joint transformation based on its type and angle
        if joint.type == "revolute":
            # Example for revolute joint (simplified)
            pass
        elif joint.type == "prismatic":
            # Example for prismatic joint (simplified)
            pass
        all_transforms.append(all_transforms[-1] @ T_joint)

    return all_transforms

def forward_kinematics(robot: Robot, q: np.ndarray) -> np.ndarray:
    """
    Computes the final transformation matrix by multiplying all joint transformations.
    Currently assumes a serial chain in the order defined in the URDF.
    """
    all_transforms = forward_kinematics_full(robot, q)
    return all_transforms[-1]
