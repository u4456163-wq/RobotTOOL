import sys
import typing
import math
import numpy as np
from models import Robot, Joint, Link
# --- ROUTE CONFIGURATION (Modify this according to your environment) ---
# If you are in Docker, we leave your route active. 
# We use the exact route we found in your container
if __name__ == "__main__":
    DOCKER_FREECAD_PATH = "/usr/lib/freecad-daily-python3/lib"
    
    if DOCKER_FREECAD_PATH:
        sys.path.append(DOCKER_FREECAD_PATH)

    try:
        import FreeCAD
    except ImportError:
        print("\n[ERROR] Could not find FreeCAD library.")
        print(f"Current route attempted: '{DOCKER_FREECAD_PATH}'")
        print("👉 If you are running this natively, please edit the 'DOCKER_FREECAD_PATH' variable or add FreeCAD to your PYTHONPATH.\n")
        sys.exit(1)

    print("=== Docker & FreeCAD Environment Verified ===")

def get_axis_from_placement(placement) -> typing.Tuple[float, float, float]:
    """In RobotCAD the axis of the joint is the Z axis of the rotated Placement.
    Yaw-Pitch-Roll=(0,0,0) → Z axis = (0, 0, 1)"""
    rot = placement.Rotation
    # Get the Z axis of the rotation
    z_local = FreeCAD.Vector(0, 0, 1)
    axis = rot.multVec(z_local)
    return (axis.x, axis.y, axis.z)

def placement_to_xyz_rpy(placement) -> typing.Tuple [tuple, tuple]:
    """Convert FreeCAD Placement to position (x, y, z) and orientation (roll, pitch, yaw)"""
    pos = placement.Base
    xyz = (pos.x / 1000.0, pos.y / 1000.0, pos.z / 1000.0)  # mm → m

    yaw, pitch, roll = placement.Rotation.toEuler() # degrees, YPR order
    rpy = (
        math.radians(roll), #Roll
        math.radians(pitch), #Pitch
        math.radians(yaw) #Yaw
    )
    return xyz, rpy

def build_robot_from_mock(joint_data: list, link_data: list) -> Robot:
    """
    Builds a Robot model from raw FreeCAD-like data.
    In production this reads from FreeCAD.ActiveDocument.Objects
    """
    robot = Robot(name="TestRobot")

    for name in link_data:
        robot.links.append(Link(name=name))

    for jd in joint_data:
        origin = jd['origin'] # FreeCAD.Placement
        xyz, rpy = placement_to_xyz_rpy(origin)
        axis = get_axis_from_placement(origin)

        robot.joints.append(Joint(
            name = jd["name"],
            joint_type = jd["type"],
            parent = jd["parent"],
            child = jd["child"],
            origin_xyz = xyz,
            origin_rpy = rpy,
            axis = axis,
            limit_lower    = jd.get("lower", None),
            limit_upper    = jd.get("upper", None),
            velocity_limit = jd.get("velocity", None),
            effort_limit   = jd.get("effort", None),
        ))
    return robot

# --- LOCAL TEST BENCH ---
if __name__ == "__main__":
    print("=== Docker & FreeCAD Environment Verified ===")
    
    def build_robot_from_document(doc=None) -> Robot:
        """
        Read the REAL robot from FreeCAD.ActiveDocument.
        Replaces the mock in production.
        """
        if doc is None:
            doc = FreeCAD.ActiveDocument
        # Avoid errors if FreeCAD does not have any files open in the interface
        if doc is None:
            print("No active document found in FreeCAD.")
            return None

        robot = Robot(name=doc.Name)

        for obj in doc.Objects:
            # Links de RobotCAD
            if hasattr(obj, 'Real') and hasattr(obj, 'Visual'):
                robot.links.append(Link(name=obj.Label))
            
            # Joints of RobotCAD
            if hasattr(obj, "Type") and hasattr(obj, "Parent") and hasattr(obj, "Child") and hasattr(obj, "Origin"):
                xyz, rpy = placement_to_xyz_rpy(obj.Origin)
                axis = get_axis_from_placement(obj.Origin)

                robot.joints.append(Joint(
                    name           = obj.Label,
                    joint_type     = obj.Type,
                    parent         = obj.Parent,
                    child          = obj.Child,
                    origin_xyz     = xyz,
                    origin_rpy     = rpy,
                    axis           = axis,
                    limit_lower    = getattr(obj, "LowerLimit", None),
                    limit_upper    = getattr(obj, "UpperLimit", None),
                    velocity_limit = getattr(obj, "VelocityLimit", None),
                    effort_limit   = getattr(obj, "EffortLimit", None),
                ))
        for j in robot.joints:
            print(f"Joint detectado de FreeCAD: {j.name}, Type: {j.joint_type}")
            print(f"  Parent: {j.parent} -> Child: {j.child}")
            print(f"  Origin XYZ: {j.origin_xyz}, RPY: {j.origin_rpy}, Axis: {j.axis}")
        return robot
        
        

    # Simulate the Robot_2DOF.FCStd you created
    mock_joints = [
        {
            "name" : "joint1",
            "type" : "revolute",
            "parent" : "Link",
            "child" : "Eslabon_1",
            "origin" : FreeCAD.Placement(FreeCAD.Vector(0, 0, 50),FreeCAD.Rotation(0, 0, 0)),
            "lower" : -math.pi/2,
            "upper" : math.pi/2,
            "velocity" : math.pi,
            "effort" : 10
        },
        {
            "name" : "joint2",
            "type" : "revolute",
            "parent" : "Eslabon_1",
            "child" : "Eslabon_2",
            "origin" : FreeCAD.Placement(FreeCAD.Vector(0,0,100),FreeCAD.Rotation(0,0,0)),
            "lower" : -math.pi/2,
            "upper" : math.pi/2,
            "velocity" : math.pi,
            "effort" : 10
        }
    ]
    mock_links = ["Link", "Eslabon_1", "Eslabon_2"]         

    robot = build_robot_from_mock(mock_joints, mock_links)
    print(f"Robot : {robot.name}")
    print(f"Links : {[l.name for l in robot.links]}")
    print(f"Joints: {len(robot.joints)}\n")

    for j in robot.joints:
        print(f"Joint: {j.name}")
        print(f" Type: {j.joint_type}")
        print(f" Parent: {j.parent}")
        print(f" Child: {j.child}")
        print(f" Origin (XYZ): {j.origin_xyz}")
        print(f" Origin (RPY): {j.origin_rpy}")
        print(f" Axis: {j.axis}")
        print(f" Limits: [{j.limit_lower}, {j.limit_upper}]")
        print(f" Velocity Limit: {j.velocity_limit}")
        print(f" Effort Limit: {j.effort_limit}")
        
    from kinematics import forward_kinematics
    q = np.zeros(2)
    T = forward_kinematics(robot, q)
    z_result = round(T[2, 3], 4)

    print(f"\nFK q=0 → Z = {z_result} m")

    # --- TEST WITH REAL DOCUMENT ---
    print("\n=== Test with Real Document ===")
    real_robot = build_robot_from_document()
    
    if real_robot and len(real_robot.joints) > 0:
        q_real = np.zeros(len([j for j in real_robot.joints if j.joint_type != "fixed"]))
        T_real = forward_kinematics(real_robot, q_real)
        print(f"\nFK robot real q=0 → position:")
        print(f"  X = {round(T_real[0,3], 4)} m")
        print(f"  Y = {round(T_real[1,3], 4)} m")
        print(f"  Z = {round(T_real[2,3], 4)} m")
    else:
        print("No joints were detected in the document.")