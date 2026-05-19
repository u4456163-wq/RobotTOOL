import numpy as np
from models import Robot, Joint
from kinematics import compute_forward_kinematics_full

def compute_jacobian(robot: Robot, q: np.ndarray) -> np.ndarray:
    """
    Calculates the Jacobian matrix 6xN for the given robot configuration.
    """
    active_joints = [j for j in robot.joints if j.joint_type != "fixed"]
    n_active = len(active_joints)
    J = np.zeros((6, n_active))

    all_transforms = compute_forward_kinematics_full(robot, q)
    T_0_n = all_transforms[-1]
    p_n = all_transforms[-1][:3, 3]  # Position of the end effector in global space

    col = 0 # Column active for the Jacobian

    # 1. Obtain the global transformation of the end effector(T_0_n)
    # 2. For each joint i from 1 to n:
    #    - Calculate the transformation from the base to the joint i (T_0_i) 
    #    - Extract the rotation/traslation axis (Z_i) 
    #    - Calculate the position vector of the end effector relative to joint i (p_n - p_i)
    
    for i, joint in enumerate(robot.joints):
        if joint.joint_type == "fixed":
            continue
            
        T_0_i = all_transforms[i+1]  # T_0_i is the transformation up to joint i (i+1 in the list)
        p_i = T_0_i[:3, 3]
        R_0_i = T_0_i[:3, :3]
        
        # Project the joint axis specified in the URDF to the global frame
        axis_local = np.array(joint.axis) / np.linalg.norm(joint.axis)
        z_i = R_0_i @ axis_local  # <--- THIS ensures it works for any axle

        if joint.joint_type in ["revolute", "continuous"]:
            r = p_n - p_i
            J[:3, col] = np.cross(z_i, r)  # Jv
            J[3:, col] = z_i               # Jw 

        elif joint.joint_type == "prismatic":
            J[:3,col] = z_i #jv
            J[3:,col] = np.zeros(3) #jw
        else:
            continue

        col += 1

    return J
def validate_jacobian_numerically(robot, q, eps=1e-6):
    """Finite difference check against analytic Jacobian."""
    J_analytic = compute_jacobian(robot, q)
    n = J_analytic.shape[1]
    J_numeric = np.zeros((6, n))

    T0 = compute_forward_kinematics_full(robot, q)[-1]
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]

    for i in range(n):
        dq = q.copy()
        dq[i] += eps
        T1 = compute_forward_kinematics_full(robot, dq)[-1]
        p1 = T1[:3, 3]
        R1 = T1[:3, :3]
        J_numeric[:3, i] = (p1 - p0) / eps  # Velocity part from linear difference
        dR = R1 @ R0.T
        # Extract the angular velocity from the rotation difference using the logarithm map
        J_numeric[3:,i] = np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0], dR[1,0]-dR[0,1]]) / (2 * eps)

    return J_analytic, J_numeric