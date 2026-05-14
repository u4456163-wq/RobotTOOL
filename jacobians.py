import numpy as np
from models import Robot
from kinematics import forward_kinematics, forward_kinematics_full

def compute_jacobian(robot: Robot, q: np.ndarray) -> np.ndarray:
    """
    Calculates the Jacobian matrix 6xN for the given robot configuration.
    """
    n = len(robot.joints)
    J = np.zeros((6, n))

    all_transforms = forward_kinematics(robot, q)

    all_transforms = forward_kinematics_full(robot, q) 
    T_0_n = all_transforms[-1]
    p_n = T_0_n[:3, 3]

    # 1. Obtain the global transformation of the end effector(T_0_n)
    # 2. For each joint i from 1 to n:
    #    - Calculate the transformation from the base to the joint i (T_0_i) 
    #    - Extract the rotation/traslation axis (Z_i) 
    #    - Calculate the position vector of the end effector relative to joint i (p_n - p_i)
    
    for i in range(n):
        # T_0_i es la transformación del frame anterior a la articulación actual
        T_0_i = all_transforms[i]
        
        z_i = T_0_i[:3, 2] # Eje Z en coordenadas globales
        p_i = T_0_i[:3, 3] # Posición del joint en coordenadas globales
        
        # Para articulación de REVOLUTA:
        r = p_n - p_i
        Jv = np.cross(z_i, r)
        Jw = z_i
        
        # Si fuera PRISMÁTICA:
        # Jv = z_i
        # Jw = np.zeros(3)

        J[:, i] = np.hstack([Jv, Jw])
        
    return J
