import numpy as np
from kinematics import compute_forward_kinematics_full
from jacobians import compute_jacobian
from models import Robot

def inverse_kinematics(robot: Robot, target_pos: np.ndarray, initial_guess: np.ndarray, max_iterations: int = 1000, tolerance: float = 1e-6) -> np.ndarray:
    q = initial_guess.copy()
    
    for i in range(max_iterations):
        # 1. Get all the transformations and extract the position [x, y, z] from the end
        all_transforms = compute_forward_kinematics_full(robot, q)
        current_pos = all_transforms[-1][:3, 3]  # Vector de (3,)
        
        # 2. Calculate the linear error in the workspace
        error = target_pos - current_pos
        
        # Check for convergence (Stopping criterion)
        if np.linalg.norm(error) < tolerance:
            print(f"Converged in {i} iterations.")
            return q
        
        # 3. Get the complete Jacobian (6xN) and truncate it to (3xN) for Jv
        J_full = compute_jacobian(robot, q)
        J_v = J_full[:3, :]  # Only the rows for linear velocity
        
        # 4. Calculate the change in joint angles using the pseudo-inverse (N,3) @ (3,)
        delta_q = np.linalg.pinv(J_v) @ error

        # Limit step size to avoid explosions in singularities
        max_step = 0.2 # two tenths of a radian or meter per iteration
        step_norm = np.linalg.norm(delta_q)
        if step_norm > max_step:
            delta_q = (delta_q / step_norm) * max_step
        
        # 5. Update and normalize the joint angles
        q += delta_q
        q = np.mod(q + np.pi, 2 * np.pi) - np.pi  # Normalize to [-pi, pi]
    
    return q