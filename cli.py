import sys
from urdf_parser import URDFParser
from kinematics import forward_kinematics
from jacobians import compute_jacobian, validate_jacobian_numerically
import numpy as np

# Set numpy printing to be more readable
np.set_printoptions(suppress=True, precision=4)

def main():
    if len(sys.argv) < 2:
        print("Usage: python cli.py robot.urdf")
        return

    file_path = sys.argv[1]

    parser = URDFParser()
    robot = parser.parse(file_path)

    fk_result = forward_kinematics(robot, np.zeros(len(robot.joints)))
    n_active = sum(1 for j in robot.joints if j.joint_type != "fixed")
    q = np.zeros(n_active)
    jacobians = compute_jacobian(robot, q)
    J_analytic, J_numeric = validate_jacobian_numerically(robot, q)

    print("Forward Kinematics Result (Transformation Matrix):")
    print(fk_result)
    print("\nJacobians:")
    print(jacobians)
    print("Max Jv error:", np.max(np.abs(J_analytic[:3] - J_numeric[:3])))
    print("Max Jw error:", np.max(np.abs(J_analytic[3:] - J_numeric[3:])))

if __name__ == "__main__":
    main()