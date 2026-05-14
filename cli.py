import sys
from urdf_parser import URDFParser
from kinematics import forward_kinematics 
from jacobians import compute_jacobian
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
    q = np.zeros(len(robot.joints))
    jacobians = compute_jacobian(robot, q)

    print("Forward Kinematics Result (Transformation Matrix):")
    print(fk_result)
    print("\nJacobians:")
    print(jacobians)

if __name__ == "__main__":
    main()