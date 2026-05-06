import sys
from urdf_parser import URDFParser
from kinematics import forward_kinematics 
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

    fk_result = forward_kinematics(robot)

    print("Forward Kinematics Result (Transformation Matrix):")
    print(fk_result)

if __name__ == "__main__":
    main()