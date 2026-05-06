import sys
from urdf_parser import URDFParser
from kinematics import forward_kinematics 
import numpy as np
np.set_printoptions(suppress=True, precision=4)

def main():
    if len(sys.argv) < 2:
        print("Uso: python cli.py robot.urdf")
        return

    file_path = sys.argv[1]

    parser = URDFParser()
    robot = parser.parse(file_path)

    fk = forward_kinematics(robot)

    print("Resultado FK:")
    print(fk)

if __name__ == "__main__":
    main()