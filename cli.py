import sys
from urdf_parser import URDFParser
from kinematics import forward_kinematics, compute_forward_kinematics_full
from jacobians import compute_jacobian, validate_jacobian_numerically
from inverse_kinematics import inverse_kinematics
import numpy as np
import argparse 

# Set numpy printing to be more readable
np.set_printoptions(suppress=True, precision=4)

def main():
    parser_cli = argparse.ArgumentParser()
    parser_cli.add_argument("urdf_file", help = "Path to the URDF file describing the robot")
    parser_cli.add_argument("--target", nargs=3, type=float, 
                            metavar=('X', 'Y', 'Z'),
                            help="IK target position in meters")
    args = parser_cli.parse_args()

    if len(sys.argv) < 2:
        print("Usage: python cli.py robot.urdf")
        return

    file_path = sys.argv[1]

    parser = URDFParser()
    robot = parser.parse(file_path)

    q = np.zeros(len([j for j in robot.joints if j.joint_type != "fixed"]))

    fk_result = forward_kinematics(robot, q)
    jacobians = compute_jacobian(robot, q)
    J_analytic, J_numeric = validate_jacobian_numerically(robot, q)

    # Print results
    print("Forward Kinematics Result (Transformation Matrix):")
    print(fk_result)
    print("\nJacobians:")
    print(jacobians)
    print("Max Jv error:", np.max(np.abs(J_analytic[:3] - J_numeric[:3])))
    print("Max Jw error:", np.max(np.abs(J_analytic[3:] - J_numeric[3:])))

    if args.target:
        target = np.array(args.target)
        ik_result = inverse_kinematics(robot, target_pos=target, initial_guess=q)
        final_pos = compute_forward_kinematics_full(robot, ik_result)[-1][:3, 3]
        print("\nInverse Kinematics Result:")
        print(ik_result)
        print(f"Target:         {np.round(target, 4)}")
        print(f"Achieved:       {np.round(final_pos, 4)}")
        print(f"Position error: {np.linalg.norm(target - final_pos):.6f} m")

if __name__ == "__main__":
    main()