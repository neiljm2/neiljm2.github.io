import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import config

# Unpack robot dimensions and home position from config
h, L1, offset1, L2, offset2, L3, offset3, L4, offset4, L5, offset5, L6 = config.LENGTHS
home_position = config.ANGLES  # assumed to be a 6-element list/array

def customIK3D(target, guess):
    """
    Custom inverse kinematics solver for a 3D target.
    Decouples the base rotation from the planar solution.
    
    Parameters
    ----------
    target : array-like
        A 3D target point [x, y, z].
    guess : array-like
        A 6-element initial guess for joint angles (in radians).
    
    Returns
    -------
    solution : np.ndarray
        A 6-element array of joint angles [θ₁, θ₂, θ₃, θ₄, θ₅, θ₆],
        where θ₁ handles the base rotation and θ₂-θ₄ are computed via least squares.
        For this demo, joints 5 and 6 are set to zero.
    """
    x, y, z = target
    # Compute the base rotation (θ₁) from the target's x and y.
    theta1 = np.arctan2(y, x)
    
    # Effective horizontal distance in the plane
    r = np.sqrt(x**2 + y**2)
    # The planar target that the remaining joints must reach:
    #   r should equal: L1 + L2*cos(-θ₂) + L3*cos(-θ₂+θ₃) + (L5+offset4)*cos(-θ₂+θ₃-θ₄)
    #   z should equal: h  + L2*sin(-θ₂) + L3*sin(-θ₂+θ₃) + (L5+offset4)*sin(-θ₂+θ₃-θ₄)
    target_planar = np.array([r, z])
    
    def residual(thetas):
        theta2, theta3, theta4 = thetas
        x_guess = L1 + L2*np.cos(-theta2) + L3*np.cos(-theta2+theta3) + (L5+offset4)*np.cos(-theta2+theta3-theta4)
        z_guess = h  + L2*np.sin(-theta2) + L3*np.sin(-theta2+theta3) + (L5+offset4)*np.sin(-theta2+theta3-theta4)
        return np.array([x_guess - target_planar[0], z_guess - target_planar[1]])
    
    # Use the provided guess for the planar joints (θ₂, θ₃, θ₄)
    planar_guess = guess[1:4]
    # Set bounds (adjust these as needed for your robot's joint limits)
    bounds_lower = [-41*np.pi/36, -np.pi, -np.pi]
    bounds_upper = [ 5*np.pi/36,  np.pi,  np.pi]
    result = least_squares(residual, planar_guess, bounds=(bounds_lower, bounds_upper))
    theta2, theta3, theta4 = result.x
    
    # For joints 5 and 6, set zero for this demo.
    theta5 = 0.0
    theta6 = 0.0
    return np.array([theta1, theta2, theta3, theta4, theta5, theta6])

def create_command_from_angles(joint_angles_list):
    """
    Creates a command string from a list of joint angle sets.
    The command format is assumed to be:
      /S*H*/M*<angles1>/M*<angles2> ... /E*X,<home_angles>*
    Angles are output in degrees, comma-separated.
    """
    command = "/S*H*"
    command += "/M*"
    for angles in joint_angles_list:
        # Convert each angle from radians to degrees
        for theta in angles:
            command += str(round(theta * 180 / np.pi, 2)) + ","
    command = command.rstrip(",")
    command += "*/E*X,"
    for theta in home_position:
        command += str(round(theta, 2)) + ","
    command = command.rstrip(",") + "*"
    return command

def demo_motion_3d():
    """
    Demonstrates a 3D motion by moving the robot to six different target points.
    The customIK3D function is used to compute a 6-DOF solution for each target.
    
    Returns
    -------
    command : str
        A command string that concatenates the computed joint angle sets.
    target_points : list of np.ndarray
        The list of 3D target points.
    joint_angles_sequence : list of np.ndarray
        The corresponding joint angle solutions for each target.
    """
    # Define six target points in 3D space.
    # These points are examples and should span a large portion of the robot's workspace.
    target_points = [
        np.array([L1 + L2 + L3,    0,     h + 5]),      # Straight ahead, high
        np.array([L1 + L2,         L2,    h + 2]),      # Front-right, medium
        np.array([L1 + L2,        -L2,    h + 3]),      # Front-left, medium
        np.array([L1 + L2/2,       L2/2,  h + 1]),      # Closer, lower
        np.array([L1 + L2/2,      -L2/2,  h + 4]),      # Closer, higher
        np.array([L1 + L2 + L3/2,   0,     h + 2])       # Extended reach, medium height
    ]
    
    # Start with the home position as the initial guess.
    current_guess = np.array(home_position)
    joint_angles_sequence = []
    
    for pt in target_points:
        solution = customIK3D(pt, current_guess)
        joint_angles_sequence.append(solution)
        current_guess = solution  # update guess for better convergence
    
    command = create_command_from_angles(joint_angles_sequence)
    return command, target_points, joint_angles_sequence

# Run the demo
command_string, targets, solutions = demo_motion_3d()
print("Command String for 3D Motion Demo:")
print(command_string)

# --- 3D Plotting of the target points for visualization ---
fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection='3d')
targets_array = np.array(targets)
ax.plot(targets_array[:,0], targets_array[:,1], targets_array[:,2],
        marker='o', linestyle='-', color='b', label='Motion Path')
for idx, pt in enumerate(targets):
    ax.text(pt[0], pt[1], pt[2], f'P{idx+1}', color='red', fontsize=12)
ax.set_title("3D Motion Demo: Target Points")
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.legend()
plt.show()
