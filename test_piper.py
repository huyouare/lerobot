from lerobot.common.robot_devices.robots.utils import make_robot

# Create a Piper robot instance
robot = make_robot("piper")

# Connect to the arm
robot.follower_arms["main"].connect()

try:
    # Get current joint positions
    positions = robot.follower_arms["main"].get_joint_positions()
    print("Current joint positions:", positions)
    
    # Move a single joint (e.g., elbow)
    robot.follower_arms["main"].set_joint_position("elbow", positions[2] + 0.1)  # Move elbow up by 0.1 rad
    
    # Wait a bit
    import time
    time.sleep(2)
    
    # Move all joints to a new position
    new_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # All joints at 0 radians
    robot.follower_arms["main"].set_joint_positions(new_positions)
    
finally:
    # Always disconnect when done
    robot.follower_arms["main"].disconnect()