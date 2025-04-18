import time
from lerobot.common.robot_devices.robots.utils import make_robot

def main():
    # Create robot instance using the type name
    robot = make_robot("piper")
    
    # Connect to the arm
    print("Connecting to robot...")
    robot.connect()
    
    # Get current positions
    current_positions = robot.follower_arms["main"].read()
    print(f"Current joint positions: {current_positions}")
    
    # Move each joint slightly (5 degrees = ~0.087 radians)
    small_movement = 0.087
    for joint_name in current_positions.keys():
        print(f"\nMoving {joint_name} slightly...")
        # Get current position
        current_pos = current_positions[joint_name]
        print(f"Current position: {current_pos}")
        # Move slightly
        target_pos = current_pos + small_movement
        print(f"Target position: {target_pos}")
        robot.follower_arms["main"].write({joint_name: target_pos})
        # Wait a bit
        time.sleep(1.0)
        # Get new position
        new_positions = robot.follower_arms["main"].read()
        print(f"New position: {new_positions[joint_name]}")
        # Move back
        robot.follower_arms["main"].write({joint_name: current_pos})
        time.sleep(1.0)
        # Get final position
        final_positions = robot.follower_arms["main"].read()
        print(f"Final position: {final_positions[joint_name]}")
    
    # Clean up
    print("\nDisconnecting...")
    robot.disconnect()

if __name__ == "__main__":
    main()