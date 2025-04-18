import time
import traceback
from piper_control import piper_control

def main():
    try:
        print("Initializing robot...")
        robot = piper_control.PiperControl(can_port="can0")
        
        print("\nResetting robot...")
        robot.reset()
        
        print("\nGetting initial state:")
        print("Joint positions:", robot.get_joint_positions())
        print("Joint velocities:", robot.get_joint_velocities())
        print("Joint efforts:", robot.get_joint_efforts())
        print("Gripper state:", robot.get_gripper_state())
        print("Robot status:", robot.get_status())
        
        print("\nTesting gripper control...")
        print(f"GRIPPER_ANGLE_MAX: {piper_control.GRIPPER_ANGLE_MAX}")
        print(f"GRIPPER_EFFORT_MAX: {piper_control.GRIPPER_EFFORT_MAX}")
        
        # Try small incremental movements
        positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
        effort = 0.5  # 50% effort
        
        print("\nMoving gripper in small increments...")
        for pos in positions:
            target_pos = piper_control.GRIPPER_ANGLE_MAX * pos
            print(f"\nSetting gripper to {pos*100}% - position: {target_pos}")
            robot.set_gripper_ctrl(target_pos, piper_control.GRIPPER_EFFORT_MAX * effort)
            time.sleep(1)
            current_state = robot.get_gripper_state()
            print(f"Current gripper state: {current_state}")
            print(f"Position difference: {current_state[0] - target_pos}")
        
        # Try moving back down
        print("\nMoving gripper back down...")
        for pos in reversed(positions):
            target_pos = piper_control.GRIPPER_ANGLE_MAX * pos
            print(f"\nSetting gripper to {pos*100}% - position: {target_pos}")
            robot.set_gripper_ctrl(target_pos, piper_control.GRIPPER_EFFORT_MAX * effort)
            time.sleep(1)
            current_state = robot.get_gripper_state()
            print(f"Current gripper state: {current_state}")
            print(f"Position difference: {current_state[0] - target_pos}")
        
        print("\nTest complete!")
        
    except Exception as e:
        print(f"Error occurred: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main() 