import os
import sys
import inspect
import math
import numpy as np
import time

# --- Dynamixel SDK path bootstrap -----------------------
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sdk_path = os.path.join(parentdir, "DynamixelSDK-3.7.31", "python", "src", "dynamixel_sdk")
if sdk_path not in sys.path:
    sys.path.insert(0, sdk_path)

from dynamixel_sdk import PortHandler, PacketHandler


def cosd(x_deg):  # cos with degrees
    return math.cos(math.radians(x_deg))


def sind(x_deg):  # sin with degrees
    return math.sin(math.radians(x_deg))


class MyRobot:
    # ----------------------------- Constants (AX-12A) -------------------------
    ADDR_MX_TORQUE_ENABLE = 24       # 1 byte
    ADDR_MX_GOAL_POSITION = 30       # 2 bytes
    ADDR_MX_MOVING_SPEED = 32        # 2 bytes
    ADDR_MX_TORQUE_LIMIT = 34        # 2 bytes
    ADDR_MX_PRESENT_POSITION = 36    # 2 bytes

    PROTOCOL_VERSION = 1.0
    BAUDRATE = 1_000_000
    DEVICENAME = "COM8"              # e.g., Windows: 'COM3' | Linux: '/dev/ttyUSB0' | Mac: '/dev/tty.usbserial-*'

    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0
    COMM_SUCCESS = 0
    DXL_MOVING_STATUS_THRESHOLD = 10  # [ticks], not used directly here
    TICKS_PER_DEG = 1.0 / 0.29        # ≈ 3.448275862 ticks per degree
    DEG_PER_TICK = 0.29

    # ------------------------------- Init -------------------------------------
    def __init__(self):
        # Robot + kinematics state
        self.motor_ids = [1, 2, 3, 4]
        self.gripper_motor_id = 5
        self.forward_transform = np.zeros((4, 4))
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0])  # deg
        self.joint_pos = np.zeros((4, 4))
        self.draw_robot_flag = False
        self.use_smooth_speed_flag = False
        self.gripper_open_flag = True

        # deg limits: [-130,130], [-180,0], [-140,140], [-140,100]
        self.joint_limits = np.array([[-90, 90], [-180, 180], [-140, 140], [-140, 140]])

        # IK config (simple analytical below; no external toolbox)
        self.ik_weights = np.array([0.25, 0.25, 0.25, 1, 1, 1])

        # Motor offsets (deg)
        self.joint_offsets = np.array([150, 150, 150, 150], dtype=float)

        self.joint_angle_error = np.array([0.0, 0.0, 0.0, 0.0])
        self.init_status = 0

        self.movement_history = []  # list of dicts
        self.motor_speed = np.array([0.1, 0.1, 0.1, 0.1])   # [0..1]
        self.motor_torque = np.array([1.0, 1.0, 1.0, 1.0])  # [0..1]
        self.pitch = 0.0

        # --- SDK setup ---
        try:
            self.port_handler = PortHandler(self.DEVICENAME)
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

            if self.port_handler.openPort():
                print("Succeeded to open the port!")
            else:
                print("Failed to open the port! Reconnect Robot.")
                raise RuntimeError("openPort failed")

            if self.port_handler.setBaudRate(self.BAUDRATE):
                print("Succeeded to change the baudrate!")
            else:
                print("Failed to change the baudrate! Reconnect Robot.")
                raise RuntimeError("setBaudRate failed")

            # Initialize defaults
            self.set_speed(self.motor_speed, overwrite_speeds=True)
            self.set_torque_limit(self.motor_torque)
            self.move_j(0, 0, 0, 0)
            self.enable_motors()
            self.init_status = 1

        except Exception as e:
            print(f"Initialization error: {e}")
            self.init_status = 0

    # ----------------------------- Utilities ----------------------------------
    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @classmethod
    def deg_to_ticks(cls, deg):
        return int(round(deg * cls.TICKS_PER_DEG))

    @classmethod
    def ticks_to_deg(cls, ticks):
        return ticks * cls.DEG_PER_TICK

    def _check_limits_one(self, deg, idx):
        lo, hi = self.joint_limits[idx]
        if not (lo <= deg <= hi):
            if idx == 0:
                raise ValueError("Angle Limits for first Axis Reached, Min/Max: ±130°")
            elif idx == 1:
                raise ValueError("Angle Limits for second Axis Reached, Min/Max: [−180°, 0°]")
            else:
                raise ValueError("Angle Limits Reached, Min/Max: ±100°")
        return deg

    # --------------------------- Motor I/O wrappers ---------------------------
    def _write1(self, dxl_id, addr, value):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, addr, int(value))
        if dxl_comm_result != self.COMM_SUCCESS or dxl_error != 0:
            raise IOError(f"DXL write1 error id={dxl_id}, addr={addr}, result={dxl_comm_result}, error={dxl_error}")

    def _write2(self, dxl_id, addr, value):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, addr, int(value))
        if dxl_comm_result != self.COMM_SUCCESS or dxl_error != 0:
            raise IOError(f"DXL write2 error id={dxl_id}, addr={addr}, result={dxl_comm_result}, error={dxl_error}")

    def _read2(self, dxl_id, addr):
        value, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, addr)
        if dxl_comm_result != self.COMM_SUCCESS or dxl_error != 0:
            raise IOError(f"DXL read2 error id={dxl_id}, addr={addr}, result={dxl_comm_result}, error={dxl_error}")
        return value

    # ------------------------------- Speeds/Torque ----------------------------
    def smooth_speed(self, joint_angles_delta_deg):
        max_angle = float(np.max(np.abs(joint_angles_delta_deg)))
        if max_angle == 0:
            return
        speed_per_deg = max_angle / 100.0
        if speed_per_deg != 0:
            new_speeds = np.abs(joint_angles_delta_deg / speed_per_deg) * 0.01
            # preserve zeros by falling back to previous speeds
            for i in range(len(self.motor_speed)):
                if new_speeds[i] == 0:
                    new_speeds[i] = self.motor_speed[i]
                else:
                    new_speeds[i] = new_speeds[i] * self.motor_speed[i]
            self.set_speed(new_speeds, overwrite_speeds=False)

    def set_speed(self, speeds, overwrite_speeds=True):
        speeds = np.asarray(speeds, dtype=float)
        if overwrite_speeds:
            self.motor_speed = speeds.copy()

        for i, dxl_id in enumerate(self.motor_ids):
            s = float(speeds[i])
            if 0 < s <= 1:
                raw = int(round(s * 1023))
                self._write2(dxl_id, self.ADDR_MX_MOVING_SPEED, raw)
            else:
                print("Movement speed out of range, enter value between (0, 1].")

    def set_torque_limit(self, torques):
        torques = np.asarray(torques, dtype=float)
        self.motor_torque = torques.copy()
        for i, dxl_id in enumerate(self.motor_ids):
            t = float(torques[i])
            if 0 < t <= 1:
                self._write2(dxl_id, self.ADDR_MX_TORQUE_LIMIT, int(round(t * 1023)))
            else:
                print("Torque limit out of range, enter value between (0, 1].")

    # --------------------------------- Power ----------------------------------
    def enable_motors(self):
        for dxl_id in self.motor_ids:
            self._write1(dxl_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("Dynamixel torque enabled on all motors.")

    def disable_motors(self):
        for dxl_id in self.motor_ids:
            try:
                self._write1(dxl_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)
            except Exception as e:
                print(f"Disable motor {dxl_id} error: {e}")
        try:
            self.port_handler.closePort()
        finally:
            self.init_status = 0
        print("Dynamixels disabled and port closed.")

    # ------------------------------- Telemetry --------------------------------
    def get_position(self, motor_id):
        ticks = self._read2(motor_id, self.ADDR_MX_PRESENT_POSITION)
        return self.ticks_to_deg(ticks)

    def read_joint_angles(self):
        j_a = np.zeros(4, dtype=float)
        for i, dxl_id in enumerate(self.motor_ids):
            ticks = self._read2(dxl_id, self.ADDR_MX_PRESENT_POSITION)
            deg = self.ticks_to_deg(ticks) - self.joint_offsets[i]
            j_a[i] = deg
            self.joint_angle_error[i] = deg - self.joint_angles[i]
        return j_a

    # --------------------------- Kinematics (FK / IK) -------------------------
   # Please input your forward and inverse kinematics here for position and pose conversion to joint angles.
   #  The output should be in the format of an array

    # ------------------------------- Motion -----------------------------------
    def move_j(self, j1_deg, j2_deg, j3_deg, j4_deg, wait=True, tol_deg=2.0, poll_period=0.05):
        # Limits
        desired = np.array([j1_deg, j2_deg, j3_deg, j4_deg], dtype=float)
        for i in range(4):
            desired[i] = self._check_limits_one(desired[i], i)

        # Smooth speed option
        if self.use_smooth_speed_flag:
            self.smooth_speed(desired - self.joint_angles)

        # Update internal & FK
        #self.joint_angles = desired.copy()
        #self.forward(desired)

        # Apply motor offsets for command
        cmd_deg = desired + self.joint_offsets
        for i, dxl_id in enumerate(self.motor_ids):
            self._write2(dxl_id, self.ADDR_MX_GOAL_POSITION, self.deg_to_ticks(cmd_deg[i]))

        if not wait:
            return


        # Wait until all joints within tolerance
        while True:
            try:
                ja = self.read_joint_angles()
            except Exception:
                # If read fails transiently, just try again shortly.
                time.sleep(poll_period)
                continue
            if np.all(np.abs(ja - desired) < tol_deg):
                break
            time.sleep(poll_period)

    #def move_c(self, x, y, z, pitch_deg, **kwargs):
     #   j_deg = self.inverse(x, y, z, math.radians(pitch_deg))
      #  self.move_j(j_deg[0], j_deg[1], j_deg[2], j_deg[3], **kwargs)

    #def read_ee_position(self):
     #   j_a = self.read_joint_angles()
      #  return self.forward(j_a)

    # --------------------------- History / Macros ------------------------------
    def record_configuration(self):
        entry = {
            "j1": float(self.joint_angles[0]),
            "j2": float(self.joint_angles[1]),
            "j3": float(self.joint_angles[2]),
            "j4": float(self.joint_angles[3]),
            "speed": float(self.motor_speed[0]),
            "torque": float(self.motor_torque[0]),
            "gripper_open": bool(self.gripper_open_flag),
        }
        self.movement_history.append(entry)
        print(
            f"Recorded Speed: {entry['speed']:.3f}, Torque: {entry['torque']:.3f}\n"
            f"Joint Positions: {entry['j1']:.2f}, {entry['j2']:.2f}, {entry['j3']:.2f}, {entry['j4']:.2f}\n"
            f"Gripper open: {int(entry['gripper_open'])}"
        )

    def delete_last_recorded_configuration(self):
        if not self.movement_history:
            print("No last history position")
            return
        self.movement_history.pop()

    def play_configuration_history(self, dwell_after_move_s=1.0, dwell_after_grip_s=3.0):
        if not self.movement_history:
            print("No recorded configurations.")
            return
        for rec in self.movement_history:
            s = rec["speed"]
            t = rec["torque"]
            self.set_speed([s, s, s, s], overwrite_speeds=True)
            self.set_torque_limit([t, t, t, t])
            self.move_j(rec["j1"], rec["j2"], rec["j3"], rec["j4"])
            time.sleep(dwell_after_move_s)
            if self.gripper_open_flag != rec["gripper_open"]:
                self.actuate_gripper()
                time.sleep(dwell_after_grip_s)


# ------------------------------- Example usage --------------------------------
# Example 1:
# Demonstrates how to move the robot joints into desired joint positions [degrees]
# You might need to adjust the offsets in the code above to make the zero point suit your needs.
# Just comment out the code snippet, input desired joint angles in robot.move_j() and run.
'''
if __name__ == "__main__":
    robot = MyRobot()
    if robot.init_status:
        try:

            #Reading off position for calibration:
            joint_angles = robot.read_joint_angles()
            print("Joint angles (deg):", joint_angles)


            # Example motions:
            robot.move_j(0, -45, 0, 0) #Degrees

            robot.record_configuration()

            # Play it back
            #robot.play_configuration_history()

        finally:
            pass
            #robot.disable_motors()
'''
#Example 2:
#Demonstrates how to move the robot joints into desired a list of joint positions [degrees]
#You might need to adjust the offsets in the code above to make the zero point suit your needs.
#Just comment out the code snippet, input desired joint angle sequence in robot.move_j() and run.

'''
if __name__ == "__main__":
    import time
    import numpy as np

    # Create robot instance
    robot = MyRobot()

    if robot.init_status:
        try:
            # ----------------- Define the sequence of joint angles -----------------
            # Each row = [J1, J2, J3, J4] in DEGREES
            # Adjust these based on your mechanical limits / workspace
            joint_sequence = np.array([
                [-12.02, -7.99, -95.76, 13.75],
                [-11.75, -7.21, -93.14, 10.35],
                [-11.24, -6.60, -90.47, 7.07],
                [-10.49, -6.14, -87.85, 3.99],
                [-9.33, -5.47, -85.76, 1.24],
                [-7.69, -5.20, -83.63, -1.17],
                [-5.96, -5.05, -81.82, -3.13],
                [-4.25, -4.67, -80.78, -4.55],
                [-2.04, -4.61, -79.95, -5.44],
                [-0.00, -4.60, -79.66, -5.74],
                [2.04, -4.61, -79.95, -5.44],
                [4.25, -4.67, -80.78, -4.55],
                [5.96, -5.05, -81.82, -3.13],
                [7.69, -5.20, -83.63, -1.17],
                [9.33, -5.47, -85.76, 1.24],
                [10.49, -6.14, -87.85, 3.99],
                [11.24, -6.60, -90.47, 7.07],
                [11.75, -7.21, -93.14, 10.35],
                [12.02, -7.99, -95.76, 13.75],
                [11.75, -8.85, -98.32, 17.17],
                [11.24, -9.82, -100.68, 20.50],
                [10.49, -10.87, -102.80, 23.67],
                [9.33, -11.60, -105.02, 26.62],
                [7.69, -12.54, -106.61, 29.14],
                [5.96, -13.37, -107.84, 31.21],
                [4.25, -13.76, -109.06, 32.82],
                [2.04, -14.16, -109.60, 33.76],
                [-0.00, -14.30, -109.78, 34.08],
                [-2.04, -14.16, -109.60, 33.76],
                [-4.25, -13.76, -109.06, 32.82],
                [-5.96, -13.37, -107.84, 31.21],
                [-7.69, -12.54, -106.61, 29.14],
                [-9.33, -11.60, -105.02, 26.62],
                [-10.49, -10.87, -102.80, 23.67],
                [-11.24, -9.82, -100.68, 20.50],
                [-11.75, -8.85, -98.32, 17.17],
                [-12.02, -7.99, -95.76, 13.75]
                # return home
            ])

            # ----------------- Run the motion sequence -----------------
            robot.enable_motors()
            print("\nStarting joint sequence...")
            time.sleep(1.0)

            #Number of loops
            loops = 5

            for loop in range(loops):
                print(f"\n Loop {loop + 1} of {loops} ----------------------------")
                for idx, pose in enumerate(joint_sequence):
                    print(f"\n  Moving to pose {idx+1}: {pose}°")
                    robot.move_j(*pose)
                    robot.record_configuration()  # save to history
                    # Read and print measured joint positions
                    measured = robot.read_joint_angles()
                    print("Measured (deg):", np.round(measured, 2))
                    time.sleep(0)  # pause between poses



        except Exception as e:
            print("Error during sequence:", e)

        finally:
            robot.disable_motors()
            print("\nMotors disabled. Program finished.")
'''
