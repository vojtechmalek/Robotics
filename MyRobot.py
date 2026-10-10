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
    BAUDRATE = 1000000
    DEVICENAME = "COM3"              # e.g., Windows: 'COM3' | Linux: '/dev/ttyUSB0' | Mac: '/dev/tty.usbserial-*'

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

    # ------------------------------ Helper Functions -----------------------------

    def effective_joint_limits(self):
        """Intersect configured limits with representable AX-12A position commands."""
        limits = np.asarray(self.joint_limits, dtype=float).copy()
        offsets = np.asarray(self.joint_offsets, dtype=float)
        limits[:, 0] = np.maximum(limits[:, 0], -offsets)
        limits[:, 1] = np.minimum(limits[:, 1], self.ticks_to_deg(1023) - offsets)
        if np.any(limits[:, 0] > limits[:, 1]):
            raise ValueError('Configured limits and servo offsets are incompatible.')
        return limits

    def _checked_angles(self, q_deg):
        q = np.asarray(q_deg, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected four finite joint angles in degrees.')
        limits = self.effective_joint_limits()
        for i, angle in enumerate(q):
            if not limits[i, 0] <= angle <= limits[i, 1]:
                raise ValueError(f'Joint {i + 1}: {angle:.3f} deg outside {limits[i]}.')
            self._check_limits_one(float(angle), i)
            ticks = self.deg_to_ticks(float(angle + self.joint_offsets[i]))
            if not 0 <= ticks <= 1023:
                raise ValueError(f'Joint {i + 1}: invalid servo goal {ticks}.')
        return q.copy()

    def _move_checked(self, q_deg, timeout=20.0, tol_deg=2.0):
        """Use move_j, but implement bounded feedback waiting outside its infinite loop."""
        if not self.init_status:
            raise RuntimeError('Robot initialization failed.')
        q = self._checked_angles(q_deg)
        self.move_j(*q, wait=False)
        self.joint_angles = q.copy()
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                measured = np.asarray(self.read_joint_angles(), dtype=float)
                if measured.shape == (4,) and np.all(np.isfinite(measured)):
                    if np.all(np.abs(measured - q) < tol_deg):
                        return measured
            except (IOError, OSError) as exc:
                last_error = exc
            time.sleep(0.05)
        self.disable_motors()
        raise TimeoutError(f'Motion timed out for {q}; last read error: {last_error}')


    # ------------------------------- Forward Kinematics -----------------------------------
    def forward_kinematics(self, q_deg, check_limits=True):
        """Return (T04, T05). Degrees in, mm translations out."""
        q = self._checked_angles(q_deg) if check_limits else np.asarray(q_deg, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)):
            raise ValueError('Expected four finite joint angles in degrees.')
        q1, q2, q3, q4 = np.deg2rad(q)
        beta = q2 + q3 + q4
        c1, s1 = (np.cos(q1), np.sin(q1))
        cb, sb = (np.cos(beta), np.sin(beta))
        r = -93 * np.sin(q2) - 93 * np.sin(q2 + q3) - 50 * sb
        h = 50 + 93 * np.cos(q2) + 93 * np.cos(q2 + q3) + 50 * cb
        T04 = np.array([[-c1 * sb, -c1 * cb, s1, c1 * r], [-s1 * sb, -s1 * cb, -c1, s1 * r], [cb, -sb, 0, h], [0, 0, 0, 1]])
        T45 = np.eye(4)
        T45[:3, 3] = [-15, 45, 0]
        return (T04, T04 @ T45)

    def end_effector_position(self, q1, q2, q3, q4):
        return self.forward_kinematics([q1, q2, q3, q4])[0][:3, 3].copy()

    def camera_position(self, q1, q2, q3, q4):
        return self.forward_kinematics([q1, q2, q3, q4])[1][:3, 3].copy()

    # ------------------------------- Inverse Kinematics -----------------------------------
    def inverse_kinematics(self, position, k, reference_q=None):
        """Return ALL limit-valid solutions as (N,4) degrees, nearest reference first.

        k is the desired vertical component of the stylus x-axis (0=horizontal).
        A reference resolves free angles at singularities when possible.
        """
        p = np.asarray(position, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError('position must be three finite coordinates in mm.')
        k = float(k)
        if not np.isfinite(k) or not -1 <= k <= 1:
            raise ValueError('k must lie in [-1, 1].')
        limits = self.effective_joint_limits()
        ref = None if reference_q is None else self._checked_angles(reference_q)
        X, Y, Z = p
        rho = np.hypot(X, Y)
        if rho < 1e-09:
            yaw = np.deg2rad(ref[0]) if ref is not None else np.deg2rad(np.clip(0, *limits[0]))
            bases = [(yaw, 0.0)]
        else:
            yaw = np.arctan2(Y, X)
            bases = [(yaw, rho), (yaw + np.pi, -rho)]
        results = []
        for q1, r in bases:
            for beta in (np.arccos(k), -np.arccos(k)):
                rw, zw = (r + 50 * np.sin(beta), Z - 50 - 50 * np.cos(beta))
                D = (rw * rw + zw * zw - 2 * 93 ** 2) / (2 * 93 ** 2)
                if not -1 - 1e-09 <= D <= 1 + 1e-09:
                    continue
                elbow = np.arccos(np.clip(D, -1, 1))
                for q3 in (elbow, -elbow):
                    if np.hypot(rw, zw) < 1e-09:
                        total = np.rad2deg(beta - q3)
                        pairs = []
                        for shift in range(-3, 4):
                            b = total + 360 * shift
                            lo = max(limits[1, 0], b - limits[3, 1])
                            hi = min(limits[1, 1], b - limits[3, 0])
                            if lo <= hi:
                                pairs.append(np.deg2rad(np.clip(ref[1] if ref is not None else 0, lo, hi)))
                    else:
                        pairs = [np.arctan2(-rw, zw) - np.arctan2(93 * np.sin(q3), 93 + 93 * np.cos(q3))]
                    for q2 in pairs:
                        raw = np.rad2deg([q1, q2, q3, beta - q2 - q3])
                        choices = []
                        for angle, (lo, hi) in zip(raw, limits):
                            first = int(np.ceil((lo - angle - 1e-08) / 360))
                            last = int(np.floor((hi - angle + 1e-08) / 360))
                            choices.append([np.clip(angle + 360 * n, lo, hi) for n in range(first, last + 1)])
                        for candidate in itertools.product(*choices):
                            try:
                                candidate = self._checked_angles(candidate)
                            except ValueError:
                                continue
                            if not any((np.allclose(candidate, old, atol=1e-07, rtol=0) for old in results)):
                                results.append(candidate)
        if not results:
            raise ValueError('No inverse-kinematics solution within joint/servo limits.')
        Q = np.array(results)
        if ref is not None:
            Q = Q[np.argsort(np.linalg.norm(Q - ref, axis=1))]
        return Q

    # ------------------------------- Circle -----------------------------------
    def circle_configurations(self, center=(150, 0, 120), radius=32, segments=36, initial_q=None, max_step_deg=20):
            """Return (phi radians, positions mm, Q degrees); validate before any move.

            Circle is parallel to yz, stylus horizontal. Reject discontinuous sequences.
            """
            center = np.asarray(center, dtype=float)
            if center.shape != (3,) or not np.all(np.isfinite(center)):
                raise ValueError('center must contain three finite coordinates.')
            if not np.isfinite(radius) or radius <= 0:
                raise ValueError('radius must be positive and finite.')
            if not isinstance(segments, (int, np.integer)) or segments < 3:
                raise ValueError('segments must be an integer >= 3.')
            if not np.isfinite(max_step_deg) or max_step_deg <= 0:
                raise ValueError('max_step_deg must be positive and finite.')
            ref = None if initial_q is None else self._checked_angles(initial_q)
            phi = np.linspace(0, 2 * np.pi, segments + 1)
            positions = center + np.column_stack((np.zeros_like(phi), radius * np.cos(phi), radius * np.sin(phi)))
            positions[-1] = positions[0]
            candidates = [self.inverse_kinematics(p, k=0) for p in positions]
            paths = []
            for start in candidates[0]:
                path = [start]
                for choices in candidates[1:-1]:
                    chosen = choices[np.argmin(np.linalg.norm(choices - path[-1], axis=1))]
                    if np.max(np.abs(chosen - path[-1])) > max_step_deg:
                        break
                    path.append(chosen)
                if len(path) != segments:
                    continue
                if np.max(np.abs(start - path[-1])) > max_step_deg:
                    continue
                path.append(start.copy())
                Q = np.array(path)
                cost = np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1))
                if ref is not None:
                    cost += np.linalg.norm(start - ref)
                paths.append((cost, Q))
            if not paths:
                raise ValueError('No continuous limit-valid circle found; try a smaller radius.')
            Q = min(paths, key=lambda item: item[0])[1]
            for p, q in zip(positions, Q):
                T04, _ = self.forward_kinematics(q)
                if not np.allclose(T04[:3, 3], p, atol=1e-06, rtol=0):
                    raise RuntimeError('Circle position validation failed.')
                if abs(T04[2, 0]) > 1e-08:
                    raise RuntimeError('Circle orientation validation failed.')
            return (phi, positions, Q)

    #  ------------------------------ Tests -----------------------------------
    def test_random_fk_ik(self, rng=None, pause_s=1.0):
        """Random joints -> home -> IK of the same position AND k -> same joints."""
        rng = np.random.default_rng() if rng is None else rng
        limits = self.effective_joint_limits()
        low = np.maximum(limits[:, 0] + 2, [-40, -35, -110, -15])
        high = np.minimum(limits[:, 1] - 2, [40, -5, -60, 35])
        if np.any(low >= high):
            raise ValueError('Configured limits exclude the test sampling region.')
        self._checked_angles([0, 0, 0, 0])
        q_random = rng.uniform(low, high)
        T04, _ = self.forward_kinematics(q_random)
        position, k = (T04[:3, 3].copy(), float(T04[2, 0]))
        q_ik = self.inverse_kinematics(position, k, reference_q=q_random)[0]
        np.testing.assert_allclose(q_ik, q_random, atol=1e-06, rtol=0)
        print('Random q (deg):', q_random)
        first_measured = self._move_checked(q_random)
        time.sleep(pause_s)
        self._move_checked([0, 0, 0, 0])
        time.sleep(pause_s)
        print('FK position (mm):', position, 'k:', k)
        print('Recovered IK q (deg):', q_ik)
        second_measured = self._move_checked(q_ik)
        time.sleep(pause_s)
        print('Measured first/second (deg):', first_measured, second_measured)
        return dict(q_random=q_random, position=position, k=k, q_ik=q_ik, first_measured=first_measured, second_measured=second_measured)

    def test_random_circle(self, rng=None, segments=72, pause_s=0.05):
        """Sample and prevalidate a random circle, then visit every point once."""
        rng = np.random.default_rng() if rng is None else rng
        reference = np.asarray(self.read_joint_angles(), dtype=float)
        self._checked_angles(reference)
        for _ in range(100):
            center = rng.uniform([140, -10, 115], [165, 10, 140])
            radius = float(rng.uniform(8, 20))
            try:
                phi, positions, Q = self.circle_configurations(center, radius, segments, initial_q=reference)
                break
            except ValueError:
                continue
        else:
            raise ValueError('Could not find a random circle within the configured limits.')
        print('Circle center (mm):', center, 'radius (mm):', radius)
        for q in Q:
            self._move_checked(q)
            time.sleep(pause_s)
        return dict(center=center, radius=radius, phi=phi, positions=positions, Q=Q)
    
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
            robot.move_j(-12.02, -7.99, -95.76, 13.75) #Degrees

            robot.record_configuration()

            # Play it back
            #robot.play_configuration_history()

        finally:
            pass
            #robot.disable_motors()
'''

'''
#Example 2:
#Demonstrates how to move the robot joints into desired a list of joint positions [degrees]
#You might need to adjust the offsets in the code above to make the zero point suit your needs.
#Just comment out the code snippet, input desired joint angle sequence in robot.move_j() and run.


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
            loops = 15

            for loop in range(loops):
                print(f"\n🔁 Loop {loop + 1} of {loops} ----------------------------")
                for idx, pose in enumerate(joint_sequence):
                    print(f"\n➡️  Moving to pose {idx+1}: {pose}°")
                    robot.move_j(*pose)
                    robot.record_configuration()  # save to history
                    # Read and print measured joint positions
                    measured = robot.read_joint_angles()
                    print("Measured (deg):", np.round(measured, 2))
                    time.sleep(0)  # pause between poses
            
            robot.move_j(0, 0, 0, 0) #Degrees
            time.sleep(0.25)



        except Exception as e:
            print("Error during sequence:", e)

        finally:
            robot.disable_motors()
            print("\nMotors disabled. Program finished.")
'''


if __name__ == "__main__":
    import time
    import numpy as np

    # Create robot instance
    robot = MyRobot()

    if robot.init_status:
        try:
            robot.set_speed([0.05] * 4, overwrite_speeds=True)

            result1 = robot.test_random_fk_ik()
            result2 = robot.test_random_circle()

            robot._move_checked([0, 0, 0, 0])
        finally:
            robot.disable_motors()
