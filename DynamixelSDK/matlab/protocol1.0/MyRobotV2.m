classdef MyRobotV2 < handle
    % MyRobotV2 - MATLAB translation of the Python MyRobot class (ASCII-safe)
    % Includes two static example methods: example1 and example2.
    %
    % Usage:
    %   MyRobotV2.example1('COM3');
    %   MyRobotV2.example2('COM3', 5);
    %
    % Or create object and use methods:
    %   robot = MyRobotV2('COM3');
    %   robot.move_j(0,-45,0,0);
    %   robot.disable_motors();
    %

    properties (Constant)
        % Constants (AX-12A)
        ADDR_MX_TORQUE_ENABLE = int32(24);
        ADDR_MX_GOAL_POSITION = int32(30);
        ADDR_MX_MOVING_SPEED = int32(32);
        ADDR_MX_TORQUE_LIMIT = int32(34);
        ADDR_MX_PRESENT_POSITION = int32(36);

        PROTOCOL_VERSION = 1.0;
        BAUDRATE = int32(1000000);

        TORQUE_ENABLE = int32(1);
        TORQUE_DISABLE = int32(0);
        COMM_SUCCESS = int32(0);

        TICKS_PER_DEG = 1.0 / 0.29;
        DEG_PER_TICK = 0.29;
    end

    properties
        DEVICENAME = 'COM8'; % default, can be overridden in constructor
        motor_ids = int32([1,2,3,4]);
        gripper_motor_id = int32(5);

        forward_transform = zeros(4,4);
        joint_angles = zeros(1,4);
        joint_pos = zeros(4,4);

        draw_robot_flag = false;
        use_smooth_speed_flag = false;
        gripper_open_flag = true;

        joint_limits = [-90 90; -180 180; -140 140; -140 140];
        ik_weights = [0.25 0.25 0.25 1 1 1];

        joint_offsets = 150 * ones(1,4);
        joint_angle_error = zeros(1,4);
        init_status = 0;

        movement_history = {};
        motor_speed = 0.1 * ones(1,4);
        motor_torque = ones(1,4);

        port_handler;
        packet_handler;
    end

    methods (Static)
        function out = clamp(v, lo, hi)
            out = max(lo, min(hi, v));
        end

        function ticks = deg_to_ticks(deg)
            ticks = int32(round(deg * MyRobotV2.TICKS_PER_DEG));
        end

        function deg = ticks_to_deg(ticks)
            deg = double(ticks) * MyRobotV2.DEG_PER_TICK;
        end

        function example1(devicename)
            % Simple single-move example
            if nargin < 1 || isempty(devicename)
                devicename = 'COM8';
            end
            robot = MyRobotV2(devicename);
            if ~robot.init_status
                fprintf('Robot failed to initialize in example1. Exiting.\n');
                return
            end
            try
                % Read current joint angles for calibration
                joint_angles = robot.read_joint_angles();
                fprintf('Joint angles (deg): [%.2f %.2f %.2f %.2f]\n', joint_angles);

                % Example motion
                robot.move_j(0, -45, 0, 0);

                % Record current configuration into history
                robot.record_configuration();

                % Optionally play back: robot.play_configuration_history();
            catch ME
                fprintf('Error in example1: %s\n', ME.message);
            end

            % Always disable motors on exit
            robot.disable_motors();
        end

        function example2(devicename, loops)
            % Sequence example using a long joint trajectory.
            % Usage: MyRobotV2.example2('COM3', 3)
            if nargin < 1 || isempty(devicename)
                devicename = 'COM8';
            end
            if nargin < 2 || isempty(loops)
                loops = 5;
            end

            robot = MyRobotV2(devicename);
            if ~robot.init_status
                fprintf('Robot failed to initialize in example2. Exiting.\n');
                return
            end

            % Joint sequence translated from Python list (each row = [J1 J2 J3 J4])
            joint_sequence = [
                -12.02, -7.99,  -95.76, 13.75;
                -11.75, -7.21,  -93.14, 10.35;
                -11.24, -6.60,  -90.47, 7.07;
                -10.49, -6.14,  -87.85, 3.99;
                -9.33,  -5.47,  -85.76, 1.24;
                -7.69,  -5.20,  -83.63, -1.17;
                -5.96,  -5.05,  -81.82, -3.13;
                -4.25,  -4.67,  -80.78, -4.55;
                -2.04,  -4.61,  -79.95, -5.44;
                0.00,   -4.60,  -79.66, -5.74;
                2.04,   -4.61,  -79.95, -5.44;
                4.25,   -4.67,  -80.78, -4.55;
                5.96,   -5.05,  -81.82, -3.13;
                7.69,   -5.20,  -83.63, -1.17;
                9.33,   -5.47,  -85.76, 1.24;
                10.49,  -6.14,  -87.85, 3.99;
                11.24,  -6.60,  -90.47, 7.07;
                11.75,  -7.21,  -93.14, 10.35;
                12.02,  -7.99,  -95.76, 13.75;
                11.75,  -8.85,  -98.32, 17.17;
                11.24,  -9.82, -100.68, 20.50;
                10.49, -10.87, -102.80, 23.67;
                9.33,  -11.60, -105.02, 26.62;
                7.69,  -12.54, -106.61, 29.14;
                5.96,  -13.37, -107.84, 31.21;
                4.25,  -13.76, -109.06, 32.82;
                2.04,  -14.16, -109.60, 33.76;
                0.00,  -14.30, -109.78, 34.08;
               -2.04,  -14.16, -109.60, 33.76;
               -4.25,  -13.76, -109.06, 32.82;
               -5.96,  -13.37, -107.84, 31.21;
               -7.69,  -12.54, -106.61, 29.14;
               -9.33,  -11.60, -105.02, 26.62;
               -10.49, -10.87, -102.80, 23.67;
               -11.24,  -9.82, -100.68, 20.50;
               -11.75,  -8.85,  -98.32, 17.17;
               -12.02,  -7.99,  -95.76, 13.75
            ];

            try
                robot.enable_motors();
                fprintf('\nStarting joint sequence...\n');
                pause(1.0);

                for loop = 1:loops
                    fprintf('\nLoop %d of %d ----------------------------\n', loop, loops);
                    for idx = 1:size(joint_sequence, 1)
                        pose = joint_sequence(idx, :);
                        fprintf('Moving to pose %d: [%.2f %.2f %.2f %.2f]\n', idx, pose);
                        robot.move_j(pose(1), pose(2), pose(3), pose(4));
                        robot.record_configuration();
                        measured = robot.read_joint_angles();
                        fprintf('Measured (deg): [%.2f %.2f %.2f %.2f]\n', measured);
                        pause(0); % no delay between poses by default
                    end
                end
            catch ME
                fprintf('Error during sequence: %s\n', ME.message);
            end

            % always disable motors at the end
            robot.disable_motors();
            fprintf('\nMotors disabled. Example finished.\n');
        end
    end

    methods
        function obj = MyRobotV2(devicename)
            % Constructor: optionally provide device name (COM port)
            if nargin >= 1 && ~isempty(devicename)
                obj.DEVICENAME = devicename;
            end
            try
                % Add Dynamixel SDK path (adjust if you installed SDK elsewhere)
                thisfile = mfilename('fullpath');
                currentdir = fileparts(thisfile);
                parentdir = fileparts(currentdir);
                sdk_path = fullfile(parentdir,'DynamixelSDK-3.7.31','python','src','dynamixel_sdk');
                if count(py.sys.path, sdk_path) == 0
                    insert(py.sys.path, int32(0), sdk_path);
                end

                % Create Python SDK objects
                obj.port_handler = py.dynamixel_sdk.PortHandler(obj.DEVICENAME);
                obj.packet_handler = py.dynamixel_sdk.PacketHandler(obj.PROTOCOL_VERSION);

                opened = logical(obj.port_handler.openPort());
                if ~opened
                    error('Could not open port %s', obj.DEVICENAME);
                end
                fprintf('Port %s opened.\n', obj.DEVICENAME);

                br_ok = logical(obj.port_handler.setBaudRate(obj.BAUDRATE));
                if ~br_ok
                    error('Could not set baudrate on %s', obj.DEVICENAME);
                end
                fprintf('Baudrate set on %s.\n', obj.DEVICENAME);

                % Initialize defaults
                obj.set_speed(obj.motor_speed, true);
                obj.set_torque_limit(obj.motor_torque);

                % move to home, enable motors
                obj.move_j(0,0,0,0);
                obj.enable_motors();

                obj.init_status = 1;
            catch ME
                fprintf('Initialization error: %s\n', ME.message);
                obj.init_status = 0;
            end
        end

        function deg = check_limit(obj, deg, idx)
            lo = obj.joint_limits(idx,1);
            hi = obj.joint_limits(idx,2);
            if ~(deg >= lo && deg <= hi)
                error('Joint %d angle out of range.', idx);
            end
        end

        function _write1(obj, dxl_id, addr, value)
            res = cell(obj.packet_handler.write1ByteTxRx(obj.port_handler, int32(dxl_id), int32(addr), int32(value)));
            if length(res) >= 2
                dxl_comm_result = int32(res{1});
                dxl_error = int32(res{2});
                if dxl_comm_result ~= obj.COMM_SUCCESS || dxl_error ~= 0
                    error('DXL write1 error id=%d, addr=%d, result=%d, error=%d', dxl_id, addr, dxl_comm_result, dxl_error);
                end
            else
                error('Unexpected return from write1ByteTxRx');
            end
        end

        function _write2(obj, dxl_id, addr, value)
            res = cell(obj.packet_handler.write2ByteTxRx(obj.port_handler, int32(dxl_id), int32(addr), int32(value)));
            if length(res) >= 2
                dxl_comm_result = int32(res{1});
                dxl_error = int32(res{2});
                if dxl_comm_result ~= obj.COMM_SUCCESS || dxl_error ~= 0
                    error('DXL write2 error id=%d, addr=%d, result=%d, error=%d', dxl_id, addr, dxl_comm_result, dxl_error);
                end
            else
                error('Unexpected return from write2ByteTxRx');
            end
        end

        function val = _read2(obj, dxl_id, addr)
            res = cell(obj.packet_handler.read2ByteTxRx(obj.port_handler, int32(dxl_id), int32(addr)));
            if length(res) >= 3
                value_py = res{1};
                dxl_comm_result = int32(res{2});
                dxl_error = int32(res{3});
                if dxl_comm_result ~= obj.COMM_SUCCESS || dxl_error ~= 0
                    error('DXL read2 error id=%d, addr=%d, result=%d, error=%d', dxl_id, addr, dxl_comm_result, dxl_error);
                end
                val = double(value_py);
            else
                error('Unexpected return from read2ByteTxRx');
            end
        end

        function smooth_speed(obj, joint_angles_delta_deg)
            max_angle = max(abs(joint_angles_delta_deg));
            if max_angle == 0
                return
            end
            speed_per_deg = max_angle / 100.0;
            if speed_per_deg ~= 0
                new_speeds = abs(joint_angles_delta_deg / speed_per_deg) * 0.01;
                for i = 1:length(obj.motor_speed)
                    if new_speeds(i) == 0
                        new_speeds(i) = obj.motor_speed(i);
                    else
                        new_speeds(i) = new_speeds(i) * obj.motor_speed(i);
                    end
                end
                obj.set_speed(new_speeds, false);
            end
        end

        function set_speed(obj, speeds, overwrite_speeds)
            if nargin < 3
                overwrite_speeds = true;
            end
            speeds = double(speeds);
            if overwrite_speeds
                obj.motor_speed = speeds;
            end
            for i = 1:length(obj.motor_ids)
                s = double(speeds(i));
                if s > 0 && s <= 1
                    raw = int32(round(s * 1023));
                    obj._write2(obj.motor_ids(i), obj.ADDR_MX_MOVING_SPEED, raw);
                else
                    fprintf('Movement speed out of range, enter value between (0, 1].\n');
                end
            end
        end

        function set_torque_limit(obj, torques)
            torques = double(torques);
            obj.motor_torque = torques;
            for i = 1:length(obj.motor_ids)
                t = double(torques(i));
                if t > 0 && t <= 1
                    obj._write2(obj.motor_ids(i), obj.ADDR_MX_TORQUE_LIMIT, int32(round(t * 1023)));
                else
                    fprintf('Torque limit out of range, enter value between (0, 1].\n');
                end
            end
        end

        function enable_motors(obj)
            for dxl_id = obj.motor_ids
                obj._write1(dxl_id, obj.ADDR_MX_TORQUE_ENABLE, obj.TORQUE_ENABLE);
            end
            fprintf('Dynamixel torque enabled on all motors.\n');
        end

        function disable_motors(obj)
            for dxl_id = obj.motor_ids
                try
                    obj._write1(dxl_id, obj.ADDR_MX_TORQUE_ENABLE, obj.TORQUE_DISABLE);
                catch ME
                    fprintf('Disable motor %d error: %s\n', dxl_id, ME.message);
                end
            end
            try
                obj.port_handler.closePort();
            end
            obj.init_status = 0;
            fprintf('Dynamixels disabled and port closed.\n');
        end

        function deg = get_position(obj, motor_id)
            ticks = obj._read2(motor_id, obj.ADDR_MX_PRESENT_POSITION);
            deg = obj.ticks_to_deg(ticks);
        end

        function j_a = read_joint_angles(obj)
            j_a = zeros(1,4);
            for i = 1:length(obj.motor_ids)
                ticks = obj._read2(obj.motor_ids(i), obj.ADDR_MX_PRESENT_POSITION);
                deg = obj.ticks_to_deg(ticks) - obj.joint_offsets(i);
                j_a(i) = deg;
                obj.joint_angle_error(i) = deg - obj.joint_angles(i);
            end
        end

        function move_j(obj, j1_deg, j2_deg, j3_deg, j4_deg, wait, tol_deg, poll_period)
            if nargin < 6 || isempty(wait)
                wait = true;
            end
            if nargin < 7 || isempty(tol_deg)
                tol_deg = 2.0;
            end
            if nargin < 8 || isempty(poll_period)
                poll_period = 0.05;
            end

            desired = [j1_deg, j2_deg, j3_deg, j4_deg];
            for i = 1:4
                desired(i) = obj.check_limit(desired(i), i);
            end

            if obj.use_smooth_speed_flag
                obj.smooth_speed(desired - obj.joint_angles);
            end

            cmd_deg = desired + obj.joint_offsets;
            for i = 1:length(obj.motor_ids)
                obj._write2(obj.motor_ids(i), obj.ADDR_MX_GOAL_POSITION, obj.deg_to_ticks(cmd_deg(i)));
            end

            if ~wait
                return
            end

            while true
                try
                    ja = obj.read_joint_angles();
                catch
                    pause(poll_period);
                    continue
                end
                if all(abs(ja - desired) < tol_deg)
                    break
                end
                pause(poll_period);
            end
        end

        function record_configuration(obj)
            entry.j1 = double(obj.joint_angles(1));
            entry.j2 = double(obj.joint_angles(2));
            entry.j3 = double(obj.joint_angles(3));
            entry.j4 = double(obj.joint_angles(4));
            entry.speed = double(obj.motor_speed(1));
            entry.torque = double(obj.motor_torque(1));
            entry.gripper_open = logical(obj.gripper_open_flag);
            obj.movement_history{end+1} = entry;
            fprintf('Recorded Speed: %.3f, Torque: %.3f\nJoint Positions: %.2f, %.2f, %.2f, %.2f\nGripper open: %d\n', ...
                entry.speed, entry.torque, entry.j1, entry.j2, entry.j3, entry.j4, int32(entry.gripper_open));
        end

        function delete_last_recorded_configuration(obj)
            if isempty(obj.movement_history)
                fprintf('No last history position\n');
                return
            end
            obj.movement_history(end) = [];
        end

        function play_configuration_history(obj, dwell_after_move_s, dwell_after_grip_s)
            if nargin < 2 || isempty(dwell_after_move_s)
                dwell_after_move_s = 1.0;
            end
            if nargin < 3 || isempty(dwell_after_grip_s)
                dwell_after_grip_s = 3.0;
            end
            if isempty(obj.movement_history)
                fprintf('No recorded configurations.\n');
                return
            end
            for k = 1:length(obj.movement_history)
                rec = obj.movement_history{k};
                s = rec.speed; t = rec.torque;
                obj.set_speed([s s s s], true);
                obj.set_torque_limit([t t t t]);
                obj.move_j(rec.j1, rec.j2, rec.j3, rec.j4);
                pause(dwell_after_move_s);
                if obj.gripper_open_flag ~= rec.gripper_open
                    % Placeholder for gripper actuation
                    pause(dwell_after_grip_s);
                end
            end
        end
    end
end
