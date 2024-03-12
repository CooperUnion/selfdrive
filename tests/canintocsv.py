import csv
import os
import re
from datetime import datetime, timezone


def decode_node_info(message_data):
    try:
        match = re.search(
            r'.*THROTTLE_gitHash: (\d+), THROTTLE_gitDirty: (\d+), THROTTLE_eepromIdentity: (\d+)',
            message_data,
        )
        if match:
            git_hash = int(match.group(1))
            git_dirty = int(match.group(2))
            eeprom_identity = int(match.group(3))
            return git_hash, git_dirty, eeprom_identity
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding THROTTLE_NodeInfo message: {e}")
        return None  # Return None in case of an error


def decode_brake_node_info(message_data):
    try:
        match = re.search(
            r'.*BRAKE_gitHash: (\d+), BRAKE_gitDirty: (\d+), BRAKE_eepromIdentity: (\d+)',
            message_data,
        )
        if match:
            git_hash = int(match.group(1))
            git_dirty = int(match.group(2))
            eeprom_identity = int(match.group(3))
            return git_hash, git_dirty, eeprom_identity
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding BRAKE_NodeInfo message: {e}")
        return None


def decode_ctrl_node_info(message_data):
    try:
        match = re.search(
            r'.*CTRL_gitHash: (\d+), CTRL_gitDirty: (\d+), CTRL_eepromIdentity: (\d+)',
            message_data,
        )
        if match:
            git_hash = int(match.group(1))
            git_dirty = int(match.group(2))
            eeprom_identity = int(match.group(3))
            return git_hash, git_dirty, eeprom_identity
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding CTRL_NodeInfo message: {e}")
        return None


def decode_throttle_node_status(message_data):
    try:
        match = re.search(
            r'THROTTLE_sysStatus: (\w+), THROTTLE_requestedSysStatus: (\w+), THROTTLE_temperature: (\d+), THROTTLE_counter: (\d+), THROTTLE_resetReason: (\w+), THROTTLE_esp32ResetReasonCode: (\d+)',
            message_data,
        )
        if match:
            sys_status = match.group(1)
            requested_sys_status = match.group(2)
            temperature = int(match.group(3))
            counter = int(match.group(4))
            reset_reason = match.group(5)
            esp32_reset_reason_code = int(match.group(6))
            return (
                sys_status,
                requested_sys_status,
                temperature,
                counter,
                reset_reason,
                esp32_reset_reason_code,
            )
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding THROTTLE_NodeStatus message: {e}")
        return None


def decode_brake_node_status(message_data):
    try:
        match = re.search(
            r'BRAKE_sysStatus: (\w+), BRAKE_requestedSysStatus: (\w+), BRAKE_temperature: (\d+), BRAKE_counter: (\d+), BRAKE_resetReason: (\w+), BRAKE_esp32ResetReasonCode: (\d+)',
            message_data,
        )
        if match:
            sys_status = match.group(1)
            requested_sys_status = match.group(2)
            temperature = int(match.group(3))
            counter = int(match.group(4))
            reset_reason = match.group(5)
            esp32_reset_reason_code = int(match.group(6))
            return (
                sys_status,
                requested_sys_status,
                temperature,
                counter,
                reset_reason,
                esp32_reset_reason_code,
            )
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding BRAKE_NodeStatus message: {e}")
        return None


def decode_ctrl_node_status(message_data):
    try:
        match = re.search(
            r'CTRL_sysStatus: (\w+), CTRL_requestedSysStatus: (\w+), CTRL_temperature: (\d+), CTRL_counter: (\d+), CTRL_resetReason: (\w+), CTRL_esp32ResetReasonCode: (\d+)',
            message_data,
        )
        if match:
            sys_status = match.group(1)
            requested_sys_status = match.group(2)
            temperature = int(match.group(3))
            counter = int(match.group(4))
            reset_reason = match.group(5)
            esp32_reset_reason_code = int(match.group(6))
            return (
                sys_status,
                requested_sys_status,
                temperature,
                counter,
                reset_reason,
                esp32_reset_reason_code,
            )
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding CTRL_NodeStatus message: {e}")
        return None


def decode_sup_authorization(message_data):
    try:
        match = re.search(
            r'SUP_brakeAuthorized: (\d+), SUP_throttleAuthorized: (\d+), SUP_steerAuthorized: (\d+)',
            message_data,
        )
        if match:
            brake_authorized = int(match.group(1))
            throttle_authorized = int(match.group(2))
            steer_authorized = int(match.group(3))
            return brake_authorized, throttle_authorized, steer_authorized
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding SUP_Authorization message: {e}")
        return None


def decode_sup_node_status(message_data):
    try:
        match = re.search(
            r'SUP_sysStatus: (\w+), SUP_requestedSysStatus: (\w+), SUP_temperature: (\d+), SUP_counter: (\d+), SUP_resetReason: (\w+), SUP_esp32ResetReasonCode: (\d+)',
            message_data,
        )
        if match:
            sys_status = match.group(1)
            requested_sys_status = match.group(2)
            temperature = int(match.group(3))
            counter = int(match.group(4))
            reset_reason = match.group(5)
            esp32_reset_reason_code = int(match.group(6))

            return (
                sys_status,
                requested_sys_status,
                temperature,
                counter,
                reset_reason,
                esp32_reset_reason_code,
            )
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding SUP_NodeStatus message: {e}")
        return None


def decode_sup_node_info(message_data):
    try:
        match = re.search(
            r'SUP_gitHash: (\d+), SUP_gitDirty: (\d), SUP_eepromIdentity: (\d+)',
            message_data,
        )
        if match:
            git_hash = int(match.group(1))
            git_dirty = int(match.group(2))
            eeprom_identity = int(match.group(3))

            return git_hash, git_dirty, eeprom_identity
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding SUP_NodeInfo message: {e}")
        return None


def decode_whl_absolute_encoder(message_data):
    try:
        match = re.search(
            r'WHL_foo: (\d+), WHL_encoder: (\d+), WHL_bar: (\d+)', message_data
        )
        if match:
            whl_foo = int(match.group(1))
            whl_encoder = int(match.group(2))
            whl_bar = int(match.group(3))

            return whl_foo, whl_encoder, whl_bar
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding WHL_AbsoluteEncoder message: {e}")
        return None


def decode_throttle_accel_data(message_data):
    try:
        match = re.search(
            r'THROTTLE_throttleADutyCycle: (\d+), THROTTLE_throttleFDutyCycle: (\d+), THROTTLE_percent: ([\d.]+), THROTTLE_relayState: (\d+)',
            message_data,
        )
        if match:
            throttle_a_duty_cycle = int(match.group(1))
            throttle_f_duty_cycle = int(match.group(2))
            throttle_percent = float(match.group(3))
            relay_state = int(match.group(4))

            return (
                throttle_a_duty_cycle,
                throttle_f_duty_cycle,
                throttle_percent,
                relay_state,
            )
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding THROTTLE_AccelData message: {e}")
        return None


def decode_brake_data(message_data):
    try:
        match = re.match(
            r'.*BRAKE_motorPercent: (\d+\.\d+).*BRAKE_pressurePercent: (\d+\.\d+).*BRAKE_limitSwitchMax: (\d+).*BRAKE_limitSwitchMin: (\d+).*',
            message_data,
        )

        if match:
            motorPercent = float(match.group(1))
            pressurePercent = float(match.group(2))
            limitSwitchMax = int(match.group(3))
            limitSwitchMin = int(match.group(4))

            return (
                motorPercent,
                pressurePercent,
                limitSwitchMax,
                limitSwitchMin,
            )
        else:
            raise ValueError("Invalid BRAKE_data message format")

    except Exception as e:
        print(f"Error decoding BRAKE_data message: {e}")
        print(f"Message data causing the error: {message_data}")
        return None


def decode_encoder_data(message_data):
    try:
        match = re.search(
            r'.*CTRL_encoderLeft: (\d+), CTRL_encoderRight: (\d+)',
            message_data,
        )
        if match:
            encoder_left = int(match.group(1))
            encoder_right = int(match.group(2))
            return encoder_left, encoder_right
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding CTRL_EncoderData message: {e}")
        return None


def decode_ctrl_velocity_command(message_data):
    try:
        match = re.search(
            r'CTRL_brakePercent: ([\d.]+), CTRL_throttlePercent: ([\d.]+)',
            message_data,
        )
        if match:
            brake_percent = float(match.group(1))
            throttle_percent = float(match.group(2))

            return brake_percent, throttle_percent
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding CTRL_VelocityCommand message: {e}")
        return None


def decode_ctrl_alarms(message_data):
    try:
        match = re.search(
            r'CTRL_alarmsRaised: (\d+), CTRL_speedAlarm: (\d+)', message_data
        )
        if match:
            alarms_raised = int(match.group(1))
            speed_alarm = int(match.group(2))

            return alarms_raised, speed_alarm
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding CTRL_Alarms message: {e}")
        return None


def decode_ctrl_controller_data(message_data):
    try:
        start_index = message_data.find("CTRL_averageVelocity: ") + len(
            "CTRL_averageVelocity: "
        )
        end_index = message_data.find(",", start_index)
        average_velocity = float(message_data[start_index:end_index])

        start_index = message_data.find("CTRL_desiredAcceleration: ") + len(
            "CTRL_desiredAcceleration: "
        )
        end_index = message_data.find(")", start_index)
        desired_acceleration = float(message_data[start_index:end_index])

        return average_velocity, desired_acceleration
    except Exception as e:
        print(f"Error decoding CTRL_ControllerData message: {e}")
        return None  # Return None in case of an error


def decode_ctrl_controller_gains(message_data):
    try:
        match = re.search(
            r'CTRL_gainKp: ([\d.]+), CTRL_gainKi: ([\d.]+), CTRL_gainKd: ([\d.]+)',
            message_data,
        )
        if match:
            gain_kp = float(match.group(1))
            gain_ki = float(match.group(2))
            gain_kd = float(match.group(3))

            return gain_kp, gain_ki, gain_kd
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding CTRL_ControllerGains message: {e}")
        return None


def decode_odrive_controller_error(message_data):
    try:
        match = re.search(r'ODRIVE_controllerError: (\w+)', message_data)
        if match:
            controller_error = match.group(1)

            return controller_error
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding ODRIVE_ControllerError message: {e}")
        return None


def decode_brake_gains(message_data):
    try:
        match = re.search(
            r'BRAKE_gainKp: ([\d.]+), BRAKE_gainKi: ([\d.]+), BRAKE_gainKd: ([\d.]+)',
            message_data,
        )
        if match:
            brake_gain_kp = float(match.group(1))
            brake_gain_ki = float(match.group(2))
            brake_gain_kd = float(match.group(3))

            return brake_gain_kp, brake_gain_ki, brake_gain_kd
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding BRAKE_BrakeGains message: {e}")
        return None


def decode_odrive_status(message_data):
    try:
        match = re.search(
            r'ODRIVE_axisError: (\w+), ODRIVE_axisState: (\w+), ODRIVE_motorErrorAlarm: (\d+), ODRIVE_encoderErrorAlarm: (\d+), ODRIVE_controllerErrorAlarm: (\d+), ODRIVE_trajectoryDone: (\d+)',
            message_data,
        )
        if match:
            odrive_axis_error = match.group(1)
            odrive_axis_state = match.group(2)
            odrive_motor_error_alarm = int(match.group(3))
            odrive_encoder_error_alarm = int(match.group(4))
            odrive_controller_error_alarm = int(match.group(5))
            odrive_trajectory_done = int(match.group(6))

            return (
                odrive_axis_error,
                odrive_axis_state,
                odrive_motor_error_alarm,
                odrive_encoder_error_alarm,
                odrive_controller_error_alarm,
                odrive_trajectory_done,
            )
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding ODRIVE_Status message: {e}")
        return None  # Return None in case of an error


def decode_odrive_motor_error(message_data):
    try:
        match = re.search(r'ODRIVE_motorError: (\w+)', message_data)
        if match:
            odrive_motor_error = match.group(1)
            return odrive_motor_error
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding ODRIVE_MotorError message: {e}")
        return None


def decode_odrive_encoder_error(message_data):
    try:
        match = re.search(r'ODRIVE_encoderError: (\w+)', message_data)
        if match:
            odrive_encoder_error = match.group(1)
            return odrive_encoder_error
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding ODRIVE_EncoderError message: {e}")
        return None


def decode_dbw_raw_velocity_command(message_data):
    try:
        match = re.search(
            r'DBW_brakePercent: ([\d.]+), DBW_throttlePercent: ([\d.]+)',
            message_data,
        )
        if match:
            brake_percent = float(match.group(1))
            throttle_percent = float(match.group(2))
            return brake_percent, throttle_percent
        else:
            raise ValueError("Match not found in message data")
    except Exception as e:
        print(f"Error decoding DBW_RawVelocityCommand message: {e}")
        return None


def convert_can_dump_to_csv(input_file, output_folder):
    with open(input_file, 'r') as input_file:
        lines = input_file.readlines()

    data = {}

    decoding_functions = {
        '0D1': decode_node_info,
        '0D2': decode_brake_node_info,
        '0D4': decode_ctrl_node_info,
        '0E1': decode_throttle_node_status,
        '0E2': decode_brake_node_status,
        '0E4': decode_ctrl_node_status,
        '01A': decode_sup_authorization,
        '01B': decode_sup_node_status,
        '01C': decode_sup_node_info,
        '1E5': decode_whl_absolute_encoder,
        '010': decode_throttle_accel_data,
        '011': decode_brake_data,
        '014': decode_encoder_data,
        '020': decode_ctrl_velocity_command,
        '022': decode_ctrl_alarms,
        '024': decode_ctrl_controller_data,
        '025': decode_ctrl_controller_gains,
        '31D': decode_odrive_controller_error,
        '253': decode_brake_gains,
        '301': decode_odrive_status,
        '303': decode_odrive_motor_error,
        '304': decode_odrive_encoder_error,
        '667': decode_dbw_raw_velocity_command,
    }

    message_names = {
        '0D1': 'NodeInfo',
        '0D2': 'BrakeNodeInfo',
        '0D4': 'CtrlNodeInfo',
        '0E1': 'ThrottleNodeStatus',
        '0E2': 'BrakeNodeStatus',
        '0E4': 'CtrlNodeStatus',
        '01A': 'SupAuthorization',
        '01B': 'SupNodeStatus',
        '01C': 'SupNodeInfo',
        '1E5': 'WhlAbsoluteEncoder',
        '010': 'ThrottleAccelData',
        '011': 'BrakeData',
        '014': 'EncoderData',
        '020': 'CtrlVelocityCommand',
        '022': 'CtrlAlarms',
        '024': 'CtrlControllerData',
        '025': 'CtrlControllerGains',
        '31D': 'ODriveControllerError',
        '253': 'BrakeGains',
        '301': 'ODriveStatus',
        '303': 'ODriveMotorError',
        '304': 'ODriveEncoderError',
        '667': 'DBWRawVelocityCommand',
    }

    for line in lines:
        parts = line.split()
        timestamp_str = parts[0][1:-1]  # Remove parentheses
        timestamp = datetime.fromtimestamp(float(timestamp_str), timezone.utc)

        message_id = parts[2]
        message_name = message_names.get(
            message_id, message_id
        )  # Use the message name if available, otherwise use the message ID
        message_data = ' '.join(parts[5:])

        if message_name not in data:
            data[message_name] = []  # Use message_name instead of message_id

        decoding_function = decoding_functions.get(message_id, lambda x: x)
        decoded_message = decoding_function(message_data)

        if isinstance(decoded_message, tuple):
            data[message_name].append(
                (timestamp,) + decoded_message
            )  # Use message_name instead of message_id
        else:
            data[message_name].append(
                (timestamp, decoded_message)
            )  # Use message_name instead of message_id

    os.makedirs(
        output_folder, exist_ok=True
    )  # Create the output folder if it doesn't exist

    for (
        message_name,
        messages,
    ) in data.items():  # Use message_name instead of message_id
        if not messages:
            continue  # Skip empty messages

        # Use the message name for the output file
        output_file = f"{output_folder}/{message_name}_{output_folder_path}.csv"  # Fix the output file naming

        with open(output_file, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            header_row = ['Timestamp'] + [
                f"{message_name}_{i}" for i in range(1, len(messages[0]))
            ]  # Use message_name instead of message_id
            csv_writer.writerow(header_row)

            for timestamp, *timestamp_data in messages:
                timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                row = [timestamp_str] + [
                    value if value is not None else ' '
                    for value in timestamp_data
                ]
                csv_writer.writerow(row)


if __name__ == "__main__":
    input_file_path = input("name of decoded .txt file: ")
    output_folder_path = input(
        "name of directory to make to store csv files: "
    )

    convert_can_dump_to_csv(input_file_path, output_folder_path)
    print(f"Conversion complete. CSV files saved at: {output_folder_path}")
