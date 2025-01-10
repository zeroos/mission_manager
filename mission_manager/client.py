from datetime import datetime

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from mission_manager_msgs.msg import MissionCommand

from .mission_manager import MISSION_TOPIC_NAME, get_time_msg


def _datetime_from_time_msg(time):
    return datetime.fromtimestamp(
        time.sec + time.nanosec * 1e-9
    )


class MissionExecutor:
    def __init__(self, custom_command_helper=None):
        self.custom_cmd_dict = {}
        if custom_command_helper:
            for func_name in dir(custom_command_helper):
                if not func_name.startswith("__"):
                    func = getattr(custom_command_helper, func_name)
                    if callable(func):
                        self.custom_cmd_dict[func_name.upper()] = func

    def _delay_execution(self, func, timestamp):
        timer = None

        def _timer_callback():
            self.destroy_timer(timer)
            func()

        wait_time = timestamp-datetime.now()
        timer = self.create_timer(
            max(wait_time.total_seconds(), 0),
            _timer_callback
        )

    def custom_cmd_at_timestamp(self, custom_command, args, timestamp):
        if custom_command not in self.custom_cmd_dict:
            print(f"Unknown custom command: {custom_command}")
            return
        def _custom_cmd_callback():
            self.custom_cmd_dict[custom_command](*args)

        self._delay_execution(_custom_cmd_callback, timestamp)

    def start_mission_at_timestamp(self, timestamp):
        def _start_mission_callback():
            self.start_mission(timestamp)

        self._delay_execution(_start_mission_callback, timestamp)

    def end_mission_at_timestamp(self, timestamp):
        def _end_mission_callback():
            self.end_mission(timestamp)

        self._delay_execution(_end_mission_callback, timestamp)

    def change_params_at_timestamp(self, params, timestamp):
        def _change_params_callback():
            self.change_params(params, timestamp)

        self._delay_execution(_change_params_callback, timestamp)

    def start_mission(self, timestamp):
        raise NotImplementedError()

    def end_mission(self, timestamp):
        raise NotImplementedError()

    def change_params(self, params, timestamp):
        raise NotImplementedError()

    def progress_reported(self, progress):
        pass

    def add_progress_callback(self, progress_callback):
        self.report_progress = progress_callback

    def report_progress(self, progress):
        raise Exception(
            'MissionExecutor needs to be registered with MissionClient '
            'before reporting mission progress'
        )


class MissionClientMixin(object):
    def __init__(self):
        super().__init__('mission_client')
        self._executors = set()

        self.publisher_ = self.create_publisher(
            MissionCommand,
            MISSION_TOPIC_NAME,
            qos_profile=QoSPresetProfiles.get_from_short_key('PARAMETER_EVENTS')
        )

        self.subscription = self.create_subscription(
            MissionCommand,
            MISSION_TOPIC_NAME,
            self.listener_callback,
            qos_profile=QoSPresetProfiles.get_from_short_key('PARAMETER_EVENTS')
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.command == MissionCommand.MISSION_START:
            self.get_logger().info(
                'Received mission start'
            )

            def op(exe):
                exe.start_mission_at_timestamp(
                    _datetime_from_time_msg(msg.stamp)
                )
        elif msg.command == MissionCommand.MISSION_END:
            self.get_logger().info(
                'Received mission end'
            )

            def op(exe):
                exe.end_mission_at_timestamp(
                    _datetime_from_time_msg(msg.stamp)
                )
        elif msg.command == MissionCommand.CHANGE_PARAMS:
            self.get_logger().info(
                'Received change params with params: {}'.format(msg.args)
            )
            params = {
                name: value
                for name, value in [p.split('=') for p in msg.args]
            }

            def op(exe):
                exe.change_params_at_timestamp(
                    params,
                    _datetime_from_time_msg(msg.stamp)
                )
        elif msg.command == MissionCommand.REPORT_PROGRESS:
            def op(exe):
                exe.progress_reported(msg.args)

        elif msg.command == MissionCommand.CUSTOM_COMMAND:
            self.get_logger().info(f'Received custom command: {msg.custom_command_func}')

            if msg.custom_command_func:
                custom_command_func = msg.custom_command_func.upper()
                additional_args = msg.args
                timestamp = _datetime_from_time_msg(msg.stamp)

                def op(exe):
                    if hasattr(exe, "custom_cmd_at_timestamp"):
                        exe.custom_cmd_at_timestamp(custom_command_func, additional_args, timestamp)
                    else:
                        self.get_logger().warn(f'MissionExecutor does not support custom commands.')

            else:
                self.get_logger().warn('No custom command provided.')

                def op(exe):
                    pass

        else:
            def op(exe):
                pass
            self.get_logger().warn(
                'Unknown mission command: "{}"'.format(msg.command)
            )

        for exe in self._executors:
            op(exe)

    def progress_callback(self, progress):
        msg = MissionCommand()
        msg.command = MissionCommand.REPORT_PROGRESS
        msg.stamp = get_time_msg()
        msg.args = [progress]
        self.publisher_.publish(msg)

    def add_mission_executor(self, mission_executor):
        mission_executor.add_progress_callback(self.progress_callback)
        self._executors.add(mission_executor)


class MissionClient(MissionClientMixin, Node):
    pass
