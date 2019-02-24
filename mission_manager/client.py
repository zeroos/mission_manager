from datetime import datetime

from rclpy.node import Node

from mission_manager_msgs.msg import MissionCommand

from .mission_manager import MISSION_TOPIC_NAME


def _datetime_from_time_msg(time):
    return datetime.fromtimestamp(
        time.sec + time.nanosec * 1e-9
    )


class MissionExecutor:
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

    def start_mission_at_timestamp(self, timestamp):
        def _start_mission_callback():
            self.start_mission(timestamp)

        self._delay_execution(_start_mission_callback, timestamp)

    def end_mission_at_timestamp(self, timestamp):
        def _end_mission_callback():
            self.end_mission(timestamp)

        self._delay_execution(_end_mission_callback, timestamp)

    def start_mission(self, timestamp):
        raise NotImplementedError()

    def end_mission(self, timestamp):
        raise NotImplementedError()

    def change_params(self, params):
        raise NotImplementedError()


class MissionClientMixin(object):
    def __init__(self):
        super().__init__('mission_client')
        self._executors = set()

        self.subscription = self.create_subscription(
            MissionCommand,
            MISSION_TOPIC_NAME,
            self.listener_callback
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
        else:
            def op(exe):
                pass
            self.get_logger().warn(
                'Unknown mission command: "{}"'.format(msg.command)
            )

        for exe in self._executors:
            op(exe)

    def add_mission_executor(self, mission_executor):
        self._executors.add(mission_executor)


class MissionClient(MissionClientMixin, Node):
    pass
