from rclpy.node import Node

from mission_manager_msgs.msg import MissionCommand
from .mission_manager import MISSION_TOPIC_NAME


class MissionExecutor:
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
            def op(exe):
                exe.start_mission(msg.stamp)
        elif msg.command == MissionCommand.MISSION_END:
            def op(exe):
                exe.end_mission(msg.stamp)
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
