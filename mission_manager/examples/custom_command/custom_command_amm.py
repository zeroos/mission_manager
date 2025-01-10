import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from mission_manager_msgs.msg import MissionCommand
from ...mission_manager import MISSION_TOPIC_NAME, get_time_msg

class AutoMissionManager_custom_cmd(Node):
    def __init__(self, node_name='Automatic_mission_manager'):
        super().__init__(node_name)

        self.publisher = self.create_publisher(
            MissionCommand,
            MISSION_TOPIC_NAME,
            qos_profile=QoSPresetProfiles.get_from_short_key('PARAMETER_EVENTS')
        )

        self.timer_counter = 0
        self.timer_period = 3.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Automatic_mission_manager started.")

    def timer_callback(self):
        if self.timer_counter % 3 == 0:
            msg = MissionCommand()
            msg.command = MissionCommand.MISSION_START
            msg.stamp = get_time_msg()
            self.publisher.publish(msg)
            self.get_logger().info("Automatic_mission_manager Timer: Publishing MISSION_START command.")

        elif self.timer_counter % 3 == 1:
            msg = MissionCommand()
            msg.command = MissionCommand.CUSTOM_COMMAND
            msg.stamp = get_time_msg()
            msg.custom_command_func = "PAUSE_MISSION"
            msg.args = ["duration=10s", "reason=safety_check"]
            self.publisher.publish(msg)
            self.get_logger().info("Automatic_mission_manager Timer: Publishing CUSTOM_COMMAND (PAUSE_MISSION).")
        
        elif self.timer_counter % 3 == 2:
            msg = MissionCommand()
            msg.command = MissionCommand.MISSION_END
            msg.stamp = get_time_msg()
            self.publisher.publish(msg)
            self.get_logger().info("Automatic_mission_manager Timer: Publishing MISSION_END command.")


        self.timer_counter += 1


def start_auto_mission_manager(args=None):
    try:
        rclpy.init(args=args)
        fsm = AutoMissionManager_custom_cmd()
        rclpy.spin(fsm)
    finally:
        fsm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    start_auto_mission_manager()
