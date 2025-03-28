import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from mission_manager_msgs.msg import MissionCommand
from ...mission_manager import MISSION_TOPIC_NAME, get_time_msg

class AutoMissionManager(Node):
    """
    A ROS2 node that demonstrates mission management without a CLI interface.

    This node simulates a mission by publishing a sequence of mission commands at regular 
        intervals using a timer.
    It cycles through the following commands:
      - MISSION_START: To initiate the mission.
      - CHANGE_PARAMS: To adjust mission parameters (e.g., setting "p=42").
      - MISSION_END: To conclude the mission.

    The node is designed to be onboard a drone or similar system, reacting to events or 
        external sensor inputs in a real-world application. In this example, the timer 
        mimics mission execution by triggering the command sequence based on the
        current timestamp.
    """
    def __init__(self, node_name='Automatic_mission_manager'):
        super().__init__(node_name)

        self.publisher = self.create_publisher(
            MissionCommand,
            MISSION_TOPIC_NAME,
            qos_profile=QoSPresetProfiles.get_from_short_key('PARAMETER_EVENTS')
        )

        self.timer_counter = 0
        self.timer_period = 5.0
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
            msg.command = MissionCommand.CHANGE_PARAMS
            msg.stamp = get_time_msg()
            msg.args = ["p=42"]
            self.publisher.publish(msg)
            self.get_logger().info("Automatic_mission_manager Timer: Publishing CHANGE_PARAMS command with p=42.")
        
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
        fsm = AutoMissionManager()
        rclpy.spin(fsm)
    finally:
        fsm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    start_auto_mission_manager()
