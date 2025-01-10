import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from mission_manager_msgs.msg import MissionCommand
from ..mission_manager import MISSION_TOPIC_NAME, get_time_msg

class FSM_manager(Node):
    def __init__(self, node_name='stateful_mission_executor'):
        super().__init__(node_name)

        self.publisher = self.create_publisher(
            MissionCommand,
            MISSION_TOPIC_NAME,
            qos_profile=QoSPresetProfiles.get_from_short_key('PARAMETER_EVENTS')
        )

        self.timer_counter = 0
        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("FSM Manager started.")

    def timer_callback(self):
        
        if self.timer_counter % 3 == 0:
            msg = MissionCommand()
            msg.command = MissionCommand.MISSION_START
            msg.stamp = get_time_msg()
            self.publisher.publish(msg)
            self.get_logger().info("FSM Timer: Publishing MISSION_START command.")

        elif self.timer_counter % 3 == 1:
            msg = MissionCommand()
            msg.command = MissionCommand.CHANGE_PARAMS
            msg.stamp = get_time_msg()
            msg.args = ["p=42"]
            self.publisher.publish(msg)
            self.get_logger().info("FSM Timer: Publishing CHANGE_PARAMS command with p=42.")
        
        elif self.timer_counter % 3 == 2:
            msg = MissionCommand()
            msg.command = MissionCommand.MISSION_END
            msg.stamp = get_time_msg()
            self.publisher.publish(msg)
            self.get_logger().info("FSM Timer: Publishing MISSION_END command.")
        self.timer_counter += 1



def start_fsm_manager(args=None):
    try:
        rclpy.init(args=args)
        fsm = FSM_manager()
        rclpy.spin(fsm)
    finally:
        fsm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    start_fsm_manager()
