import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ...client import MissionExecutor, MissionClient

class CustomCmdHelper:
    """
    A helper class that defines custom command functions for mission operations.

    The custom commands are implemented as methods in this class. Each method's name 
        serves as the command identifier.

    Only methods intended to be used as custom commands should be included, as all 
        callable methods (except those with names containing double underscores)
        may be invoked as custom commands.
    """
    def pause_mission(self, *args):
        print(f"Pausing mission with args: {args}")

class CustomCmdExecutor(Node, MissionExecutor):
    def __init__(self, node_name='custom_cmd_client', custom_command_helper=None):
        super().__init__(node_name)
        MissionExecutor.__init__(self, custom_command_helper=custom_command_helper)

    def start_mission(self, timestamp):
        self.get_logger().info(
            'Starting mission'
        )

    def end_mission(self, timestamp):
        self.get_logger().info(
            'Ending mission'
        )

    def change_params(self, params, timestamp):
        self.get_logger().info(
            'Changing parameters to {}'.format(params)
        )
        self.report_progress(params.get('p', 'unknown'))


def start_custom_cmd_client(args=None):
    rclpy.init(args=args)

    custom_helper = CustomCmdHelper()
    mock_executor = CustomCmdExecutor('custom_cmd_client', custom_command_helper=custom_helper)
    mission_client = MissionClient()
    mission_client.add_mission_executor(mock_executor)

    executor = SingleThreadedExecutor()
    executor.add_node(mock_executor)
    executor.add_node(mission_client)

    mission_client.get_logger().info('Node initialized, waiting for events.')
    executor.spin()

    rclpy.shutdown()



if __name__ == '__main__':
    start_custom_cmd_client()

