import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ...client import MissionExecutor, MissionClient

class MockMissionExecutor(Node, MissionExecutor):
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


def start_mock_client(args=None):
    rclpy.init(args=args)

    mock_executor = MockMissionExecutor('mock_mission_client')
    mission_client = MissionClient()
    mission_client.add_mission_executor(mock_executor)

    executor = SingleThreadedExecutor()
    executor.add_node(mock_executor)
    executor.add_node(mission_client)

    mission_client.get_logger().info(
        'Node initialized, waiting for events.'
    )
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    start_mock_client()