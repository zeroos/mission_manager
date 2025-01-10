import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ..client import (
    MissionClient,
    MissionExecutor,
)

class FSMStates:
    IDLE = 0
    STARTING = 1
    RUNNING = 2
    CHANGING_PARAMS = 3
    ENDING = 4


class FSMMissionExecutor(Node, MissionExecutor):
    def __init__(self, node_name='fsm_mission_executor'):
        super().__init__(node_name)
        self.state = FSMStates.IDLE

    def start_mission(self, timestamp):
        self.get_logger().info('FSM: start_mission called')
        if self.state == FSMStates.IDLE:
            self.state = FSMStates.STARTING
            self.get_logger().info('FSM: Transition to STARTING')
            self.state = FSMStates.RUNNING
            self.get_logger().info('FSM: Transition to RUNNING')

    def end_mission(self, timestamp):
        self.get_logger().info('FSM: end_mission called')
        if self.state in (FSMStates.RUNNING, FSMStates.CHANGING_PARAMS):
            self.state = FSMStates.ENDING
            self.get_logger().info('FSM: Transition to ENDING')
            self.state = FSMStates.IDLE
            self.get_logger().info('FSM: Transition to IDLE')

    def change_params(self, params, timestamp):
        self.get_logger().info(f'FSM: change_params called with {params}')
        if self.state == FSMStates.RUNNING:
            self.state = FSMStates.CHANGING_PARAMS
            self.get_logger().info('FSM: Transition to CHANGING_PARAMS')
            self.report_progress(f"Parameters changed: {params}")
            self.state = FSMStates.RUNNING
            self.get_logger().info('FSM: Transition back to RUNNING')


def start_fsm_client(args=None):

    rclpy.init(args=args)
    fsm_executor = FSMMissionExecutor()
    mission_client = MissionClient()
    mission_client.add_mission_executor(fsm_executor)

    executor = SingleThreadedExecutor()
    executor.add_node(fsm_executor)
    executor.add_node(mission_client)

    mission_client.get_logger().info('FSM client node started, waiting for mission commands.')
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    start_fsm_client()
