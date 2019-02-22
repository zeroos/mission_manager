import math
import time

from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node

from mission_manager_msgs.msg import MissionCommand

MISSION_TOPIC_NAME = 'mission'

MISSION_CMD_START = 'start'
MISSION_CMD_END = 'end'

USER_CMD_MISSION_START = 's'
USER_CMD_MISSION_END = 'e'
USER_CMD_HELP = 'h'
USER_CMD_CLOSE = 'c'

INTERACTIVE_COMMANDS_DESCRIPTIONS = {
    USER_CMD_MISSION_START: 'start mission',
    USER_CMD_MISSION_END: 'end mission',
    USER_CMD_HELP: 'display a list of allowed commands',
    USER_CMD_CLOSE: 'close the interactive mode',
}


def get_current_time_msg():
    modf = math.modf(time.time())
    nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
    return Time(sec=sec, nanosec=nanosec)


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.publisher_ = self.create_publisher(
            MissionCommand,
            MISSION_TOPIC_NAME
        )

    def start_mission(self):
        msg = MissionCommand()
        msg.command = MissionCommand.MISSION_START
        msg.stamp = get_current_time_msg()
        self.publisher_.publish(msg)
        return msg.stamp

    def end_mission(self):
        msg = MissionCommand()
        msg.command = MissionCommand.MISSION_END
        msg.stamp = get_current_time_msg()
        self.publisher_.publish(msg)
        return msg.stamp

    def change_params(self, params):
        raise NotImplementedError()  # not implemented yet

    def display_help(self):
        print('Available commands:')
        for cmd, desc in INTERACTIVE_COMMANDS_DESCRIPTIONS.items():
            print('\t{} --- {}'.format(cmd, desc))

    def interpret_command(self, cmd):
        if cmd == USER_CMD_CLOSE:
            return
        elif cmd == USER_CMD_HELP:
            self.display_help()
        elif cmd == USER_CMD_MISSION_END:
            print('Ending mission')
            ts = self.end_mission()
            print('{}.{}'.format(ts.sec, ts.nanosec))
        elif cmd == USER_CMD_MISSION_START:
            print('Starting mission')
            ts = self.start_mission()
            print('{}.{}'.format(ts.sec, ts.nanosec))
        else:
            print('Unknown command. Use "h" for help.')

    def interact(self):
        self.display_help()
        cmd = ''
        while cmd != USER_CMD_CLOSE:
            cmd = input('CMD >> ')
            self.interpret_command(cmd)
        print('Bye!')


def cmd(args=None):
    print('Not implemented yet')


def main(args=None):
    rclpy.init(args=args)
    manager = MissionManager()
    manager.interact()


if __name__ == '__main__':
    main()
