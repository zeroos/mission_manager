import math
from datetime import timedelta, datetime
from dateutil.parser import parse as parse_timestamp
import sys
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
USER_CMD_WAIT_NODES = 'n'
USER_CMD_SLEEP = 'l'
USER_CMD_QUIT = 'q'

INTERACTIVE_COMMANDS_DESCRIPTIONS = {
    USER_CMD_MISSION_START: 'start mission',
    USER_CMD_MISSION_END: 'end mission',
    USER_CMD_HELP: 'display a list of allowed commands',
    USER_CMD_CLOSE: 'close the interactive mode',
    USER_CMD_WAIT_NODES: 'wait for n nodes to subscribe',
    USER_CMD_SLEEP: 'sleep for n seconds',
    USER_CMD_QUIT: 'exit the program',
}


INTERACTIVE_COMMANDS_ADDITIONAL_HELP = (
    'Commands {} and {} additionally accepts an optional argument, '
    'start time, which can be either a date in a format accepted by '
    'dateutil.parser.parse function or "+s" where s is a number of '
    'seconds after which the mission should start.'.format(
        USER_CMD_MISSION_START, USER_CMD_MISSION_END)
)


def get_time_msg(timestamp):
    modf = math.modf(timestamp.timestamp())
    nanosec, sec = map(int, (modf[0]*10**9, modf[1]))
    return Time(sec=sec, nanosec=nanosec)


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.publisher_ = self.create_publisher(
            MissionCommand,
            MISSION_TOPIC_NAME
        )

    def start_mission(self, timestamp):
        msg = MissionCommand()
        msg.command = MissionCommand.MISSION_START
        msg.stamp = get_time_msg(timestamp)
        self.publisher_.publish(msg)
        return msg.stamp

    def end_mission(self, timestamp):
        msg = MissionCommand()
        msg.command = MissionCommand.MISSION_END
        msg.stamp = get_time_msg(timestamp)
        self.publisher_.publish(msg)
        return msg.stamp

    def change_params(self, params):
        raise NotImplementedError()  # not implemented yet

    def display_help(self):
        print('Available commands:')
        for cmd, desc in INTERACTIVE_COMMANDS_DESCRIPTIONS.items():
            print('\t{} --- {}'.format(cmd, desc))

    def wait_for_subscribed_nodes(self, n, timeout):
        while self.count_managed_nodes() < n:
            rclpy.spin_once(self, timeout_sec=1)
            print(
                "{} subscribed nodes, waiting for {} (timeout in {}s)".format(
                    self.count_managed_nodes(),
                    n,
                    timeout
                )
            )
            timeout -= 1
            if timeout == 0:
                break

    def delayed_quit(self, n):
        def _shutdown():
            print("Bye, bye!")
            sys.exit(0)

        self.create_timer(n, _shutdown)

    @staticmethod
    def parse_cmd_timestamp(cmd):
        args = cmd.split()
        if len(args) == 1:
            return datetime.now()
        elif args[1][0] == '+':
            s = timedelta(seconds=int(args[1][1:]))
            timestamp = datetime.now()+s
        else:
            timestamp = parse_timestamp(args[1])

        return timestamp

    def interpret_command(self, cmd):
        cmd = cmd.strip()
        if cmd == USER_CMD_CLOSE:
            return
        elif cmd == USER_CMD_HELP:
            self.display_help()
        elif cmd.startswith(USER_CMD_MISSION_END):
            print('Ending mission')
            timestamp = self.parse_cmd_timestamp(cmd)
            self.end_mission(timestamp)
            print('@{}'.format(timestamp))
        elif cmd.startswith(USER_CMD_MISSION_START):
            print('Starting mission')
            timestamp = self.parse_cmd_timestamp(cmd)
            self.start_mission(timestamp)
            print('@{}'.format(timestamp))
        elif cmd.startswith(USER_CMD_WAIT_NODES):
            print('Waiting for subscribers')
            args = cmd.split()
            print(args, len(args))
            n = int(args[1])
            if len(args) > 2:
                timeout = int(args[2])
            else:
                timeout = 0
            self.wait_for_subscribed_nodes(n, timeout)
        elif cmd.startswith(USER_CMD_SLEEP):
            print('Sleeping...')
            n = int(cmd.split()[1])
            print('...{} seconds'.format(n))
            time.sleep(n)
        elif cmd.startswith(USER_CMD_QUIT):
            print('Setting quit time...')
            n = int(cmd.split()[1])
            print('...in {} seconds'.format(n))
            self.delayed_quit(n)
        else:
            print('Unknown command. Use "h" for help.')

    def interact(self):
        self.display_help()
        cmd = ''
        while cmd != USER_CMD_CLOSE:
            cmd = input('CMD >> ')
            self.interpret_command(cmd)
        print('Bye!')

    def count_managed_nodes(self):
        return self.count_subscribers(MISSION_TOPIC_NAME)


def cmd(args=None):
    rclpy.init(args=args)
    manager = MissionManager()

    cmds = sys.argv[-1].split(';')
    print("Executing commands '{}'".format(cmds))
    for cmd in cmds:
        manager.interpret_command(cmd)

    print('Spinning')
    rclpy.spin(manager)


def main(args=None):
    rclpy.init(args=args)
    manager = MissionManager()
    manager.interact()


if __name__ == '__main__':
    main()
