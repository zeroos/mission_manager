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
USER_CMD_PARAMS = 'p'
USER_CMD_WAIT_REPORTS = 'r'
USER_CMD_RESET_REPORTS = 'rr'

INTERACTIVE_COMMANDS_DESCRIPTIONS = {
    USER_CMD_MISSION_START: 'start mission',
    USER_CMD_MISSION_END: 'end mission',
    USER_CMD_HELP: 'display a list of allowed commands',
    USER_CMD_CLOSE: 'close the interactive mode',
    USER_CMD_WAIT_NODES: 'wait for n nodes to subscribe',
    USER_CMD_SLEEP: 'sleep for n seconds',
    USER_CMD_QUIT: 'exit the program',
    USER_CMD_PARAMS: 'set mission parameters, in the following format: '
                     '"A=1,B=2,C=3"',
    USER_CMD_WAIT_REPORTS: 'wait for reports from agents, accepts two '
    'optional arguments: number of occurrences, name of a report',
    USER_CMD_RESET_REPORTS: 'resets all counted reports'
}


INTERACTIVE_COMMANDS_ADDITIONAL_HELP = (
    'Commands {}, {} and {} additionally accepts an optional argument, '
    'start time, which can be either a date in a format accepted by '
    'dateutil.parser.parse function or "+s" where s is a number of '
    'seconds after which the mission should start.'.format(
        USER_CMD_MISSION_START, USER_CMD_MISSION_END,
        USER_CMD_PARAMS,
    )
)


def get_time_msg(timestamp=None):
    if timestamp is None:
        timestamp = datetime.now()
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
        self.subscription = self.create_subscription(
            MissionCommand,
            MISSION_TOPIC_NAME,
            self.listener_callback
        )
        self.subscription  # prevent unused variable warning
        self.reset_reports()

    def listener_callback(self, msg):
        if msg.command == MissionCommand.REPORT_PROGRESS:
            if msg.args[0] not in self.reports_counter:
                self.reports_counter[msg.args[0]] = 0
            self.reports_counter[msg.args[0]] += 1
            print("Progress report: {}, #{}".format(
                msg.args[0],
                self.reports_counter[msg.args[0]]
            ))
        else:
            pass

    def reset_reports(self):
        self.reports_counter = {}

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

    def change_params(self, params_raw, timestamp):
        params = [p.strip() for p in params_raw.split(',')]
        assert(all('=' in p for p in params))
        print("Parsed parameters: {}".format(params))

        msg = MissionCommand()
        msg.command = MissionCommand.CHANGE_PARAMS
        msg.stamp = get_time_msg(timestamp)
        msg.args = params
        self.publisher_.publish(msg)
        return msg.stamp

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
        print("{} nodes subscribed.".format(self.count_managed_nodes()))

    def delayed_quit(self, n):
        def _shutdown():
            print("Bye, bye!")
            sys.exit(0)

        self.create_timer(n, _shutdown)

    @staticmethod
    def parse_cmd_timestamp(args):
        if len(args) == 0:
            return datetime.now()
        elif args[0][0] == '+':
            s = timedelta(seconds=int(args[0][1:]))
            timestamp = datetime.now()+s
        else:
            timestamp = parse_timestamp(args[0])

        return timestamp

    def interpret_command(self, user_input):
        cmd_all = user_input.strip().split()
        if len(cmd_all) == 0:
            return
        cmd = cmd_all[0]
        args = cmd_all[1:]

        if cmd == USER_CMD_CLOSE:
            return
        elif cmd == USER_CMD_HELP:
            self.display_help()
        elif cmd == USER_CMD_MISSION_END:
            print('Ending mission')
            timestamp = self.parse_cmd_timestamp(args)
            self.end_mission(timestamp)
            print('@{}'.format(timestamp))
        elif cmd == USER_CMD_MISSION_START:
            print('Starting mission')
            timestamp = self.parse_cmd_timestamp(args)
            self.start_mission(timestamp)
            print('@{}'.format(timestamp))
        elif cmd == USER_CMD_PARAMS:
            print('Changing parameters')
            timestamp = self.parse_cmd_timestamp(args[1:])
            self.change_params(args[0], timestamp)
            print('@{}'.format(timestamp))
        elif cmd == USER_CMD_WAIT_NODES:
            print('Waiting for subscribers (numbers include mission manager)')
            n = int(args[0])+1
            if len(args) > 1:
                timeout = int(args[0])
            else:
                timeout = 0
            self.wait_for_subscribed_nodes(n, timeout)
        elif cmd == USER_CMD_RESET_REPORTS:
            print('Reseting reports')
            self.reset_reports()
        elif cmd == USER_CMD_WAIT_REPORTS:
            print("Waiting for progress reports...")
            try:
                occurrences_num = int(args[0])
            except IndexError:
                occurrences_num = 1
            try:
                event_name = args[1]
            except IndexError:
                event_name = None

            while (
                (
                    event_name is not None and
                    self.reports_counter.get(event_name, 0) < occurrences_num
                ) or (
                    event_name is None and
                    sum(self.reports_counter.values()) < occurrences_num
                )
            ):
                rclpy.spin_once(self)
                print('.', end='')
            print('Done waiting')

        elif cmd == USER_CMD_SLEEP:
            print('Sleeping...')
            n = int(args[0])
            print('...{} seconds'.format(n))
            time.sleep(n)
        elif cmd == USER_CMD_QUIT:
            print('Setting quit time...')
            try:
                n = int(args[0])
            except IndexError:
                n = 0
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


def cli(args=None):
    rclpy.init(args=args)
    manager = MissionManager()
    manager.interact()


if __name__ == '__main__':
    cli()
