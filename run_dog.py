#!/usr/bin/env python3
"""控制一只狗踢球。一般不直接运行，用 ./start.sh 1 或 ./start.sh 2。"""

import argparse
import signal
import sys


def parse(argv):
    parser = argparse.ArgumentParser(description='CyberDog 2v2 踢球（单只狗）')
    parser.add_argument('--dog', type=int, choices=(1, 2), required=True,
                        help='1 号或 2 号狗')
    parser.add_argument('--dry-run', action='store_true',
                        help='只打印决策，不发运动指令、不起身')
    parser.add_argument('--side', choices=('auto', 'left', 'right'), default='auto',
                        help='己方球门在哪边；auto 按开场时两只狗所在半场判断')
    parser.add_argument('--kickoff', choices=('us', 'them'), default='us',
                        help='开场由谁开球；them 表示先等对方踢动球')
    parser.add_argument('--role', choices=('auto', 'striker', 'defender'), default='auto',
                        help='固定角色（队友坏了时用）；默认两只狗自动分工')
    return parser.parse_known_args(argv)


def main():
    args, ros_args = parse(sys.argv[1:])
    # 让 Ctrl+C 先由 Python 处理，保证退出前能发出结束帧。
    signal.signal(signal.SIGINT, signal.default_int_handler)
    from soccer.ros_node import run
    run(args, [sys.argv[0]] + ros_args)


if __name__ == '__main__':
    main()
