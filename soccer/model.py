"""决策用到的数据结构，全部在场地坐标系下（兼容 Python 3.6，不依赖 dataclasses）。"""

from typing import Tuple

Point = Tuple[float, float]


class Pose:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    @property
    def xy(self) -> Point:
        return (self.x, self.y)

    def __repr__(self):
        return 'Pose(x={!r}, y={!r}, yaw={!r})'.format(self.x, self.y, self.yaw)


class World:
    def __init__(self, me, mate=None, others=None, ball=None, ball_vel=(0.0, 0.0)):
        self.me = me
        self.mate = mate
        self.others = [] if others is None else list(others)   # 对方和其他不认识的狗
        self.ball = ball
        self.ball_vel = ball_vel


class Command:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0, mode='idle', halt=False):
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.mode = mode
        self.halt = halt        # True：发结束帧，原地站住不踏步

    @staticmethod
    def stop(mode):
        return Command(mode=mode, halt=True)

    @property
    def speed(self):
        return (self.vx ** 2 + self.vy ** 2) ** 0.5

    def __repr__(self):
        return 'Command(vx={!r}, vy={!r}, wz={!r}, mode={!r}, halt={!r})'.format(
            self.vx, self.vy, self.wz, self.mode, self.halt)
