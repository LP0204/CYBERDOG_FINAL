"""决策用到的数据结构，全部在场地坐标系下。"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Point = Tuple[float, float]


@dataclass
class Pose:
    x: float
    y: float
    yaw: float

    @property
    def xy(self) -> Point:
        return (self.x, self.y)


@dataclass
class World:
    me: Optional[Pose]
    mate: Optional[Pose] = None
    others: List[Point] = field(default_factory=list)   # 对方和其他不认识的狗
    ball: Optional[Point] = None
    ball_vel: Point = (0.0, 0.0)


@dataclass
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    mode: str = 'idle'
    halt: bool = False      # True：发结束帧，原地站住不踏步

    @staticmethod
    def stop(mode):
        return Command(mode=mode, halt=True)

    @property
    def speed(self):
        return (self.vx ** 2 + self.vy ** 2) ** 0.5
