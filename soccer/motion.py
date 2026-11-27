"""运动层：避让、限速、世界速度转机身速度、卡住脱困。"""

import math

from . import geometry as geo
from .model import Command


def avoid(p, vel, obstacles, gain):
    """简单人工势场。obstacles 为 [(位置, 影响半径, 接触距离)]。

    进入影响半径后加一个向外的斥力，并按距离削掉朝向障碍的速度分量，
    到接触距离时朝向障碍的分量被完全削掉：只会绕开或停下，不会顶上去。
    """
    out = vel
    for center, radius, contact in obstacles:
        rel = geo.sub(p, center)
        d = geo.norm(rel)
        if d >= radius or d < 1e-6:
            continue
        away = geo.unit(rel)
        closeness = (radius - d) / radius
        toward = -geo.dot(out, away)
        if toward > 0:
            cut = geo.clamp((radius - d) / max(radius - contact, 1e-3), 0.0, 1.0)
            out = geo.add(out, geo.scale(away, toward * cut))
        out = geo.add(out, geo.scale(away, gain * closeness))
    return geo.limit(out, max(geo.norm(vel), 0.15))


def front_blocked(pose, others, distance, half_angle=0.6):
    """正前方 distance 内是否有别的狗。"""
    for o in others:
        rel = geo.sub(o, pose.xy)
        d = geo.norm(rel)
        if d < distance and abs(geo.wrap(geo.angle_of(rel) - pose.yaw)) < half_angle:
            return True
    return False


def body_command(pose, world_vel, face, cfg, mode):
    """把场地系期望速度转成 MotionServoCmd 的 [vx, vy, wz]（机身系）。"""
    body = geo.rotate(world_vel, -pose.yaw)
    heading_err = geo.wrap(face - pose.yaw)
    wz = geo.clamp(cfg.k_turn * heading_err, -cfg.max_wz, cfg.max_wz)
    # 偏差大时先转身再走，狗前进方向走得最稳。
    s = 1.0
    if abs(heading_err) > 0.5:
        s = geo.clamp(1.0 - (abs(heading_err) - 0.5), 0.3, 1.0)
    vx = geo.clamp(body[0] * s, -cfg.max_back, cfg.max_vx)
    vy = geo.clamp(body[1] * s, -cfg.max_vy, cfg.max_vy)
    return Command(vx, vy, wz, mode)


class StuckDetector:
    """有速度指令却几乎没动（顶墙、和别的狗顶住）时，后退并转身脱困。

    规则 4.3/4.4：卡在边界会被抱起，两狗顶住 8 s 会被判死球，所以 2.5 s 就主动退出来。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.history = []
        self.escape_until = -1.0
        self.cooldown_until = -1.0
        self.turn_sign = 1.0

    def reset(self):
        self.history = []

    def escape_command(self, t):
        if t < self.escape_until:
            return Command(-self.cfg.max_back, 0.0, 0.6 * self.turn_sign, 'escape')
        return None

    def update(self, t, pose, cmd):
        if cmd.halt or cmd.speed < 0.12 or t < self.cooldown_until:
            self.history = []
            return
        self.history.append((t, pose.x, pose.y))
        while self.history and t - self.history[0][0] > self.cfg.stuck_time:
            self.history.pop(0)
        t0, x0, y0 = self.history[0]
        if t - t0 >= self.cfg.stuck_time - 0.15 and \
                math.hypot(pose.x - x0, pose.y - y0) < self.cfg.stuck_move:
            self.escape_until = t + self.cfg.escape_time
            self.cooldown_until = self.escape_until + self.cfg.escape_cooldown
            self.turn_sign = -self.turn_sign
            self.history = []
