"""不需要 ROS 的 2v2 运动学仿真，按复赛规则跑完整一场（上下半场各 4 分钟）。

红队两只狗跑本项目的 Brain（和实机同一套代码，含 1 号狗分工、进球暂停、开球等待）；
蓝队可选同一套代码、简单追球（chaser）或站着不动（static）。

仿真只用来验证决策逻辑和规则处理，不代表真实步态和球的物理特性。
"""

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from soccer import config  # noqa: E402
from soccer import geometry as geo  # noqa: E402
from soccer.brain import Brain  # noqa: E402
from soccer.field import Field, detect_attack_dir  # noqa: E402
from soccer.game import GameMonitor  # noqa: E402
from soccer.model import Command, Pose, World  # noqa: E402
from soccer.roles import RoleArbiter, striker_cost  # noqa: E402

DT = 0.05
DOG_R = 0.28        # 狗近似成圆，半径 m
BALL_FRICTION = 0.45
CONTROL_EVERY = 2   # 10 Hz 决策


class SimDog:
    def __init__(self, name, team, number, pose):
        self.name, self.team, self.number = name, team, number
        self.pose = pose
        self.v = [0.0, 0.0, 0.0]          # 实际机身速度（有滞后）
        self.cmd = Command.stop('init')
        self.brain = None
        self.game = None
        self.arbiter = None

    def step(self, dt):
        tau = 0.3
        target = [0.0, 0.0, 0.0] if self.cmd.halt else [self.cmd.vx, self.cmd.vy, self.cmd.wz]
        for i in range(3):
            self.v[i] += (target[i] - self.v[i]) * min(1.0, dt / tau)
        vw = geo.rotate((self.v[0], self.v[1]), self.pose.yaw)
        self.pose = Pose(self.pose.x + vw[0] * dt, self.pose.y + vw[1] * dt,
                         geo.wrap(self.pose.yaw + self.v[2] * dt))

    def world_vel(self):
        return geo.rotate((self.v[0], self.v[1]), self.pose.yaw)


class Match:
    def __init__(self, opponent='same', seed=0, half_time=240.0, noise=0.08, log=None):
        self.rng = random.Random(seed)
        self.field = Field(config.FIELD)
        self.cfg = config.CONTROL
        self.opponent = opponent
        self.half_time = half_time
        self.noise = noise
        self.log = log or (lambda text: None)
        self.t = 0.0
        self.score = {'red': 0, 'blue': 0}
        self.contacts = {'red': 0.0, 'blue': 0.0}   # 与对方狗接触的累计时长
        self.dead_resets = 0
        self.escapes = 0
        self.half = 1
        self.red_left = True                          # 上半场红队守左门
        self.dogs = [SimDog('red1', 'red', 1, None), SimDog('red2', 'red', 2, None),
                     SimDog('blue1', 'blue', 1, None), SimDog('blue2', 'blue', 2, None)]
        self.ball = self.field.center
        self.ball_v = (0.0, 0.0)
        self.pause_until = None
        self.dead_since = None
        self.jam_since = None
        self.placed = False
        self.pending_kickoff_left = True
        self.reset_positions(kickoff_left=True, new_brains=True)

    # ------------------------------------------------------------ 规则执行
    def jitter(self, p):
        return (p[0] + self.rng.uniform(-self.noise, self.noise),
                p[1] + self.rng.uniform(-self.noise, self.noise))

    def team_left(self, team):
        return self.red_left if team == 'red' else not self.red_left

    def reset_positions(self, kickoff_left, new_brains=False):
        sp = self.field.start_points
        for dog in self.dogs:
            left = self.team_left(dog.team)
            base = (sp[0], sp[1]) if left else (sp[2], sp[3])
            p = self.jitter(base[dog.number - 1])
            yaw = (0.0 if left else math.pi) + self.rng.uniform(-0.5, 0.5)
            dog.pose = Pose(p[0], p[1], yaw)
            dog.v = [0.0, 0.0, 0.0]
            if new_brains:
                dog.brain = None
        self.ball = self.field.kickoff_point(kickoff_left)
        self.ball_v = (0.0, 0.0)
        if new_brains:
            for dog in self.dogs:
                mates = [d.pose.xy for d in self.dogs if d.team == dog.team]
                attack = detect_attack_dir(mates, self.field)
                kick_us = self.team_left(dog.team) == kickoff_left
                dog.brain = Brain(dog.number, self.field, self.cfg, attack)
                dog.game = GameMonitor(self.field, attack, self.cfg, kickoff_us=kick_us)
                dog.arbiter = RoleArbiter(self.cfg.role_margin)

    def check_rules(self):
        side = self.field.ball_in_goal(self.ball)
        if side is not None:
            scorer = 'red' if (side == 'right') == self.red_left else 'blue'
            self.score[scorer] += 1
            self.log('{:6.1f}s 进球：{} 进球，比分 红 {} : {} 蓝'.format(
                self.t, '红队' if scorer == 'red' else '蓝队',
                self.score['red'], self.score['blue']))
            self.pause_until = self.t + self.cfg.goal_pause
            conceded = 'blue' if scorer == 'red' else 'red'
            self.pending_kickoff_left = self.team_left(conceded)
            self.ball_v = (0.0, 0.0)
            return
        if self.field.needs_reset(self.ball):
            if self.dead_since is None:
                self.dead_since = self.t
            slow = geo.norm(self.ball_v) < 0.1
            if (slow and self.t - self.dead_since > 1.5) or self.t - self.dead_since > 4.0:
                self.ball = self.field.nearest_serve_point(self.ball)
                self.ball_v = (0.0, 0.0)
                self.dead_resets += 1
                self.dead_since = None
        else:
            self.dead_since = None
        # 两只狗顶住 8 s：就近发球（规则 4.4）
        jammed = any(geo.dist(a.pose.xy, b.pose.xy) < 2 * DOG_R + 0.02
                     for a in self.dogs for b in self.dogs if a.team != b.team)
        if jammed:
            if self.jam_since is None:
                self.jam_since = self.t
            if self.t - self.jam_since > 8.0:
                self.ball = self.field.nearest_serve_point(self.ball)
                self.ball_v = (0.0, 0.0)
                self.jam_since = None
                self.log('{:6.1f}s 死球：顶住 8 s，就近发球'.format(self.t))
        else:
            self.jam_since = None

    # ------------------------------------------------------------ 物理
    def physics(self):
        for dog in self.dogs:
            dog.step(DT)
            x = geo.clamp(dog.pose.x, DOG_R, self.field.length - DOG_R)
            y = geo.clamp(dog.pose.y, DOG_R, self.field.width - DOG_R)
            if self.field.in_goal_mouth_band(y, DOG_R):
                gd = self.field.cfg.goal_depth
                x = geo.clamp(x, gd + DOG_R * 0.5, self.field.length - gd - DOG_R * 0.5)
            dog.pose = Pose(x, y, dog.pose.yaw)
        for i, a in enumerate(self.dogs):
            for b in self.dogs[i + 1:]:
                rel = geo.sub(b.pose.xy, a.pose.xy)
                d = geo.norm(rel)
                if d < 2 * DOG_R and d > 1e-6:
                    push = geo.scale(geo.unit(rel), (2 * DOG_R - d) / 2)
                    a.pose = Pose(a.pose.x - push[0], a.pose.y - push[1], a.pose.yaw)
                    b.pose = Pose(b.pose.x + push[0], b.pose.y + push[1], b.pose.yaw)
                    if a.team != b.team:
                        for dog, other in ((a, b), (b, a)):
                            toward = geo.dot(dog.world_vel(), geo.unit(geo.sub(other.pose.xy, dog.pose.xy)))
                            if toward > 0.1:
                                self.contacts[dog.team] += DT
        r = self.field.cfg.ball_radius
        for dog in self.dogs:
            rel = geo.sub(self.ball, dog.pose.xy)
            d = geo.norm(rel)
            if d < DOG_R + r:
                n = geo.unit(rel)
                self.ball = geo.add(dog.pose.xy, geo.scale(n, DOG_R + r))
                dv = geo.dot(dog.world_vel(), n)
                bv = geo.dot(self.ball_v, n)
                if dv * 1.4 > bv:
                    self.ball_v = geo.add(self.ball_v, geo.scale(n, dv * 1.4 - bv + 0.05))
        speed = geo.norm(self.ball_v)
        if speed > 0:
            new = max(0.0, speed - BALL_FRICTION * DT)
            self.ball_v = geo.scale(self.ball_v, new / speed)
        bx, by = self.ball[0] + self.ball_v[0] * DT, self.ball[1] + self.ball_v[1] * DT
        vx, vy = self.ball_v
        in_mouth = self.field.in_goal_mouth_band(by)
        if bx < r:
            bx, vx = r, -0.5 * vx
        if bx > self.field.length - r:
            bx, vx = self.field.length - r, -0.5 * vx
        if by < r or by > self.field.width - r:
            by, vy = geo.clamp(by, r, self.field.width - r), -0.5 * vy
        if not in_mouth:     # 门框两侧是实体，球不能从侧面钻进球门
            gd = self.field.cfg.goal_depth
            if abs(by - self.field.cy) < self.field.cfg.goal_width / 2 + 0.3:
                if bx < gd and self.ball[0] >= gd:
                    bx, vx = gd, -0.5 * vx
                if bx > self.field.length - gd and self.ball[0] <= self.field.length - gd:
                    bx, vx = self.field.length - gd, -0.5 * vx
        self.ball, self.ball_v = (bx, by), (vx, vy)

    # ------------------------------------------------------------ 决策
    def decide(self):
        for team in ('red', 'blue'):
            team_dogs = [d for d in self.dogs if d.team == team]
            others = [d.pose.xy for d in self.dogs if d.team != team]
            if team == 'blue' and self.opponent != 'same':
                for d in team_dogs:
                    d.cmd = self.simple_opponent(d, others)
                continue
            leader = team_dogs[0]
            goal = self.field.opp_goal(leader.brain.attack_dir)
            ball_for_roles = self.ball
            if self.field.needs_reset(self.ball):
                ball_for_roles = self.field.nearest_serve_point(self.ball)
            costs = {d.number: striker_cost(d.pose, ball_for_roles, goal, self.cfg.standoff)
                     for d in team_dogs}
            striker = leader.arbiter.update(costs)
            for d in team_dogs:
                mate = [m for m in team_dogs if m is not d][0]
                world = World(me=d.pose, mate=mate.pose, others=others,
                              ball=self.ball, ball_vel=self.ball_v)
                state = d.game.update(self.t, self.ball, geo.norm(self.ball_v))
                if d.game.event and d.number == 1:
                    self.log('{:6.1f}s [{}] {}'.format(self.t, team, d.game.event))
                d.game.event = None
                before = d.brain.stuck.escape_until
                d.cmd = d.brain.step(world, striker, state, self.t)
                if d.brain.stuck.escape_until != before:
                    self.escapes += 1

    def simple_opponent(self, dog, others):
        if self.opponent == 'static' or self.pause_until is not None:
            return Command.stop('static')
        attack = 1 if not self.red_left else -1
        goal = self.field.opp_goal(attack)
        u = geo.unit(geo.sub(goal, self.ball))
        target = geo.sub(self.ball, geo.scale(u, 0.35)) if dog.number == 1 else \
            geo.add(self.field.own_goal(attack), (attack * 1.0, 0.0))
        if dog.number == 1 and geo.dist(dog.pose.xy, target) < 0.25:
            target = geo.add(self.ball, geo.scale(u, 0.5))
        err = geo.sub(target, dog.pose.xy)
        vel = geo.scale(geo.unit(err), min(0.45, geo.norm(err)))
        face = geo.angle_of(err) if geo.norm(err) > 0.3 else geo.angle_of(u)
        b = geo.rotate(vel, -dog.pose.yaw)
        herr = geo.wrap(face - dog.pose.yaw)
        return Command(geo.clamp(b[0], -0.2, 0.5), geo.clamp(b[1], -0.3, 0.3),
                       geo.clamp(1.5 * herr, -1.0, 1.0), 'chaser')

    # ------------------------------------------------------------ 主循环
    def run(self):
        step = 0
        total = 2 * self.half_time
        while self.t < total:
            if self.half == 1 and self.t >= self.half_time:
                self.half = 2
                self.red_left = not self.red_left
                self.pause_until = None
                self.log('{:6.1f}s 下半场：换边，蓝队开球'.format(self.t))
                self.reset_positions(kickoff_left=not self.red_left, new_brains=True)
            if self.pause_until is not None:
                if self.t >= self.pause_until - 12.0 and not self.placed:
                    self.reset_positions(self.pending_kickoff_left)   # 人工把狗和球放回
                    self.placed = True
                if self.t >= self.pause_until:
                    self.pause_until = None
                    self.placed = False
            if step % CONTROL_EVERY == 0:
                self.decide()
            if self.pause_until is not None:
                for d in self.dogs:
                    d.cmd = Command.stop('held')
            self.physics()
            if self.pause_until is None:
                self.check_rules()
            self.t += DT
            step += 1
        return self.score


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--opponent', choices=('same', 'chaser', 'static'), default='chaser')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--half', type=float, default=240.0, help='半场时长 s')
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()
    match = Match(args.opponent, args.seed, args.half, log=None if args.quiet else print)
    score = match.run()
    print('终场：红 {} : {} 蓝（对手 {}，seed {}）'.format(
        score['red'], score['blue'], args.opponent, args.seed))
    print('红队顶向对方狗累计 {:.1f} s，蓝队 {:.1f} s；死区重发 {} 次；脱困 {} 次'.format(
        match.contacts['red'], match.contacts['blue'], match.dead_resets, match.escapes))


if __name__ == '__main__':
    main()
