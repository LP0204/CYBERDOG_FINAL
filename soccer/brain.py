"""单只狗每个控制周期的决策：前锋绕球推射，后卫守门前；遇到特殊情况先处理。

优先级：
1. 没有自己的位姿 / 进球暂停 → 原地停（发结束帧）
2. 卡住 → 后退转身脱困
3. 对方开球前 → 退回己方半场，离球 1 m 以上
4. 球在死区或出界 → 不去墙边抢，提前站到裁判会放球的发球点后方
5. 前锋：选射门点 → 绕到球后 → 对准后推射；后卫：站在球与己方球门连线上
"""

from . import geometry as geo
from .game import GOAL_PAUSE, WAIT_KICKOFF
from .model import Command
from .motion import StuckDetector, avoid, body_command, front_blocked


def choose_aim(field, ball, attack_dir, others):
    """在对方球门里选一个射门点：球到该点的线路离对方狗最远。"""
    gx, gy = field.opp_goal(attack_dir)
    best, best_score = (gx, gy), -1e9
    half = field.cfg.goal_width / 2.0 - 0.3
    for offset in (0.0, -half, half):
        aim = (gx, gy + offset)
        clearance = 1.0
        for o in others:
            d, t = geo.point_segment_distance(o, ball, aim)
            if t > 0.05:
                clearance = min(clearance, d)
        score = clearance - 0.15 * abs(offset)
        if score > best_score + 1e-9:
            best, best_score = aim, score
    return best


class Brain:
    def __init__(self, number, field, cfg, attack_dir, role_override='auto'):
        self.number = number
        self.field = field
        self.cfg = cfg
        self.attack_dir = attack_dir
        self.role_override = role_override
        self.pushing = False
        self.stuck = StuckDetector(cfg)
        self.last_ball = None
        self.role = None

    # ------------------------------------------------------------------
    def step(self, world, striker, game_state, t):
        if world.me is None:
            return Command.stop('no_pose')
        if game_state == GOAL_PAUSE:
            self.pushing = False
            self.stuck.reset()
            return Command.stop('goal_pause')
        escape = self.stuck.escape_command(t)
        if escape is not None:
            return self.limit_translation(world.me, escape, self.obstacles(world))

        if world.ball is not None:
            self.last_ball = world.ball
        role = self.role_override
        if role == 'auto':
            role = 'striker' if striker == self.number else 'defender'
        self.role = role

        if world.ball is None:
            cmd = self.defend(world, self.last_ball or self.field.center, 'no_ball')
        elif game_state == WAIT_KICKOFF:
            cmd = self.wait_kickoff(world)
        elif self.field.needs_reset(world.ball):
            cmd = self.dead_ball(world, role)
        elif role == 'striker':
            cmd = self.attack(world, world.ball)
        else:
            cmd = self.defend(world, world.ball, 'defend')

        self.stuck.update(t, world.me, cmd)
        return cmd

    # ------------------------------------------------------------------
    def obstacles(self, world, ball_obstacle=None):
        cfg = self.cfg
        obs = [(o, cfg.opp_radius, cfg.dog_contact) for o in world.others]
        if world.mate is not None:
            obs.append((world.mate.xy, cfg.mate_radius, cfg.dog_contact))
        if ball_obstacle is not None:
            obs.append((ball_obstacle, cfg.ball_avoid_radius, cfg.ball_contact))
        return obs

    def limit_translation(self, me, cmd, obstacles):
        """对最终的机身速度做限速。统一缩放平移，保留原转向和行进方向。"""
        velocity = geo.rotate((cmd.vx, cmd.vy), me.yaw)
        wall_velocity = self.field.wall_limit(me.xy, velocity, self.cfg.wall_soft)
        scale = 1.0
        for requested, allowed in zip(velocity, wall_velocity):
            if abs(requested) > 1e-9:
                scale = min(scale, allowed / requested)

        for center, radius, contact in obstacles:
            toward = geo.sub(center, me.xy)
            distance = geo.norm(toward)
            if distance < 1e-6:
                return Command(0.0, 0.0, cmd.wz, cmd.mode)
            if geo.dot(velocity, toward) > 0.0:
                if distance <= contact:
                    return Command(0.0, 0.0, cmd.wz, cmd.mode)
                if distance < radius:
                    scale = min(scale, (distance - contact) / (radius - contact))
        return Command(cmd.vx * scale, cmd.vy * scale, cmd.wz, cmd.mode)

    def go_to(self, world, target, face, vmax, mode, ball_obstacle=None,
              face_travel=True):
        cfg = self.cfg
        me = world.me
        target = self.field.clamp_target(target)
        err = geo.sub(target, me.xy)
        d = geo.norm(err)
        speed = 0.0 if d < cfg.arrive_tol else min(vmax, cfg.k_pos * d)
        vel = geo.scale(geo.unit(err), speed)
        vel = avoid(me.xy, vel, self.obstacles(world, ball_obstacle), cfg.repulse_gain)
        vel = self.field.wall_limit(me.xy, vel, cfg.wall_soft)
        if face_travel and d > cfg.travel_face_dist and geo.norm(vel) > 0.15:
            face = geo.angle_of(vel)
        cmd = body_command(me, vel, face, cfg, mode)
        if cmd.vx > cfg.block_slow_speed and front_blocked(
                me, world.others, cfg.block_slow_dist):
            cmd.vx = cfg.block_slow_speed          # 前方有对方狗：只能慢慢靠近，绝不冲撞
        return cmd

    # ------------------------------------------------------------------
    def attack(self, world, ball):
        cfg = self.cfg
        me = world.me
        aim = choose_aim(self.field, ball, self.attack_dir, world.others)
        u = geo.unit(geo.sub(aim, ball))
        rel = geo.sub(me.xy, ball)
        along = geo.dot(rel, u)                 # <0：在球后方
        lateral = geo.cross(u, rel)             # 偏离“球→射门点”直线的距离
        heading_err = geo.wrap(geo.angle_of(u) - me.yaw)
        dball = geo.norm(rel)

        if self.pushing:
            if (along > 0.10 or abs(lateral) > cfg.push_exit_lat
                    or abs(heading_err) > cfg.push_exit_yaw or dball > cfg.push_exit_dist):
                self.pushing = False
        elif (-cfg.standoff - 0.25 < along < -0.12 and abs(lateral) < cfg.push_enter_lat
              and abs(heading_err) < cfg.push_enter_yaw):
            self.pushing = True

        if self.pushing:
            # 沿射门方向前推，横向偏差用横移纠正。
            vel = geo.add(geo.scale(u, cfg.push_speed),
                          geo.scale(geo.perp(u), -1.5 * lateral))
            dogs = self.obstacles(world)
            vel = avoid(me.xy, vel, dogs, cfg.repulse_gain)
            cmd = body_command(me, vel, geo.angle_of(u), cfg, 'push')
            if cmd.vx > cfg.block_slow_speed and front_blocked(
                    me, (center for center, _, _ in dogs), cfg.block_slow_dist):
                cmd.vx = cfg.block_slow_speed
            return self.limit_translation(me, cmd, dogs)

        behind = geo.sub(ball, geo.scale(u, cfg.standoff))
        if along > -0.15 and abs(lateral) < cfg.circle_radius:
            # 站在球前面：先横着绕到球的侧面，不从球上碾过去，避免把球踢向自家门。
            side = 1.0 if lateral >= 0 else -1.0
            target = geo.add(geo.add(ball, geo.scale(geo.perp(u), side * cfg.circle_radius)),
                             geo.scale(u, -0.15))
            mode = 'circle'
        else:
            target = behind
            mode = 'approach'
        vmax = cfg.max_vx if dball > 0.8 else cfg.near_ball_speed
        return self.go_to(world, target, geo.angle_of(u), vmax, mode, ball_obstacle=ball)

    def defend(self, world, ball, mode):
        cfg = self.cfg
        me = world.me
        goal = self.field.own_goal(self.attack_dir)
        to_ball = geo.sub(ball, goal)
        d_goal = geo.norm(to_ball)
        # 球已经压到门前、自己又在球和球门之间：上去解围（推向对方半场）。
        if (d_goal < cfg.clear_dist and geo.dist(me.xy, ball) < 0.9
                and geo.dist(me.xy, goal) < d_goal and mode == 'defend'):
            cmd = self.attack(world, ball)
            cmd.mode = 'clear_' + cmd.mode
            return cmd
        self.pushing = False
        d = geo.clamp(cfg.defend_ratio * d_goal, cfg.defend_min, cfg.defend_max)
        d = max(0.5, min(d, d_goal - 0.6))
        target = geo.add(goal, geo.scale(geo.unit(to_ball), d))
        face = geo.angle_of(geo.sub(ball, me.xy))
        return self.go_to(world, target, face, cfg.max_vx, mode, ball_obstacle=ball)

    def wait_kickoff(self, world):
        """对方开球：两只狗都退到球和己方球门之间，离球至少 kickoff_keep。"""
        cfg = self.cfg
        ball = world.ball
        goal = self.field.own_goal(self.attack_dir)
        to_ball = geo.sub(ball, goal)
        d_goal = geo.norm(to_ball)
        u = geo.unit(to_ball)
        d = max(0.6, min(cfg.defend_max, d_goal - cfg.kickoff_keep - 0.3))
        side = 0.5 if self.number == 1 else -0.5
        target = geo.add(geo.add(goal, geo.scale(u, d)), geo.scale(geo.perp(u), side))
        face = geo.angle_of(geo.sub(ball, world.me.xy))
        self.pushing = False
        return self.go_to(world, target, face, cfg.near_ball_speed, 'wait_kickoff',
                          ball_obstacle=ball)

    def dead_ball(self, world, role):
        """球进死区/出界：裁判会把球放到最近的发球点，提前站到那个点的球后方。"""
        cfg = self.cfg
        serve = self.field.nearest_serve_point(world.ball)
        self.pushing = False
        if role != 'striker':
            return self.defend(world, serve, 'defend_dead')
        aim = self.field.opp_goal(self.attack_dir)
        u = geo.unit(geo.sub(aim, serve))
        target = geo.sub(serve, geo.scale(u, cfg.standoff + 0.25))
        return self.go_to(world, target, geo.angle_of(u), cfg.max_vx, 'dead_ball',
                          ball_obstacle=serve)
