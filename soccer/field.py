"""复赛场地：球门、死区、发球点、出发点和坐标换算。"""

from . import geometry as geo


class Field:
    def __init__(self, cfg):
        self.cfg = cfg
        self.length = cfg.length
        self.width = cfg.width
        self.cy = cfg.width / 2.0
        L, W = cfg.length, cfg.width
        # 规则图 1：四角出发点距两边各 0.5 m；9 个发球点在 x=2.5/5/7.5、y=1.5/2.75/4.0。
        self.start_points = [(0.5, 0.5), (0.5, W - 0.5), (L - 0.5, 0.5), (L - 0.5, W - 0.5)]
        xs = (L / 4.0, L / 2.0, 3.0 * L / 4.0)
        ys = (1.5, self.cy, W - 1.5)
        self.serve_points = [(x, y) for x in xs for y in ys]
        self.center = (L / 2.0, self.cy)

    # ---------- 坐标换算 ----------
    def from_mocap(self, x, y, yaw=0.0):
        p = geo.rotate((x - self.cfg.origin_x, y - self.cfg.origin_y), -self.cfg.rotation)
        return p[0], p[1], geo.wrap(yaw - self.cfg.rotation)

    # ---------- 球门 ----------
    def goal_line_x(self, left):
        return self.cfg.goal_depth if left else self.length - self.cfg.goal_depth

    def goal_center(self, left):
        return (self.goal_line_x(left), self.cy)

    def own_goal(self, attack_dir):
        """attack_dir=+1 表示进攻右侧球门（己方球门在 x=0 一侧）。"""
        return self.goal_center(attack_dir > 0)

    def opp_goal(self, attack_dir):
        return self.goal_center(attack_dir < 0)

    def ball_in_goal(self, ball):
        """球完全进入球门返回 'left' / 'right'，否则 None（规则 3.4）。"""
        if ball is None:
            return None
        r = self.cfg.ball_radius
        if abs(ball[1] - self.cy) > self.cfg.goal_width / 2.0:
            return None
        if ball[0] <= self.cfg.goal_depth - r:
            return 'left'
        if ball[0] >= self.length - self.cfg.goal_depth + r:
            return 'right'
        return None

    # ---------- 区域 ----------
    def in_field(self, p, margin=0.0):
        return (margin <= p[0] <= self.length - margin
                and margin <= p[1] <= self.width - margin)

    def in_goal_mouth_band(self, y, extra=0.0):
        return abs(y - self.cy) <= self.cfg.goal_width / 2.0 + extra

    def in_dead_zone(self, p):
        """死区：距边线 0.8 m 以内，球门前方死球线断开的那一段除外。"""
        if not self.in_field(p) or self.ball_in_goal(p):
            return False
        m = self.cfg.dead_margin
        x, y = p
        inside_x = m <= x <= self.length - m
        inside_y = m <= y <= self.width - m
        if inside_x and inside_y:
            return False
        if inside_y and abs(y - self.cy) <= self.cfg.dead_gap_half:
            return False
        return True

    def needs_reset(self, ball):
        """球在死区或出界，裁判会就近放到发球点。"""
        return ball is not None and (not self.in_field(ball) or self.in_dead_zone(ball)) \
            and not self.ball_in_goal(ball)

    def nearest_serve_point(self, p):
        return min(self.serve_points, key=lambda s: geo.dist(s, p))

    def kickoff_point(self, left_team):
        """被进球方半场的开球点，距中央发球点 0.5 m。"""
        return (self.length / 2.0 - 0.5 if left_team else self.length / 2.0 + 0.5, self.cy)

    def clamp_target(self, p, margin=0.40):
        """狗的目标点不能贴墙，也不能进球门。"""
        x = geo.clamp(p[0], margin, self.length - margin)
        y = geo.clamp(p[1], margin, self.width - margin)
        if self.in_goal_mouth_band(y, 0.15):
            x = geo.clamp(x, self.cfg.goal_depth + 0.30, self.length - self.cfg.goal_depth - 0.30)
        return (x, y)

    def wall_limit(self, p, vel, soft):
        """离围挡/门框太近时压低朝墙方向的速度分量。"""
        vx, vy = vel
        x_min = self.cfg.goal_depth + 0.25 if self.in_goal_mouth_band(p[1], 0.15) else 0.30
        x_max = self.length - x_min
        y_min, y_max = 0.30, self.width - 0.30

        def factor(gap):
            return geo.clamp(gap / soft, 0.0, 1.0)

        if vx < 0:
            vx *= factor(p[0] - x_min)
        else:
            vx *= factor(x_max - p[0])
        if vy < 0:
            vy *= factor(p[1] - y_min)
        else:
            vy *= factor(y_max - p[1])
        return (vx, vy)


def detect_attack_dir(our_positions, field):
    """开场两只狗趴在己方半场出发点：平均 x 在左半场就进攻右门（+1）。"""
    xs = [p[0] for p in our_positions if p is not None]
    if not xs:
        return None
    return 1 if sum(xs) / len(xs) < field.length / 2.0 else -1


def side_to_attack_dir(side):
    """命令行 --side：left 表示己方球门在 x=0 一侧。"""
    if side == 'left':
        return 1
    if side == 'right':
        return -1
    return None

