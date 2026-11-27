"""根据动捕里球的位置自己跟踪比赛进程（进球暂停、开球）。

规则 3.4：进球后暂停 15 s，裁判把球放到被进球方的开球点，双方把狗放回出发点，
15 s 后由被进球方开球。狗在这 15 s 里会被人搬动，所以这段时间只发结束帧。
"""

from . import geometry as geo

PLAY = 'play'
GOAL_PAUSE = 'goal_pause'
WAIT_KICKOFF = 'wait_kickoff'


class GameMonitor:
    def __init__(self, field, attack_dir, cfg, kickoff_us=True):
        self.field = field
        self.attack_dir = attack_dir
        self.cfg = cfg
        self.state = PLAY if kickoff_us else WAIT_KICKOFF
        self.kickoff_us = kickoff_us
        self.score_us = 0
        self.score_them = 0
        self.goal_frames = 0
        self.goal_side = None
        self.pause_t = None
        self.still_since = None
        self.wait_t = None
        self.wait_ref = None
        self.event = None

    def opp_goal_side(self):
        return 'right' if self.attack_dir > 0 else 'left'

    def update(self, t, ball, ball_speed):
        """每个控制周期调用一次，返回当前状态。"""
        if self.state == PLAY:
            self._play(t, ball)
        elif self.state == GOAL_PAUSE:
            self._pause(t, ball, ball_speed)
        if self.state == WAIT_KICKOFF:
            self._wait(t, ball)
        return self.state

    def _play(self, t, ball):
        side = self.field.ball_in_goal(ball)
        if side is None:
            self.goal_frames = 0
            return
        if side != self.goal_side:
            self.goal_side, self.goal_frames = side, 0
        self.goal_frames += 1
        if self.goal_frames < self.cfg.goal_confirm:
            return
        we_scored = side == self.opp_goal_side()
        if we_scored:
            self.score_us += 1
        else:
            self.score_them += 1
        self.kickoff_us = not we_scored           # 被进球方开球
        self.state = GOAL_PAUSE
        self.pause_t = t
        self.still_since = None
        self.goal_frames = 0
        self.event = '{}进球！比分 我方 {} : {} 对方，暂停 {:.0f} s'.format(
            '我方' if we_scored else '对方', self.score_us, self.score_them,
            self.cfg.goal_pause)

    def _pause(self, t, ball, ball_speed):
        ready = (ball is not None and self.field.in_field(ball)
                 and self.field.ball_in_goal(ball) is None
                 and ball_speed < self.cfg.ball_still_speed)
        if not ready:
            self.still_since = None
            return
        if self.still_since is None:
            self.still_since = t
        if (t - self.pause_t >= self.cfg.goal_pause
                and t - self.still_since >= self.cfg.ball_still_time):
            if self.kickoff_us:
                self.state = PLAY
                self.event = '暂停结束，我方开球'
            else:
                self.state = WAIT_KICKOFF
                self.wait_t = None
                self.event = '暂停结束，等待对方开球'

    def _wait(self, t, ball):
        if self.wait_t is None:
            self.wait_t = t
        if ball is not None and self.wait_ref is None:
            self.wait_ref = ball
        moved = (ball is not None and self.wait_ref is not None
                 and geo.dist(ball, self.wait_ref) > self.cfg.kickoff_move)
        if moved or t - self.wait_t >= self.cfg.kickoff_wait:
            self.state = PLAY
            self.wait_t = None
            self.wait_ref = None
            self.event = '对方已开球，继续比赛' if moved else '等待开球超时，继续比赛'
