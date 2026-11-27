"""单元测试：覆盖几何、场地规则、角色分工、比赛状态机、避让脱困与核心决策。"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from run_dog import parse as parse_cli
from soccer import config
from soccer import geometry as geo
from soccer.brain import Brain, choose_aim
from soccer.field import Field, detect_attack_dir, side_to_attack_dir
from soccer.game import GOAL_PAUSE, PLAY, WAIT_KICKOFF, GameMonitor
from soccer.model import Command, Pose, World
from soccer.motion import StuckDetector, avoid, body_command, front_blocked
from soccer.roles import RoleArbiter, striker_cost


class TestGeometry(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(geo.wrap(0.0), 0.0)
        self.assertAlmostEqual(geo.wrap(3.0 * math.pi), math.pi, places=5)
        self.assertAlmostEqual(geo.wrap(-1.5 * math.pi), 0.5 * math.pi, places=5)

    def test_yaw_from_quaternion(self):
        half = math.pi / 4.0  # 90 deg yaw
        yaw = geo.yaw_from_quaternion(0.0, 0.0, math.sin(half), math.cos(half))
        self.assertAlmostEqual(yaw, math.pi / 2.0, places=5)

    def test_point_segment_distance(self):
        d, t = geo.point_segment_distance((1.0, 2.0), (0.0, 0.0), (2.0, 0.0))
        self.assertAlmostEqual(d, 2.0)
        self.assertAlmostEqual(t, 0.5)
        d2, t2 = geo.point_segment_distance((-1.0, 0.0), (0.0, 0.0), (2.0, 0.0))
        self.assertAlmostEqual(d2, 1.0)
        self.assertAlmostEqual(t2, 0.0)


class TestField(unittest.TestCase):
    def setUp(self):
        self.field = Field(config.FieldConfig())

    def test_from_mocap_transform(self):
        cfg = config.FieldConfig(origin_x=1.0, origin_y=2.0, rotation=math.pi / 2.0)
        f = Field(cfg)
        x, y, yaw = f.from_mocap(1.0, 3.0, math.pi / 2.0)
        self.assertAlmostEqual(x, 1.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)
        self.assertAlmostEqual(yaw, 0.0, places=5)

    def test_ball_in_goal(self):
        cy = self.field.cy
        # 球心在门线 x=0.5 上不算完全进入（半径 0.11）
        self.assertIsNone(self.field.ball_in_goal((0.5, cy)))
        self.assertIsNone(self.field.ball_in_goal((0.45, cy)))
        # 完全进入左门
        self.assertEqual(self.field.ball_in_goal((0.38, cy)), 'left')
        # 超出球门宽度不算进球
        self.assertIsNone(self.field.ball_in_goal((0.2, cy + 1.0)))
        # 完全进入右门
        self.assertEqual(self.field.ball_in_goal((9.62, cy)), 'right')

    def test_dead_zone_and_goal_gap(self):
        cy = self.field.cy
        # 中场不是死区
        self.assertFalse(self.field.in_dead_zone((5.0, cy)))
        # 边线 0.8m 内是死区
        self.assertTrue(self.field.in_dead_zone((5.0, 0.3)))
        self.assertTrue(self.field.in_dead_zone((0.4, 1.0)))
        # 球门前死球线断开处不是死区
        self.assertFalse(self.field.in_dead_zone((0.6, cy)))
        # 已进球不算死区
        self.assertFalse(self.field.in_dead_zone((0.2, cy)))

    def test_serve_points_and_kickoff(self):
        self.assertEqual(len(self.field.serve_points), 9)
        self.assertEqual(self.field.nearest_serve_point((0.2, 0.2)), (2.5, 1.5))
        self.assertEqual(self.field.nearest_serve_point((5.1, 2.7)), (5.0, 2.75))
        self.assertEqual(self.field.kickoff_point(True), (4.5, 2.75))
        self.assertEqual(self.field.kickoff_point(False), (5.5, 2.75))

    def test_detect_attack_dir(self):
        self.assertEqual(detect_attack_dir([(0.5, 0.5), (0.5, 5.0)], self.field), 1)
        self.assertEqual(detect_attack_dir([(9.5, 0.5), (9.5, 5.0)], self.field), -1)
        self.assertEqual(side_to_attack_dir('left'), 1)
        self.assertEqual(side_to_attack_dir('right'), -1)
        self.assertIsNone(side_to_attack_dir('auto'))


class TestRoles(unittest.TestCase):
    def test_striker_cost_penalizes_being_ahead(self):
        ball = (5.0, 2.75)
        goal = (9.5, 2.75)
        behind = Pose(4.4, 2.75, 0.0)
        ahead = Pose(5.6, 2.75, math.pi)
        self.assertLess(striker_cost(behind, ball, goal, 0.5),
                        striker_cost(ahead, ball, goal, 0.5))

    def test_arbiter_hysteresis(self):
        arb = RoleArbiter(margin=0.5)
        self.assertEqual(arb.update({1: 2.0, 2: 2.8}), 1)
        # 2 号只比 1 号好 0.3，不超过滞回 0.5，不切换
        self.assertEqual(arb.update({1: 2.3, 2: 2.0}), 1)
        # 2 号好 0.6，超过滞回，切换
        self.assertEqual(arb.update({1: 2.6, 2: 2.0}), 2)
        # 2 号掉线（None），切回 1 号
        self.assertEqual(arb.update({1: 3.0, 2: None}), 1)


class TestGameMonitor(unittest.TestCase):
    def setUp(self):
        self.field = Field(config.FieldConfig())
        self.cfg = config.ControlConfig()

    def test_goal_pause_and_kickoff_wait(self):
        gm = GameMonitor(self.field, attack_dir=1, cfg=self.cfg, kickoff_us=True)
        self.assertEqual(gm.state, PLAY)
        # 连续 3 帧在右门内 -> 我方进球，进入 15s 暂停，随后等对方开球
        for i in range(3):
            st = gm.update(float(i) * 0.1, (9.7, 2.75), 0.0)
        self.assertEqual(st, GOAL_PAUSE)
        self.assertEqual(gm.score_us, 1)
        self.assertFalse(gm.kickoff_us)

        # 暂停未满 15s 不恢复
        st = gm.update(10.0, (5.5, 2.75), 0.0)
        self.assertEqual(st, GOAL_PAUSE)

        # 满 15s 且球静止满 1s -> WAIT_KICKOFF
        gm.update(14.5, (5.5, 2.75), 0.0)
        st = gm.update(15.6, (5.5, 2.75), 0.0)
        self.assertEqual(st, WAIT_KICKOFF)

        # 对方把球踢动超过 kickoff_move -> PLAY
        st = gm.update(16.0, (5.2, 2.75), 0.3)
        self.assertEqual(st, PLAY)

    def test_conceded_goal_gives_us_kickoff(self):
        gm = GameMonitor(self.field, attack_dir=1, cfg=self.cfg, kickoff_us=True)
        for i in range(3):
            gm.update(float(i) * 0.1, (0.2, 2.75), 0.0)
        self.assertEqual(gm.score_them, 1)
        self.assertTrue(gm.kickoff_us)
        gm.update(14.5, (4.5, 2.75), 0.0)
        st = gm.update(15.6, (4.5, 2.75), 0.0)
        self.assertEqual(st, PLAY)


class TestMotion(unittest.TestCase):
    def test_avoid_cancels_toward_velocity_at_contact(self):
        vel = avoid((5.0, 2.75), (0.5, 0.0), [((5.5, 2.75), 0.75, 0.62)], gain=0.6)
        # 距离 0.5 < contact 0.62，不应再朝障碍 (+x) 运动
        self.assertLessEqual(vel[0], 0.0)

    def test_front_blocked(self):
        me = Pose(5.0, 2.75, 0.0)
        self.assertTrue(front_blocked(me, [(5.4, 2.75)], 0.6))
        self.assertFalse(front_blocked(me, [(4.4, 2.75)], 0.6))
        self.assertFalse(front_blocked(me, [(6.0, 2.75)], 0.6))

    def test_stuck_detector(self):
        cfg = config.ControlConfig()
        det = StuckDetector(cfg)
        pose = Pose(2.0, 2.0, 0.0)
        cmd = Command(0.4, 0.0, 0.0, 'approach')
        for i in range(30):
            det.update(i * 0.1, pose, cmd)
        esc = det.escape_command(3.0)
        self.assertIsNotNone(esc)
        self.assertEqual(esc.mode, 'escape')
        self.assertLess(esc.vx, 0.0)

    def test_body_command_clamps(self):
        cfg = config.ControlConfig()
        cmd = body_command(Pose(5.0, 2.75, 0.0), (2.0, 2.0), 1.5, cfg, 'test')
        self.assertLessEqual(cmd.vx, cfg.max_vx)
        self.assertLessEqual(cmd.vy, cfg.max_vy)
        self.assertLessEqual(abs(cmd.wz), cfg.max_wz)


class TestBrain(unittest.TestCase):
    def setUp(self):
        self.field = Field(config.FieldConfig())
        self.cfg = config.ControlConfig()
        self.brain = Brain(1, self.field, self.cfg, attack_dir=1)

    def test_choose_aim_avoids_blocker(self):
        # 对方堵在球门中路，射门点应选偏上或偏下
        aim = choose_aim(self.field, (7.0, 2.75), 1, [(8.5, 2.75)])
        self.assertNotAlmostEqual(aim[1], 2.75, places=2)

    def test_halt_on_no_pose_and_goal_pause(self):
        cmd1 = self.brain.step(World(me=None), 1, PLAY, 0.0)
        self.assertTrue(cmd1.halt)
        cmd2 = self.brain.step(World(me=Pose(2.0, 2.75, 0.0), ball=(5.0, 2.75)),
                               1, GOAL_PAUSE, 0.0)
        self.assertTrue(cmd2.halt)

    def test_striker_circle_then_push(self):
        # 站在球前面 -> circle
        w_ahead = World(me=Pose(5.3, 2.75, math.pi), ball=(5.0, 2.75))
        cmd = self.brain.step(w_ahead, 1, PLAY, 0.0)
        self.assertEqual(cmd.mode, 'circle')
        # 站在球后方对准球门 -> push
        w_behind = World(me=Pose(4.6, 2.75, 0.0), ball=(5.0, 2.75))
        cmd = self.brain.step(w_behind, 1, PLAY, 0.1)
        self.assertEqual(cmd.mode, 'push')
        self.assertGreater(cmd.vx, 0.2)

    def test_front_blocked_caps_speed(self):
        w = World(me=Pose(4.6, 2.75, 0.0), ball=(5.0, 2.75), others=[(5.05, 2.75)])
        cmd = self.brain.step(w, 1, PLAY, 0.0)
        self.assertLessEqual(cmd.vx, self.cfg.block_slow_speed + 1e-6)

    def test_dead_ball_prepares_behind_serve_point(self):
        w = World(me=Pose(3.0, 2.75, 0.0), ball=(5.0, 0.2))  # 球在死区
        cmd = self.brain.step(w, 1, PLAY, 0.0)
        self.assertEqual(cmd.mode, 'dead_ball')

    def test_defender_stands_between_goal_and_ball(self):
        w = World(me=Pose(2.0, 2.75, 0.0), ball=(6.0, 2.75))
        cmd = self.brain.step(w, 2, PLAY, 0.0)  # 2 号是前锋，1 号防守
        self.assertEqual(self.brain.role, 'defender')
        self.assertEqual(cmd.mode, 'defend')

    def test_cli_parse(self):
        args, extra = parse_cli(['--dog', '2', '--dry-run', '--side', 'right',
                                 '--kickoff', 'them', '--ros-args'])
        self.assertEqual(args.dog, 2)
        self.assertTrue(args.dry_run)
        self.assertEqual(args.side, 'right')
        self.assertEqual(args.kickoff, 'them')
        self.assertEqual(extra, ['--ros-args'])


if __name__ == '__main__':
    unittest.main()
