"""验证推射和脱困时，实际发往机身的速度会响应附近的狗和围挡。"""

import math
import unittest

from soccer import config, geometry as geo
from soccer.brain import Brain
from soccer.field import Field
from soccer.game import PLAY
from soccer.model import Pose, World


class TestMatchClearance(unittest.TestCase):
    def setUp(self):
        self.cfg = config.ControlConfig()
        self.brain = Brain(1, Field(config.FieldConfig()), self.cfg, attack_dir=1)

    def test_push_yields_when_teammate_crosses_shot(self):
        world = World(me=Pose(4.6, 2.75, 0.0), ball=(5.0, 2.75))
        clear = self.brain.step(world, 1, PLAY, 0.0)
        self.assertEqual(clear.mode, 'push')
        self.assertAlmostEqual(clear.vx, self.cfg.push_speed)

        world.mate = Pose(5.05, 2.75, 0.0)
        blocked = self.brain.step(world, 1, PLAY, 0.1)
        self.assertLessEqual(blocked.vx, 0.0)
        world.mate = None
        self.assertAlmostEqual(self.brain.step(world, 1, PLAY, 0.2).vx,
                               self.cfg.push_speed)

    def first_escape(self, world):
        # 位姿保持不变，模拟有速度指令但机身卡住的连续控制周期。
        for frame in range(50):
            now = frame / self.cfg.rate
            cmd = self.brain.step(world, 1, PLAY, now)
            if cmd.mode == 'escape':
                return now, cmd
        self.fail('卡住后未进入脱困动作')

    def test_escape_rechecks_teammate_and_opponent_each_tick(self):
        me = Pose(5.0, 2.75, math.pi / 4)
        world = World(me=me, ball=(7.0, 2.75))
        now, clear = self.first_escape(world)
        self.assertAlmostEqual(clear.vx, -self.cfg.max_back)
        self.assertNotEqual(clear.wz, 0.0)

        behind = (4.7, 2.45)
        for index, who in enumerate(('mate', 'opponent'), 1):
            with self.subTest(who=who):
                world.mate = Pose(*behind, 0.0) if who == 'mate' else None
                world.others = [behind] if who == 'opponent' else []
                cmd = self.brain.step(world, 1, PLAY, now + index / self.cfg.rate)
                velocity = geo.rotate((cmd.vx, cmd.vy), me.yaw)
                self.assertLessEqual(geo.dot(velocity, geo.sub(behind, me.xy)), 1e-9)

    def test_escape_checks_wall_in_field_coordinates(self):
        # 狗头朝场地 +Y，后退会撞到 y=0 的围挡。
        world = World(me=Pose(5.0, 0.30, math.pi / 2), ball=(7.0, 2.75))
        _, cmd = self.first_escape(world)
        _, field_vy = geo.rotate((cmd.vx, cmd.vy), world.me.yaw)
        self.assertGreaterEqual(field_vy, -1e-9)

    def test_avoid_dogs_false_ignores_dogs_but_keeps_wall_limit(self):
        cfg = config.ControlConfig(avoid_dogs=False)
        brain = Brain(1, Field(config.FieldConfig()), cfg, attack_dir=-1)
        # 复现现场：GEXY2 在 (1.51, 2.10) 时，关闭避让后不再在球前停死
        world = World(me=Pose(1.87, 2.64, math.radians(163.0)),
                      ball=(1.22, 2.53), others=[(1.51, 2.10)])
        cmd = brain.step(world, 1, PLAY, 0.0)
        self.assertGreater(cmd.speed, 0.15)
        # 推射时也不因前方有狗减速
        world_push = World(me=Pose(1.65, 2.40, math.radians(163.0)),
                           ball=(1.22, 2.53), mate=Pose(1.30, 2.50, 0.0),
                           others=[(1.51, 2.10)])
        push_cmd = brain.step(world_push, 1, PLAY, 0.1)
        self.assertEqual(push_cmd.mode, 'push')
        self.assertAlmostEqual(push_cmd.vx, cfg.push_speed, places=4)
        # 围挡限速仍然有效
        wall_world = World(me=Pose(5.0, 0.30, math.pi / 2), ball=(7.0, 2.75))
        _, esc = self._first_escape_with(brain, cfg, wall_world)
        _, field_vy = geo.rotate((esc.vx, esc.vy), wall_world.me.yaw)
        self.assertGreaterEqual(field_vy, -1e-9)

    def _first_escape_with(self, brain, cfg, world):
        for frame in range(50):
            now = frame / cfg.rate
            cmd = brain.step(world, 1, PLAY, now)
            if cmd.mode == 'escape':
                return now, cmd
        self.fail('卡住后未进入脱困动作')
