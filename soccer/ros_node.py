"""ROS2 适配层：收动捕、发运动指令、起身、两只狗共享分工。

只有这个文件依赖 ROS；决策逻辑在 brain.py 等纯 Python 模块里，可以离线测试。
"""

import re
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from protocol.msg import MotionServoCmd
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

try:
    from protocol.srv import MotionResultCmd
except ImportError:          # 旧固件没有这个服务时只能跳过起身
    MotionResultCmd = None

from . import config
from . import geometry as geo
from .brain import Brain
from .field import Field, detect_attack_dir, side_to_attack_dir
from .game import GameMonitor
from .model import Pose, World
from .roles import RoleArbiter, striker_cost

VRPN_POSE = re.compile(r'^/vrpn/([^/]+)/pose$')
STAND_UP_MOTION_ID = 111     # 恢复站立


class BallTracker:
    def __init__(self):
        self.pos = None
        self.stamp = None
        self.vel = (0.0, 0.0)

    def update(self, p, t):
        if self.pos is not None and t - self.stamp > 0.005:
            dt = t - self.stamp
            raw = ((p[0] - self.pos[0]) / dt, (p[1] - self.pos[1]) / dt)
            if dt > 0.5:
                raw = (0.0, 0.0)
            self.vel = (0.6 * self.vel[0] + 0.4 * raw[0], 0.6 * self.vel[1] + 0.4 * raw[1])
        self.pos, self.stamp = p, t

    def fresh(self, t, timeout):
        return self.pos is not None and t - self.stamp <= timeout


class SoccerNode(Node):
    def __init__(self, args):
        super().__init__('soccer_dog{}'.format(args.dog))
        self.cfg = config.CONTROL
        self.number = args.dog
        self.mate_number = 3 - args.dog
        self.dog = config.DOGS[self.number]
        self.mate = config.DOGS[self.mate_number]
        self.dry_run = args.dry_run
        self.field = Field(config.FIELD)
        self.kickoff_us = args.kickoff == 'us'
        self.role_override = args.role

        self.poses = {}               # 刚体名 -> (Pose, 收到时刻)
        self.ball = BallTracker()
        self.subscribed = set()
        self.ignore = set(config.IGNORE_RIGID_BODIES)

        self.attack_dir = side_to_attack_dir(args.side)
        self.brain = None
        self.game = None
        if self.attack_dir is not None:
            self.setup_side(self.attack_dir, '命令行指定')

        self.arbiter = RoleArbiter(self.cfg.role_margin)
        self.shared_striker = None
        self.shared_stamp = -1e9
        role_topic = '/soccer/{}/striker'.format(config.TEAM)
        self.role_pub = self.create_publisher(String, role_topic, 10)
        self.create_subscription(String, role_topic, self.on_role, 10)

        motion_topic = '/{}/motion_servo_cmd'.format(self.dog.namespace)
        self.motion_pub = self.create_publisher(MotionServoCmd, motion_topic, 10)

        for name in (self.dog.rigid_body, self.mate.rigid_body, config.BALL_RIGID_BODY):
            self.subscribe_rigid(name)
        self.discover()
        self.create_timer(1.0, self.discover)

        self.phase = 'standup'
        self.stand_client = None
        self.stand_future = None
        self.stand_t0 = time.monotonic()
        self.last_log = 0.0
        self.last_halt = False
        self.create_timer(1.0 / self.cfg.rate, self.tick)
        self.get_logger().info('{} 号狗启动：刚体 {}，运动话题 {}（{}）'.format(
            self.number, self.dog.rigid_body, motion_topic,
            '预览，不发运动指令' if self.dry_run else '实机'))

    # ------------------------------------------------------------ 订阅
    def subscribe_rigid(self, name):
        if name in self.subscribed or name in self.ignore:
            return
        self.subscribed.add(name)
        self.create_subscription(
            PoseStamped, '/vrpn/{}/pose'.format(name),
            lambda msg, n=name: self.on_pose(n, msg), qos_profile_sensor_data)

    def discover(self):
        for topic, _ in self.get_topic_names_and_types():
            m = VRPN_POSE.match(topic)
            if m:
                self.subscribe_rigid(m.group(1))

    def on_pose(self, name, msg):
        t = time.monotonic()
        p, q = msg.pose.position, msg.pose.orientation
        yaw = geo.yaw_from_quaternion(q.x, q.y, q.z, q.w)
        x, y, yaw = self.field.from_mocap(p.x, p.y, yaw)
        if name == config.BALL_RIGID_BODY:
            self.ball.update((x, y), t)
            return
        if name == self.dog.rigid_body:
            yaw = geo.wrap(yaw + self.dog.yaw_offset)
        elif name == self.mate.rigid_body:
            yaw = geo.wrap(yaw + self.mate.yaw_offset)
        self.poses[name] = (Pose(x, y, yaw), t)

    def on_role(self, msg):
        if self.number == 1:
            return
        try:
            self.shared_striker = int(msg.data)
            self.shared_stamp = time.monotonic()
        except ValueError:
            pass

    def pose_of(self, name, t, timeout):
        item = self.poses.get(name)
        if item is None or t - item[1] > timeout:
            return None
        return item[0]

    # ------------------------------------------------------------ 起身
    def standup(self, t):
        """规则 3.3：比赛开始时狗趴在出发点，程序启动后先起身。"""
        if self.dry_run or MotionResultCmd is None:
            if MotionResultCmd is None:
                self.get_logger().warn('没有 protocol/srv/MotionResultCmd，跳过起身')
            self.phase = 'run'
            return
        if self.stand_client is None:
            self.stand_client = self.create_client(
                MotionResultCmd, '/{}/motion_result_cmd'.format(self.dog.namespace))
        if self.stand_future is None:
            if self.stand_client.service_is_ready():
                req = MotionResultCmd.Request()
                req.motion_id = STAND_UP_MOTION_ID
                self.stand_future = self.stand_client.call_async(req)
                self.get_logger().info('发送起身指令')
            elif t - self.stand_t0 > 5.0:
                self.get_logger().warn('起身服务 5 s 未就绪，直接进入比赛')
                self.phase = 'run'
            return
        if self.stand_future.done():
            res = self.stand_future.result()
            self.get_logger().info('起身完成：result={} code={}'.format(
                getattr(res, 'result', '?'), getattr(res, 'code', '?')))
            self.phase = 'run'
        elif t - self.stand_t0 > 15.0:
            self.get_logger().warn('起身超时，直接进入比赛')
            self.phase = 'run'

    # ------------------------------------------------------------ 主循环
    def setup_side(self, attack_dir, how):
        self.attack_dir = attack_dir
        self.brain = Brain(self.number, self.field, self.cfg, attack_dir, self.role_override)
        self.game = GameMonitor(self.field, attack_dir, self.cfg, self.kickoff_us)
        self.get_logger().info('{}：己方球门在 {}，进攻 {} 球门'.format(
            how, '左侧 x=0' if attack_dir > 0 else '右侧 x=10',
            '右侧' if attack_dir > 0 else '左侧'))

    def others(self, t):
        out = []
        for name, (pose, stamp) in self.poses.items():
            if name in (self.dog.rigid_body, self.mate.rigid_body):
                continue
            if t - stamp <= self.cfg.other_timeout and self.field.in_field(pose.xy, -0.3):
                out.append(pose.xy)
        return out

    def tick(self):
        t = time.monotonic()
        if self.phase == 'standup':
            self.standup(t)
            return
        me = self.pose_of(self.dog.rigid_body, t, self.cfg.pose_timeout)
        mate = self.pose_of(self.mate.rigid_body, t, self.cfg.pose_timeout)
        if self.brain is None:
            if me is None:
                self.send_halt()
                self.log_throttled(t, '等待自己的动捕 /vrpn/{}/pose'.format(self.dog.rigid_body))
                return
            self.setup_side(detect_attack_dir([me.xy, mate.xy if mate else None],
                                              self.field), '按开场位置判断')

        ball = self.ball.pos if self.ball.fresh(t, self.cfg.ball_timeout) else None
        world = World(me=me, mate=mate, others=self.others(t), ball=ball,
                      ball_vel=self.ball.vel)
        state = self.game.update(t, ball, geo.norm(self.ball.vel) if ball else 1.0)
        if self.game.event:
            self.get_logger().info(self.game.event)
            self.game.event = None

        striker = self.decide_striker(t, world)
        cmd = self.brain.step(world, striker, state, t)
        self.send(cmd)

        if t - self.last_log >= 1.0:
            where = '球未知' if ball is None else '球({:.2f},{:.2f})'.format(*ball)
            pos = '无位姿' if me is None else '狗({:.2f},{:.2f},{:.0f}°)'.format(
                me.x, me.y, me.yaw * 57.3)
            self.get_logger().info('{} | {} | {} | {} | {} | v=({:.2f},{:.2f}) w={:.2f}'.format(
                state, self.brain.role, cmd.mode, pos, where, cmd.vx, cmd.vy, cmd.wz))
            self.last_log = t

    def decide_striker(self, t, world):
        if world.ball is None:
            return self.arbiter.striker
        ball = world.ball
        if self.field.needs_reset(ball):
            ball = self.field.nearest_serve_point(ball)
        goal = self.field.opp_goal(self.attack_dir)
        costs = {
            self.number: striker_cost(world.me, ball, goal, self.cfg.standoff),
            self.mate_number: striker_cost(world.mate, ball, goal, self.cfg.standoff),
        }
        local = self.arbiter.update(costs)
        if self.number == 1:
            if local is not None:
                self.role_pub.publish(String(data=str(local)))
            return local
        if t - self.shared_stamp <= self.cfg.role_timeout:
            self.arbiter.striker = self.shared_striker
            return self.shared_striker
        return local

    # ------------------------------------------------------------ 运动指令
    def send(self, cmd):
        if cmd.halt:
            self.send_halt()
            return
        self.last_halt = False
        if self.dry_run:
            return
        msg = MotionServoCmd()
        msg.motion_id = self.cfg.motion_id
        msg.cmd_type = 1                         # 数据帧
        msg.value = 2
        msg.vel_des = [float(cmd.vx), float(cmd.vy), float(cmd.wz)]
        msg.step_height = [self.cfg.step_height, self.cfg.step_height]
        self.motion_pub.publish(msg)

    def send_halt(self):
        if self.dry_run:
            return
        msg = MotionServoCmd()
        msg.motion_id = self.cfg.motion_id
        msg.cmd_type = 2                         # 结束帧
        msg.vel_des = [0.0, 0.0, 0.0]
        self.motion_pub.publish(msg)
        self.last_halt = True

    def log_throttled(self, t, text):
        if t - self.last_log >= 1.0:
            self.get_logger().info(text)
            self.last_log = t

    def shutdown(self):
        for _ in range(3):
            self.send_halt()
            time.sleep(0.1)


def run(args, ros_args):
    rclpy.init(args=ros_args)
    node = SoccerNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
