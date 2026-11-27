"""比赛现场需要改的参数都在这里。

坐标系与复赛规则图 1 一致：原点在场地左下角，X 沿 10 m 长边，Y 沿 5.5 m 短边。
如果动捕世界坐标系的原点不在左下角，用 FieldConfig.origin_x / origin_y / rotation 换算。
"""

from dataclasses import dataclass, field
from typing import Dict


@dataclass
class FieldConfig:
    length: float = 10.0          # 场地长，m
    width: float = 5.5            # 场地宽，m
    goal_width: float = 1.5       # 球门宽，m
    goal_depth: float = 0.5       # 球门在围挡内侧的深度：门线在 x=0.5 和 x=9.5
    dead_margin: float = 0.8      # 死球线距边线 0.8 m
    dead_gap_half: float = 0.85   # 死球线在球门前断开的半宽（门宽一半 + 0.1）
    ball_radius: float = 0.11     # 球半径，m（现场量一下）
    origin_x: float = 0.0         # 场地左下角在动捕坐标系中的 x
    origin_y: float = 0.0         # 场地左下角在动捕坐标系中的 y
    rotation: float = 0.0         # 场地 X 轴相对动捕 X 轴的转角，rad


@dataclass
class DogConfig:
    rigid_body: str               # FZMotion 里的刚体名，话题 /vrpn/<rigid_body>/pose
    namespace: str                # 运动话题名字空间，/<namespace>/motion_servo_cmd
    host: str = ''                # ssh 模式下这只狗的地址（start.sh 用）
    yaw_offset: float = 0.0       # 刚体 X 轴与狗头不一致时的补偿，rad


@dataclass
class ControlConfig:
    rate: float = 10.0            # 控制频率，Hz（课件示例 0.1 s 一帧）
    motion_id: int = 303          # 伺服行走步态
    step_height: float = 0.05     # 抬腿高度，m
    max_vx: float = 0.55          # 前进最大速度，m/s
    max_back: float = 0.25        # 后退最大速度，m/s
    max_vy: float = 0.30          # 横移最大速度，m/s
    max_wz: float = 1.0           # 最大角速度，rad/s
    k_pos: float = 1.2            # 走点比例增益
    k_turn: float = 1.6           # 转向比例增益
    arrive_tol: float = 0.08      # 到点容差，m
    travel_face_dist: float = 1.0 # 离目标远于此距离时朝着行进方向走
    near_ball_speed: float = 0.30 # 离球 0.8 m 内、还没对准时的限速

    standoff: float = 0.50        # 绕到球后方的距离（狗中心到球心）
    push_speed: float = 0.50      # 带球/推射速度
    push_enter_lat: float = 0.12  # 进入推射：横向偏差
    push_enter_yaw: float = 0.40  # 进入推射：朝向偏差，rad
    push_exit_lat: float = 0.28   # 退出推射：横向偏差
    push_exit_yaw: float = 0.75   # 退出推射：朝向偏差，rad
    push_exit_dist: float = 0.90  # 退出推射：离球距离
    circle_radius: float = 0.60   # 站在球前方时绕球的半径

    defend_min: float = 0.9       # 防守站位离己方门线中心的距离范围
    defend_max: float = 2.4
    defend_ratio: float = 0.45
    clear_dist: float = 1.8       # 球离己方球门这么近时，防守狗允许上抢解围

    role_margin: float = 0.5      # 换前锋的滞回，m
    role_timeout: float = 0.6     # 1 号狗的分工消息超过这么久没收到，2 号狗自己算

    opp_radius: float = 0.75      # 对方/其他狗的避让影响半径（中心距）
    mate_radius: float = 0.80     # 队友避让半径
    ball_avoid_radius: float = 0.40  # 没对准时把球当障碍，避免把球碰歪
    dog_contact: float = 0.62     # 两狗中心距小于此值视为贴身，朝对方的速度清零
    ball_contact: float = 0.30
    repulse_gain: float = 0.6     # 避让斥力
    block_slow_dist: float = 0.60 # 正前方这么近有别的狗就减速，杜绝主动冲撞
    block_slow_speed: float = 0.08
    wall_soft: float = 0.35       # 离围挡这么近开始压低朝墙速度

    stuck_time: float = 2.5       # 有速度指令但几乎不动超过此时间判卡住
    stuck_move: float = 0.10      # 卡住判定的位移阈值，m
    escape_time: float = 1.2      # 脱困动作时长
    escape_cooldown: float = 2.0

    pose_timeout: float = 0.3     # 自己的动捕超过此时间没更新就停
    ball_timeout: float = 0.5
    other_timeout: float = 0.5

    goal_confirm: int = 3         # 连续多少帧球在门内才算进球
    goal_pause: float = 15.0      # 进球后暂停 15 s（规则 3.4）
    ball_still_speed: float = 0.06
    ball_still_time: float = 1.0
    kickoff_wait: float = 5.0     # 对方开球时最多等多久
    kickoff_move: float = 0.15    # 球被踢动多少算开球
    kickoff_keep: float = 1.0     # 对方开球前离球至少多远


FIELD = FieldConfig()
CONTROL = ControlConfig()

TEAM = 'red'                      # 只用于分工话题名 /soccer/<TEAM>/striker
BALL_RIGID_BODY = 'ball'

# 1 号狗、2 号狗（规则 6.2：左侧终端 1 号，右侧终端 2 号）。
DOGS: Dict[int, DogConfig] = {
    1: DogConfig(rigid_body='red1', namespace='fixed_value', host='mi@192.168.1.101'),
    2: DogConfig(rigid_body='red2', namespace='fixed_value2', host='mi@192.168.1.102'),
}

# 不属于我方和球的其他刚体都当作“其他狗”避让。场外刚体（下一场赛队）会按坐标过滤。
IGNORE_RIGID_BODIES = []
