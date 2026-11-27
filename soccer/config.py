"""比赛现场需要改的参数都在这里。

坐标系与复赛规则图 1 一致：原点在场地左下角，X 沿 10 m 长边，Y 沿 5.5 m 短边。
如果动捕世界坐标系的原点不在左下角，用 FieldConfig.origin_x / origin_y / rotation 换算。
"""

from typing import Dict


class _ConfigBase:
    """兼容 Python 3.6 的轻量配置基类，支持用关键字覆盖默认字段。"""

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            if not hasattr(type(self), key) or key.startswith('_'):
                raise TypeError('未知配置项: {}'.format(key))
            setattr(self, key, value)


class FieldConfig(_ConfigBase):
    length = 10.0                 # 场地长，m
    width = 5.5                   # 场地宽，m
    goal_width = 1.5              # 球门宽，m
    goal_depth = 0.5              # 球门在围挡内侧的深度：门线在 x=0.5 和 x=9.5
    dead_margin = 0.8             # 死球线距边线 0.8 m
    dead_gap_half = 0.85          # 死球线在球门前断开的半宽（门宽一半 + 0.1）
    ball_radius = 0.11            # 球半径，m（现场量一下）
    origin_x = 0.0                # 场地左下角在动捕坐标系中的 x
    origin_y = 0.0                # 场地左下角在动捕坐标系中的 y
    rotation = 0.0                # 场地 X 轴相对动捕 X 轴的转角，rad


class DogConfig:
    def __init__(self, rigid_body, namespace, host='', yaw_offset=0.0):
        self.rigid_body = rigid_body    # FZMotion 里的刚体名，话题 /vrpn/<rigid_body>/pose
        self.namespace = namespace      # 运动话题名字空间，/<namespace>/motion_servo_cmd
        self.host = host                # ssh 模式下这只狗的地址（start.sh 用）
        self.yaw_offset = yaw_offset    # 刚体 X 轴与狗头不一致时的补偿，rad


class ControlConfig(_ConfigBase):
    rate = 10.0                   # 控制频率，Hz（课件示例 0.1 s 一帧）
    motion_id = 303               # 伺服行走步态
    step_height = 0.05            # 抬腿高度，m
    max_vx = 0.55                 # 前进最大速度，m/s
    max_back = 0.25               # 后退最大速度，m/s
    max_vy = 0.30                 # 横移最大速度，m/s
    max_wz = 1.0                  # 最大角速度，rad/s
    k_pos = 1.2                   # 走点比例增益
    k_turn = 1.6                  # 转向比例增益
    arrive_tol = 0.08             # 到点容差，m
    travel_face_dist = 1.0        # 离目标远于此距离时朝着行进方向走
    near_ball_speed = 0.30        # 离球 0.8 m 内、还没对准时的限速

    standoff = 0.50               # 绕到球后方的距离（狗中心到球心）
    push_speed = 0.50             # 带球/推射速度
    push_enter_lat = 0.12         # 进入推射：横向偏差
    push_enter_yaw = 0.40         # 进入推射：朝向偏差，rad
    push_exit_lat = 0.28          # 退出推射：横向偏差
    push_exit_yaw = 0.75          # 退出推射：朝向偏差，rad
    push_exit_dist = 0.90         # 退出推射：离球距离
    circle_radius = 0.60          # 站在球前方时绕球的半径

    defend_min = 0.9              # 防守站位离己方门线中心的距离范围
    defend_max = 2.4
    defend_ratio = 0.45
    clear_dist = 1.8              # 球离己方球门这么近时，防守狗允许上抢解围

    role_margin = 0.5             # 换前锋的滞回，m
    role_timeout = 0.6            # 1 号狗的分工消息超过这么久没收到，2 号狗自己算

    opp_radius = 0.75             # 对方/其他狗的避让影响半径（中心距）
    mate_radius = 0.80            # 队友避让半径
    ball_avoid_radius = 0.40      # 没对准时把球当障碍，避免把球碰歪
    dog_contact = 0.62            # 两狗中心距小于此值视为贴身，朝对方的速度清零
    ball_contact = 0.30
    repulse_gain = 0.6            # 避让斥力
    block_slow_dist = 0.60        # 正前方这么近有别的狗就减速，杜绝主动冲撞
    block_slow_speed = 0.08
    wall_soft = 0.35              # 离围挡这么近开始压低朝墙速度

    stuck_time = 2.5              # 有速度指令但几乎不动超过此时间判卡住
    stuck_move = 0.10             # 卡住判定的位移阈值，m
    escape_time = 1.2             # 脱困动作时长
    escape_cooldown = 2.0

    pose_timeout = 0.3            # 自己的动捕超过此时间没更新就停
    ball_timeout = 0.5
    other_timeout = 0.5

    goal_confirm = 3              # 连续多少帧球在门内才算进球
    goal_pause = 15.0             # 进球后暂停 15 s（规则 3.4）
    ball_still_speed = 0.06
    ball_still_time = 1.0
    kickoff_wait = 5.0            # 对方开球时最多等多久
    kickoff_move = 0.15           # 球被踢动多少算开球
    kickoff_keep = 1.0            # 对方开球前离球至少多远


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
