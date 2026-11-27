"""两只狗分工：谁去抢球（前锋），谁守门前（后卫）。

两只狗由两个进程控制。1 号狗算出前锋编号后发布到 /soccer/<TEAM>/striker，
2 号狗直接采用；1 号狗的消息超时（例如 1 号被抱起重启）时 2 号狗自己算，
两边用的是同一个代价函数，所以结果基本一致。
"""

from . import geometry as geo


def striker_cost(pose, ball, opp_goal, standoff):
    """到达“球后方击球点”的大致代价，单位约等于米。"""
    if pose is None or ball is None:
        return None
    u = geo.unit(geo.sub(opp_goal, ball))
    behind = geo.sub(ball, geo.scale(u, standoff))
    cost = geo.dist(pose.xy, behind)
    ahead = geo.dot(geo.sub(pose.xy, ball), u)
    if ahead > -0.2:                      # 站在球前面要绕一圈
        cost += 1.5 * (ahead + 0.2)
    face = abs(geo.wrap(geo.angle_of(geo.sub(ball, pose.xy)) - pose.yaw))
    return cost + 0.25 * face


class RoleArbiter:
    """带滞回地选前锋，避免两只狗来回换角色。"""

    def __init__(self, margin):
        self.margin = margin
        self.striker = None

    def update(self, costs):
        valid = {k: v for k, v in costs.items() if v is not None}
        if not valid:
            return self.striker
        best = min(valid, key=lambda k: (valid[k], k))
        if self.striker in valid and valid[self.striker] <= valid[best] + self.margin:
            return self.striker
        self.striker = best
        return best
