"""二维向量小工具，点用 (x, y) 元组表示。"""

import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    """把角度归一化到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def scale(a, k):
    return (a[0] * k, a[1] * k)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def norm(a):
    return math.hypot(a[0], a[1])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(a, fallback=(1.0, 0.0)):
    n = norm(a)
    if n < 1e-9:
        return fallback
    return (a[0] / n, a[1] / n)


def perp(a):
    """逆时针转 90 度。"""
    return (-a[1], a[0])


def angle_of(a):
    return math.atan2(a[1], a[0])


def from_angle(theta):
    return (math.cos(theta), math.sin(theta))


def rotate(a, theta):
    c, s = math.cos(theta), math.sin(theta)
    return (c * a[0] - s * a[1], s * a[0] + c * a[1])


def limit(a, max_norm):
    n = norm(a)
    if n <= max_norm or n < 1e-9:
        return a
    return scale(a, max_norm / n)


def yaw_from_quaternion(x, y, z, w):
    """课件第四讲给出的四元数转偏航角公式。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def point_segment_distance(p, a, b):
    """点 p 到线段 ab 的距离，以及投影参数 t∈[0, 1]。"""
    ab = sub(b, a)
    length2 = dot(ab, ab)
    if length2 < 1e-12:
        return dist(p, a), 0.0
    t = clamp(dot(sub(p, a), ab) / length2, 0.0, 1.0)
    return dist(p, add(a, scale(ab, t))), t
