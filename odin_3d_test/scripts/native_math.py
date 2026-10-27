"""Rigid transforms and conservative native-localization acceptance."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def matrix(t):
    p = t.translation
    q = t.rotation
    values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:]) - 1) > 0.01:
        raise ValueError("Invalid native transform")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def close(a, b, distance=0.10, angle=0.10):
    return (
        np.linalg.norm(a[:3, 3] - b[:3, 3]) <= distance
        and Rotation.from_matrix(a[:3, :3].T @ b[:3, :3]).magnitude() <= angle
    )


class Alignment:
    def __init__(self):
        self.pose = None
        self.candidate = None
        self.count = 0
        self.fault = False

    def observe(self, pose):
        if self.fault:
            return
        if self.pose is not None:
            if not close(pose, self.pose, 0.5, 0.5):
                self.fault = True
            else:
                self.pose = pose
            return
        self.count = (
            self.count + 1
            if self.candidate is not None and close(pose, self.candidate)
            else 1
        )
        self.candidate = pose
        if self.count >= 3:
            self.pose = pose

    def invalidate(self):
        self.fault = True


def fresh(stamp, now, limit):
    return math.isfinite(stamp) and math.isfinite(now) and -0.05 <= now - stamp <= limit
