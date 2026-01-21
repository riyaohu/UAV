# src_uav_searching_platform/observation/observation_model.py
import math
import random
from .. import config
class ObservationModel:
    """
    观测模型：把“真实世界”变成“传感器看到的东西”
    - 漏检：在范围内也可能检测不到
    - 误检：不在范围内也可能冒出一个假检测
    - 测距噪声：检测到时返回一个带噪声的距离（以后可用来做 belief 更新）
    """

    def __init__(self, p_false_negative=0.0, p_false_positive=0.0, distance_noise_std=0.0):
        self.p_fn = float(p_false_negative)
        self.p_fp = float(p_false_positive)
        self.noise_std = float(distance_noise_std)

    def _true_distance(self, uav_x, uav_y, target_x, target_y):
        return math.hypot(target_x - uav_x, target_y - uav_y)

    def observe_target(self, uav_x, uav_y, uav_height,
                       target_x, target_y, detection_radius):

        """
        对“某一个目标”生成观测结果。
        返回 dict：
        - detected: True/False （是否检测到这个目标）
        - noisy_distance: 带噪距离（检测到才有意义）
        - true_distance: 真实距离（方便调试/论文，可选）
        - is_false_positive: 是否为误检（这里对单个目标一般是 False）
        """
        true_d = self._true_distance(uav_x, uav_y, target_x, target_y)

        # 不在探测半径内：必然检测不到“这个目标”
        if true_d > detection_radius:
            return {"detected": False, "true_distance": true_d, "noisy_distance": None, "is_false_positive": False}

        # 在探测半径内：可能漏检
        # === height-modulated false negative rate ===
        height_factor = uav_height / getattr(config, "HEIGHT_REF", 1.0)

        # 高度越高，effective_fn 越小（更不容易漏检）
        effective_fn = self.p_fn * (height_factor ** -getattr(config, "HEIGHT_FN_ALPHA", 1.0))

        # 限制在 [0, 1]
        effective_fn = max(0.0, min(1.0, effective_fn))

        if random.random() < effective_fn:
            return {
                "detected": False,
                "true_distance": true_d,
                "noisy_distance": None,
                "is_false_positive": False
            }

        # 检测到了：加入测距噪声
        noisy_d = true_d
        if self.noise_std > 0:
            noisy_d = max(0.0, random.gauss(true_d, self.noise_std))

        return {"detected": True, "true_distance": true_d, "noisy_distance": noisy_d, "is_false_positive": False}

    def maybe_false_positive(self, map_width, map_height):
        """
        误检：以一定概率产生一个“假检测点”。
        先不影响找到目标（不标记 found），只是用于统计/可视化/论文说明。
        """
        if random.random() >= self.p_fp:
            return None
        x = random.uniform(0, map_width)
        y = random.uniform(0, map_height)
        return {"detected": True, "pos": (x, y), "is_false_positive": True}
