import os
import random

def set_seed(seed: int):
    """Set random seed for reproducibility."""
    random.seed(seed)

    # 如果以后用到 numpy，这里也一起固定
    try:
        import numpy as np
        np.random.seed(seed)
    except Exception:
        pass

    # 一些第三方库可能会读 PYTHONHASHSEED（可选）
    os.environ["PYTHONHASHSEED"] = str(seed)
