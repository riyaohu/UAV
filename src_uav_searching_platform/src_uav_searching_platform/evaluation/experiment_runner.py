import csv
import os
import random

from ..simulator import Simulator
from datetime import datetime
# 终端运行python -m src_uav_searching_platform.evaluation.experiment_runner
# 在E:\Graduation project\src_uav_searching_platform>下运


RESULT_DIR = "results"
RESULT_FILE = os.path.join(RESULT_DIR, f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")


def run_experiments(num_runs=10, base_seed=0):
    os.makedirs(RESULT_DIR, exist_ok=True)

    with open(RESULT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["run_id", "found", "frames", "distance", "seed", "stop_reason"])

        for i in range(num_runs):
            seed = base_seed + i
            random.seed(seed)

            sim = Simulator(render=False, mode="experiment")
            sim.run()

            writer.writerow([
                i,
                int(sim.target_found),
                sim.frames,
                round(sim.uav.distance_traveled, 2),
                seed,
                sim.stop_reason
            ])

            print(f"[Run {i}] found={sim.target_found}, frames={sim.frames}, dist={sim.uav.distance_traveled:.1f}, reason={sim.stop_reason}")


if __name__ == "__main__":
    run_experiments(num_runs=30, base_seed=100)
