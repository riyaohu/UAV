import csv
import os
import random

from ..simulator import Simulator
from datetime import datetime
from ..utils.seed import set_seed

# 终端运行python -m src_uav_searching_platform.evaluation.experiment_runner
# 在E:\Graduation project\src_uav_searching_platform>下运


RESULT_DIR = "results"
RESULT_FILE = os.path.join(RESULT_DIR, f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")



def run_experiments(num_runs=10, base_seed=0):
    os.makedirs(RESULT_DIR, exist_ok=True)

    with open(RESULT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["run_id", "found_all", "found_count", "total_targets", "discovery_rate", "frames", "distance", "seed",
             "stop_reason"])

        successes = 0
        sum_discovery_rate = 0.0

        for i in range(num_runs):
            seed = base_seed + i
            set_seed(seed)


            sim = Simulator(render=False, mode="experiment")
            sim.run()

            if sim.target_found:
                successes += 1

            found_count = sim.get_found_count()
            total_targets = len(sim.targets)
            discovery_rate = found_count / total_targets if total_targets > 0 else 0.0

            sum_discovery_rate += discovery_rate

            writer.writerow([
                i,
                int(sim.target_found),
                found_count,
                total_targets,
                round(discovery_rate, 4),
                sim.frames,
                round(sim.uav.distance_traveled, 2),
                seed,
                sim.stop_reason
            ])

            print(f"[Run {i}] found={sim.target_found}, frames={sim.frames}, dist={sim.uav.distance_traveled:.1f}, reason={sim.stop_reason}")


        writer.writerow([])
        writer.writerow(["SUMMARY", "success_rate", successes / num_runs, "avg_discovery_rate",
                         round(sum_discovery_rate / num_runs, 4)])
if __name__ == "__main__":
    run_experiments(num_runs=30, base_seed=200)
