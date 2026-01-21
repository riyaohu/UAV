import csv
import os
import random
from .. import config
from ..simulator import Simulator
from datetime import datetime
from ..utils.seed import set_seed

# 终端运行python -m src_uav_searching_platform.evaluation.experiment_runner
# 在E:\Graduation project\src_uav_searching_platform>下运


RESULT_DIR = "results"
RESULT_FILE = os.path.join(RESULT_DIR, f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")



def run_experiments(num_runs=10, base_seed=0, use_grid_map=None):
    os.makedirs(RESULT_DIR, exist_ok=True)

    with open(RESULT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["run_id", "found_all", "found_count", "total_targets", "discovery_rate", "num_false_negatives","frames", "distance", "use_grid_map","map_width", "map_height", "cell_size", "obstacle_density", "seed",
             "stop_reason"])

        successes = 0
        sum_discovery_rate = 0.0

        for i in range(num_runs):
            seed = base_seed + i
            set_seed(seed)

            sim = Simulator(render=False, mode="experiment", use_grid_map=use_grid_map)
            sim.run()

            if sim.target_found:
                successes += 1
            use_grid = int(sim.use_grid_map)

            map_w = getattr(sim, "map_width", None)
            map_h = getattr(sim, "map_height", None)

            cell_size = getattr(sim.grid_map, "cell_size", None) if sim.grid_map is not None else None
            obstacle_density = getattr(config, "OBSTACLE_DENSITY", None) if sim.grid_map is not None else 0.0

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
                sim.num_false_negatives,
                sim.frames,
                round(sim.uav.distance_traveled, 2),
                int(sim.use_grid_map),
                map_w, map_h, cell_size, obstacle_density,
                seed,
                sim.stop_reason
            ])

            print(f"[Run {i}] found={sim.target_found}, frames={sim.frames}, dist={sim.uav.distance_traveled:.1f}, reason={sim.stop_reason}")


        writer.writerow([])
        writer.writerow(["SUMMARY", "success_rate", successes / num_runs, "avg_discovery_rate",
                         round(sum_discovery_rate / num_runs, 4)])



if __name__ == "__main__":
    run_experiments(num_runs=30, base_seed=200,use_grid_map=True)
