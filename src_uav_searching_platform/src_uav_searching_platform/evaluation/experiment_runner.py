import csv
import os
import random
from .. import config
from ..simulator import Simulator
from datetime import datetime
from ..utils.seed import set_seed
from ..evaluation.metrics import compute_metrics
from .plot_results import plot_algorithm_stats
import math
import pygame

from ..algorithms import RandomSearch
from ..baseline_template_matching_cruise import config_template_matching as tm_cfg
from ..baseline_template_matching_cruise.template_matching_cruise import TemplateMatchingCruise
from ..environment.grid_map import GridMap
# 终端运行python -m src_uav_searching_platform.evaluation.experiment_runner
# 在E:\Graduation project\src_uav_searching_platform>下运


RESULT_ROOT = "results"
RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")

RESULT_DIR = os.path.join(RESULT_ROOT, RUN_ID)
RESULT_FILE = os.path.join(RESULT_DIR, "run_results.csv")
SUMMARY_FILE = os.path.join(RESULT_DIR, "summary.txt")


def run_experiments(num_runs=10, base_seed=0, use_grid_map=None, algorithm_name="random"):

    os.makedirs(RESULT_DIR, exist_ok=True)

    algo_dir = os.path.join(RESULT_DIR, algorithm_name)
    os.makedirs(algo_dir, exist_ok=True)

    result_file = os.path.join(algo_dir, "run_results.csv")

    with open(result_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["run_id", "algorithm","found_all", "found_count", "total_targets", "discovery_rate", "num_false_negatives","frames", "distance", "use_grid_map","map_width", "map_height", "cell_size", "obstacle_density", "seed",
             "stop_reason","start_x","start_y","target_positions"])

        successes = 0
        sum_discovery_rate = 0.0

        for i in range(num_runs):
            seed = base_seed + i

            fair_start_pos = None

            if use_grid_map:
                set_seed(seed)
                temp_grid = GridMap(
                    width_px=config.MAP_WIDTH,
                    height_px=config.MAP_HEIGHT,
                    cell_size=config.CELL_SIZE,
                    obstacle_density=config.OBSTACLE_DENSITY,
                    border_blocked=getattr(config, "BORDER_BLOCKED", False),
                )
                fair_start_pos = temp_grid.random_free_position()

            set_seed(seed)

            sim = Simulator(
                render=False,
                mode="experiment",
                use_grid_map=use_grid_map,
                algorithm_name=algorithm_name,
                start_pos=fair_start_pos
            )
            sim.run()

            # ===== 保存逐帧日志 =====
            frame_logs = sim.get_frame_logs()

            log_dir = os.path.join(RESULT_DIR, algorithm_name)
            os.makedirs(log_dir, exist_ok=True)

            log_file = os.path.join(log_dir, f"回合_{i:03d}_逐帧日志.csv")

            with open(log_file, "w", newline="", encoding="utf-8-sig") as lf:
                log_writer = csv.writer(lf)
                log_writer.writerow(["时间帧", "x坐标", "y坐标", "累计距离", "已发现目标数", "是否完成"])

                for row in frame_logs:
                    log_writer.writerow([
                        row["frame"],
                        int(row["x"]),
                        int(row["y"]),
                        round(row["distance"], 2),
                        row["found_count"],
                        row["target_found"],
                    ])

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
            target_positions = ";".join([f"({int(t.x)},{int(t.y)})" for t in sim.targets])

            sum_discovery_rate += discovery_rate
            start_x = getattr(sim, "start_x", None)
            start_y = getattr(sim, "start_y", None)
            writer.writerow([
                i,
                algorithm_name,
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
                sim.stop_reason,
                round(start_x, 2),
                round(start_y, 2),
                target_positions
            ])

            print(f"[Run {i}] found={sim.target_found}, frames={sim.frames}, dist={sim.uav.distance_traveled:.1f}, reason={sim.stop_reason}")


        writer.writerow([])
        writer.writerow(["SUMMARY", "success_rate", successes / num_runs, "avg_discovery_rate",
                         round(sum_discovery_rate / num_runs, 4)])
        return result_file

def run_template_matching_teacher(num_runs=5, base_seed=200):
    """
    完全对齐老师 comparison_runner 的视觉实验：
    - 不使用 Simulator
    - 随机搜索和模板匹配共享同一个场景
    - 起点固定为 tm_cfg.UAV_START_X / tm_cfg.UAV_START_Y
    - 每回合随机目标由同一个 rng 顺序生成
    - 模板匹配直接调用 TemplateMatchingCruise.get_next_position(scene_surface=...)
    """

    print("\n===== Running VISION EXPERIMENT (Teacher Mode) =====")

    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    pygame.display.set_mode((1, 1))

    map_width = int(tm_cfg.WINDOW_WIDTH)
    map_height = int(tm_cfg.WINDOW_HEIGHT)
    fps = float(tm_cfg.FPS)
    detection_radius = float(tm_cfg.UAV_DETECTION_RADIUS)
    max_frames = int(tm_cfg.MAX_FRAMES)

    start_x = float(tm_cfg.UAV_START_X)
    start_y = float(tm_cfg.UAV_START_Y)

    map_img = pygame.image.load(tm_cfg.MAP_IMAGE_PATH)
    map_surface = pygame.transform.scale(map_img, (map_width, map_height))

    target_img = pygame.image.load(tm_cfg.TARGET_TEMPLATE_PATH).convert_alpha()
    target_surface = pygame.transform.scale(
        target_img,
        (tm_cfg.TARGET_SIZE, tm_cfg.TARGET_SIZE)
    )

    def distance(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def build_scene_surface(target_x, target_y):
        surface = map_surface.copy()
        rect = target_surface.get_rect(center=(int(target_x), int(target_y)))
        surface.blit(target_surface, rect)
        return surface

    def run_random_episode(run_id, target_x, target_y, rng_seed):
        random.seed(rng_seed)

        algorithm = RandomSearch(map_width, map_height)

        x, y = start_x, start_y
        heading = 0.0
        total_distance = 0.0

        found = False
        found_frame = -1
        end_frame = max_frames - 1

        log_dir = os.path.join(RESULT_DIR, "random_search")
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, f"回合_{run_id:03d}_逐帧日志.csv")

        with open(log_file, "w", encoding="utf-8-sig", newline="") as lf:
            writer = csv.writer(lf)
            writer.writerow([
                "时间帧", "算法", "模式", "匹配分数", "平滑分数",
                "x坐标", "y坐标", "目标是否锁定", "是否发现目标",
                "累计飞行距离", "目标x", "目标y"
            ])

            for frame in range(max_frames):
                next_x, next_y = algorithm.get_next_position(x, y, heading)

                dx = next_x - x
                dy = next_y - y

                if dx != 0 or dy != 0:
                    heading = math.degrees(math.atan2(dy, dx)) % 360.0

                total_distance += math.sqrt(dx ** 2 + dy ** 2)
                x, y = next_x, next_y

                detected = distance(x, y, target_x, target_y) <= detection_radius

                writer.writerow([
                    frame,
                    "随机搜索",
                    "随机搜索",
                    "0.0000",
                    "0.0000",
                    int(x),
                    int(y),
                    "否",
                    "是" if detected else "否",
                    f"{total_distance:.2f}",
                    int(target_x),
                    int(target_y),
                ])

                if detected:
                    found = True
                    found_frame = frame
                    end_frame = frame
                    break

        return {
            "algorithm": "随机搜索",
            "run_id": run_id,
            "found": found,
            "found_frame": found_frame,
            "end_frame": end_frame,
            "elapsed_seconds": end_frame / fps,
            "distance": total_distance,
            "target_x": target_x,
            "target_y": target_y,
            "rng_seed": rng_seed,
        }

    def run_template_episode(run_id, target_x, target_y, rng_seed):
        random.seed(rng_seed)

        algorithm = TemplateMatchingCruise(map_width, map_height, tm_cfg)
        algorithm.reset(start_x, start_y, 0.0)

        scene_surface = build_scene_surface(target_x, target_y)

        x, y = start_x, start_y
        heading = 0.0
        total_distance = 0.0

        found = False
        found_frame = -1
        end_frame = max_frames - 1

        log_dir = os.path.join(RESULT_DIR, "template_matching_cruise")
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, f"回合_{run_id:03d}_逐帧日志.csv")

        with open(log_file, "w", encoding="utf-8-sig", newline="") as lf:
            writer = csv.writer(lf)
            writer.writerow([
                "时间帧", "算法", "模式", "匹配分数", "平滑分数",
                "x坐标", "y坐标", "目标是否锁定", "是否发现目标",
                "累计飞行距离", "目标x", "目标y"
            ])

            for frame in range(max_frames):
                next_x, next_y = algorithm.get_next_position(
                    x,
                    y,
                    heading,
                    scene_surface=scene_surface
                )

                debug = algorithm.get_debug_info()

                dx = next_x - x
                dy = next_y - y

                if dx != 0 or dy != 0:
                    heading = math.degrees(math.atan2(dy, dx)) % 360.0

                total_distance += math.sqrt(dx ** 2 + dy ** 2)
                x, y = next_x, next_y

                detected = distance(x, y, target_x, target_y) <= detection_radius

                writer.writerow([
                    frame,
                    "模板匹配引导搜索",
                    debug.get("模式", "未知"),
                    f"{float(debug.get('匹配分数', 0.0)):.4f}",
                    f"{float(debug.get('平滑分数', 0.0)):.4f}",
                    int(x),
                    int(y),
                    debug.get("目标是否锁定", "否"),
                    "是" if detected else "否",
                    f"{total_distance:.2f}",
                    int(target_x),
                    int(target_y),
                ])

                if detected:
                    found = True
                    found_frame = frame
                    end_frame = frame
                    break

        return {
            "algorithm": "模板匹配引导搜索",
            "run_id": run_id,
            "found": found,
            "found_frame": found_frame,
            "end_frame": end_frame,
            "elapsed_seconds": end_frame / fps,
            "distance": total_distance,
            "target_x": target_x,
            "target_y": target_y,
            "rng_seed": rng_seed,
        }

    rng = random.Random(base_seed)
    margin = int(getattr(tm_cfg, "TARGET_SAFE_MARGIN", 60))

    all_results = []
    scene_rows = []

    for episode_id in range(1, num_runs + 1):
        target_x = float(rng.randint(margin, map_width - margin))
        target_y = float(rng.randint(margin, map_height - margin))

        rng_seed = base_seed + episode_id * 1009

        for algo_label in ["随机搜索", "模板匹配搜索"]:
            scene_rows.append([
                "视觉搜索实验",
                algo_label,
                episode_id,
                rng_seed,
                0,
                map_width,
                map_height,
                "",
                0.0,
                start_x,
                start_y,
                f"({int(target_x)},{int(target_y)})",
            ])

        random_res = run_random_episode(episode_id, target_x, target_y, rng_seed)
        template_res = run_template_episode(episode_id, target_x, target_y, rng_seed)

        all_results.append(random_res)
        all_results.append(template_res)

        print(
            f"视觉回合 {episode_id:03d}: "
            f"随机={'成功' if random_res['found'] else '失败'}(帧{random_res['found_frame']}), "
            f"模板={'成功' if template_res['found'] else '失败'}(帧{template_res['found_frame']})"
        )

    episode_rows = []
    for r in all_results:
        episode_rows.append([
            "视觉搜索实验",
            r["algorithm"],
            r["run_id"],
            r["rng_seed"],
            1 if r["found"] else 0,
            1 if r["found"] else 0,
            1,
            1.0 if r["found"] else 0.0,
            r["end_frame"],
            round(r["distance"], 2),
            "success" if r["found"] else "timeout",
            0,
            map_width,
            map_height,
            "",
            0.0,
            start_x,
            start_y,
            f"({int(r['target_x'])},{int(r['target_y'])})",
        ])

    summary_rows = []
    vision_results = {}

    for algo_name in ["随机搜索", "模板匹配引导搜索"]:
        sub = [r for r in all_results if r["algorithm"] == algo_name]
        success = [r for r in sub if r["found"]]

        success_rate = len(success) / len(sub) if sub else 0.0
        avg_end_frame = sum(r["end_frame"] for r in sub) / len(sub) if sub else 0.0
        avg_distance = sum(r["distance"] for r in sub) / len(sub) if sub else 0.0

        # 老师口径：只统计成功回合
        avg_success_frame = sum(r["found_frame"] for r in success) / len(success) if success else 0.0
        avg_success_time = avg_success_frame / fps if success else 0.0
        summary_rows.append([
            "视觉搜索实验",
            algo_name,
            len(sub),
            round(success_rate, 4),
            round(avg_end_frame, 2),
            round(avg_distance, 2),
            round(avg_success_frame, 2),
            round(avg_success_time, 2),
        ])

        vision_results[algo_name] = {
            "num_runs": len(sub),
            "success_rate": success_rate,
            "avg_frames": avg_end_frame,
            "avg_distance": avg_distance,
            "avg_success_frame": avg_success_frame,
            "avg_success_time": avg_success_time,
        }

    return episode_rows, summary_rows, scene_rows, vision_results

if __name__ == "__main__":
    episode_rows = []
    summary_rows = []
    scene_rows = []
    algorithms = [
        ("random", True),
        ("coverage", True),
        ("frontier", True),
        ("information_gain", True),
    ]
    grid_results = {}
    vision_results = {}

    for algo, use_grid in algorithms:
        print(f"\n===== Running {algo} =====")

        output_file = run_experiments(
            num_runs=5,
            base_seed=200,
            use_grid_map=use_grid,
            algorithm_name=algo
        )

        metrics = compute_metrics(output_file)

        if use_grid:
            grid_results[algo] = metrics
            experiment_group = "GridMap搜索实验"
        else:
            vision_results[algo] = metrics
            experiment_group = "视觉搜索实验"

        # ===== 汇总到“回合级结果.csv” =====
        with open(output_file, "r", newline="") as rf:
            reader = csv.DictReader(rf)
            for row in reader:
                if row["run_id"] == "" or row["run_id"] == "SUMMARY":
                    continue

                episode_rows.append([
                    experiment_group,
                    row["algorithm"],
                    row["run_id"],
                    row["seed"],
                    row["found_all"],
                    row["found_count"],
                    row["total_targets"],
                    row["discovery_rate"],
                    row["frames"],
                    row["distance"],
                    row["stop_reason"],
                    row["use_grid_map"],
                    row["map_width"],
                    row["map_height"],
                    row["cell_size"],
                    row["obstacle_density"],
                    row.get("start_x", ""),
                    row.get("start_y", ""),
                    row.get("target_positions", ""),
                ])
                scene_rows.append([
                    experiment_group,
                    row["algorithm"],
                    row["run_id"],
                    row["seed"],
                    row["use_grid_map"],
                    row["map_width"],
                    row["map_height"],
                    row["cell_size"],
                    row["obstacle_density"],
                    row.get("start_x", ""),
                    row.get("start_y", ""),
                    row.get("target_positions", ""),
                ])
        # ===== 汇总到“算法对照统计.csv” =====
        summary_rows.append([
            experiment_group,
            algo,
            metrics["num_runs"],
            round(metrics["success_rate"], 4),
            round(metrics["avg_frames"], 2),
            round(metrics["avg_distance"], 2),
            "",
            "",
        ])

    # =========================
    # ⭐ 加入老师模式视觉实验
    # =========================
    vision_episode, vision_summary, vision_scene, teacher_vision_results = run_template_matching_teacher(
        num_runs=5,
        base_seed=20260408
    )

    episode_rows.extend(vision_episode)
    summary_rows.extend(vision_summary)
    scene_rows.extend(vision_scene)

    vision_results = teacher_vision_results




    # =========================
    # 老师风格：回合级结果总表
    # =========================

    # =========================
    # 老师风格：场景列表
    # =========================
    scene_file = os.path.join(RESULT_DIR, "场景列表.csv")

    with open(scene_file, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "实验组",
            "算法",
            "回合",
            "随机种子",
            "是否使用GridMap",
            "地图宽度",
            "地图高度",
            "栅格大小",
            "障碍物密度",
            "起始x",
            "起始y",
            "目标位置",
        ])
        writer.writerows(scene_rows)

    print(f"场景列表 saved to: {scene_file}")

    episode_file = os.path.join(RESULT_DIR, "回合级结果.csv")



    with open(episode_file, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "实验组",
            "算法",
            "回合",
            "随机种子",
            "是否成功",
            "已发现目标数",
            "总目标数",
            "发现率",
            "结束帧数",
            "结束距离",
            "停止原因",
            "是否使用GridMap",
            "地图宽度",
            "地图高度",
            "栅格大小",
            "障碍物密度",
            "起始x",
            "起始y",
            "目标位置",
        ])
        writer.writerows(episode_rows)

    print(f"回合级结果 saved to: {episode_file}")

    # =========================
    # 老师风格：算法对照统计总表
    # =========================
    summary_file = os.path.join(RESULT_DIR, "算法对照统计.csv")

    with open(summary_file, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "实验组",
            "算法",
            "总回合数",
            "成功率",
            "平均结束帧数",
            "平均结束距离",
            "平均成功帧数",
            "平均成功时间(秒)",
        ])
        writer.writerows(summary_rows)

    print(f"算法对照统计 saved to: {summary_file}")

    # =========================
    # 老师风格：对照结论
    # =========================
    conclusion_file = os.path.join(RESULT_DIR, "对照结论.txt")

    with open(conclusion_file, "w", encoding="utf-8") as f:
        f.write("算法对照结论\n")
        f.write("====================\n\n")

        f.write("一、GridMap 搜索实验\n")
        if grid_results:
            sorted_grid = sorted(grid_results.items(), key=lambda x: x[1]["avg_frames"])
            for i, (algo, m) in enumerate(sorted_grid, 1):
                f.write(
                    f"{i}. {algo}: 成功率={m['success_rate']:.2f}, "
                    f"平均结束帧数={m['avg_frames']:.2f}, "
                    f"平均结束距离={m['avg_distance']:.2f}\n"
                )
            best_grid = sorted_grid[0][0]
            f.write(f"GridMap 实验中，平均结束帧数最小的算法为：{best_grid}。\n\n")

        f.write("二、视觉搜索实验\n")
        if vision_results:
            sorted_vision = sorted(vision_results.items(), key=lambda x: x[1]["avg_frames"])
            for i, (algo, m) in enumerate(sorted_vision, 1):
                f.write(
                    f"{i}. {algo}: 成功率={m['success_rate']:.2f}, "
                    f"平均结束帧数={m['avg_frames']:.2f}, "
                    f"平均结束距离={m['avg_distance']:.2f}\n"
                )

            if "随机搜索" in vision_results and "模板匹配引导搜索" in vision_results:
                r = vision_results["随机搜索"]
                tm = vision_results["模板匹配引导搜索"]
                if r["avg_success_time"] > 0:
                    improvement = (r["avg_success_time"] - tm["avg_success_time"]) / r["avg_success_time"] * 100
                    success_gap = (tm["success_rate"] - r["success_rate"]) * 100

                    f.write(f"平均成功时间相对改进(模板匹配相对随机): {improvement:.2f}%\n")
                    f.write(f"成功率差值(模板匹配-随机): {success_gap:.2f}%\n")
            f.write("\n")

        f.write("三、说明\n")
        f.write(
            "GridMap 实验主要用于比较基于空间探索的搜索策略；视觉搜索实验主要用于比较模板匹配引导方法与图片地图下随机搜索的差异。\n")

    print(f"对照结论 saved to: {conclusion_file}")

    # ✅ 最终对比输出
    print("\n\n===== GRID SEARCH COMPARISON =====")
    print(f"{'Algorithm':<20}{'Success':<10}{'Frames':<12}{'Distance':<12}")

    for algo, m in grid_results.items():
        print(f"{algo:<20}{m['success_rate']:<10.2f}{m['avg_frames']:<12.1f}{m['avg_distance']:<12.1f}")

    print("\n\n===== VISION METHOD COMPARISON =====")
    print(f"{'Algorithm':<20}{'Success':<10}{'Frames':<12}{'Distance':<12}")

    for algo, m in vision_results.items():
        print(f"{algo:<20}{m['success_rate']:<10.2f}{m['avg_frames']:<12.1f}{m['avg_distance']:<12.1f}")

    plot_algorithm_stats(RESULT_DIR)

