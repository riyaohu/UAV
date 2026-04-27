import csv
import os
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _setup_chinese_font():
    plt.rcParams["font.sans-serif"] = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "Arial Unicode MS",
        "DejaVu Sans",
    ]
    plt.rcParams["axes.unicode_minus"] = False


def _read_csv_dict_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def _to_float(value: Optional[str], default: float = 0.0) -> float:
    if value is None:
        return default
    text = str(value).strip()
    if text == "":
        return default
    try:
        return float(text)
    except ValueError:
        return default


def _extract_algorithm_names(summary_rows: List[Dict[str, str]]) -> List[str]:
    names = []
    for row in summary_rows:
        name = (row.get("算法") or "").strip()
        if name:
            names.append(name)

    seen = set()
    ordered = []
    for n in names:
        if n not in seen:
            seen.add(n)
            ordered.append(n)
    return ordered


def _index_by_algorithm(rows: List[Dict[str, str]]) -> Dict[str, Dict[str, str]]:
    data = {}
    for row in rows:
        name = (row.get("算法") or "").strip()
        if name:
            data[name] = row
    return data


def _group_episode_rows(episode_rows: List[Dict[str, str]]) -> Dict[str, Dict[int, Dict[str, str]]]:
    grouped = {}
    for row in episode_rows:
        algo = (row.get("算法") or "").strip()
        ep = int(_to_float(row.get("回合"), 0))
        if not algo:
            continue
        grouped.setdefault(algo, {})[ep] = row
    return grouped


def _plot_core_metrics(summary_rows, algorithms, out_path, dpi):
    data = _index_by_algorithm(summary_rows)

    metrics = [
        ("成功率", "成功率(%)", True),
        ("平均结束帧数", "平均结束帧数", False),
        ("平均结束距离", "平均结束距离", False),
        ("总回合数", "总回合数", False),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2", "#B279A2"]

    for idx, (field, ylabel, as_percent) in enumerate(metrics):
        ax = axes[idx]
        values = []

        for algo in algorithms:
            value = _to_float(data.get(algo, {}).get(field), np.nan)
            if as_percent and not np.isnan(value):
                value *= 100.0
            values.append(value)

        bars = ax.bar(algorithms, values, color=colors[:len(algorithms)], alpha=0.9)

        for bar, value in zip(bars, values):
            if np.isnan(value):
                continue
            label = f"{value:.1f}%" if as_percent else f"{value:.1f}"
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height(),
                label,
                ha="center",
                va="bottom",
                fontsize=9,
            )

        ax.set_title(field)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", linestyle="--", alpha=0.3)
        ax.tick_params(axis="x", rotation=20)

    fig.suptitle("算法核心指标对照", fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)

def _plot_vision_core_metrics(summary_rows, algorithms, out_path, dpi):
    data = _index_by_algorithm(summary_rows)

    metrics = [
        ("成功率", "成功率(%)", True),
        ("平均成功时间(秒)", "平均成功时间(秒)", False),
        ("平均成功帧数", "平均成功帧数", False),
        ("平均结束距离", "平均结束距离(像素)", False),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756"]

    for idx, (field, ylabel, as_percent) in enumerate(metrics):
        ax = axes[idx]
        values = []

        for algo in algorithms:
            value = _to_float(data.get(algo, {}).get(field), np.nan)
            if as_percent and not np.isnan(value):
                value *= 100.0
            values.append(value)

        bars = ax.bar(algorithms, values, color=colors[:len(algorithms)], alpha=0.9)

        for bar, value in zip(bars, values):
            if np.isnan(value):
                continue
            label = f"{value:.2f}%" if as_percent else f"{value:.2f}"
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height(),
                label,
                ha="center",
                va="bottom",
                fontsize=9,
            )

        ax.set_title(field)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", linestyle="--", alpha=0.3)
        ax.tick_params(axis="x", rotation=15)

    fig.suptitle("两算法核心指标对照", fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def _plot_episode_speed(episode_rows, algorithms, out_path, dpi):
    grouped = _group_episode_rows(episode_rows)

    all_eps = sorted({
        int(_to_float(r.get("回合"), 0))
        for r in episode_rows
        if int(_to_float(r.get("回合"), 0)) >= 0
    })

    if not all_eps:
        return

    max_end_frame = max(_to_float(r.get("结束帧数"), 0) for r in episode_rows)
    fail_y = max_end_frame * 1.03 if max_end_frame > 0 else 1.0

    fig, ax = plt.subplots(figsize=(12, 6))

    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2", "#B279A2"]
    markers = ["o", "s", "^", "D", "x", "*"]

    for i, algo in enumerate(algorithms):
        xs_success = []
        ys_success = []
        xs_fail = []
        ys_fail = []

        for ep in all_eps:
            row = grouped.get(algo, {}).get(ep)
            if not row:
                continue

            ok = str(row.get("是否成功", "")).strip() in ["1", "是", "True", "true"]

            if ok:
                xs_success.append(ep)
                ys_success.append(_to_float(row.get("结束帧数"), np.nan))
            else:
                xs_fail.append(ep)
                ys_fail.append(fail_y)

        if xs_success:
            ax.plot(
                xs_success,
                ys_success,
                marker=markers[i % len(markers)],
                color=colors[i % len(colors)],
                linewidth=1.8,
                label=f"{algo}（成功）",
            )

        if xs_fail:
            ax.scatter(
                xs_fail,
                ys_fail,
                marker="x",
                color=colors[i % len(colors)],
                s=60,
                label=f"{algo}（失败）",
            )

    ax.axhline(max_end_frame, color="#999999", linestyle="--", linewidth=1.2, label=f"最大结束帧={int(max_end_frame)}")
    ax.set_xlabel("回合编号")
    ax.set_ylabel("结束帧数（越低越快）")
    ax.set_title("各回合发现速度对照")
    ax.set_xticks(all_eps)
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def _plot_cumulative_success_rate(episode_rows, algorithms, out_path, dpi):
    grouped = _group_episode_rows(episode_rows)

    total_episodes = len({
        int(_to_float(r.get("回合"), 0))
        for r in episode_rows
        if int(_to_float(r.get("回合"), 0)) >= 0
    })

    if total_episodes == 0:
        return

    max_end_frame = int(max(_to_float(r.get("结束帧数"), 0) for r in episode_rows))
    x = np.linspace(0, max_end_frame, num=220)

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = ["#4C78A8", "#F58518", "#54A24B", "#E45756", "#72B7B2", "#B279A2"]

    for i, algo in enumerate(algorithms):
        rows = grouped.get(algo, {})
        success_frames = []

        for _, row in rows.items():
            ok = str(row.get("是否成功", "")).strip() in ["1", "是", "True", "true"]
            if ok:
                success_frames.append(_to_float(row.get("结束帧数"), np.nan))

        success_frames = [v for v in success_frames if not np.isnan(v)]

        y = []
        for t in x:
            cnt = sum(1 for f in success_frames if f <= t)
            y.append(cnt / total_episodes)

        ax.plot(x, y, color=colors[i % len(colors)], linewidth=2.0, label=algo)

    ax.set_xlabel("帧阈值")
    ax.set_ylabel("累计成功率")
    ax.set_title("累计成功率曲线（同阈值下越高越好）")
    ax.set_ylim(0, 1.05)
    ax.grid(linestyle="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def _plot_pairwise_frame_delta(episode_rows, algorithms, out_path, dpi):
    if len(algorithms) < 2:
        return

    grouped = _group_episode_rows(episode_rows)

    # 默认比较同一实验组里的前两个算法
    algo_a = algorithms[0]
    algo_b = algorithms[1]

    eps = sorted(set(grouped.get(algo_a, {}).keys()) & set(grouped.get(algo_b, {}).keys()))

    pair_eps = []
    deltas = []

    for ep in eps:
        ra = grouped[algo_a][ep]
        rb = grouped[algo_b][ep]

        ok_a = str(ra.get("是否成功", "")).strip() in ["1", "是", "True", "true"]
        ok_b = str(rb.get("是否成功", "")).strip() in ["1", "是", "True", "true"]

        if not (ok_a and ok_b):
            continue

        fa = _to_float(ra.get("结束帧数"), np.nan)
        fb = _to_float(rb.get("结束帧数"), np.nan)

        if np.isnan(fa) or np.isnan(fb):
            continue

        pair_eps.append(ep)
        deltas.append(fa - fb)

    fig, ax = plt.subplots(figsize=(10, 5))

    if pair_eps:
        colors = ["#2CA02C" if d > 0 else "#D62728" for d in deltas]
        ax.bar(pair_eps, deltas, color=colors, alpha=0.85)
        ax.axhline(0, color="#444444", linewidth=1.2)
        ax.set_xlabel("回合编号")
        ax.set_ylabel(f"帧差（{algo_a} - {algo_b}）")
        ax.set_title("配对回合速度差（正值表示第二种算法更快）")
        ax.set_xticks(pair_eps)
        ax.grid(axis="y", linestyle="--", alpha=0.3)
    else:
        ax.text(0.5, 0.5, "无可配对成功回合，无法绘制帧差图", ha="center", va="center", fontsize=12)
        ax.set_axis_off()

    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def _write_report(out_dir, result_dir, stats_file, episode_file, generated_files):
    report_file = os.path.join(out_dir, "可视化说明.txt")

    with open(report_file, "w", encoding="utf-8") as f:
        f.write("实验结果可视化说明\n")
        f.write("====================\n\n")
        f.write(f"实验目录: {result_dir}\n")
        f.write(f"统计数据: {stats_file}\n")
        f.write(f"回合数据: {episode_file}\n\n")

        f.write("生成图表:\n")
        for fp in generated_files:
            f.write(f"- {fp}\n")

    return report_file


def _filter_rows_by_group(rows, group_name):
    return [r for r in rows if (r.get("实验组") or "").strip() == group_name]


def _extract_algorithm_names_by_group(summary_rows, group_name):
    names = []
    for row in summary_rows:
        if (row.get("实验组") or "").strip() == group_name:
            algo = (row.get("算法") or "").strip()
            if algo:
                names.append(algo)

    return list(dict.fromkeys(names))


def _plot_relative_to_random(episode_rows, algorithms, out_path, dpi, base_algo):
    grouped = _group_episode_rows(episode_rows)

    if base_algo not in grouped:
        return

    fig, ax = plt.subplots(figsize=(12, 6))

    width = 0.22
    base_eps = sorted(grouped[base_algo].keys())
    compare_algos = [a for a in algorithms if a != base_algo]

    if not compare_algos:
        return

    for idx, algo in enumerate(compare_algos):
        xs = []
        deltas = []

        for ep in base_eps:
            if ep not in grouped.get(algo, {}):
                continue

            r_base = grouped[base_algo][ep]
            r_algo = grouped[algo][ep]

            ok_base = str(r_base.get("是否成功", "")).strip() in ["1", "是", "True", "true"]
            ok_algo = str(r_algo.get("是否成功", "")).strip() in ["1", "是", "True", "true"]

            if not (ok_base and ok_algo):
                continue

            base_frame = _to_float(r_base.get("结束帧数"), np.nan)
            algo_frame = _to_float(r_algo.get("结束帧数"), np.nan)

            if np.isnan(base_frame) or np.isnan(algo_frame):
                continue

            xs.append(ep + (idx - len(compare_algos) / 2) * width)
            deltas.append(base_frame - algo_frame)

        if xs:
            ax.bar(xs, deltas, width=width, label=f"{base_algo} - {algo}", alpha=0.85)

    ax.axhline(0, color="#444444", linewidth=1.2)
    ax.set_xlabel("回合编号")
    ax.set_ylabel("帧差")
    ax.set_title(f"相对{base_algo}的配对回合帧差（正值表示对比算法更快）")
    ax.grid(axis="y", linestyle="--", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def _plot_group(summary_rows, episode_rows, group_name, prefix, out_dir, dpi, base_algo):
    group_summary = _filter_rows_by_group(summary_rows, group_name)
    group_episode = _filter_rows_by_group(episode_rows, group_name)
    algorithms = _extract_algorithm_names_by_group(summary_rows, group_name)

    generated = []

    if not algorithms:
        return generated

    p1 = os.path.join(out_dir, f"{prefix}_图1_核心指标对照.png")

    if prefix == "Vision":
        _plot_vision_core_metrics(group_summary, algorithms, p1, dpi)
    else:
        _plot_core_metrics(group_summary, algorithms, p1, dpi)

    generated.append(p1)

    p2 = os.path.join(out_dir, f"{prefix}_图2_回合发现速度对照.png")
    _plot_episode_speed(group_episode, algorithms, p2, dpi)
    generated.append(p2)

    p3 = os.path.join(out_dir, f"{prefix}_图3_累计成功率曲线.png")
    _plot_cumulative_success_rate(group_episode, algorithms, p3, dpi)
    generated.append(p3)

    p4 = os.path.join(out_dir, f"{prefix}_图4_相对随机帧差.png")
    _plot_relative_to_random(group_episode, algorithms, p4, dpi, base_algo)
    generated.append(p4)

    return generated


def plot_algorithm_stats(result_dir, dpi=160):
    _setup_chinese_font()

    stats_file = os.path.join(result_dir, "算法对照统计.csv")
    episode_file = os.path.join(result_dir, "回合级结果.csv")

    if not os.path.isfile(stats_file):
        raise FileNotFoundError(f"缺少文件: {stats_file}")

    if not os.path.isfile(episode_file):
        raise FileNotFoundError(f"缺少文件: {episode_file}")

    summary_rows = _read_csv_dict_rows(stats_file)
    episode_rows = _read_csv_dict_rows(episode_file)

    out_dir = os.path.join(result_dir, "可视化图表")
    os.makedirs(out_dir, exist_ok=True)

    generated = []

    # GridMap 四算法单独画
    generated.extend(
        _plot_group(
            summary_rows,
            episode_rows,
            group_name="GridMap搜索实验",
            prefix="GridMap",
            out_dir=out_dir,
            dpi=dpi,
            base_algo="random",
        )
    )

    # 视觉实验单独画
    generated.extend(
        _plot_group(
            summary_rows,
            episode_rows,
            group_name="视觉搜索实验",
            prefix="Vision",
            out_dir=out_dir,
            dpi=dpi,
            base_algo="随机搜索",
        )
    )

    report = _write_report(out_dir, result_dir, stats_file, episode_file, generated)

    print(f"图表已保存到: {out_dir}")
    print(f"说明文件: {report}")