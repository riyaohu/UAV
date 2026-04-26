import csv


def compute_metrics(csv_file):
    results = []

    with open(csv_file, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # ✅ 跳过空行和SUMMARY行
            if row["run_id"] == "" or row["run_id"] == "SUMMARY":
                continue
            results.append(row)

    total_runs = len(results)

    success_count = 0
    total_frames = 0.0
    total_distance = 0.0

    for r in results:
        # ✅ 成功统计
        if r["stop_reason"] == "success":
            success_count += 1

        # ✅ 防止异常数据（很关键）
        try:
            total_frames += float(r["frames"])
            total_distance += float(r["distance"])
        except (ValueError, TypeError):
            continue

    # ✅ 防止除0（更稳）
    if total_runs == 0:
        return {
            "success_rate": 0,
            "avg_frames": 0,
            "avg_distance": 0,
            "num_runs": 0,
        }

    metrics = {
        "success_rate": success_count / total_runs,
        "avg_frames": total_frames / total_runs,
        "avg_distance": total_distance / total_runs,
        "num_runs": total_runs,
    }

    return metrics