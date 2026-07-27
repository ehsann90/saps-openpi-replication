#!/usr/bin/env python3
"""
Analyze the completed autonomous LIBERO displacement experiment.

Usage:
    LOCAL_UID="$(id -u)" \
    LOCAL_GID="$(id -g)" \
    docker compose \
        -f compose.yml \
        run --rm --no-deps \
        runtime \
        /bin/bash -lc \
        'source /.venv/bin/activate &&
        python /workspace/scripts/analyze_autonomous_results.py \
            /workspace/outputs/autonomous_n20_state0/sweep_summary.json \
            --output-dir /workspace/results/autonomous_n20_state0 &&
        chown -R "$LOCAL_UID:$LOCAL_GID" \
            /workspace/results'
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats


def wilson_interval(
    successes: int,
    trials: int,
    confidence: float = 0.95,
) -> tuple[float, float]:
    if trials <= 0:
        raise ValueError("trials must be positive")

    z = stats.norm.ppf(0.5 + confidence / 2.0)
    proportion = successes / trials
    denominator = 1.0 + z**2 / trials

    center = (
        proportion + z**2 / (2.0 * trials)
    ) / denominator

    half_width = (
        z
        * math.sqrt(
            proportion * (1.0 - proportion) / trials
            + z**2 / (4.0 * trials**2)
        )
        / denominator
    )

    return (
        max(0.0, center - half_width),
        min(1.0, center + half_width),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "summary",
        type=Path,
        help="Path to sweep_summary.json",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
    )
    args = parser.parse_args()

    data = json.loads(
        args.summary.read_text(encoding="utf-8")
    )

    if not data.get("complete", False):
        raise RuntimeError(
            "The experiment summary is not marked complete."
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = []

    for condition in data["conditions"]:
        successes = int(condition["successes"])
        trials = int(condition["completed_episodes"])
        rate = successes / trials
        ci_low, ci_high = wilson_interval(
            successes,
            trials,
        )

        rows.append(
            {
                "condition_id": condition["condition_id"],
                "delta_x": float(condition["delta_x"]),
                "delta_y": float(condition["delta_y"]),
                "distance": float(
                    condition["offset_distance"]
                ),
                "trials": trials,
                "successes": successes,
                "timeouts": int(condition["timeouts"]),
                "success_rate": rate,
                "success_ci_low": ci_low,
                "success_ci_high": ci_high,
                "mean_successful_completion_seconds": condition[
                    "mean_successful_completion_seconds"
                ],
                "std_successful_completion_seconds": condition[
                    "std_successful_completion_seconds"
                ],
            }
        )

    csv_path = args.output_dir / "autonomous_results.csv"

    with csv_path.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(rows[0].keys()),
        )
        writer.writeheader()
        writer.writerows(rows)

    distances = np.asarray(
        [row["distance"] for row in rows],
        dtype=np.float64,
    )
    rates = np.asarray(
        [row["success_rate"] for row in rows],
        dtype=np.float64,
    )

    all_correlation = stats.pearsonr(
        distances,
        rates,
    )

    perturbed_rows = [
        row
        for row in rows
        if row["condition_id"] != "nominal"
    ]

    perturbed_distances = np.asarray(
        [row["distance"] for row in perturbed_rows],
        dtype=np.float64,
    )
    perturbed_rates = np.asarray(
        [row["success_rate"] for row in perturbed_rows],
        dtype=np.float64,
    )

    perturbed_correlation = stats.pearsonr(
        perturbed_distances,
        perturbed_rates,
    )

    high_distance_rows = [
        row
        for row in rows
        if row["distance"] >= 0.15
    ]

    high_distance_successes = sum(
        row["successes"]
        for row in high_distance_rows
    )
    high_distance_trials = sum(
        row["trials"]
        for row in high_distance_rows
    )
    high_distance_rate = (
        high_distance_successes
        / high_distance_trials
    )

    statistics = {
        "source_summary": str(args.summary),
        "conditions": len(rows),
        "episodes": sum(row["trials"] for row in rows),
        "pearson_all_conditions": {
            "r": float(all_correlation.statistic),
            "two_sided_p": float(all_correlation.pvalue),
        },
        "pearson_perturbed_conditions": {
            "r": float(
                perturbed_correlation.statistic
            ),
            "two_sided_p": float(
                perturbed_correlation.pvalue
            ),
        },
        "distance_ge_0_15": {
            "conditions": len(high_distance_rows),
            "successes": high_distance_successes,
            "episodes": high_distance_trials,
            "success_rate": high_distance_rate,
        },
        "paper_reported_values": {
            "pearson_r": -0.800,
            "two_sided_p": 0.005,
            "distance_ge_0_15_success_rate": 0.229,
        },
    }

    statistics_path = (
        args.output_dir / "autonomous_statistics.json"
    )
    statistics_path.write_text(
        json.dumps(statistics, indent=2),
        encoding="utf-8",
    )

    ci_low = np.asarray(
        [row["success_ci_low"] for row in rows]
    )
    ci_high = np.asarray(
        [row["success_ci_high"] for row in rows]
    )

    lower_errors = rates - ci_low
    upper_errors = ci_high - rates

    figure, axis = plt.subplots(figsize=(8, 5))

    axis.errorbar(
        distances,
        rates * 100.0,
        yerr=np.vstack(
            (
                lower_errors * 100.0,
                upper_errors * 100.0,
            )
        ),
        fmt="o",
        capsize=4,
        label="Replication: autonomous π0.5",
    )

    coefficients = np.polyfit(
        perturbed_distances,
        perturbed_rates * 100.0,
        deg=1,
    )
    trend_x = np.linspace(
        perturbed_distances.min(),
        perturbed_distances.max(),
        200,
    )
    trend_y = np.polyval(coefficients, trend_x)

    axis.plot(
        trend_x,
        trend_y,
        linestyle="--",
        label="Perturbed-condition trend",
    )

    for row in rows:
        axis.annotate(
            row["condition_id"],
            (
                row["distance"],
                row["success_rate"] * 100.0,
            ),
            xytext=(4, 5),
            textcoords="offset points",
            fontsize=8,
        )

    axis.set_xlabel("Cream-cheese displacement distance (m)")
    axis.set_ylabel("Task success rate (%)")
    axis.set_ylim(-5, 110)
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()

    plot_path = (
        args.output_dir
        / "autonomous_success_vs_distance.png"
    )
    figure.savefig(plot_path, dpi=200)
    plt.close(figure)

    print(f"CSV: {csv_path}")
    print(f"Statistics: {statistics_path}")
    print(f"Plot: {plot_path}")
    print()
    print(
        "Perturbed-condition Pearson correlation: "
        f"r={perturbed_correlation.statistic:.3f}, "
        f"p={perturbed_correlation.pvalue:.5f}"
    )
    print(
        "Success for distance >= 0.15 m: "
        f"{high_distance_successes}/{high_distance_trials} "
        f"({100.0 * high_distance_rate:.1f}%)"
    )


if __name__ == "__main__":
    main()
