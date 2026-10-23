import csv
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from robust_flight.experiments.runner import ExperimentConfig, run_experiment

RESULTS_DIR = Path(__file__).resolve().parents[3] / "results"
SEEDS = range(20)
OUTAGE_START = 8.0

# feedback options compared at every level; the true state is the reference
FEEDBACK = [
    ("Held GPS", {"feedback": "gps"}),
    ("Naive EKF", {"feedback": "ekf", "ekf_delay_compensation": False}),
    ("EKF", {"feedback": "ekf"}),
]


def outage(length):
    return {"link_outages": ((OUTAGE_START, OUTAGE_START + length),)} if length else {}


# each sweep changes one kind of degradation and holds the rest at baseline
SWEEPS = [
    (
        "gps_noise",
        "GPS position noise (m)",
        [0.5, 1.0, 2.0, 4.0, 8.0],
        lambda level: {"gps_position_noise": level},
    ),
    (
        "gps_dropout",
        "GPS dropout probability",
        [0.0, 0.25, 0.5, 0.75, 0.9],
        lambda level: {"gps_dropout_probability": level},
    ),
    (
        "link_delay",
        "Link delay (s)",
        [0.0, 0.25, 0.5, 1.0, 2.0],
        lambda level: {"link_delay": level},
    ),
    (
        "link_jitter",
        "Link jitter (s) on a 0.1 s delay",
        [0.0, 0.2, 0.4, 0.8],
        lambda level: {"link_delay": 0.1, "link_jitter": level},
    ),
    (
        "packet_loss",
        "Packet loss probability",
        [0.0, 0.25, 0.5, 0.75, 0.9],
        lambda level: {"link_loss_probability": level},
    ),
    (
        "outage",
        f"Outage length (s) from {OUTAGE_START:g} s",
        [0.0, 2.0, 5.0, 10.0, 20.0],
        outage,
    ),
    (
        "imu_bias",
        "IMU acceleration bias (m/s²)",
        [0.0, 0.05, 0.1, 0.2],
        lambda level: {"imu_acceleration_bias": level},
    ),
]

METRICS = [
    "mission_complete",
    "final_time",
    "cross_track_rmse",
    "max_cross_track_error",
    "feedback_position_rmse",
    "feedback_heading_rmse",
    "ekf_position_rmse",
    "ekf_nees",
    "gps_lost_count",
    "gps_discarded_count",
]


def run_summary(config):
    # runs in a worker process, so only the scalar metrics travel back
    result = run_experiment(config)
    return {name: getattr(result, name) for name in METRICS}


def build_runs(baseline):
    runs = []
    for sweep, _, levels, settings in SWEEPS:
        for level in levels:
            for feedback, feedback_settings in FEEDBACK:
                for seed in SEEDS:
                    config = replace(
                        baseline, seed=seed, **settings(level), **feedback_settings
                    )
                    runs.append(
                        {
                            "sweep": sweep,
                            "level": level,
                            "feedback": feedback,
                            "seed": seed,
                            "config": config,
                        }
                    )
    return runs


def save_rows(rows, path):
    path.parent.mkdir(exist_ok=True)
    with open(path, "w", newline="") as file:
        writer = csv.DictWriter(
            file, fieldnames=["sweep", "level", "feedback", "seed", *METRICS]
        )
        writer.writeheader()
        writer.writerows(rows)


def print_sweep(title, rows, reference):
    print(f"\n{title}")
    print(
        f"{'Level':>7} {'Feedback':>10} {'Done':>6} {'Time (s)':>16} "
        f"{'XTE RMSE (m)':>16} {'Position err. (m)':>18} {'NEES':>9}"
    )

    for level in dict.fromkeys(row["level"] for row in rows):
        for feedback, _ in FEEDBACK:
            group = [
                r for r in rows if r["level"] == level and r["feedback"] == feedback
            ]
            done = sum(r["mission_complete"] for r in group)

            def stat(name, scale=1.0):
                values = np.array([r[name] for r in group]) * scale
                return f"{np.mean(values):>7.3f} ± {np.std(values):<6.3f}"

            print(
                f"{level:>7g} {feedback:>10} {done:>3}/{len(group):<2} "
                f"{stat('final_time')} {stat('cross_track_rmse')} "
                f"{stat('feedback_position_rmse'):>18} "
                f"{np.mean([r['ekf_nees'] for r in group]):>9.2f}"
            )

    print(
        f"{'':>7} {'True state':>10} {'yes':>6} {reference.final_time:>7.3f} "
        f"{'':>8} {reference.cross_track_rmse:>7.3f}"
    )


def main():
    baseline = ExperimentConfig()
    reference = run_experiment(baseline)
    runs = build_runs(baseline)

    with ProcessPoolExecutor() as pool:
        summaries = list(
            pool.map(run_summary, [run["config"] for run in runs], chunksize=8)
        )

    rows = [
        {key: run[key] for key in ("sweep", "level", "feedback", "seed")} | summary
        for run, summary in zip(runs, summaries)
    ]
    save_rows(rows, RESULTS_DIR / "robustness.csv")

    for sweep, title, _, _ in SWEEPS:
        print_sweep(
            f"{title}, {len(SEEDS)} seeds (mean ± standard deviation)",
            [row for row in rows if row["sweep"] == sweep],
            reference,
        )
    print(f"\n{len(rows)} runs saved to {RESULTS_DIR / 'robustness.csv'}")


if __name__ == "__main__":
    main()
