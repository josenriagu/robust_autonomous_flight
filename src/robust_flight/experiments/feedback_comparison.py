from dataclasses import replace

import matplotlib.pyplot as plt
import numpy as np

from robust_flight.experiments.runner import ExperimentConfig, run_experiment

FEEDBACK_MODES = [
    ("true", "True state"),
    ("gps", "Held GPS"),
    ("ekf", "EKF"),
]


def run_seeds(config, seeds):
    return [run_experiment(replace(config, seed=seed)) for seed in seeds]


def summarize(values):
    return f"{np.mean(values):>7.3f} ± {np.std(values):<6.3f}"


def print_comparison(title, cases):
    print(f"\n{title}")
    print(
        f"{'Feedback':>11} {'Done':>6} {'Time (s)':>16} {'XTE RMSE (m)':>16} "
        f"{'Max XTE (m)':>16} {'Position err. (m)':>18} {'Heading err. (°)':>17}"
    )

    for label, results in cases:
        done = sum(result.mission_complete for result in results)
        print(
            f"{label:>11} {done:>3}/{len(results):<2} "
            f"{summarize([r.final_time for r in results])} "
            f"{summarize([r.cross_track_rmse for r in results])} "
            f"{summarize([r.max_cross_track_error for r in results])} "
            f"{summarize([r.feedback_position_rmse for r in results]):>18} "
            f"{summarize([np.degrees(r.feedback_heading_rmse) for r in results]):>17}"
        )


def plot_trajectories(title, cases, waypoints):
    plt.figure()

    for label, result in cases:
        plt.plot(
            result.history[:, 0],
            result.history[:, 1],
            label=f"{label} ({'done' if result.mission_complete else 'not done'})",
        )

    points = np.array(waypoints)
    plt.plot(points[:, 0], points[:, 1], "kx", markersize=10, label="Waypoints")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(title)
    plt.axis("equal")
    plt.legend()
    plt.grid()

    plt.show()


def main():
    baseline = ExperimentConfig()
    seeds = range(20)

    cases = [
        (label, run_seeds(replace(baseline, feedback=mode), seeds))
        for mode, label in FEEDBACK_MODES
    ]

    print_comparison(
        f"Feedback comparison over {len(seeds)} seeds (mean ± standard deviation)",
        cases,
    )

    # the seed where steering on held GPS took longest shows the difference best
    gps_results = cases[1][1]
    slowest = max(range(len(gps_results)), key=lambda i: gps_results[i].final_time)
    plot_trajectories(
        f"Trajectories by Feedback Source (seed {slowest})",
        [(label, results[slowest]) for label, results in cases],
        baseline.waypoints,
    )


if __name__ == "__main__":
    main()
