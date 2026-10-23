import csv

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FormatStrFormatter, LogLocator, NullFormatter

from robust_flight.experiments.robustness import FEEDBACK, RESULTS_DIR, SWEEPS
from robust_flight.experiments.runner import ExperimentConfig, run_experiment

# a panel uses a log scale only when its values span more than this ratio;
# narrower ranges read better on a linear scale with evenly spaced labels
LOG_RANGE = 5.0

MARKERS = {"Held GPS": "o", "Naive EKF": "s", "EKF": "^", "Bias EKF": "D"}


def load_rows(path):
    with open(path, newline="") as file:
        rows = list(csv.DictReader(file))
    for row in rows:
        row["level"] = float(row["level"])
        row["mission_complete"] = row["mission_complete"] == "True"
        for name in row:
            if name not in ("sweep", "level", "feedback", "mission_complete"):
                row[name] = float(row[name])
    return rows


def plot_metric(rows, metric, ylabel, title, reference=None, log=False, scale=1.0):
    # one panel per sweep; levels are evenly spaced so uneven steps stay legible
    figure, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()

    for axis, (sweep, label, levels, _) in zip(axes, SWEEPS):
        positions = np.arange(len(levels))
        panel_means = []

        for feedback, _ in FEEDBACK:
            means, spreads = [], []
            for level in levels:
                values = [
                    row[metric] * scale
                    for row in rows
                    if row["sweep"] == sweep
                    and row["level"] == level
                    and row["feedback"] == feedback
                ]
                means.append(np.mean(values))
                spreads.append(np.std(values))
            panel_means.extend(means)
            # a spread is meaningless for a completion rate and would run
            # past 0 and 100%
            axis.errorbar(
                positions,
                means,
                yerr=None if metric == "mission_complete" else spreads,
                marker=MARKERS[feedback],
                capsize=3,
                label=feedback,
            )

        if reference is not None:
            axis.axhline(reference, color="black", linestyle="--", label="True state")
        axis.set_xticks(positions, [f"{level:g}" for level in levels])
        axis.set_xlabel(label)
        axis.set_ylabel(ylabel)
        if log and max(panel_means) > LOG_RANGE * min(panel_means):
            axis.set_yscale("log")
            # label 1, 2 and 5 in each decade as plain numbers, nothing between
            axis.yaxis.set_major_locator(LogLocator(base=10, subs=(1.0, 2.0, 5.0)))
            axis.yaxis.set_major_formatter(FormatStrFormatter("%g"))
            axis.yaxis.set_minor_formatter(NullFormatter())
            # start the axis at a labelled tick, so a number sits below the
            # lowest line
            bottom = axis.get_ylim()[0]
            decade = 10 ** np.floor(np.log10(bottom))
            tick = max(t * decade for t in (1.0, 2.0, 5.0) if t * decade <= bottom)
            axis.set_ylim(bottom=tick)
        if metric == "mission_complete":
            axis.set_ylim(-5, 105)
        axis.grid(True, which="both", alpha=0.4)

    # the eighth panel carries the legend
    handles, labels = axes[0].get_legend_handles_labels()
    axes[-1].axis("off")
    axes[-1].legend(handles, labels, loc="center")
    axes[-1].text(
        0.5,
        0.12,
        "Naive EKF matches EKF wherever\n"
        "there is no link delay, so its\n"
        "line is hidden underneath.",
        ha="center",
        transform=axes[-1].transAxes,
    )

    figure.suptitle(title)
    figure.tight_layout()
    return figure


def main():
    rows = load_rows(RESULTS_DIR / "robustness.csv")
    reference = run_experiment(ExperimentConfig())

    figures = {
        "robustness_completion.png": plot_metric(
            rows,
            "mission_complete",
            "Missions completed (%)",
            "Mission Completion Under Degradation (20 seeds per point)",
            scale=100.0,
        ),
        "robustness_tracking.png": plot_metric(
            rows,
            "cross_track_rmse",
            "Cross-track RMSE (m)",
            "Path Tracking Under Degradation (mean ± standard deviation)",
            reference=reference.cross_track_rmse,
            log=True,
        ),
        "robustness_estimation.png": plot_metric(
            rows,
            "feedback_position_rmse",
            "Position error seen by controller (m)",
            "Navigation Error Under Degradation (mean ± standard deviation)",
            log=True,
        ),
    }

    for name, figure in figures.items():
        figure.savefig(RESULTS_DIR / name, dpi=120)

    plt.show()


if __name__ == "__main__":
    main()
