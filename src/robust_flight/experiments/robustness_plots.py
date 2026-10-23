import csv

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LogNorm, Normalize
from matplotlib.lines import Line2D
from matplotlib.ticker import FormatStrFormatter, LogLocator, NullFormatter

from robust_flight.experiments.robustness import (
    FEEDBACK,
    GRID_DELAY,
    GRID_NOISE,
    RESULTS_DIR,
    SWEEPS,
)
from robust_flight.experiments.runner import ExperimentConfig, run_experiment

# a panel uses a log scale only when its values span more than this ratio;
# narrower ranges read better on a linear scale with evenly spaced labels
LOG_RANGE = 5.0

# a mission is truly complete when every waypoint was actually passed within
# twice the arrival radius, however close the aircraft believed it was
TRUE_MISS_TOLERANCE = 2.0 * ExperimentConfig().arrival_radius

MARKERS = {"Held GPS": "o", "Naive EKF": "s", "EKF": "^", "Bias EKF": "D"}
PRESET_NAMES = {1.0: "Mild", 2.0: "Moderate", 3.0: "Severe"}


def load_rows(path):
    with open(path, newline="") as file:
        rows = list(csv.DictReader(file))
    for row in rows:
        row["level"] = float(row["level"])
        row["mission_complete"] = row["mission_complete"] == "True"
        for name in row:
            if name not in ("sweep", "level", "feedback", "mission_complete"):
                row[name] = float(row[name])
        row["truly_complete"] = (
            row["mission_complete"] and row["max_capture_miss"] <= TRUE_MISS_TOLERANCE
        )
    return rows


def select(rows, sweep, level, feedback):
    return [
        row
        for row in rows
        if row["sweep"] == sweep
        and row["level"] == level
        and row["feedback"] == feedback
    ]


def plot_metric(
    rows,
    metric,
    ylabel,
    title,
    reference=None,
    log=False,
    scale=1.0,
    believed_metric=None,
):
    # one panel per sweep; levels are evenly spaced so uneven steps stay legible
    columns = 4
    panels = len(SWEEPS) + 1
    figure, axes = plt.subplots(
        -(-panels // columns), columns, figsize=(16, 4 * -(-panels // columns))
    )
    axes = axes.flatten()
    rate = metric in ("mission_complete", "truly_complete")

    for axis, (sweep, label, levels, _) in zip(axes, SWEEPS):
        positions = np.arange(len(levels))
        panel_means = []

        for feedback, _ in FEEDBACK:
            groups = [select(rows, sweep, level, feedback) for level in levels]
            means = [np.mean([row[metric] * scale for row in g]) for g in groups]
            spreads = [np.std([row[metric] * scale for row in g]) for g in groups]
            panel_means.extend(means)
            # a spread is meaningless for a completion rate and would run
            # past 0 and 100%
            plotted = axis.errorbar(
                positions,
                means,
                yerr=None if rate else spreads,
                marker=MARKERS[feedback],
                capsize=3,
                label=feedback,
            )
            if believed_metric is not None:
                believed = [
                    np.mean([row[believed_metric] * scale for row in g]) for g in groups
                ]
                axis.plot(
                    positions,
                    believed,
                    linestyle="--",
                    color=plotted.lines[0].get_color(),
                    alpha=0.6,
                )

        # the reference flies the baseline gains, so it does not apply to the
        # panel that changes the controller gain
        if reference is not None and sweep != "speed_gain":
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
        if rate:
            axis.set_ylim(-5, 105)
        axis.grid(True, which="both", alpha=0.4)

    # the remaining panels are empty except the last, which carries the legend
    for axis in axes[len(SWEEPS) :]:
        axis.axis("off")
    handles, labels = axes[0].get_legend_handles_labels()
    if believed_metric is not None:
        handles += [
            Line2D([], [], color="grey", label="Truly complete (solid)"),
            Line2D([], [], color="grey", linestyle="--", label="Believed complete"),
        ]
        labels += ["Truly complete (solid)", "Believed complete (dashed)"]
    axes[-1].legend(handles, labels, loc="center")
    axes[-1].text(
        0.5,
        0.05,
        "Naive EKF matches EKF wherever\n"
        "there is no link delay, so its\n"
        "line is hidden underneath.",
        ha="center",
        transform=axes[-1].transAxes,
    )

    figure.suptitle(title)
    figure.tight_layout()
    return figure


def grid_values(rows, metric, scale, feedback):
    return np.array(
        [
            [
                np.mean(
                    [
                        row[metric] * scale
                        for row in select(
                            rows, f"delay_at_noise_{noise:g}", delay, feedback
                        )
                    ]
                )
                for delay in GRID_DELAY
            ]
            for noise in GRID_NOISE
        ]
    )


def plot_grid(rows):
    # sensor noise crossed with link delay, one heat map per navigation option;
    # each row of maps shares one colour scale so the options compare fairly
    figure, axes = plt.subplots(2, len(FEEDBACK), figsize=(16, 8), layout="constrained")
    maps = [
        ("truly_complete", 100.0, "Truly complete (%)", "Greens", "{:.0f}"),
        ("feedback_position_rmse", 1.0, "Position error seen (m)", "Oranges", "{:.2g}"),
    ]

    for row_axes, (metric, scale, name, colours, label) in zip(axes, maps):
        values = {
            feedback: grid_values(rows, metric, scale, feedback)
            for feedback, _ in FEEDBACK
        }
        every = np.concatenate([v.ravel() for v in values.values()])
        norm = (
            Normalize(0.0, 100.0)
            if metric == "truly_complete"
            else LogNorm(every.min(), every.max())
        )
        for axis, (feedback, _) in zip(row_axes, FEEDBACK):
            image = axis.imshow(
                values[feedback], cmap=colours, origin="lower", norm=norm
            )
            for i in range(len(GRID_NOISE)):
                for j in range(len(GRID_DELAY)):
                    value = values[feedback][i, j]
                    axis.text(
                        j,
                        i,
                        label.format(value),
                        ha="center",
                        va="center",
                        color="white" if norm(value) > 0.6 else "black",
                    )
            axis.set_xticks(range(len(GRID_DELAY)), [f"{d:g}" for d in GRID_DELAY])
            axis.set_yticks(range(len(GRID_NOISE)), [f"{n:g}" for n in GRID_NOISE])
            axis.set_xlabel("Link delay (s)")
            axis.set_ylabel("GPS noise (m)")
            if metric == "truly_complete":
                axis.set_title(feedback)
        figure.colorbar(image, ax=list(row_axes), shrink=0.8, label=name)

    figure.suptitle("GPS Noise Combined with Link Delay (20 seeds per cell)")
    return figure


def plot_presets(rows):
    # three bad-day presets, every navigation option side by side
    figure, axes = plt.subplots(1, 3, figsize=(16, 5))
    levels = sorted(PRESET_NAMES)
    width = 0.8 / len(FEEDBACK)
    metrics = [
        ("truly_complete", 100.0, "Truly complete (%)", False),
        ("max_capture_miss", 1.0, "Worst true miss at capture (m)", True),
        ("cross_track_rmse", 1.0, "Cross-track RMSE (m)", False),
    ]

    for axis, (metric, scale, label, completed_only) in zip(axes, metrics):
        for index, (feedback, _) in enumerate(FEEDBACK):
            values = []
            for level in levels:
                group = select(rows, "preset", level, feedback)
                if completed_only:
                    group = [row for row in group if row["mission_complete"]]
                values.append(
                    np.mean([row[metric] * scale for row in group]) if group else np.nan
                )
            positions = np.arange(len(levels)) + (index - 1.5) * width
            axis.bar(positions, values, width, label=feedback)
            # label every bar, including those at zero and the ones with no
            # completed flight to measure
            for position, value in zip(positions, values):
                axis.text(
                    position,
                    0 if np.isnan(value) else value,
                    "none\ndone"
                    if np.isnan(value)
                    else f"{value:.0f}"
                    if metric == "truly_complete"
                    else f"{value:.1f}",
                    ha="center",
                    va="bottom",
                    fontsize=8,
                )
        if metric == "max_capture_miss":
            axis.axhline(
                TRUE_MISS_TOLERANCE,
                color="black",
                linestyle="--",
                label=f"{TRUE_MISS_TOLERANCE:g} m tolerance",
            )
        axis.set_xticks(range(len(levels)), [PRESET_NAMES[level] for level in levels])
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.4)

    axes[1].legend()
    figure.suptitle("Bad-Day Presets: Several Degradations at Once (20 seeds per bar)")
    figure.tight_layout()
    return figure


def main():
    rows = load_rows(RESULTS_DIR / "robustness.csv")
    reference = run_experiment(ExperimentConfig())

    figures = {
        "robustness_completion.png": plot_metric(
            rows,
            "truly_complete",
            "Missions completed (%)",
            "Mission Completion Under Degradation (20 seeds per point)",
            scale=100.0,
            believed_metric="mission_complete",
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
        "robustness_combined.png": plot_grid(rows),
        "robustness_presets.png": plot_presets(rows),
    }

    for name, figure in figures.items():
        figure.savefig(RESULTS_DIR / name, dpi=120)

    plt.show()


if __name__ == "__main__":
    main()
