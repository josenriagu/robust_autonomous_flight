import csv
from dataclasses import replace

import numpy as np
import pytest

from robust_flight.experiments.robustness import (
    FEEDBACK,
    METRICS,
    RESULTS_DIR,
    SWEEPS,
    run_summary,
)
from robust_flight.experiments.runner import ExperimentConfig, run_experiment

BASELINE = ExperimentConfig()


def test_baseline_flight_is_unchanged():
    # pinned so that any unintended change to the flight fails loudly
    result = run_experiment(BASELINE)

    assert result.mission_complete
    assert result.final_time == pytest.approx(25.84)
    assert result.cross_track_rmse == pytest.approx(2.454, abs=5e-4)
    assert result.max_overspeed == pytest.approx(2.499, abs=5e-4)


def test_unknown_feedback_is_rejected():
    with pytest.raises(ValueError, match="unknown feedback"):
        run_experiment(replace(BASELINE, feedback="estimate"))


def test_the_same_seed_repeats_exactly():
    config = replace(BASELINE, feedback="ekf", seed=4)

    assert np.array_equal(
        run_experiment(config).history, run_experiment(config).history
    )


def test_different_seeds_give_different_flights_on_measured_feedback():
    first = run_experiment(replace(BASELINE, feedback="gps", seed=0))
    second = run_experiment(replace(BASELINE, feedback="gps", seed=1))

    assert not np.array_equal(first.history, second.history)


def test_steering_on_the_true_state_ignores_the_sensors():
    first = run_experiment(BASELINE)
    noisy = run_experiment(replace(BASELINE, seed=9, gps_position_noise=8.0))

    assert np.array_equal(first.history, noisy.history)


# rows of the committed results, chosen to cover every sweep and filter
PUBLISHED_ROWS = [
    ("gps_noise", 4.0, "Held GPS", 2),
    ("gps_dropout", 0.75, "EKF", 5),
    ("link_delay", 1.0, "Naive EKF", 3),
    ("link_jitter", 0.8, "EKF", 0),
    ("packet_loss", 0.5, "Bias EKF", 13),
    ("outage", 10.0, "Held GPS", 7),
    ("imu_bias", 0.2, "Bias EKF", 11),
]


@pytest.fixture(scope="module")
def published():
    with open(RESULTS_DIR / "robustness.csv", newline="") as file:
        return {
            (row["sweep"], float(row["level"]), row["feedback"], int(row["seed"])): row
            for row in csv.DictReader(file)
        }


@pytest.mark.parametrize("sweep, level, feedback, seed", PUBLISHED_ROWS)
def test_published_results_reproduce_exactly(published, sweep, level, feedback, seed):
    settings = {name: change for name, _, _, change in SWEEPS}[sweep](level)
    config = replace(BASELINE, seed=seed, **settings, **dict(FEEDBACK)[feedback])
    saved = published[(sweep, level, feedback, seed)]

    summary = run_summary(config)

    for name in [*METRICS, "recovery_time"]:
        assert str(summary[name]) == saved[name], name
