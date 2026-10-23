from dataclasses import replace

import numpy as np
import pytest

from robust_flight.experiments.runner import ExperimentConfig, run_experiment

BASELINE = ExperimentConfig()
PERFECT_SENSORS = dict(
    gps_position_noise=0.0,
    gps_velocity_noise=0.0,
    imu_acceleration_noise=0.0,
    imu_yaw_rate_noise=0.0,
)


@pytest.mark.parametrize(
    "settings",
    [
        {},
        {"ekf_estimate_imu_bias": True},
        {"link_delay": 0.5, "link_jitter": 0.3},
        {"link_delay": 0.5, "link_jitter": 0.3, "ekf_estimate_imu_bias": True},
    ],
    ids=["4-state", "bias", "delay-compensated", "bias with delay"],
)
def test_ekf_reproduces_the_truth_exactly_with_perfect_sensors(settings):
    # the filter's model is the aircraft's model, so with nothing to correct
    # any difference from the true path is a bug
    result = run_experiment(replace(BASELINE, **PERFECT_SENSORS, **settings))

    assert np.array_equal(result.ekf_states, result.history)


def test_naive_ekf_does_not_reproduce_the_truth_under_delay():
    result = run_experiment(
        replace(
            BASELINE,
            **PERFECT_SENSORS,
            link_delay=0.5,
            ekf_delay_compensation=False,
        )
    )

    assert np.max(np.abs(result.ekf_states - result.history)) > 1.0


def test_ekf_is_far_more_accurate_than_gps_alone():
    result = run_experiment(BASELINE)

    assert result.ekf_position_rmse < 0.3 * result.gps_position_rmse


def test_ekf_reported_uncertainty_matches_its_real_error():
    # NEES averages 4 for a consistent four-state filter; averaged over five
    # seeds it should land close to that
    nees = [run_experiment(replace(BASELINE, seed=seed)).ekf_nees for seed in range(5)]

    assert 3.0 < np.mean(nees) < 4.5


def test_delay_compensation_removes_the_delay_error():
    delayed = replace(BASELINE, link_delay=0.5)
    naive = run_experiment(replace(delayed, ekf_delay_compensation=False))
    compensated = run_experiment(delayed)

    assert naive.ekf_position_rmse > 2.0
    assert compensated.ekf_position_rmse < 0.3
    assert 2.0 < compensated.ekf_nees < 6.0


def test_bias_ekf_recovers_an_accelerometer_bias():
    result = run_experiment(
        replace(BASELINE, imu_acceleration_bias=0.1, ekf_estimate_imu_bias=True)
    )

    assert result.ekf_bias_estimates[-1, 0] == pytest.approx(0.1, abs=0.01)
    assert result.ekf_position_rmse < 0.3
