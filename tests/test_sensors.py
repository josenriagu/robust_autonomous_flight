import numpy as np
import pytest

from robust_flight.sensors.gps import GPS
from robust_flight.sensors.imu import IMU

DT = 0.01
STATE = np.array([10.0, -5.0, 12.0, np.pi / 6])
TRUE_VELOCITY = np.array([12.0 * np.cos(np.pi / 6), 12.0 * np.sin(np.pi / 6)])


def sample_gps(gps, steps=6000):
    """Feed the GPS 60 s of steps, accumulating time as Aircraft does."""
    time = 0.0
    samples = []
    for _ in range(steps):
        time += DT
        measurement = gps.measure(time, STATE)
        if measurement is not None:
            samples.append((time, measurement))
    return samples


def test_gps_samples_at_its_rate_despite_rounding_in_the_time():
    samples = sample_gps(GPS(np.random.default_rng(0)))
    times = np.array([time for time, _ in samples])

    assert len(samples) == 300
    assert np.allclose(np.diff(times), 0.2)
    assert times[0] == pytest.approx(0.2)


def test_gps_without_noise_measures_the_true_position_and_velocity():
    gps = GPS(np.random.default_rng(0), position_noise=0.0, velocity_noise=0.0)
    for _, measurement in sample_gps(gps):
        assert np.allclose(measurement[:2], STATE[:2])
        assert np.allclose(measurement[2:], TRUE_VELOCITY)


def test_gps_noise_and_bias_have_the_configured_size():
    gps = GPS(np.random.default_rng(1), position_bias=(3.0, -2.0))
    measurements = np.array([m for _, m in sample_gps(gps, steps=60000)])
    position_errors = measurements[:, :2] - STATE[:2]

    assert np.allclose(position_errors.mean(axis=0), [3.0, -2.0], atol=0.1)
    assert np.allclose(position_errors.std(axis=0), 1.0, atol=0.05)
    assert np.allclose(
        (measurements[:, 2:] - TRUE_VELOCITY).std(axis=0), 0.1, atol=0.01
    )


def test_gps_dropout_loses_samples_but_leaves_the_kept_ones_unchanged():
    full = dict(sample_gps(GPS(np.random.default_rng(2))))
    kept = sample_gps(GPS(np.random.default_rng(2), dropout_probability=0.5))

    assert 100 < len(kept) < 200
    assert all(np.array_equal(full[time], measurement) for time, measurement in kept)


def test_imu_without_noise_measures_the_applied_control():
    imu = IMU(np.random.default_rng(0), acceleration_noise=0.0, yaw_rate_noise=0.0)
    assert np.array_equal(imu.measure([1.5, -0.3]), [1.5, -0.3])


def test_imu_bias_shows_as_the_mean_error():
    imu = IMU(np.random.default_rng(3), acceleration_bias=0.05, yaw_rate_bias=-0.002)
    errors = np.array([imu.measure([1.5, -0.3]) for _ in range(20000)]) - [1.5, -0.3]

    assert np.allclose(errors.mean(axis=0), [0.05, -0.002], atol=[0.002, 0.0003])
    assert np.allclose(errors.std(axis=0), [0.05, 0.01], rtol=0.05)


def test_sensors_repeat_exactly_for_the_same_seed():
    first = sample_gps(GPS(np.random.default_rng(7)))
    second = sample_gps(GPS(np.random.default_rng(7)))
    other = sample_gps(GPS(np.random.default_rng(8)))

    assert all(np.array_equal(a[1], b[1]) for a, b in zip(first, second))
    assert not all(np.array_equal(a[1], b[1]) for a, b in zip(first, other))
