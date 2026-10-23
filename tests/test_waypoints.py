import numpy as np
import pytest

from robust_flight.guidance.waypoints import Waypoints


@pytest.fixture
def waypoints():
    return Waypoints([[10.0, 0.0]])


def test_cross_track_error_is_positive_left_of_the_path(waypoints):
    assert waypoints.cross_track_error([5.0, 2.0], [0.0, 0.0], [10.0, 0.0]) == 2.0


def test_cross_track_error_is_negative_right_of_the_path(waypoints):
    assert waypoints.cross_track_error([5.0, -3.0], [0.0, 0.0], [10.0, 0.0]) == -3.0


def test_cross_track_error_is_zero_on_the_path(waypoints):
    assert waypoints.cross_track_error([7.0, 0.0], [0.0, 0.0], [10.0, 0.0]) == 0.0


def test_cross_track_error_follows_a_diagonal_path(waypoints):
    # (0, 1) is one metre from the line y = x, on its left
    error = waypoints.cross_track_error([0.0, 1.0], [0.0, 0.0], [5.0, 5.0])
    assert error == pytest.approx(1.0 / np.sqrt(2.0))


def test_heading_error_turns_the_short_way_round(waypoints):
    # from 170° to -170° is a 20° left turn, not a 340° right turn
    error = waypoints.heading_error(np.radians(170.0), np.radians(-170.0))
    assert error == pytest.approx(np.radians(20.0))


def test_heading_error_stays_within_plus_or_minus_pi(waypoints):
    for current in np.linspace(-10.0, 10.0, 41):
        for desired in np.linspace(-10.0, 10.0, 41):
            assert -np.pi <= waypoints.heading_error(current, desired) < np.pi


def test_desired_speed_brakes_towards_the_waypoint(waypoints):
    # braking speed sqrt(2 * 1.5 * d), capped at the maximum speed
    assert waypoints.desired_speed([0.0, 0.0], [12.0, 0.0], 20.0, 1.5) == 6.0
    assert waypoints.desired_speed([0.0, 0.0], [500.0, 0.0], 20.0, 1.5) == 20.0
    assert waypoints.desired_speed([3.0, 4.0], [3.0, 4.0], 20.0, 1.5) == 0.0
