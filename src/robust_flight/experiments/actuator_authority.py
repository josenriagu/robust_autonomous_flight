from dataclasses import replace

import numpy as np

from robust_flight.experiments.runner import ExperimentConfig, run_experiment


def print_sweep(title, cases, authority_label):
    print(f"\n{title}")
    print(
        f"{authority_label:>18} {'Speed RMSE':>11} {'Max speed err.':>15} "
        f"{'Final dist.':>12} {'Min dist.':>11} {'Final speed':>12} "
        f"{'Time':>8} {'Accel sat.':>11} {'Turn sat.':>10} "
        f"{'Mean accel int.':>16} {'Max accel int.':>15} "
        f"{'Mean turn int.':>15} {'Max turn int.':>14} "
        f"{'Accel effort':>13} {'Turn effort':>12}"
    )

    for authority, result in cases:
        print(
            f"{authority:>18} "
            f"{result.speed_rmse:>11.3f} "
            f"{result.max_speed_error:>15.3f} "
            f"{result.final_distance:>12.3f} "
            f"{result.minimum_distance:>11.3f} "
            f"{result.final_speed:>12.3f} "
            f"{result.final_time:>8.2f} "
            f"{result.acceleration_saturation_rate * 100:>10.1f}% "
            f"{result.turn_rate_saturation_rate * 100:>9.1f}% "
            f"{result.mean_acceleration_intervention:>16.3f} "
            f"{result.max_acceleration_intervention:>15.3f} "
            f"{result.mean_turn_rate_intervention:>15.3f} "
            f"{result.max_turn_rate_intervention:>14.3f} "
            f"{result.acceleration_effort:>13.3f} "
            f"{result.turn_rate_effort:>12.3f}"
        )


def main():
    baseline = ExperimentConfig()

    acceleration_limits = [
        (-1.0, 1.5),
        (-1.5, 2.0),
        (-2.0, 3.0),
        (-3.0, 4.0),
        (-4.0, 5.0),
    ]
    acceleration_cases = [
        (
            f"({minimum:.1f}, {maximum:.1f})",
            run_experiment(
                replace(
                    baseline,
                    min_acceleration=minimum,
                    max_acceleration=maximum,
                )
            ),
        )
        for minimum, maximum in acceleration_limits
    ]

    max_turn_rates = [
        ("π/12", np.pi / 12),
        ("π/8", np.pi / 8),
        ("π/6", np.pi / 6),
        ("π/4", np.pi / 4),
        ("π/3", np.pi / 3),
        ("π/2", np.pi / 2),
    ]
    turn_rate_cases = [
        (
            f"{label} rad/s",
            run_experiment(
                replace(
                    baseline,
                    max_turn_rate=turn_rate,
                )
            ),
        )
        for label, turn_rate in max_turn_rates
    ]

    print_sweep("Acceleration authority sweep", acceleration_cases, "Accel limits")
    print_sweep("Turn-rate authority sweep", turn_rate_cases, "Max turn rate")


if __name__ == "__main__":
    main()
