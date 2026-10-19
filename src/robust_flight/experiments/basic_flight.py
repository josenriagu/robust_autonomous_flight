from dataclasses import replace

import matplotlib.pyplot as plt
import numpy as np

from robust_flight.experiments.runner import ExperimentConfig, run_experiment


def print_result(result):
    print(f"Speed controller gain: {result.config.speed_kp:.2f}")
    print(f"Final position: {result.final_position}")
    print(f"Final speed: {result.final_speed:.3f} m/s")
    print(f"Final distance: {result.final_distance:.3f} m")
    print(f"Minimum distance: {result.minimum_distance:.3f} m")
    print(f"Cross-track RMSE: {result.cross_track_rmse:.3f} m")
    print(f"Maximum cross-track error: {result.max_cross_track_error:.3f} m")
    print(f"Mission complete: {'yes' if result.mission_complete else 'no'}")
    print(f"Final desired speed: {result.final_desired_speed:.3f} m/s")
    print(f"Speed RMSE: {result.speed_rmse:.3f} m/s")
    print(f"Minimum acceleration: {result.min_observed_acceleration:.3f} m/s²")
    print(f"Maximum acceleration: {result.max_observed_acceleration:.3f} m/s²")
    print(f"Maximum speed error: {result.max_speed_error:.3f} m/s")
    print(f"Maximum overspeed: {result.max_overspeed:.3f} m/s")
    print(f"Final time: {result.final_time:.3f} s")
    print(
        f"Acceleration saturation rate: "
        f"{result.acceleration_saturation_rate * 100:.1f}%"
    )
    print(f"Turn-rate saturation rate: {result.turn_rate_saturation_rate * 100:.1f}%")
    print(
        f"Max acceleration intervention: "
        f"{result.max_acceleration_intervention:.3f} m/s²"
    )
    print(
        f"Mean acceleration intervention: "
        f"{result.mean_acceleration_intervention:.3f} m/s²"
    )
    print(f"Max turn-rate intervention: {result.max_turn_rate_intervention:.3f} rad/s")
    print(
        f"Mean turn-rate intervention: {result.mean_turn_rate_intervention:.3f} rad/s"
    )


def main():
    baseline = ExperimentConfig()
    speed_gains = [0.25, 0.5, 1.0, 2.0]

    experiments = []

    for speed_kp in speed_gains:
        experiment = run_experiment(replace(baseline, speed_kp=speed_kp))
        print_result(experiment)
        experiments.append(experiment)

    print("\nGain comparison")
    print("-" * 84)

    print(
        f"{'Kp':>6} "
        f"{'RMSE':>10} "
        f"{'Overspeed':>10} "
        f"{'XTE RMSE':>10} "
        f"{'Done':>6} "
        f"{'Accel Sat.':>12} "
        f"{'Mean Int.':>12} "
        f"{'Time':>10}"
    )

    print("-" * 84)

    for experiment in experiments:
        print(
            f"{experiment.config.speed_kp:>6.2f} "
            f"{experiment.speed_rmse:>10.3f} "
            f"{experiment.max_overspeed:>10.3f} "
            f"{experiment.cross_track_rmse:>10.3f} "
            f"{'yes' if experiment.mission_complete else 'no':>6} "
            f"{experiment.acceleration_saturation_rate * 100:>11.1f}% "
            f"{experiment.mean_acceleration_intervention:>12.3f} "
            f"{experiment.final_time:>10.3f}"
        )

    # plot speed tracking for different controller gains
    plt.figure()

    for experiment in experiments:
        time = np.arange(len(experiment.history)) * experiment.config.dt

        plt.plot(
            time,
            experiment.history[:, 2],
            label=f"Kp = {experiment.config.speed_kp:.2f} actual",
        )

        plt.plot(
            time,
            experiment.desired_speeds,
            "--",
            label=f"Kp = {experiment.config.speed_kp:.2f} desired",
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.title("Speed Tracking for ifferent Controller Gains")
    plt.legend()
    plt.grid()

    plt.show()

    # plot speed tracking errors for different controller gains
    plt.figure()

    for experiment in experiments:
        time = np.arange(len(experiment.speed_errors)) * experiment.config.dt

        plt.plot(
            time,
            experiment.speed_errors,
            label=f"Kp = {experiment.config.speed_kp:.2f}",
        )

    plt.axhline(
        0.0,
        linestyle="--",
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Speed Error (m/s)")
    plt.title("Speed Tracking Error for Different Controller Gains")
    plt.legend()
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()
