# Robust Autonomous Flight

A Python-based simulation and research framework for investigating robust autonomous flight under uncertainty.

The project is being developed as a research-oriented side project alongside an application to graduate study in aerospace systems and control.

---

## Research Direction

The long-term focus is:

> **Guidance, Navigation, Estimation and Control of Autonomous Aerospace Systems**

The initial research question is:

> **How do sensor uncertainty and communication degradation affect autonomous aircraft state estimation and trajectory tracking, and to what extent can feedback control and state estimation mitigate these effects?**

The project will progressively introduce realistic sources of uncertainty into an autonomous aircraft simulation and evaluate how different estimation and control strategies respond.

---

## Current Scope

The simulation currently models a simplified planar aircraft with four states:

```text
state = [px, py, v, ψ]
```

where:

- `px` — x-position
- `py` — y-position
- `v` — speed
- `ψ` — heading

The control input is:

```text
control = [a, r]
```

where:

- `a` — acceleration
- `r` — heading rate

The aircraft dynamics are:

```text
dpx/dt = v cos(ψ)
dpy/dt = v sin(ψ)
dv/dt  = a
dψ/dt  = r
```

The current aircraft model uses forward Euler integration.

This is intentionally a simplified model. The goal at this stage is to establish the simulation architecture and control framework before increasing physical complexity.

---

## Project Architecture

The current source tree is:

```text
src/
└── robust_flight/
    ├── __init__.py
    │
    ├── dynamics/
    │   ├── __init__.py
    │   └── aircraft.py
    │
    ├── simulation/
    │   ├── __init__.py
    │   └── engine.py
    │
    ├── experiments/
    │   ├── __init__.py
    │   └── basic_flight.py
    │
    ├── guidance/
    │   ├── __init__.py
    │   └── waypoints.py
    │
    ├── control/
    │   ├── __init__.py
    │   ├── heading.py
    │   └── speed.py
    │
    └── mission/
        ├── __init__.py
        └── mission.py
```

### Design Principles

The project is intentionally being developed with separation between:

- aircraft dynamics
- simulation execution
- guidance
- control
- mission management
- experiments

This allows individual components to evolve without turning the simulation into a single monolithic script.

---

## Getting Started

Create a virtual environment:

```bash
python3 -m venv .venv
```

Activate the virtual environment.

**macOS / Linux**

```bash
source .venv/bin/activate
```

**Windows**

```powershell
.venv\Scripts\Activate.ps1
```

Install the project and development dependencies:

```bash
pip install -e ".[dev]"
```

Run the basic flight experiment:

```bash
python -m robust_flight.experiments.basic_flight
```

The experiment prints performance metrics to stdout and opens two plots:

- desired vs. actual speed over time
- aircraft trajectory

---

## Aircraft Dynamics

The `Aircraft` class currently owns the aircraft state and dynamics.

The state is represented as:

```python
state = [px, py, v, psi]
```

The dynamics function computes the state derivative from the current state and control input.

The simulation uses forward Euler integration:

```python
state_next = state + state_derivative * dt
```

The simulation loop was deliberately separated from the aircraft model so that the aircraft describes **how it moves**, while the simulation engine describes **how the experiment is executed**.

---

## Simulation Engine

The simulation loop lives in:

```text
src/robust_flight/simulation/engine.py
```

The current interface is conceptually:

```python
simulate(
    aircraft,
    control_function,
    duration,
    dt,
    termination_condition=None,
)
```

The engine:

1. records the initial state
2. calculates control
3. advances the aircraft
4. records the new state
5. checks the optional termination condition
6. repeats until the simulation ends

The termination condition is implemented as a callback rather than making the generic simulation engine aware of mission-specific concepts.

### Numerical Detail

The original simulation used a time-based `while` loop.

This produced an undesirable floating-point edge case where a value such as:

```text
9.99999999999998
```

could still satisfy:

```python
time < 10.0
```

and produce an extra integration step.

The simulation was changed to determine the number of integration steps explicitly:

```python
num_steps = int(duration / dt)
```

When the simulation runs to its full configured duration, this gives:

- `100` integration steps for `duration = 10.0`
- `101` recorded states, including the initial state

When a mission termination condition fires earlier, fewer states are recorded.

This keeps the integration count deterministic for full-duration simulations.

---

## Guidance

Waypoint guidance is implemented by the `Waypoints` class.

Current guidance capabilities include:

### Desired Heading

The desired heading is calculated from the aircraft position to the waypoint:

```python
desired_heading = np.arctan2(direction[1], direction[0])
```

### Heading Error

Heading error is normalized to:

```text
[-π, π]
```

so that the controller takes the shortest angular path.

### Desired Speed

The current desired speed is based on the distance remaining to the waypoint.

The guidance system estimates a braking speed using:

```text
v_braking = sqrt(2 * max_deceleration * distance)
```

The desired speed is limited by the configured maximum speed:

```text
desired_speed = min(max_speed, v_braking)
```

When the distance to the waypoint is zero, the desired speed is `0.0`.

The `max_deceleration` parameter currently describes the braking assumption used by the guidance law. It is **not an enforced aircraft or actuator limit**.

This gives the aircraft a simple speed profile that reduces the desired speed as it approaches the waypoint.

---

## Controllers

Two proportional controllers currently provide closed-loop control.

### Heading Controller

The heading controller calculates heading rate from heading error:

```python
heading_control = kp * heading_error
```

### Speed Controller

The speed controller calculates acceleration from speed error:

```python
acceleration = kp * speed_error
```

The speed error is calculated outside the `Waypoints` class:

```python
speed_error = desired_speed - current_speed
```

This separation is intentional.

`Waypoints` is responsible for guidance — determining what the aircraft should do.

The controller is responsible for feedback — determining the control response to the difference between desired and actual behavior.

---

## Mission Management

The `Mission` class manages the active waypoint and mission completion.

It currently provides:

- current waypoint
- waypoint reach detection
- progression to the next waypoint
- mission completion status
- mission update logic

A waypoint is currently considered reached when the aircraft is within **1.0 m** of it.

The 1.0 m threshold is currently the default value inside the waypoint-reach check and is not yet configurable through the `Mission` constructor.

The simulation engine does not know about `Mission`.

Instead, the experiment supplies a termination callback that connects mission state to the generic simulation engine.

This keeps mission-specific logic outside the simulation engine.

---

## Experiment Configuration

The basic experiment centralizes its primary parameters:

```python
duration = 10.0
dt = 0.1

max_speed = 20.0
max_deceleration = 2.0

heading_kp = 2.0
speed_kp = 2.0
```

The simulation and guidance parameters are defined together near the top of the experiment script, making the conditions used by the current experiment easy to identify and change.

At present, changing an experimental parameter still requires editing the experiment script. Reproducible parameter sweeps are planned as a future improvement.

---

## First Experiment: Speed Controller Gain

The first controlled experiment investigated the effect of the speed-controller proportional gain:

```text
speed_kp
```

All other parameters were held constant.

The sweep was performed by manually changing `speed_kp` in the experiment script and running the simulation for each value.

The tested values were:

```text
0.25
0.50
1.00
2.00
```

The aircraft started at:

```text
position = [0, 0]
speed    = 10 m/s
heading  = 0 rad
```

and was commanded toward:

```text
waypoint = [50, 30]
```

The current repository contains the experiment implementation with the final manually tested value, `speed_kp = 2.0`. The four results below were recorded from the corresponding manual runs.

A reproducible automated sweep is planned so that these results can be regenerated without manually editing the experiment configuration.

---

## Experimental Metrics

The experiment currently records:

- final position
- final distance to waypoint
- minimum distance to waypoint
- final speed
- final desired speed
- speed-tracking RMSE
- maximum observed acceleration
- maximum observed deceleration
- elapsed simulation time

### Speed-Tracking RMSE

The speed-tracking error is:

```text
e_v = v_desired - v_actual
```

The experiment calculates:

```text
RMSE = sqrt(mean(e_v²))
```

This provides a measure of how closely the aircraft follows the desired speed throughout the simulation.

### Acceleration Metrics

Acceleration is currently derived from the recorded speed history:

```python
accelerations = np.diff(history[:, 2]) / dt
```

The reported maximum acceleration and deceleration therefore represent **observed acceleration in the simulated aircraft**, rather than a separately recorded controller command.

This distinction will become important when actuator dynamics and saturation are introduced.

### Elapsed Simulation Time

The reported final time is calculated as:

```python
final_time = (len(history) - 1) * dt
```

This represents elapsed simulation time.

At present, the experiment does not separately report whether termination occurred because the mission completed or because the simulation reached its configured duration.

---

## Speed Gain Experiment Results

| `speed_kp` | Final distance (m) | Final speed (m/s) | Speed RMSE (m/s) | Max acceleration (m/s²) | Peak deceleration magnitude (m/s²) | Elapsed time (s) |
|---:|---:|---:|---:|---:|---:|---:|
| 0.25 | 0.881 | 9.310 | 3.459 | 1.318 | 1.694 | 5.3 |
| 0.50 | 0.380 | 7.586 | 3.147 | 2.636 | 2.855 | 5.3 |
| 1.00 | 0.528 | 5.076 | 2.315 | 5.272 | 3.343 | 5.5 |
| 2.00 | 0.755 | 3.292 | 1.458 | 10.544 | 2.954 | 5.9 |

Acceleration is recorded as a signed quantity internally; the table reports the magnitude of the most negative observed acceleration as peak deceleration.

---

### Initial Observations

Increasing `speed_kp` produced lower speed-tracking RMSE across the tested range:

```text
3.459 → 3.147 → 2.315 → 1.458 m/s
```

At the same time, maximum observed acceleration increased substantially:

```text
1.318 → 2.636 → 5.272 → 10.544 m/s²
```

The results therefore demonstrate a trade-off between speed-tracking performance and control effort within the current simplified model.

Waypoint error did not improve monotonically with increasing gain:

```text
0.881 → 0.380 → 0.528 → 0.755 m
```

This illustrates why controller performance cannot be characterized adequately using a single metric.

The current results should not be interpreted as identifying an optimal gain. The experiment only characterizes the behavior of the current simplified system under the tested proportional gains.

---

## Current Limitations

The current model is deliberately simplified and does not yet represent a realistic aircraft.

Important limitations include:

- planar rather than 3D dynamics
- simplified kinematic model
- no aerodynamic model
- no actuator dynamics
- no actuator saturation
- no sensor noise
- no sensor bias
- no sensor dropout
- no communication delay
- no packet loss
- no communication outages
- no state estimator
- no EKF
- no realistic GPS/IMU models
- no disturbance model
- no wind model
- no uncertainty model
- proportional controllers only
- no automated experiment sweep
- no automated test suite

The current simulation therefore represents a **controlled development baseline**, not a flight-accurate aircraft model.

---

## Planned Development

The project will progressively introduce more realistic aerospace-system components.

### Phase 1 — Core Simulation

- [x] Aircraft state and dynamics
- [x] Simulation engine
- [x] Package structure
- [x] Waypoint guidance
- [x] Heading control
- [x] Speed control
- [x] Mission management
- [x] Mission-aware termination
- [x] Performance metrics
- [x] Controller gain experiment

### Phase 2 — Control Architecture

- [ ] Automated parameter sweep script
- [ ] Define actuator model
- [ ] Add actuator saturation
- [ ] Separate requested and achievable control
- [ ] Investigate controller behavior under actuator constraints
- [ ] Evaluate alternative controllers such as LQR
- [ ] Record mission completion separately from simulation timeout

### Phase 3 — Sensors

Introduce simplified models for:

- [ ] GPS
- [ ] IMU
- [ ] Altimeter

Add:

- [ ] measurement noise
- [ ] bias
- [ ] different sensor update rates
- [ ] measurement dropout

### Phase 4 — Communication Uncertainty

Introduce:

- [ ] communication delay
- [ ] jitter
- [ ] packet loss
- [ ] temporary communication outages

### Phase 5 — State Estimation

Implement:

- [ ] state estimation framework
- [ ] Extended Kalman Filter (EKF)
- [ ] comparison between true and estimated state
- [ ] estimation error metrics

### Phase 6 — Robust Autonomous Flight Experiments

Evaluate:

- [ ] nominal operation
- [ ] sensor degradation
- [ ] communication degradation
- [ ] combined sensor/communication uncertainty
- [ ] controller performance with estimated state
- [ ] recovery following temporary outages

Potential metrics include:

- trajectory RMSE
- position error
- speed error
- heading error
- maximum deviation
- settling time
- control effort
- recovery time
- maximum tolerable communication delay
- maximum tolerable packet-loss rate

---

## Technology

The project currently uses:

- Python 3.11+
- NumPy
- Matplotlib
- Ruff
- Git
- GitHub

Ruff is used for code formatting and linting.

The project is packaged using `pyproject.toml` and installed in editable mode during development.

The intent is to keep the core research framework open-source and reproducible, avoiding unnecessary dependence on proprietary software.

---

## Development Philosophy

The project is being developed incrementally.

Each major architectural or experimental change should:

1. introduce one meaningful change
2. run the simulation
3. measure the resulting behavior
4. record the result
5. commit the change

The goal is to maintain a reproducible chain from implementation decisions to experimental observations.

The project is intentionally being built from a simple model toward increasing levels of realism rather than attempting to construct a complete aerospace simulation immediately.

As the project matures, experimental results should become reproducible through dedicated experiment scripts rather than manual parameter changes.

---

## License

Released under the MIT License. See [LICENSE](LICENSE).

---

## Status

**Current stage:** Closed-loop planar aircraft simulation with waypoint guidance, proportional heading/speed control, mission management, performance metrics, and an initial speed-controller gain experiment.

**Current focus:** Establishing reproducible experiments and a clean control architecture before introducing actuator constraints.

**Next architectural question:** Where actuator constraints and saturation should live within the simulation architecture.