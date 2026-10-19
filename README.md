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
    │   ├── actuator.py
    │   └── aircraft.py
    │
    ├── simulation/
    │   ├── __init__.py
    │   └── engine.py
    │
    ├── experiments/
    │   ├── __init__.py
    │   ├── runner.py
    │   ├── basic_flight.py
    │   └── actuator_authority.py
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

Run the speed controller gain experiment:

```bash
python -m robust_flight.experiments.basic_flight
```

It prints metrics for each gain and a comparison table, then opens two plots:

- desired vs. actual speed over time
- speed tracking error over time

Run the actuator authority experiment:

```bash
python -m robust_flight.experiments.actuator_authority
```

It prints an acceleration-limit sweep and a turn-rate-limit sweep, then opens a plot of the aircraft trajectories for each turn-rate limit.

Each plot window must be closed before the next one opens.

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

The `max_deceleration` parameter describes the braking assumption used by the guidance law. It is **not an enforced limit**; the enforced limit belongs to the actuator model.

The guidance plans braking at `1.5 m/s²`, 75% of the actuator's `2.0 m/s²` deceleration limit. Planning at the full limit leaves the lagging speed controller no authority to catch up once it falls behind the braking profile, so the aircraft reaches each waypoint too fast.

This gives the aircraft a simple speed profile that reduces the desired speed as it approaches each waypoint.

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

Every experiment runs through a shared runner:

```text
src/robust_flight/experiments/runner.py
```

All parameters live in a frozen `ExperimentConfig` dataclass, whose defaults define the baseline scenario:

```python
# simulation parameters
duration = 60.0
dt = 0.01

# initial state [px, py, v, psi] and mission waypoints
initial_state = (0.0, 0.0, 10.0, 0.0)
waypoints = ((50.0, 0.0), (50.0, 50.0), (0.0, 50.0), (0.0, 0.0))

# guidance parameters
max_speed = 20.0
max_deceleration = 1.5

# controller gains
heading_kp = 2.0
speed_kp = 1.0

# actuator limits
min_acceleration = -2.0
max_acceleration = 3.0
max_turn_rate = π/3
```

The baseline mission is a 50 m square with three 90° turns that returns to the start. The aircraft starts aligned with the first leg, so the first leg exercises speed control only and every turn happens at a corner.

A sweep varies one parameter at a time with `dataclasses.replace`, so every other parameter stays at its baseline value:

```python
baseline = ExperimentConfig()
result = run_experiment(replace(baseline, speed_kp=0.5))
```

`run_experiment` returns an `ExperimentResult` that carries the config it ran with, the full simulation history and every metric. It performs no printing; each experiment script decides how to present its results.

### Time Step

The baseline uses `dt = 0.01 s`. At `dt = 0.1 s`, the original value, several metrics changed with the time step rather than with the system:

| `dt` (s) | Final distance (m) | Speed RMSE (m/s) |
|---:|---:|---:|
| 0.1 | 0.791 | 2.555 |
| 0.05 | 0.680 | 2.500 |
| 0.02 | 0.874 | 2.442 |
| 0.01 | 0.982 | 2.420 |
| 0.001 | 0.995 | 2.408 |

These figures come from the original single-waypoint scenario (waypoint `(50, 30)`, `duration = 10.0`, `max_deceleration = 2.0`). Final distance converges to the 1.0 m arrival radius as `dt` shrinks, so at `dt = 0.1 s` it measured where the last integration step landed, not controller behavior. At `dt = 0.01 s`, speed RMSE is within 0.5% of its converged value.

---

## Experimental Metrics

`run_experiment` computes every metric and returns it in an `ExperimentResult`. The headline metrics are:

| Group | Metric | Field |
|---|---|---|
| Mission | mission completed | `mission_complete` |
| Mission | elapsed simulation time | `final_time` |
| Path tracking | cross-track RMSE and maximum | `cross_track_rmse`, `max_cross_track_error` |
| Speed tracking | speed RMSE | `speed_rmse` |
| Speed tracking | maximum overspeed | `max_overspeed` |
| Actuator | saturation rate per channel | `acceleration_saturation_rate`, `turn_rate_saturation_rate` |
| Actuator | mean and maximum intervention per channel | `mean_acceleration_intervention`, … |
| Actuator | control effort per channel | `acceleration_effort`, `turn_rate_effort` |

The result also keeps final position, final and minimum distance to the last waypoint, final speed, final desired speed, maximum speed error and observed acceleration. These are recorded for completeness but are not used to compare runs, for the reasons below.

### Active Leg

Each leg of the mission runs from the previous waypoint, or the initial position for the first leg, to the active waypoint. The runner records which waypoint the controller steered towards at every step, and measures desired speed and cross-track error against that step's own leg.

### Cross-Track Error

Cross-track error is the signed perpendicular distance from the straight path of the active leg:

```text
e_xt = (path × offset) / |path|
```

where `path` runs from the leg start to the leg end, and `offset` from the leg start to the aircraft. It is positive when the aircraft is to the left of the path. The experiment reports its RMSE and its maximum magnitude.

### Speed Tracking

The speed-tracking error is:

```text
e_v = v_desired - v_actual
```

Speed RMSE is `sqrt(mean(e_v²))` over the whole run.

The largest positive error is set by the initial state: the aircraft starts at 10 m/s while the guidance initially asks for more. The controller's own shortfall shows up as **overspeed**, where the aircraft is faster than the braking profile allows on the approach to a waypoint:

```text
max_overspeed = max(0, -min(e_v))
```

### Actuator Saturation, Intervention and Effort

The engine records both the control the controller requested and the control the actuator applied.

- **Intervention** is `|requested - applied|` for each channel at each step.
- **Saturation rate** is the fraction of steps with non-zero intervention.
- **Control effort** is `sum(u²) * dt` over the applied control.

### Observed Acceleration

Observed acceleration is derived from the recorded speed history:

```python
observed_accelerations = np.diff(history[:, 2]) / dt
```

It describes what the simulated aircraft did, as distinct from the requested and applied control above.

### Mission Completion and Elapsed Time

`mission_complete` records whether every waypoint was reached before the configured duration ran out, so a timeout is distinguishable from a completed mission. Elapsed time is:

```python
final_time = (len(history) - 1) * dt
```

Metrics from an incomplete run must be read alongside `mission_complete`. An aircraft circling a waypoint at a steady speed tracks speed well, so its speed RMSE can be lower than that of a run that completes.

### Why Final Distance Is Not a Headline Metric

A run ends on the first step that brings the aircraft within the 1.0 m arrival radius. Final distance therefore depends on where that last step lands, not on how well the aircraft was controlled, and it converges to the arrival radius as `dt` shrinks (see [Time Step](#time-step)). Cross-track error and mission completion replace it.

---

## Results

Every result below starts from the baseline `ExperimentConfig` and changes one parameter. The speed-gain and actuator-authority tables come from the two experiment scripts; the extra turn-rate rows and the braking-margin table come from the same runner with `replace`. These results describe the current simplified model and should not be read as validated aircraft performance.

### Speed Controller Gain

| `speed_kp` | Done | Time (s) | XTE RMSE (m) | Speed RMSE (m/s) | Max overspeed (m/s) | Accel saturation | Accel effort |
|---:|---|---:|---:|---:|---:|---:|---:|
| 0.25 | yes | 23.38 | 3.61 | 2.79 | 6.20 | 0.0% | 11.4 |
| 0.50 | yes | 23.93 | 3.23 | 2.67 | 4.50 | 3.0% | 42.2 |
| 1.00 | yes | 25.84 | 2.45 | 2.75 | 2.50 | 25.4% | 86.2 |
| 2.00 | yes | 28.63 | 1.79 | 3.05 | 1.05 | 25.7% | 115.4 |

- A higher gain lowers overspeed from 6.20 to 1.05 m/s and cross-track RMSE from 3.61 to 1.79 m.
- The price is ten times the acceleration effort and a 22% longer mission. A higher gain follows the braking profile's slow-down into every corner more closely, and that takes time.
- Speed RMSE barely changes across an eightfold range of gains. After each corner the desired speed jumps back towards cruise speed faster than the 3 m/s² acceleration limit allows, and that error does not depend on the gain.

### Acceleration Authority

| Accel limits (m/s²) | Done | Time (s) | XTE RMSE (m) | Max overspeed (m/s) | Accel saturation | Accel effort |
|---|---|---:|---:|---:|---:|---:|
| (−1.0, 1.5) | yes | 23.93 | 3.30 | 5.36 | 71.8% | 27.5 |
| (−1.5, 2.0) | yes | 25.75 | 2.63 | 3.51 | 55.2% | 54.5 |
| (−2.0, 3.0) | yes | 25.84 | 2.45 | 2.50 | 25.4% | 86.2 |
| (−3.0, 4.0) | yes | 25.45 | 2.61 | 2.40 | 11.1% | 104.2 |
| (−4.0, 5.0) | yes | 25.21 | 2.77 | 2.40 | 7.2% | 116.4 |

- Overspeed falls as braking authority grows, until the actuator can brake harder than the guidance plans (1.5 m/s²). Beyond that point it stays at 2.40 m/s.
- With only 1.0 m/s² of braking, below what the guidance plans, overspeed more than doubles and the actuator saturates 72% of the time.
- Cross-track error is lowest at the baseline limits and rises slightly with more authority. The differences are small and this sweep does not explain them.

### Turn-Rate Authority

| Max turn rate (rad/s) | Done | Time (s) | XTE RMSE (m) | Max XTE (m) | Turn saturation | Turn effort |
|---|---|---:|---:|---:|---:|---:|
| π/12 | **no** | 60.00 | 28.48 | 55.15 | 90.6% | 3.73 |
| π/8 | yes | 28.58 | 9.55 | 19.59 | 44.3% | 2.11 |
| π/6 | yes | 26.77 | 5.58 | 13.08 | 29.3% | 2.39 |
| π/4 | yes | 26.12 | 3.37 | 7.27 | 17.3% | 3.30 |
| π/3 | yes | 25.84 | 2.45 | 4.99 | 11.4% | 4.14 |
| π/2 | yes | 25.61 | 1.70 | 3.29 | 5.6% | 5.52 |
| π | yes | 25.50 | 1.33 | 2.48 | 0.0% | 7.16 |
| 2π | yes | 25.50 | 1.33 | 2.48 | 0.0% | 7.16 |
| unlimited | yes | 25.50 | 1.33 | 2.48 | 0.0% | 7.16 |

The rows from π upwards were run with `replace`; the experiment script sweeps π/12 to π/2.

- **There is a failure threshold between π/12 and π/8 rad/s.** At π/12 the aircraft reaches the first corner, swings wide on the 90° turn, and circles without ever capturing the second waypoint. It is still circling after 300 s and after 600 s, so the failure is permanent rather than slow.
- **The circling sustains itself.** Missing the waypoint takes the aircraft further away, where the guidance asks for more speed, and more speed widens the turn at the saturated turn rate.
- **Between π/8 and π, more turn authority steadily improves path tracking,** with diminishing returns.
- **Above π nothing changes,** because the turn limit is never reached. On this mission the controller asks for at most 3.10 rad/s: `heading_kp` × the 90° heading change at each corner. The remaining 1.33 m of cross-track RMSE comes from the guidance and controller design (aiming at a point with a proportional heading controller), not from turn authority.

### Braking Margin

The guidance plans braking at a fraction of the actuator's 2.0 m/s² deceleration limit:

| Guidance braking (m/s²) | Share of limit | Time (s) | Max overspeed (m/s) | XTE RMSE (m) | Accel saturation | Lowest turn limit that completes |
|---:|---:|---:|---:|---:|---:|---|
| 2.00 | 100% | 22.02 | 4.23 | 3.31 | 48.3% | π/6 |
| 1.75 | 88% | 23.78 | 3.32 | 2.86 | 39.1% | π/6 |
| **1.50** | **75%** | **25.84** | **2.50** | **2.45** | **25.4%** | **π/8** |
| 1.25 | 62% | 28.26 | 1.98 | 2.15 | 14.4% | π/8 |
| 1.00 | 50% | 31.46 | 1.55 | 1.86 | 11.7% | π/8 |

More margin gives tighter, slower flight. The baseline uses 75%. Compared with planning at the full limit, it lowers overspeed by 41% and cross-track RMSE by 26%, halves braking saturation, and moves the failure threshold from between π/8 and π/6 down to between π/12 and π/8, for a 17% longer mission.

### Corrections to Earlier Results

An earlier version of this README reported a speed-gain sweep on a single waypoint at `dt = 0.1 s`, before actuator limits existed. Two conclusions drawn at that stage did not survive:

- Waypoint error that "did not improve monotonically with gain" (0.881, 0.380, 0.528, 0.755 m) was where the last integration step landed, not controller behavior.
- An apparent non-monotonic effect of turn authority on waypoint error (0.381 m at π/8 against 0.791 m at π/3) had the same cause.

Both disappear as `dt` shrinks (see [Time Step](#time-step)). The trade-off between speed tracking and control effort reported at that stage still holds.

---

## Current Limitations

The current model is deliberately simplified and does not yet represent a realistic aircraft.

**Aircraft model**

- planar rather than 3D dynamics
- simplified kinematic model with forward Euler integration
- no aerodynamic model
- actuator limits are instantaneous saturation only, with no actuator dynamics or rate limits
- no minimum airspeed or stall model; in the baseline the aircraft never drops below about 4 m/s, so this has not mattered yet
- no disturbance or wind model

**Guidance and mission**

- guidance aims directly at the active waypoint rather than following the path between waypoints, which leaves a cross-track error floor of about 1.33 m
- a waypoint counts as reached only inside a fixed 1.0 m radius, with no pass-by switching, so an aircraft that cannot turn tightly enough circles a waypoint indefinitely
- proportional controllers only
- one baseline mission

**Sensing, communication and estimation**

- the controller uses the true state
- no sensor models, noise, bias or dropout
- no communication delay, jitter, packet loss or outages
- no state estimator

**Method**

- each configuration is a single deterministic run; there is no randomness yet, so there are no repeated trials
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

- [x] Automated parameter sweep script
- [x] Define actuator model
- [x] Add actuator saturation
- [x] Separate requested and achievable control
- [x] Investigate controller behavior under actuator constraints
- [x] Record mission completion separately from simulation timeout
- [x] Multi-waypoint mission with per-leg metrics
- [ ] Evaluate alternative controllers such as LQR (future work)

### Phase 3 — Sensors

Introduce simplified models for:

- [ ] GPS
- [ ] IMU
- [ ] Altimeter (only once the model has altitude)

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

### Future Work

Beyond the phases above:

- 3D and 6-DOF fixed-wing dynamics
- PID and LQR controller comparisons
- a minimum-airspeed and stall model
- pass-by waypoint switching and path-following guidance

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

Experimental results are reproduced by the experiment scripts and the shared runner, never by editing parameters by hand.

---

## License

Released under the MIT License. See [LICENSE](LICENSE).

---

## Status

**Current stage:** Closed-loop planar aircraft simulation on a square mission with three 90° turns, with actuator saturation, a shared experiment runner, per-leg path and speed metrics, and reproducible sweeps of controller gain, acceleration authority, turn authority and braking margin.

**Current focus:** Phase 3. Adding sensor models so the controller steers on measurements instead of the true state.

**Next architectural question:** Where sensing and estimation sit between the simulation engine and the controller, so the same runner can compare true-state, raw-measurement and estimated-state feedback.