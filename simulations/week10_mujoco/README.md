# Week 10 — MuJoCo beam and observer

Compare a nonlinear MuJoCo beam/slider with the course's linear model using the
same pole-placement gains, angle-only observer, initial condition, 2 ms control
period, and 10 N m torque limit. MuJoCo uses a 1 ms RK4 physics step; the linear
plant and observer updates use exact zero-order-hold discretization.

`compute_control(xhat, K, torque_limit)` calculates the saturated torque.
`update_observer(xhat, u, y, Ao, Bo)` calculates the next estimate from that
applied torque and the measured angle. Both the recorded simulations and the
native viewer call these functions. The simulation loop handles measurement,
plant stepping, and logging; the observer cannot access the true plant state.

## Run locally

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python beam_control.py --output output
```

This produces the comparison plot, simulation data, validation results, model
XML, MP4 video, and GIF animation. For numerical results without a graphics
context, add `--no-render`.

For a native interactive viewer on macOS:

```bash
mjpython beam_control.py --live
```

On other desktop platforms, use `python beam_control.py --live`. The live viewer
runs at a 1 ms control period and accepts mouse perturbations; the recorded
comparison uses the 2 ms period described above. The scripted numerical and
rendering workflows are tested; the interactive viewer is optional.

The self-contained `notebooks/week10_mujoco_beam.ipynb` companion can be run in
Jupyter or Colab. It installs missing dependencies, executes the comparison, and
embeds a playable MP4. Linux without a display defaults to EGL; native rendering
requires a working OpenGL/EGL context. All numerical checks and plots work without
rendering.

## What the model represents

- State order: `[d, d_dot, theta, theta_dot]`, SI units, counterclockwise angle.
- A frictionless slider of mass 1 kg on a balanced beam with pivot inertia
  2 kg m². A unit-gear motor applies torque at the hinge.
- The orange sphere is a **display marker for a sliding point mass**. It is not
  a rolling/contact simulation. Its visual center is offset above the beam, but
  its inertial center follows the beam centerline, as in the course model.
- Slider rotational inertia is `1e-8 kg m²`, the small positive value used to
  approximate a point mass. Beam mass/inertia are explicitly specified, rather
  than inferred from drawing dimensions.
- The blue ring is a display-only observer estimate. It uses the actual beam
  angle to isolate the position error and never changes engine dynamics.
- The right-hand animation replays the linear trajectory in MuJoCo's renderer;
  only the left-hand trajectory is integrated by the physics engine.
- The slider has no end stops and collisions are disabled. The baseline stays
  within the drawn ±0.4 m beam. Changing initial conditions can exceed that range.

## Checks

The script verifies equilibrium, the engine's finite-difference linearization,
nonlinear accelerations, sampled closed-loop stability, small-disturbance
agreement, physics-timestep convergence, and regulation with full-state and
observer feedback. Metrics retain separate physical units for each state.

Reference: [MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html).
