"""MAE 6246 - mjsim: design controllers and observers, test them on MuJoCo.

You never call MuJoCo directly. Every dynamic model is used the same way:

    import mjsim
    beam_dyn = mjsim.load_dynamics("beam", u_limit=10.0)   # dt = 0.002 s by default
    A, B, C, D = beam_dyn.A, beam_dyn.B, beam_dyn.C, beam_dyn.D
    x_eq, u_eq, y_eq = beam_dyn.x_eq, beam_dyn.u_eq, beam_dyn.y_eq
    n, m, p, dt = beam_dyn.n, beam_dyn.m, beam_dyn.p, beam_dyn.dt

    res = mjsim.simulate(beam_dyn, x0, T, controller, observer=None, xhat0=None)
    res.t, res.x, res.xhat, res.u, res.y

Standard signatures (write these yourself; they never use MuJoCo):

    u         = controller(t, z)          z = xhat (or the true x if observer=None)
    xhat_next = observer(t, xhat, u, y)   update over one sample period dyn.dt

At each sample t_k = k*dt (dt = dyn.dt) the simulator
    1. computes u_k = controller(t_k, xhat_k)  (true x_k if no observer),
    2. measures y_k = C x_k + D u_k from the MuJoCo sensors,
    3. computes xhat_{k+1} = observer(t_k, xhat_k, u_k, y_k),
    4. holds u_k for dt while MuJoCo integrates the nonlinear dynamics.
The controller and observer never see the true state when an observer is used.

`dyn.linear()` returns the linear model (in deviation form, see below) with the
same interface, so the same call compares MuJoCo with linear theory.
A, B, C, D describe deviations from an isolated equilibrium (x_eq, u_eq) with output y_eq:
    d/dt (x - x_eq) = A (x - x_eq) + B (u - u_eq),   y - y_eq = C (x - x_eq) + D (u - u_eq).
Simulation results are in physical (absolute) coordinates. Write controllers and
observers in deviation form, e.g. u = u_eq - K (xhat - x_eq).
n, m, p are the numbers of states, inputs and outputs.
"""
import os
import sys
from types import SimpleNamespace

# Headless Linux/Colab rendering. Must be set before importing MuJoCo.
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import mujoco
from scipy.linalg import expm

__all__ = ["load_dynamics", "simulate", "discretize", "plot_response", "animate",
           "available_dynamics"]


# ----------------------------------------------------------------------------
# Shared tools
# ----------------------------------------------------------------------------
def discretize(A, B, dt):
    """Zero-order-hold discretization of x_dot = A x + B w with w held over dt.

    Returns (Ad, Bd) such that x_{k+1} = Ad x_k + Bd w_k exactly.
    For an observer, use w = [u; y]:
        Ad, Bd = discretize(A - L@C, np.hstack([B - L@D, L]), dyn.dt)
    """
    A, B = np.atleast_2d(A).astype(float), np.atleast_2d(B).astype(float)
    n, m = B.shape
    M = np.zeros((n+m, n+m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M*dt)
    return E[:n, :n], E[:n, n:]


class _Dynamics:
    """Common interface shared by the MuJoCo and linear dynamics."""

    def _setup(self, spec, dt, u_limit, equilibrium):
        if dt <= 0:
            raise ValueError("dt must be positive")
        equilibria = spec["equilibria"]
        equilibrium = equilibrium or next(iter(equilibria))
        if equilibrium not in equilibria:
            raise ValueError(f"Unknown equilibrium '{equilibrium}'. Available: {list(equilibria)}")
        eq = equilibria[equilibrium]      # A, B, C, D may be shared or per equilibrium
        get = lambda k: np.array(eq[k] if k in eq else spec[k], float)
        self.spec, self.equilibrium = spec, equilibrium
        self.dt = float(dt)
        self.name = spec["name"]
        self.A, self.B, self.C, self.D = get("A"), get("B"), get("C"), get("D")
        self.n, self.m, self.p = int(spec["n"]), int(spec["m"]), int(spec["p"])
        for k, shape in [("A", (self.n, self.n)), ("B", (self.n, self.m)),
                         ("C", (self.p, self.n)), ("D", (self.p, self.m))]:
            if getattr(self, k).shape != shape:
                raise ValueError(f"{k} must be {shape} for n={self.n}, m={self.m}, p={self.p}")
        self.x_eq = np.array(eq["x_eq"], float).reshape(self.n)
        self.u_eq = np.array(eq["u_eq"], float).reshape(self.m)
        self.y_eq = np.array(eq["y_eq"], float).reshape(self.p)
        self.state_names = spec["state_names"]
        self.input_names = spec["input_names"]
        self.output_names = spec["output_names"]
        self.u_limit = None if u_limit is None else np.broadcast_to(
            np.asarray(u_limit, float), (self.m,)).copy()

    def saturate(self, u):
        u = np.asarray(u, float).reshape(self.m)
        if self.u_limit is None:
            return u
        return np.clip(u, -self.u_limit, self.u_limit)

    def info(self):
        np.set_printoptions(precision=4, suppress=True)
        print(f"Dynamics '{self.name}':  n = {self.n} states, m = {self.m} inputs, "
              f"p = {self.p} outputs, dt = {self.dt} s")
        print("  x =", self.state_names)
        print("  u =", self.input_names)
        print("  y =", self.output_names)
        print(f"  equilibrium '{self.equilibrium}': x_eq =", self.x_eq, " u_eq =", self.u_eq,
              " y_eq =", self.y_eq)
        if self.u_limit is not None:
            print("  |u| <=", self.u_limit)
        for k in "ABCD":
            print(f"{k} =\n{getattr(self, k)}")


class MujocoDynamics(_Dynamics):
    """A MuJoCo model behind the common interface (students do not need this)."""

    def __init__(self, spec, dt, physics_dt=0.001, u_limit=None, equilibrium=None):
        self._setup(spec, dt, u_limit, equilibrium)
        self.substeps = int(round(dt/physics_dt))
        if self.substeps < 1 or not np.isclose(self.substeps*physics_dt, dt):
            raise ValueError(f"dt must be a multiple of the physics step {physics_dt} s")
        self.model = mujoco.MjModel.from_xml_string(spec["xml"])
        self.model.opt.timestep = physics_dt
        self.data = mujoco.MjData(self.model)
        self.physics_dt = physics_dt
        self._check_equilibrium()

    def _check_equilibrium(self):
        """Confirm with MuJoCo that (x_eq, u_eq) is an equilibrium with output y_eq."""
        self.reset(self.x_eq)
        self.data.ctrl[:] = self.u_eq
        mujoco.mj_forward(self.model, self.data)
        if np.abs(self.data.qvel).max() > 1e-9 or np.abs(self.data.qacc).max() > 1e-7:
            raise RuntimeError(f"'{self.equilibrium}' is not an equilibrium of the MuJoCo model")
        if np.abs(self.output(self.u_eq) - self.y_eq).max() > 1e-9:
            raise RuntimeError("y_eq does not match the MuJoCo sensors at the equilibrium")

    def reset(self, x):
        x = np.asarray(x, float).reshape(self.n)
        mujoco.mj_resetData(self.model, self.data)
        for value, (joint, kind) in zip(x, self.spec["state_map"]):
            getattr(self.data.joint(joint), kind)[0] = value
        mujoco.mj_forward(self.model, self.data)

    def state(self):
        return np.array([getattr(self.data.joint(j), kind)[0]
                         for j, kind in self.spec["state_map"]])

    def output(self, u):
        mujoco.mj_forward(self.model, self.data)  # refresh sensors at current state
        y = np.array([self.data.sensor(s).data[0] for s in self.spec["sensors"]])
        return y + self.D @ u

    def step(self, u):
        """Hold u for one sample period dt."""
        self.data.ctrl[:] = u
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

    def linear(self):
        return LinearDynamics(self.spec, self.dt, self.u_limit, self.equilibrium)


class LinearDynamics(_Dynamics):
    """Linearization about the equilibrium, stepped exactly over each dt (ZOH):
       x_dot = A (x - x_eq) + B (u - u_eq),   y = y_eq + C (x - x_eq) + D (u - u_eq)."""

    def __init__(self, spec, dt, u_limit=None, equilibrium=None):
        self._setup(spec, dt, u_limit, equilibrium)
        self.name = spec["name"] + " (linear)"
        self._Ad, self._Bd = discretize(self.A, self.B, self.dt)

    def reset(self, x):
        self.x = np.asarray(x, float).reshape(self.n).copy()

    def state(self):
        return self.x.copy()

    def output(self, u):
        return self.y_eq + self.C @ (self.x - self.x_eq) + self.D @ (u - self.u_eq)

    def step(self, u):
        """Hold u for one sample period dt (exact)."""
        self.x = self.x_eq + self._Ad @ (self.x - self.x_eq) + self._Bd @ (u - self.u_eq)

    def linear(self):
        return self


# ----------------------------------------------------------------------------
# The one simulation command
# ----------------------------------------------------------------------------
def simulate(dyn, x0, T, controller, observer=None, xhat0=None,
             noise_std=0.0, seed=0):
    """Simulate the closed loop for T seconds with the sample period dyn.dt.

    dyn        : from load_dynamics(...) or dyn.linear()
    x0         : initial true state (length n)
    controller : u = controller(t, z), z = xhat (or true x if observer is None)
    observer   : xhat_next = observer(t, xhat, u, y), or None for state feedback
    xhat0      : initial estimate (default x_eq; may be longer than n, e.g. with
                 integral states)
    noise_std  : standard deviation of Gaussian noise added to each output

    Returns res with arrays sampled at t_k = k*dt, k = 0..N:
      res.t (N+1,), res.x (N+1,n), res.xhat (N+1,nz), res.u (N+1,m), res.y (N+1,p)
    u[k] is the (saturated) input held over [t_k, t_k+dt); the last one is not applied.
    """
    dt = dyn.dt
    N = int(round(T/dt))
    if N < 1 or not np.isclose(N*dt, T):
        raise ValueError(f"T must be a positive integer multiple of dt = {dt}")
    rng = np.random.default_rng(seed)
    noise = np.broadcast_to(np.asarray(noise_std, float), (dyn.p,))
    dyn.reset(x0)
    xhat = dyn.x_eq.copy() if xhat0 is None else np.asarray(xhat0, float).copy()
    t = np.arange(N+1)*dt
    X = np.empty((N+1, dyn.n))
    Xh = np.full((N+1, len(xhat)), np.nan)
    U = np.empty((N+1, dyn.m))
    Y = np.empty((N+1, dyn.p))
    for k in range(N+1):
        x = dyn.state()
        z = x if observer is None else xhat
        u = np.asarray(controller(t[k], z.copy()), float).ravel()
        if u.size != dyn.m:
            raise ValueError(f"controller must return {dyn.m} input(s), got {u.size}")
        u = dyn.saturate(u)
        y = dyn.output(u) + noise*rng.standard_normal(dyn.p)
        X[k], U[k], Y[k] = x, u, y
        if observer is not None:
            Xh[k] = xhat
        if k == N:
            break
        if observer is not None:
            xhat = np.asarray(observer(t[k], xhat.copy(), u, y), float).ravel()
            if xhat.size != Xh.shape[1]:
                raise ValueError("observer must return an estimate of the same size as xhat0")
        dyn.step(u)
    if not np.isfinite(X).all():
        raise RuntimeError("Simulation diverged (non-finite state)")
    return SimpleNamespace(t=t, x=X, xhat=None if observer is None else Xh, u=U, y=Y,
                           dynamics=dyn.name, dt=dt, spec=dyn.spec,
                           state_names=dyn.state_names, input_names=dyn.input_names)


# ----------------------------------------------------------------------------
# Plotting and animation
# ----------------------------------------------------------------------------
_STYLES = [dict(color="#d18736", ls="-", lw=2), dict(color="#222222", ls=":", lw=1.6),
           dict(color="#2a9d8f", ls="-", lw=1.2), dict(color="#7a5195", ls="-", lw=1.2)]
_ESTIMATE_STYLES = [dict(color="#0072B2", ls="--"), dict(color="#7a5195", ls="-.", alpha=.65),
                    dict(color="#2a9d8f", ls="--"), dict(color="#7a5195", ls="--")]


def plot_response(results, labels=None, estimates=True, title=None):
    """Plot states (estimates dashed), inputs and the estimation error.

    results : one result from simulate(...) or a list of them (e.g. MuJoCo, linear)
    States are shown in display units (e.g. cm, deg). The bottom row shows the
    applied input(s) and, when an observer is used, the estimation error of the
    model's key state. Returns the matplotlib figure.
    """
    import matplotlib.pyplot as plt
    results = results if isinstance(results, (list, tuple)) else [results]
    labels = labels or [r.dynamics for r in results]
    disp = results[0].spec["display"]
    scale, names = np.asarray(disp["scale"]), disp["labels"]
    n = results[0].x.shape[1]
    rows = (n + 1)//2 + 1
    fig, axes = plt.subplots(rows, 2, figsize=(11, 3*rows), sharex=True)
    axes = axes.reshape(rows, 2)
    any_est = estimates and any(r.xhat is not None for r in results)
    for k, (r, lab) in enumerate(zip(results, labels)):
        st, est = _STYLES[k % len(_STYLES)], _ESTIMATE_STYLES[k % len(_ESTIMATE_STYLES)]
        for j in range(n):
            ax = axes.flat[j]
            ax.plot(r.t, r.x[:, j]*scale[j], label=f"{lab} actual", **st)
            if any_est and r.xhat is not None:
                ax.plot(r.t, r.xhat[:, j]*scale[j], label=f"{lab} estimate", **est)
        for i in range(r.u.shape[1]):
            axes[-1, 0].step(r.t, r.u[:, i], where="post", label=lab, **st)
        if any_est and r.xhat is not None:
            j = disp["error_state"]
            axes[-1, 1].plot(r.t, (r.x[:, j] - r.xhat[:, j])*scale[j], label=lab, **st)
    for j in range(n):
        axes.flat[j].set_ylabel(names[j])
    if n % 2:
        axes.flat[n].set_visible(False)
    axes[-1, 0].set_ylabel(disp["input_label"])
    if any_est:
        axes[-1, 1].set_ylabel(disp["error_label"])
        axes[-1, 1].legend(fontsize=8)
    else:
        axes[-1, 1].set_visible(False)
    for ax in axes.flat:
        ax.grid(alpha=.25)
    for ax in axes[-1]:
        ax.set_xlabel("Time [s]")
    axes[0, 0].legend(fontsize=8)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig


def animate(res, filename="sim.mp4", fps=30):
    """Render a recorded result to MP4 (replay only; nothing is re-simulated).

    In Jupyter:  from IPython.display import Video
                 Video(mjsim.animate(res), embed=True)
    """
    import imageio_ffmpeg
    spec = res.spec
    model = mujoco.MjModel.from_xml_string(spec["xml"])
    data = mujoco.MjData(model)
    width, height = spec["frame"]
    camera = mujoco.MjvCamera()
    camera.lookat[:], (camera.distance, camera.azimuth, camera.elevation) = spec["camera"]
    times = np.arange(int(round((res.t[-1]-res.t[0])*fps))+1)/fps + res.t[0]
    writer = imageio_ffmpeg.write_frames(str(filename), (width, height), fps=fps,
                                         codec="libx264", pix_fmt_out="yuv420p",
                                         macro_block_size=16, quality=8)
    writer.send(None)
    try:
        with mujoco.Renderer(model, height=height, width=width) as renderer:
            for tf in times:
                for j, (joint, kind) in enumerate(spec["state_map"]):
                    if kind == "qpos":
                        data.joint(joint).qpos[0] = np.interp(tf, res.t, res.x[:, j])
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                writer.send(renderer.render())
    finally:
        writer.close()
    return str(filename)


# ----------------------------------------------------------------------------
# Model library
# ----------------------------------------------------------------------------
def _beam_spec():
    MASS, INERTIA, G, SLIDER_INERTIA = 1.0, 2.0, 9.81, 1e-8
    xml = f'''<mujoco model="Observer-controlled mass on a beam">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option timestep="0.001" gravity="0 0 -{G}" integrator="RK4"/>
  <visual>
    <global offwidth="720" offheight="360"/>
    <quality shadowsize="2048"/>
    <headlight ambient="0.55 0.55 0.55" diffuse="0.65 0.65 0.65" specular="0.1 0.1 0.1"/>
    <rgba haze="0.96 0.97 0.99 1"/>
  </visual>
  <default>
    <joint damping="0" frictionloss="0" armature="0" limited="false"/>
    <geom contype="0" conaffinity="0"/>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient"
             rgb1="0.96 0.97 0.99" rgb2="0.96 0.97 0.99" width="256" height="256"/>
  </asset>
  <worldbody>
    <light pos="0 -1 2" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="2 2 0.01" rgba="0.89 0.91 0.94 1"/>
    <geom name="base" type="box" pos="0 0 0.025" size="0.09 0.065 0.025" rgba="0.3 0.36 0.43 1"/>
    <geom name="stand" type="cylinder" pos="0 0 0.14" size="0.018 0.09" rgba="0.42 0.48 0.55 1"/>
    <body name="beam" pos="0 0 0.25">
      <!-- A positive angle raises the right-hand end of the beam. -->
      <joint name="beam_hinge" type="hinge" axis="0 -1 0"/>
      <!-- Specified beam/drive inertia; visual dimensions do not set inertia. -->
      <inertial pos="0 0 0" mass="10" diaginertia="0.02 {INERTIA} {INERTIA}"/>
      <geom name="beam_shape" type="box" size="0.4 0.023 0.012" rgba="0.26 0.34 0.44 1"/>
      <geom name="pivot" type="sphere" size="0.018" rgba="0.25 0.3 0.37 1"/>
      <body name="slider">
        <joint name="mass_slide" type="slide" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="{MASS}"
                  diaginertia="{SLIDER_INERTIA} {SLIDER_INERTIA} {SLIDER_INERTIA}"/>
        <!-- The sphere is a visual marker above the track; the point-mass
             dynamics act on the beam centerline, with no rolling/contact. -->
        <geom name="ball" type="sphere" pos="0 0 0.035" size="0.023" rgba="0.95 0.48 0.12 1"/>
      </body>
    </body>
  </worldbody>
  <actuator><motor name="torque" joint="beam_hinge" gear="1"/></actuator>
  <sensor><jointpos name="angle" joint="beam_hinge"/></sensor>
</mujoco>'''
    return dict(
        name="beam", xml=xml, n=4, m=1, p=1,
        state_names=["d [m]", "d_dot [m/s]", "theta [rad]", "theta_dot [rad/s]"],
        input_names=["torque [N m]"], output_names=["theta [rad]"],
        state_map=[("mass_slide", "qpos"), ("mass_slide", "qvel"),
                   ("beam_hinge", "qpos"), ("beam_hinge", "qvel")],
        sensors=["angle"],
        equilibria={"level": dict(x_eq=[0, 0, 0, 0], u_eq=[0], y_eq=[0])},
        A=[[0, 1, 0, 0], [0, 0, -G, 0], [0, 0, 0, 1], [-MASS*G/INERTIA, 0, 0, 0]],
        B=[[0], [0], [0], [1/INERTIA]],
        C=[[0, 0, 1, 0]], D=[[0]],
        params=dict(mass=MASS, inertia=INERTIA, g=G),
        display=dict(scale=[100, 100, 180/np.pi, 180/np.pi],
                     labels=["Mass position [cm]", "Mass velocity [cm/s]",
                             "Beam angle [deg]", "Angular velocity [deg/s]"],
                     input_label="Applied torque [N m]",
                     error_state=0, error_label="Position estimation error [cm]"),
        camera=([0, 0, .23], (.78, 90., -15.)), frame=(720, 352))


_MODELS = {"beam": _beam_spec}


def available_dynamics():
    return sorted(_MODELS)


def load_dynamics(name, equilibrium=None, dt=0.002, u_limit=None, physics_dt=0.001):
    """Return MuJoCo dynamics with A, B, C, D linearized about an equilibrium.

    equilibrium: name of an isolated equilibrium (default: the model's first one)
    dt         : sample period of the controller/observer [s]
                 (a multiple of the 1 ms MuJoCo physics step)
    u_limit    : optional input saturation |u_i| <= u_limit
    beam       : mass sliding on a beam, x = [d, d_dot, theta, theta_dot], y = theta;
                 equilibrium "level" (x_eq = 0, u_eq = 0)
    """
    if name not in _MODELS:
        raise ValueError(f"Unknown dynamics '{name}'. Available: {available_dynamics()}")
    return MujocoDynamics(_MODELS[name](), dt, physics_dt, u_limit, equilibrium)
