"""Supplied experiments and scoring for Homework 7; no controller design here."""
import numpy as np
import mjsim

DT = 0.002
PHYSICS_DT = 0.001
TORQUE_LIMIT = 0.20
T_FINAL = 8.0
BASE_POLES = np.array([-1.0, -1.25, -1.5, -1.75])
NOMINAL_X0 = np.deg2rad([10.0, 5.0, 0.0, 0.0])
THETA_DEG = np.arange(-40.0, 41.0, 10.0)
RATE_DEG_S = np.arange(-120.0, 121.0, 40.0)
TAIL_TOL_DEG = np.array([2.0, 1.0, 5.0, 5.0])


def load_plant():
    """Use the supplied physical model and fixed experimental settings."""
    return mjsim.load_dynamics("furuta", equilibrium="upright", dt=DT,
                              u_limit=TORQUE_LIMIT, physics_dt=PHYSICS_DT)


def assess(result, local=False):
    """All angle/rate limits are checked on continuous, unwrapped coordinates.

    Local: |phi|<=2 deg, |theta|<=1 deg for every sample at t>=4 s,
           together with the fixed applied-torque bound. No local rate
           or transient-excursion limits are imposed.
    Grid:  |phi|<=120 deg, |theta|<=60 deg for the whole experiment,
           and [2,1,5,5] deg/deg/s tolerances for every sample at t>=6 s.
    Input energy uses the actually applied zero-order-held input; the last
    recorded input is not applied and is excluded from energy/saturation.
    """
    x = np.asarray(result.x)
    t = np.asarray(result.t)
    if x.shape != (len(t), 4) or result.u.shape != (len(t), 1):
        raise ValueError("Expected state (N,4) and input (N,1) histories")
    if not np.isclose(t[0], 0.) or not np.isclose(t[-1], T_FINAL):
        raise ValueError("Use the full fixed 0-to-8-second experiment")
    if not np.allclose(np.diff(t), DT):
        raise ValueError("Use the fixed 2 ms control sample period")
    finite = bool(np.isfinite(x).all() and np.isfinite(result.u).all())
    xd = np.rad2deg(x)
    peaks = np.max(np.abs(xd), axis=0)
    if local:
        inside = np.all(np.abs(xd[:, :2]) <= TAIL_TOL_DEG[:2], axis=1)
    else:
        inside = np.all(np.abs(xd) <= TAIL_TOL_DEG, axis=1)
    outside = np.flatnonzero(~inside)
    if len(outside) == 0:
        settling_time = 0.0
    elif outside[-1] == len(t) - 1:
        settling_time = float("inf")
    else:
        settling_time = float(t[outside[-1] + 1])
    start = 4.0 if local else 6.0
    tail_ok = bool(np.all(inside[t >= start - 1e-10]))
    excursion_ok = True if local else bool(peaks[0] <= 120.0 and peaks[1] <= 60.0)
    applied = result.u[:-1, 0]
    torque_ok = bool(np.max(np.abs(applied)) <= TORQUE_LIMIT + 1e-10)
    return dict(
        passed=bool(finite and tail_ok and excursion_ok and torque_ok),
        finite=finite, tail_ok=tail_ok, excursion_ok=excursion_ok,
        torque_ok=torque_ok, settling_time_s=settling_time,
        peak_phi_deg=float(peaks[0]), peak_theta_deg=float(peaks[1]),
        peak_torque_Nm=float(np.max(np.abs(applied))),
        input_energy_Nm2s=float(DT * np.sum(applied**2)),
        saturation_fraction=float(np.mean(np.abs(applied) >= TORQUE_LIMIT - 1e-9)),
        tail_max_deg=np.max(np.abs(xd[t >= start - 1e-10]), axis=0).tolist())


def evaluate_grid(dyn, controller):
    """Return mask[row=theta_dot(0), column=theta(0)] and case metrics.

    All cases start with phi(0)=phi_dot(0)=0. No angle wrapping, resetting
    during a trajectory, or termination at a favorable intermediate time.
    """
    mask = np.zeros((len(RATE_DEG_S), len(THETA_DEG)), dtype=bool)
    records = []
    for i, rate in enumerate(RATE_DEG_S):
        for j, theta in enumerate(THETA_DEG):
            x0 = np.deg2rad([0.0, theta, 0.0, rate])
            try:
                res = mjsim.simulate(dyn, x0, T_FINAL, controller, observer=None)
                score = assess(res, local=False)
            except (RuntimeError, FloatingPointError) as error:
                score = dict(passed=False, error=str(error))
            mask[i, j] = score["passed"]
            records.append(dict(theta_deg=float(theta), rate_deg_s=float(rate), **score))
    return dict(mask=mask, count=int(mask.sum()), total=int(mask.size), records=records)


def compare_recovery(baseline, candidate):
    """A sampled-set expansion retains every baseline success and adds >=10."""
    b, c = baseline["mask"], candidate["mask"]
    added = int(np.count_nonzero(c & ~b))
    lost = int(np.count_nonzero(b & ~c))
    return dict(added=added, lost=lost, target_met=bool(added >= 10 and lost == 0))


def plot_grid(result, title="Recovery tests"):
    """A success/failure map on the fixed two-dimensional initial-state slice."""
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    fig, ax = plt.subplots(figsize=(7, 4.4))
    ax.imshow(result["mask"], origin="lower", aspect="auto", interpolation="nearest",
              extent=[THETA_DEG[0]-5, THETA_DEG[-1]+5,
                      RATE_DEG_S[0]-20, RATE_DEG_S[-1]+20],
              cmap=ListedColormap(["#e0e0e0", "#0072B2"]), vmin=0, vmax=1)
    ax.set_xticks(THETA_DEG)
    ax.set_yticks(RATE_DEG_S)
    ax.set_xlabel("Initial pendulum angle theta(0) [deg]")
    ax.set_ylabel("Initial pendulum rate theta_dot(0) [deg/s]")
    ax.set_title(f"{title}: {result['count']}/{result['total']} successful")
    ax.legend(handles=[Patch(facecolor="#0072B2", label="Success"),
                       Patch(facecolor="#e0e0e0", label="Failure")],
              loc="upper center", bbox_to_anchor=(0.5, -0.20), ncol=2)
    fig.tight_layout()
    return fig
