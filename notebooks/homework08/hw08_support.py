"""Homework 8: supplied experiments, plots, and criteria; no gain/reference design."""
import numpy as np
import mjsim

SEED = 6248
SENSOR_STD = np.deg2rad([0.20, 0.10])
R_SENSOR = np.diag(SENSOR_STD**2)
QE = np.diag([0., 0., 1e-4, 1e-4])
QC = np.diag([1., 10., 0.1, 0.1])
RC = np.array([[100.]])
STABILIZATION_X0 = np.deg2rad([10., 5., 0., 0.])
T_STABILIZATION = 8.0
REFERENCE_AMPLITUDE = np.deg2rad(30.)
REFERENCE_PERIOD = 6.0
OMEGA = 2*np.pi/REFERENCE_PERIOD
T_SINUSOID = 3*REFERENCE_PERIOD


def load_plant():
    """Physical parameters, matrices, timing, and torque bound are in mjsim.py."""
    return mjsim.load_dynamics('furuta', equilibrium='upright')


def run(dyn, controller, observer=None, experiment='stabilization'):
    """Use the fixed initial conditions, actual sensor noise, and first z_1."""
    durations = {'stabilization':T_STABILIZATION, 'sinusoid':T_SINUSOID}
    if experiment not in durations:
        raise ValueError(experiment)
    x0 = STABILIZATION_X0 if experiment == 'stabilization' else dyn.x_eq.copy()
    return mjsim.simulate(dyn, x0, durations[experiment], controller,
                         observer=observer, xhat0=dyn.x_eq.copy(),
                         noise_std=SENSOR_STD, seed=SEED, first_measurement=1)


def assess(result, experiment='stabilization'):
    """Evaluate the prescribed true-state target, independently of student reference code.

    All angles are unwrapped. Torque uses held commands, excluding the terminal
    logged command. Tail angle criteria use every sample, not just the endpoint.
    No statistical coverage claim is made for this fixed-seed nonlinear trial.
    """
    durations = {'stabilization':T_STABILIZATION, 'sinusoid':T_SINUSOID}
    if experiment not in durations:raise ValueError(experiment)
    t, x, u = np.asarray(result.t), np.asarray(result.x), np.asarray(result.u)
    if x.shape != (len(t),4) or u.shape != (len(t),1):
        raise ValueError('Expected state (N,4) and input (N,1) histories')
    if not np.isclose(t[0],0) or not np.isclose(t[-1],durations[experiment]):
        raise ValueError('Run the full prescribed experiment duration')
    if not np.allclose(np.diff(t),result.dt):raise ValueError('Nonuniform sample times')
    finite = bool(np.isfinite(x).all() and np.isfinite(u).all())
    limit = 0.20  # displayed assignment evaluation criterion; model applies this bound
    peak_u = float(np.max(np.abs(u[:-1])))
    torque_ok = peak_u <= limit+1e-10
    if experiment == 'sinusoid':
        ref = REFERENCE_AMPLITUDE*np.sin(OMEGA*t)
        window = t >= REFERENCE_PERIOD-1e-10
        arm_value = float(np.rad2deg(np.sqrt(np.mean((x[window,0]-ref[window])**2))))
        pend_value = float(np.rad2deg(np.max(np.abs(x[:,1]))))
        arm_limit, pend_limit = 1.0, 5.0
        arm_label, pend_label = 'Arm RMSE, 6-18 s [deg]', 'Peak absolute theta, 0-18 s [deg]'
    else:
        ref = 0.0
        window = t >= 4.0-1e-10
        arm_value = float(np.rad2deg(np.max(np.abs(x[window,0]-ref))))
        pend_value = float(np.rad2deg(np.max(np.abs(x[window,1]))))
        arm_limit, pend_limit = 2.0, 1.0
        arm_label, pend_label = 'Max arm error, 4-8 s [deg]', 'Max absolute theta, 4-8 s [deg]'
    error_rmse = None
    if result.xhat is not None:
        error_rmse = np.rad2deg(np.sqrt(np.mean((result.xhat[window]-x[window])**2,axis=0))).tolist()
    return dict(experiment=experiment, passed=bool(finite and torque_ok and arm_value<=arm_limit and pend_value<=pend_limit),
                finite=finite, arm_value=arm_value, arm_limit=arm_limit, arm_label=arm_label,
                pend_value=pend_value, pend_limit=pend_limit, pend_label=pend_label,
                peak_torque_Nm=peak_u, torque_limit_Nm=limit, estimation_rmse=error_rmse)


def results_table(records, experiment='stabilization'):
    """Show each measured result beside the actual fixed constraints."""
    from IPython.display import display, Markdown
    metrics = {name:assess(res,experiment) for name,res in records.items()}
    first=next(iter(metrics.values()))
    lines=[f"| Design | {first['arm_label']} | {first['pend_label']} | Peak torque [N m] | Pass |",
           '|---|---:|---:|---:|:---:|',
           f"| **Required limits** | **<= {first['arm_limit']:.2f}** | **<= {first['pend_limit']:.2f}** | **<= 0.200** | |"]
    for name,m in metrics.items():
        lines.append(f"| {name} | {m['arm_value']:.3f} | {m['pend_value']:.3f} | {m['peak_torque_Nm']:.4f} | {m['passed']} |")
    display(Markdown('\n'.join(lines)))
    return metrics


def estimation_table(records):
    """No pass threshold: compare rate estimation under the same actual noise."""
    from IPython.display import display, Markdown
    lines=['| Filter | Arm-rate RMSE, 4-8 s [deg/s] | Pendulum-rate RMSE, 4-8 s [deg/s] |',
           '|---|---:|---:|']
    for name,res in records.items():
        rmse=assess(res,'stabilization')['estimation_rmse']
        if rmse is None:raise ValueError('An observer result is required')
        lines.append(f'| {name} | {rmse[2]:.3f} | {rmse[3]:.3f} |')
    display(Markdown('\n'.join(lines)))


def plot_tracking(result, reference, title):
    """True tracking error for evaluation; the controller cannot read this state."""
    import matplotlib.pyplot as plt
    samples=[reference(tk) for tk in result.t]
    xr=np.array([a[0] for a in samples]);ur=np.array([a[1] for a in samples])
    errors=np.rad2deg(result.x-xr)
    fig,axes=plt.subplots(3,2,figsize=(11,8),sharex=True,layout='constrained')
    labels=['Arm angle [deg]','Pendulum angle [deg]','Arm rate [deg/s]','Pendulum rate [deg/s]']
    for j in range(4):
        axes.flat[j].plot(result.t,errors[:,j],color='#2366b1')
        axes.flat[j].axhline(0,color='0.4',lw=.7)
        axes.flat[j].set_ylabel('Tracking error\n'+labels[j])
    axes[2,0].plot(result.t,np.rad2deg(result.x[:,0]),label='Actual arm angle')
    axes[2,0].plot(result.t,np.rad2deg(xr[:,0]),'k--',label='Reference')
    axes[2,1].plot(result.t[:-1],result.u[:-1,0],label='Applied torque')
    axes[2,1].plot(result.t[:-1],ur[:-1,0],'k--',label='Reference torque')
    axes[2,0].set_ylabel('Arm angle [deg]');axes[2,1].set_ylabel('Torque [N m]')
    for ax in axes.flat:ax.grid(alpha=.25)
    for ax in axes[-1]:ax.set_xlabel('Time [s]');ax.legend(fontsize=8)
    fig.suptitle(title)
    return fig
