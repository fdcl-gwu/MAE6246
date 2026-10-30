"""MuJoCo and linear beam comparison with sampled angle-only state estimation.

Run: python beam_control.py --output output
Optional native viewer on macOS: mjpython beam_control.py --live
"""
from pathlib import Path
import argparse
import json
import os
import sys

# Colab/Linux headless rendering. Set before importing MuJoCo.
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from scipy.linalg import expm
from scipy.signal import place_poles

MASS, INERTIA, GRAVITY = 1.0, 2.0, 9.81
# MuJoCo requires a positive rotational inertia. Its effect is negligible here.
SLIDER_INERTIA = 1e-8
A = np.array([[0, 1, 0, 0], [0, 0, -GRAVITY, 0],
              [0, 0, 0, 1], [-MASS*GRAVITY/INERTIA, 0, 0, 0.]])
B = np.array([[0.], [0.], [0.], [1/INERTIA]])
C = np.array([[0., 0., 1., 0.]])
K = place_poles(A, B, [-1, -1.5, -2, -2.5]).gain_matrix
L = place_poles(A.T, C.T, [-3, -4, -5, -6]).gain_matrix.T
X0 = np.array([0.08, 0., np.deg2rad(2.), 0.])

MODEL_XML = f'''<mujoco model="Observer-controlled mass on a beam">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option timestep="0.001" gravity="0 0 -{GRAVITY}" integrator="RK4"/>
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


def new_model(physics_dt=0.001):
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    model.opt.timestep = physics_dt
    return model, mujoco.MjData(model)


def set_state(model, data, x):
    data.joint("mass_slide").qpos[0] = x[0]
    data.joint("mass_slide").qvel[0] = x[1]
    data.joint("beam_hinge").qpos[0] = x[2]
    data.joint("beam_hinge").qvel[0] = x[3]
    mujoco.mj_forward(model, data)


def read_state(data):
    return np.array([data.joint("mass_slide").qpos[0],
                     data.joint("mass_slide").qvel[0],
                     data.joint("beam_hinge").qpos[0],
                     data.joint("beam_hinge").qvel[0]])


def zero_order_hold(a, b, dt):
    """Exact discretization of xdot=a*x+b*u with held input."""
    n, m = b.shape
    augmented = np.zeros((n+m, n+m))
    augmented[:n, :n], augmented[:n, n:] = a, b
    discrete = expm(augmented*dt)
    return discrete[:n, :n], discrete[:n, n:]


def compute_control(xhat, K, torque_limit):
    """Compute u = -K*xhat and return the torque after saturation.

    This function does not read or advance the plant. For the full-state
    baseline, pass the actual state in place of the estimate.
    """
    return float(np.clip(-(K@xhat).item(), -torque_limit, torque_limit))


def update_observer(xhat, u, y, Ao, Bo):
    """Return the next estimate using held applied torque and measured angle.

    Ao, Bo discretize xhat_dot = (A-L*C)*xhat + B*u + L*y over one control
    period. Their input columns are ordered [u, y]. This function does not
    modify xhat or access the true plant state.
    """
    return Ao@xhat + Bo@np.array([u, y])


def simulate(plant="mujoco", *, x0=X0, duration=12., control_dt=0.002,
             physics_dt=0.001, torque_limit=10., feedback="observer"):
    """Use identical sampling, gains, saturation and observer on either plant.

    The observer uses only the current angle and the actual applied torque.
    Measurements and control are held during each observer update. True state
    enters feedback only when feedback='state', for the baseline check.
    """
    if plant not in ("mujoco", "linear") or feedback not in ("observer", "state"):
        raise ValueError("Invalid plant or feedback choice")
    if min(control_dt, physics_dt, duration, torque_limit) <= 0:
        raise ValueError("Timesteps, duration and torque limit must be positive")
    substeps = int(round(control_dt/physics_dt))
    steps = int(round(duration/control_dt))
    if substeps < 1 or not np.isclose(substeps*physics_dt, control_dt):
        raise ValueError("control_dt must be an integer multiple of physics_dt")
    if not np.isclose(steps*control_dt, duration):
        raise ValueError("duration must be an integer multiple of control_dt")
    times = np.arange(steps+1)*control_dt
    x = np.array(x0, dtype=float).copy()
    xhat = np.zeros(4)
    X, Xhat = np.empty((steps+1, 4)), np.empty((steps+1, 4))
    U = np.empty(steps+1)
    Ad, Bd = zero_order_hold(A, B, control_dt)
    Ao, Bo = zero_order_hold(A-L@C, np.column_stack([B, L]), control_dt)
    if plant == "mujoco":
        model, data = new_model(physics_dt)
        set_state(model, data, x)
    for j in range(steps+1):
        if plant == "mujoco":
            x = read_state(data)
        y = float((C@x).item())  # Only the measured angle is supplied to the observer.
        state_for_control = xhat if feedback == "observer" else x
        u = compute_control(state_for_control, K, torque_limit)
        X[j], Xhat[j], U[j] = x, xhat, u
        if j == steps:
            break
        next_estimate = update_observer(xhat, u, y, Ao, Bo)
        if plant == "mujoco":
            data.ctrl[0] = u
            for _ in range(substeps):
                mujoco.mj_step(model, data)
        else:
            x = Ad@x + Bd[:, 0]*u
        xhat = next_estimate
    if not np.isfinite(X).all() or not np.isfinite(Xhat).all():
        raise RuntimeError("Non-finite simulated trajectory")
    return dict(t=times, x=X, xhat=Xhat, u=U, plant=plant,
                control_dt=control_dt, physics_dt=physics_dt,
                torque_limit=torque_limit, feedback=feedback)


def engine_rhs(model, data, x, u):
    set_state(model, data, x)
    data.ctrl[0] = u
    mujoco.mj_forward(model, data)
    return np.array([x[1], data.joint("mass_slide").qacc[0],
                     x[3], data.joint("beam_hinge").qacc[0]])


def validate_model():
    """Check the engine's equilibrium, linearization and nonlinear accelerations."""
    model, data = new_model()
    eps, origin = 1e-6, np.zeros(4)
    np.testing.assert_allclose(engine_rhs(model, data, origin, 0), 0, atol=1e-12)
    Ae = np.column_stack([(engine_rhs(model, data, origin+eps*v, 0)
                         -engine_rhs(model, data, origin-eps*v, 0))/(2*eps)
                         for v in np.eye(4)])
    Be = ((engine_rhs(model, data, origin, eps)
          -engine_rhs(model, data, origin, -eps))/(2*eps))[:, None]
    np.testing.assert_allclose(Ae, A, atol=5e-8, rtol=0)
    np.testing.assert_allclose(Be, B, atol=5e-9, rtol=0)
    for x, u in [(np.array([.1, .2, .15, -.3]), .7),
                 (np.array([-.12, -.1, -.2, .4]), -.8)]:
        d, v, theta, omega = x
        expected = np.array([v, d*omega**2-GRAVITY*np.sin(theta), omega,
            (u-MASS*GRAVITY*d*np.cos(theta)-2*MASS*d*v*omega)
            /(INERTIA+SLIDER_INERTIA+MASS*d*d)])
        np.testing.assert_allclose(engine_rhs(model, data, x, u), expected, atol=1e-10)
    return {"linearization_A_max_error": float(np.max(np.abs(Ae-A))),
            "linearization_B_max_error": float(np.max(np.abs(Be-B)))}


def compare(mj, linear):
    delta = mj["x"]-linear["x"]
    return {"max_position_difference_mm": float(1000*np.max(np.abs(delta[:, 0]))),
            "max_angle_difference_deg": float(np.rad2deg(np.max(np.abs(delta[:, 2])))),
            "final_position_mm": float(1000*mj["x"][-1, 0]),
            "final_position_error_mm": float(1000*(mj["x"][-1, 0]-mj["xhat"][-1, 0])),
            "peak_torque_Nm": float(np.max(np.abs(mj["u"])))}


def plot_comparison(mj, linear):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), sharex=True)
    scale = [100, 100, 180/np.pi, 180/np.pi]
    labels = ["Mass position [cm]", "Mass velocity [cm/s]",
              "Beam angle [deg]", "Angular velocity [deg/s]"]
    for j, ax in enumerate(axes[:2].flat):
        ax.plot(mj["t"], mj["x"][:, j]*scale[j], color="#d18736", label="MuJoCo actual", lw=2)
        ax.plot(mj["t"], mj["xhat"][:, j]*scale[j], "--", color="#0072B2", label="MuJoCo observer")
        ax.plot(linear["t"], linear["x"][:, j]*scale[j], ":", color="#222222", label="Linear actual")
        ax.plot(linear["t"], linear["xhat"][:, j]*scale[j], "-.", color="#7a5195", alpha=.65,
                label="Linear observer")
        ax.set_ylabel(labels[j])
    for r, color in [(mj, "#d18736"), (linear, "#222222")]:
        style = "-" if r is mj else ":"
        axes[2, 0].plot(r["t"], 100*(r["x"][:, 0]-r["xhat"][:, 0]), style,
                        color=color, label=r["plant"])
        axes[2, 1].plot(r["t"], r["u"], style, color=color, label=r["plant"])
    axes[2, 0].set_ylabel("Position estimation error [cm]")
    axes[2, 1].set_ylabel("Applied torque [N m]")
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.set_xlabel("Time [s]")
    axes[0, 0].legend(fontsize=8)
    axes[2, 1].legend()
    fig.suptitle("Same angle-only observer and controller: MuJoCo vs. linear plant")
    fig.tight_layout()
    return fig


def run_checks(mj, linear):
    report = validate_model()
    report["baseline"] = compare(mj, linear)
    Ad, Bd = zero_order_hold(A, B, mj["control_dt"])
    Ao, Bo = zero_order_hold(A-L@C, np.column_stack([B, L]), mj["control_dt"])
    sampled = np.block([[Ad, -Bd@K], [Bo[:, 1:2]@C, Ao-Bo[:, 0:1]@K]])
    report["sampled_linear_spectral_radius"] = float(np.max(np.abs(np.linalg.eigvals(sampled))))
    assert report["sampled_linear_spectral_radius"] < 1
    # Hold the controller rate fixed while halving only the physics timestep.
    finer = simulate(physics_dt=.0005)
    report["physics_step_halving_max_position_mm"] = float(
        1000*np.max(np.abs(mj["x"][:, 0]-finer["x"][:, 0])))
    assert report["physics_step_halving_max_position_mm"] < .01
    small_mj = simulate(x0=.1*X0)
    small_lin = simulate("linear", x0=.1*X0)
    report["small_initial_condition"] = compare(small_mj, small_lin)
    # For the baseline, clipping must be inactive and the slider stays on the drawing.
    assert np.max(np.abs(mj["u"])) < mj["torque_limit"]
    assert np.max(np.abs(mj["x"][:, 0])) < .4
    assert abs(mj["x"][-1, 0]) < .001
    assert np.max(np.abs(mj["x"][-1]-mj["xhat"][-1])) < .001
    for plant in ["mujoco", "linear"]:
        full = simulate(plant, feedback="state")
        assert np.max(np.abs(full["x"][-1])) < .001
    report["full_state_baselines"] = "Both plants converge"
    report["mujoco_version"] = mujoco.__version__
    return report


def add_estimate_ring(scene, data, dhat):
    """Draw a massless blue ring on the actual beam, using only estimated d."""
    body = data.body("beam")
    rotation = body.xmat.reshape(3, 3)
    center = body.xpos + rotation @ np.array([dhat, 0, .035])
    # The ring surrounds the orange ball when the position estimate converges.
    phis = np.linspace(0, 2*np.pi, 41)
    points = [center + rotation@np.array([.028*np.cos(p), 0, .028*np.sin(p)])
              for p in phis]
    for a, b in zip(points[:-1], points[1:]):
        if scene.ngeom >= scene.maxgeom:
            raise RuntimeError("Insufficient space for observer display geometry")
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                           np.zeros(3), np.zeros(3), np.eye(3).ravel(),
                           np.array([0., .45, .85, 1.], dtype=np.float32))
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, .0022, a, b)
        scene.ngeom += 1


def make_camera():
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .23]
    camera.distance, camera.azimuth, camera.elevation = .78, 90., -15.
    return camera


def render_comparison(mj, linear, output, fps=20, make_gif=True):
    """Native MuJoCo rendering of both trajectories; no physics is re-simulated.

    The right-hand scene replays the LINEAR trajectory in the same geometry.
    Estimated positions are display-only geoms, never physical bodies.
    """
    from PIL import Image, ImageDraw, ImageFont
    import matplotlib
    import imageio_ffmpeg
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    font_path = Path(matplotlib.get_data_path()) / "fonts/ttf/DejaVuSans.ttf"
    title_font = ImageFont.truetype(str(font_path), 22)
    panel_font = ImageFont.truetype(str(font_path), 18)
    small_font = ImageFont.truetype(str(font_path), 15)
    model, data = new_model()
    camera = make_camera()
    frame_times = np.arange(int(round(mj["t"][-1]*fps))+1)/fps
    video_path = output / "week10_mujoco_comparison.mp4"
    writer = imageio_ffmpeg.write_frames(str(video_path), (960, 480), fps=fps,
                codec="libx264", pix_fmt_out="yuv420p", quality=8,
                macro_block_size=16, output_params=["-movflags", "+faststart"])
    writer.send(None)
    gif_frames = []
    try:
        with mujoco.Renderer(model, height=300, width=480) as renderer:
            for frame_number, time in enumerate(frame_times):
                frame = Image.new("RGB", (960, 480), "#f5f7fb")
                draw = ImageDraw.Draw(frame)
                draw.text((480, 17), "Angle-only observer and feedback", fill="#25354a",
                           font=title_font, anchor="mm")
                draw.text((480, 44), f"t = {time:4.2f} s  |  same gains and sampling on both plants",
                           fill="#42556d", font=small_font, anchor="mm")
                for panel, (result, title) in enumerate([(mj, "MuJoCo nonlinear physics"),
                                                        (linear, "Linear model (replayed)")]):
                    x = np.array([np.interp(time, result["t"], result["x"][:, j]) for j in range(4)])
                    dhat = np.interp(time, result["t"], result["xhat"][:, 0])
                    u = np.interp(time, result["t"], result["u"])
                    set_state(model, data, x)
                    renderer.update_scene(data, camera=camera)
                    add_estimate_ring(renderer.scene, data, dhat)
                    frame.paste(Image.fromarray(renderer.render()), (480*panel, 86))
                    cx = 240+480*panel
                    draw.text((cx, 72), title, fill="#25354a", font=panel_font, anchor="mm")
                    draw.text((cx, 399), f"actual {100*x[0]:+5.1f} cm   estimate {100*dhat:+5.1f} cm",
                               fill="#25354a", font=small_font, anchor="mm")
                    draw.text((cx, 422), f"error {100*(x[0]-dhat):+5.1f} cm   torque {u:+5.2f} N m",
                               fill="#42556d", font=small_font, anchor="mm")
                draw.line((480, 62, 480, 434), fill="#c5cfdb", width=1)
                draw.ellipse((252, 449, 266, 463), fill="#f27b1e")
                draw.text((277, 456), "Actual position", fill="#25354a", font=small_font, anchor="lm")
                draw.ellipse((474, 448, 490, 464), outline="#0072b2", width=3)
                draw.text((501, 456), "Observer estimate", fill="#25354a", font=small_font, anchor="lm")
                writer.send(np.asarray(frame))
                if make_gif:
                    gif_frames.append(frame)
                if frame_number in [0, 4, 30, len(frame_times)-1]:
                    frame.save(output / f"frame_{frame_number:03d}.png")
    finally:
        writer.close()
    if make_gif:
        gif_frames[0].save(output / "week10_mujoco_comparison.gif", save_all=True,
                          append_images=gif_frames[1:], duration=1000//fps, loop=0,
                          optimize=False)
    return video_path


def run_live(duration=60., torque_limit=10.):
    """Native interactive demonstration; uses engine angle sensor only."""
    import time
    import mujoco.viewer
    model, data = new_model()
    set_state(model, data, X0)
    xhat = np.zeros(4)
    dt = model.opt.timestep
    Ao, Bo = zero_order_hold(A-L@C, np.column_stack([B, L]), dt)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, .23]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .78, 90, -15
        while viewer.is_running() and data.time < duration:
            start = time.monotonic()
            # Refresh sensor after mj_step, whose derived quantities can lag qpos.
            mujoco.mj_forward(model, data)
            y = float(data.sensor("angle").data[0])
            u = compute_control(xhat, K, torque_limit)
            next_estimate = update_observer(xhat, u, y, Ao, Bo)
            data.ctrl[0] = u
            mujoco.mj_step(model, data)
            xhat = next_estimate
            mujoco.mj_forward(model, data)
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                add_estimate_ring(viewer.user_scn, data, xhat[0])
            viewer.sync()
            time.sleep(max(0., dt-(time.monotonic()-start)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output"))
    parser.add_argument("--no-render", action="store_true", help="Run numerical checks and plots only")
    parser.add_argument("--live", action="store_true", help="Open the native interactive viewer")
    args = parser.parse_args()
    if args.live:
        run_live()
        return
    args.output.mkdir(parents=True, exist_ok=True)
    mj, linear = simulate(), simulate("linear")
    report = run_checks(mj, linear)
    print(json.dumps(report, indent=2))
    (args.output / "validation.json").write_text(json.dumps(report, indent=2))
    (args.output / "beam.xml").write_text(MODEL_XML)
    np.savez_compressed(args.output / "trajectories.npz", t=mj["t"],
                        mujoco_x=mj["x"], mujoco_xhat=mj["xhat"], mujoco_u=mj["u"],
                        linear_x=linear["x"], linear_xhat=linear["xhat"], linear_u=linear["u"])
    fig = plot_comparison(mj, linear)
    fig.savefig(args.output / "comparison.png", dpi=140)
    import matplotlib.pyplot as plt
    plt.close(fig)
    if not args.no_render:
        print("Video:", render_comparison(mj, linear, args.output))


if __name__ == "__main__":
    main()
