MAE 6246 - Week 13: Ball-and-beam LQG and output tracking

Run week13_ball_beam_lqg.ipynb from top to bottom.
1. Infinite-horizon LQR and full-state stabilization.
2. Infinite-horizon Kalman filter with a fixed correction gain.
3. LQG stabilization, simulation, and animation.
4. Sinusoidal reference tracking using the new state x - xr.
5. Fixed-reference tracking as a special case.

The sinusoidal position command has amplitude 0.30 m and period 8 s.
The fixed position command is 0.10 m. The first measurement is at k=1.

In Colab, the setup cell installs missing dependencies and downloads the
companion mjsim.py automatically. Internet is needed for initial setup.
Locally, keep mjsim.py beside the notebook. Plant parameters, matrices,
equilibrium, sample period, and torque limit are supplied by mjsim.py.

The saved notebook contains executed plots and three embedded animations.
The download bundle also includes the animations as MP4 files.

Course homepage: https://fdcl-gwu.github.io/MAE6246/#week13
