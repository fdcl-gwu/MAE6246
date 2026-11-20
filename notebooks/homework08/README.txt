MAE 6246 - Homework 8
Furuta Pendulum: Kalman Filtering, LQG Stabilization, and Reference Tracking
Due Friday, November 20, 2026, at 10:00 AM Eastern Time

START HERE
1. Extract this bundle into one folder.
2. Open homework08_furuta_lqg_tracking.ipynb in Jupyter or Google Colab.
3. Keep mjsim.py and hw08_support.py beside the notebook. In Colab, upload
   both Python files into the Files panel before running the setup cell.
4. Complete the marked code and response cells. The setup cell installs
   missing packages. Use the provided wrapper, not the Homework 7 version.
5. Restart, run all cells, save with outputs, and reopen the notebook to
   confirm the sinusoidal-tracking animation remains embedded.
6. Submit the completed .ipynb through Blackboard. No separate handwritten
   PDF or video is required. The course AI supplementary-record policy applies.

FILES
homework08_furuta_lqg_tracking.ipynb - Complete assignment and starter code
mjsim.py                          - Plant parameters, matrices, MuJoCo wrapper
hw08_support.py                   - Fixed experiments, evaluation, and plots
homework08.pdf                    - One-page assignment overview

PARTS (100 points)
1. Kalman filtering (30): observability, covariance derivation, fixed gain.
2. LQG stabilization (30): LQR, nonlinear tests, assumed-noise comparison.
3. Reference tracking (40): sinusoidal trajectory, new-state feedback,
   and embedded animation.

Use H and z for sensing. The first measurement is at k=1; skip correction
at k=0. Use the same baseline gains for stabilization and tracking. Plant
parameters and matrices come from mjsim.py; the supplied experiment settings
and design weights come from hw08_support.py. Keep both supplied files unchanged.
