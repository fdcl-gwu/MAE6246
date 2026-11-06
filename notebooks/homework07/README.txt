MAE 6246 - Homework 7
LQR by Hand and Furuta Pendulum Control

Due: Friday, November 13, 2026, 10:00 AM Eastern Time.
Late penalty: 1 point per 10 minutes late.

1. Read homework07.pdf for Part 1. The notebook contains the complete
   Part 2 statement, specifications, experiments, and response fields.
2. Complete Part 1 by hand and scan it as one clearly legible PDF.
3. Keep homework07_furuta_control.ipynb, mjsim.py, and hw07_support.py
   together. Open the notebook in Jupyter and complete the marked choices
   and written responses. The setup cell installs missing dependencies.
4. In Colab, open the notebook and run the setup cell. It downloads
   mjsim.py and hw07_support.py automatically if they are missing.
5. Restart the notebook kernel, run all cells, save the outputs, and reopen
   the saved notebook to confirm that the animation is embedded.
6. Submit the handwritten Part 1 PDF and the completed Part 2 IPYNB
   through Blackboard. No separate video submission is required.

This Furuta-enabled mjsim.py uses the same interface as the Week 11
MuJoCo example, including make_controller(dyn, K). Keep the supplied
model and fixed evaluation criteria.
Numerical simulation does not need a graphics context; rendering does.
The wrapper uses EGL on headless Linux/Colab and was checked with
MuJoCo 3.17.0.
