---
title: "MAE 6246 - Linear Systems and Control"
subtitle: "Week 6: PBH Tests, Observability, and State Reconstruction"
author: "Fall 2026"
date: "October 2, 2026"
geometry: margin=0.78in
fontsize: 11pt
toc: true
toc-depth: 1
numbersections: true
colorlinks: true
linkcolor: blue
urlcolor: blue
header-includes:
  - \usepackage{amsmath,amssymb,bm}
  - \usepackage{booktabs}
  - \usepackage{microtype}
  - \usepackage{needspace}
---

**Lecture:** Friday, October 2, 2026

**Main question:** Which motions can an actuator influence, and which motions can a sensor reveal?

We begin with the controllability rank condition from Week 5, introduce its modal interpretation through the PBH test, and then develop observability from measured output histories. The force-driven mass from the legacy notes connects the two topics.

**Scope:** controllability PBH; observability and indistinguishable states; discrete-time reconstruction; continuous-time observability; duality; observability PBH. The final sensor-sensitivity example is optional. Gramians, canonical decompositions, and minimal realization are deferred so that Week 7 can emphasize midterm review.

\clearpage

# From controllability to a modal test

## Recall the rank condition

For $\dot x=Ax+Bu$, with $x\in\mathbb R^n$ and $u\in\mathbb R^m$, the controllability matrix is

$$\mathcal C=[B,\ AB,\ \ldots,\ A^{n-1}B].$$

The pair $(A,B)$ is controllable if and only if $\operatorname{rank}\mathcal C=n$. Its columns describe directions the input can influence, directly through $B$ and through the dynamics. From zero initial state, the reachable states lie in the controllable subspace $\operatorname{im}\mathcal C$. A rank-deficient matrix can still allow some target states to be reached, as in Homework 4.

The same matrix test applies to $x_{k+1}=A_dx_k+B_du_k$, using $A_d,B_d$. Here controllability means that arbitrary states can be reached by choosing a finite input sequence.

## A coordinate the input cannot influence

The **Popov-Belevitch-Hautus (PBH) test** asks whether an eigenmode is disconnected from the input. A nonzero row $w^*$ is a left eigenvector when

$$w^*A=\lambda w^*.$$

The star denotes conjugate transpose; for real vectors it is just transpose. Eigenvalues and eigenvectors may be complex even when the system matrices are real.

Define the scalar modal coordinate $z=w^*x$. Then

$$\dot z=w^*(Ax+Bu)=\lambda z+w^*Bu.$$

If $w^*B=0$, the input disappears:

$$\boxed{\dot z=\lambda z,\qquad z(t)=e^{\lambda t}z(0).}$$

No choice of input can change this coordinate independently of its free evolution. In particular, from $x_0=0$ we always have $w^*x(t)=0$, so not every target can be reached. For a real left eigenvector, the reachable states remain in the hyperplane perpendicular to $w$.

The obstruction also appears in the controllability matrix:

$$w^*A^jB=\lambda^j w^*B=0\quad\Longrightarrow\quad w^*\mathcal C=0.$$

**PBH eigenvector criterion.** The pair $(A,B)$ is controllable if and only if there is no nonzero left eigenvector of $A$ satisfying $w^*B=0$. The argument above explains why such a vector prevents controllability; the converse is part of the PBH theorem.

For discrete time, the same reasoning gives $z_{k+1}=\lambda z_k+w^*B_du_k$. The test is unchanged in form. Stability and controllability remain separate questions.

\clearpage

# The controllability PBH rank test

## Statement and use

The eigenvector criterion can be written as a rank test:

$$\boxed{(A,B)\text{ controllable}\quad\Longleftrightarrow\quad
\operatorname{rank}[\lambda I-A,\ B]=n
\quad\text{for every }\lambda\in\mathbb C.}$$

The matrix has $n$ rows and $n+m$ columns. It has deficient row rank precisely when a nonzero row $w^*$ satisfies

$$w^*[\lambda I-A,\ B]=0
\quad\Longleftrightarrow\quad w^*A=\lambda w^*,\quad w^*B=0.$$

Only the **distinct eigenvalues of $A$** need to be checked: for any other $\lambda$, the block $\lambda I-A$ is already invertible. Evaluate ranks over $\mathbb C$ when necessary. The test does not require $A$ to be diagonalizable.

## Legacy example: a force-driven mass

Let $m>0$, $x=[q,v]^T$, and $m\ddot q=u$. Then

$$A=\begin{bmatrix}0&1\\0&0\end{bmatrix},\qquad
B=\begin{bmatrix}0\\1/m\end{bmatrix}.$$

The only distinct eigenvalue is zero, although its algebraic multiplicity is two. At $\lambda=0$,

$$[\lambda I-A,\ B]=\begin{bmatrix}0&-1&0\\0&0&1/m\end{bmatrix},$$

which has rank two. Thus the system is controllable. Equivalently, its left eigenvectors are $w^*=\alpha[0,1]$, $\alpha\ne0$, and $w^*B=\alpha/m\ne0$.

This agrees with

$$\mathcal C=\begin{bmatrix}0&1/m\\1/m&0\end{bmatrix},\qquad \det\mathcal C=-1/m^2\ne0.$$

Although the zero-input mass is unstable, its position and velocity can both be controlled. An actuator need not act directly on every state.

## A failed test and a repeated-eigenvalue caution

For $A=\operatorname{diag}(-1,-2)$ and $B=[1,0]^T$, the row $w^*=[0,1]$ satisfies $w^*A=-2w^*$ and $w^*B=0$. Therefore the system is not controllable: $\dot x_2=-2x_2$ is unaffected by the input. A stable mode can be uncontrollable.

PBH must hold for **every nonzero vector in each left eigenspace**, not merely for selected basis vectors. For example, with $A=-I_2$ and $B=[1,1]^T$, both coordinate eigenvectors have nonzero products with $B$, but $w^*=[1,-1]$ has $w^*B=0$. The rank test catches this immediately: $[\lambda I-A,B]=[0,B]$ has rank one at $\lambda=-1$.

\clearpage

# Observability from output histories

## What does a sensor tell us?

Controllability asks whether inputs can move the state. **Observability** asks whether known inputs and measured outputs determine the initial state uniquely over a finite observation interval. We assume the model is known and begin with exact, noiseless data.

Two initial states are **indistinguishable** if, under the same known input, they produce identical outputs throughout that interval. Observability rules out distinct indistinguishable initial states. Knowing the initial state and input then determines the later state through the state equation.

A single measurement need not reveal the whole state. For a mass, measuring position once does not determine velocity; measuring how position changes can.

## Expand the discrete-time outputs

Consider

$$x_{k+1}=A_dx_k+B_du_k,\qquad y_k=Cx_k+Du_k,$$

where $y_k\in\mathbb R^p$. Expanding the first three outputs gives

$$\begin{aligned}
y_0&=Cx_0+Du_0,\\
y_1&=CA_dx_0+CB_du_0+Du_1,\\
y_2&=CA_d^2x_0+CA_dB_du_0+CB_du_1+Du_2.
\end{aligned}$$

All input-dependent terms are known. Define the corrected measurements

$$\widetilde y_k=y_k-Du_k-\sum_{j=0}^{k-1}CA_d^{k-1-j}B_du_j
=CA_d^kx_0,$$

where the sum is empty for $k=0$. Stacking $N$ measurements gives

$$\underbrace{\begin{bmatrix}\widetilde y_0\\\widetilde y_1\\\vdots\\\widetilde y_{N-1}\end{bmatrix}}_{Y_N}
=\underbrace{\begin{bmatrix}C\\CA_d\\\vdots\\CA_d^{N-1}\end{bmatrix}}_{\mathcal O_N}x_0.$$

The unknown $x_0$ is unique exactly when $\mathcal O_N$ has full column rank. For an $n$-state LTI system, Cayley-Hamilton implies that powers beyond $A_d^{n-1}$ add no new row directions. Write $\mathcal O_d=\mathcal O_n$, the observability matrix with $np$ rows and $n$ columns. Then

$$\boxed{(A_d,C)\text{ observable}\ \Longleftrightarrow\ \operatorname{rank}\mathcal O_d=n.}$$
 If $\eta\in\ker\mathcal O_d$, then $x_0$ and $x_0+\eta$ give the same entire output sequence under the same input. Thus $\ker\mathcal O_d$ is the **unobservable subspace**. More measurements cannot remove a genuinely unobservable direction.

\clearpage

# Reconstructing the state of the mass

## Position measurements

With a held force over sampling intervals of length $h>0$, the exact model is

$$A_d=\begin{bmatrix}1&h\\0&1\end{bmatrix},\qquad
B_d=\begin{bmatrix}h^2/(2m)\\h/m\end{bmatrix}.$$

For the position sensor $y_k=q_k$, take $C_q=[1,0]$ and $D=0$. The first two measurements satisfy

$$y_0=q_0,\qquad y_1=q_0+hv_0+\frac{h^2}{2m}u_0.$$

Therefore

$$\mathcal O_d=\begin{bmatrix}1&0\\1&h\end{bmatrix},\qquad \det\mathcal O_d=h\ne0,$$

and the reconstruction is

$$\boxed{q_0=y_0,\qquad
v_0=\frac{y_1-y_0}{h}-\frac{h}{2m}u_0.}$$

The force contribution must be subtracted: output changes reflect both the unknown initial velocity and the known applied force.

**Worked example.** Let $m=2$, $h=0.5$, $u_0=4$, and measure $y_0=1$, $y_1=2$. Then

$$q_0=1,\qquad v_0=\frac{2-1}{0.5}-\frac{0.5}{4}(4)=1.5.$$

The state update gives $x_1=[2,2.5]^T$, whose first component agrees with $y_1=2$.

## Velocity measurements

For $y_k=v_k$, take $C_v=[0,1]$. Now

$$\mathcal O_d=\begin{bmatrix}0&1\\0&1\end{bmatrix},\qquad
\operatorname{rank}\mathcal O_d=1.$$

An unknown initial position offset never affects velocity. For example, $[0,1]^T$ and $[5,1]^T$ produce identical velocity histories under any common known input. Their difference lies in $\ker\mathcal O_d=\operatorname{span}\{[1,0]^T\}$.

**Interpretation.** Position sensing reveals velocity through the dynamics, while velocity sensing cannot reveal absolute position. Observability depends on the pair $(A_d,C)$, not simply on the number of sensors. Choosing a different known input does not eliminate this ambiguity in an LTI model: the input terms cancel when comparing two initial states.

\clearpage

# Continuous-time observability

## Why the same matrix appears

For $\dot x=Ax+Bu$, $y=Cx+Du$, separate the known forced response from the output:

$$\widetilde y(t)=y(t)-Du(t)-C\int_0^t e^{A(t-\tau)}Bu(\tau)\,d\tau
=Ce^{At}x_0.$$

Thus observability depends on $(A,C)$; the known input and direct feedthrough can be removed. For this corrected, zero-input output,

$$\widetilde y(0)=Cx_0,\qquad
\dot{\widetilde y}(0)=CAx_0,\qquad
\widetilde y^{(j)}(0)=CA^jx_0.$$

This motivates the continuous-time observability matrix and theorem:

$$\boxed{\mathcal O=\begin{bmatrix}C\\CA\\\vdots\\CA^{n-1}\end{bmatrix},\qquad
(A,C)\text{ observable}\ \Longleftrightarrow\ \operatorname{rank}\mathcal O=n.}$$

For an LTI system, this criterion gives uniqueness from exact continuous measurements over any interval of positive length. The series for $e^{At}$ and Cayley-Hamilton connect the derivative conditions to the entire output history. In particular,

$$\mathcal O\eta=0\quad\Longleftrightarrow\quad Ce^{At}\eta=0\text{ for every }t.$$

Derivatives explain the mathematics; they are not a recommendation to repeatedly differentiate noisy measurements. In computation, use a model-based reconstruction from output samples.

## The legacy sensor comparison in continuous time

For the same force-driven mass,

$$A=\begin{bmatrix}0&1\\0&0\end{bmatrix},\qquad
\mathcal O_q=\begin{bmatrix}1&0\\0&1\end{bmatrix},\qquad
\mathcal O_v=\begin{bmatrix}0&1\\0&0\end{bmatrix}.$$

Position sensing is observable, and velocity-only sensing is not. In physical terms, $y=q$ gives $\dot y=v$, whereas $y=v$ gives $\dot y=u/m$, which provides no information about $q_0$.

The observability rank test is the same algebraic construction in continuous and discrete time, but the matrix used must match the model: use $A$ for continuous measurements and $A_d$ for sampled measurements. Sampling can sometimes lose information even when the continuous-time pair is observable; an optional example appears later.

**Check your understanding.** The mass is controllable with a force input and observable with a position sensor, yet its zero-input origin is unstable. Stability concerns $A$; controllability concerns $(A,B)$; observability concerns $(A,C)$. None of these properties alone implies the others.

\clearpage

# Duality and the observability PBH test

## Transposing the controllability matrix

For real system matrices, form the dual pair $(A^T,C^T)$. Its controllability matrix is

$$\mathcal C(A^T,C^T)
=[C^T,\ A^TC^T,\ldots,(A^T)^{n-1}C^T]=\mathcal O(A,C)^T.$$

Transpose preserves rank, so

$$\boxed{(A,C)\text{ observable}\quad\Longleftrightarrow\quad
(A^T,C^T)\text{ controllable}.}$$

The same result holds with $A_d$ in discrete time. Duality relates two matrix pairs; it does not say that a controllable plant is observable for an arbitrary choice of sensor.

## Eigenvector and rank forms

Applying controllability PBH to the dual pair yields

$$\boxed{(A,C)\text{ observable}\quad\Longleftrightarrow\quad
\operatorname{rank}\begin{bmatrix}\lambda I-A\\C\end{bmatrix}=n
\text{ for every eigenvalue }\lambda\text{ of }A.}$$

The stacked matrix has $n+p$ rows and $n$ columns. It fails full column rank exactly when there is a nonzero **right eigenvector** $v$ satisfying

$$\boxed{Av=\lambda v,\qquad Cv=0.}$$

Such a mode is invisible: its zero-input motion is $x(t)=e^{\lambda t}v$, but its output is $y(t)=e^{\lambda t}Cv=0$. For a complex eigenvector in a real system, the real and imaginary parts give real invisible motions. In discrete time the same statement uses $x_k=\lambda^kv$.

## Apply PBH to the two sensors

For the mass, $\lambda=0$ is the only distinct eigenvalue and the right eigenspace is $\operatorname{span}\{[1,0]^T\}$. The position sensor gives $C_qv\ne0$ for every nonzero vector in that eigenspace, while the velocity sensor gives $C_vv=0$.

The rank tests make the same distinction:

$$\begin{bmatrix}-A\\C_q\end{bmatrix}
=\begin{bmatrix}0&-1\\0&0\\1&0\end{bmatrix}\quad(\text{rank }2),\qquad
\begin{bmatrix}-A\\C_v\end{bmatrix}
=\begin{bmatrix}0&-1\\0&0\\0&1\end{bmatrix}\quad(\text{rank }1).$$

**Remember the direction:** controllability uses left eigenvectors and $w^*B$; observability uses right eigenvectors and $Cv$. For repeated eigenvalues, checking selected eigenvectors individually is insufficient if a linear combination in the same eigenspace is invisible. The PBH rank form avoids that ambiguity.

\clearpage

# Optional extension: uniqueness versus reliable reconstruction

## Small measurement errors can produce large state errors

For the sampled mass, suppose the two position measurements contain errors $e_0,e_1$, while the input and model are exact. The reconstruction formula gives

$$\widehat q_0-q_0=e_0,\qquad
\widehat v_0-v_0=\frac{e_1-e_0}{h}.$$

If $|e_0|,|e_1|\le\varepsilon$, the velocity error can be as large as $2\varepsilon/h$. Thus $\det\mathcal O_d=h\ne0$ guarantees uniqueness for any $h>0$, but a very short two-sample observation window can make velocity reconstruction sensitive to noise. This does not mean that faster sampling is inherently worse: collecting more samples over a fixed time window changes the comparison.

For a longer corrected measurement stack, solve

$$\widehat x_0=\arg\min_x\|\mathcal O_Nx-Y_N\|_2^2.$$

With full column rank, the estimate is unique. Use QR or SVD numerically. A small minimum singular value indicates a direction that produces little output and can be difficult to reconstruct. Compare singular values only with consistent state and output scaling; numerical sensitivity depends on units as well as sensor choice.

## An oscillator can become unobservable after sampling

Let $\omega>0$ and

$$A=\begin{bmatrix}0&1\\-\omega^2&0\end{bmatrix},\qquad C=[1,0].$$

In continuous time, $\mathcal O=[C;CA]=I_2$, so position measurements reveal both initial position and velocity. After exact sampling,

$$A_d=e^{Ah}=\begin{bmatrix}
\cos\omega h&\sin(\omega h)/\omega\\
-\omega\sin\omega h&\cos\omega h
\end{bmatrix},$$

and

$$\mathcal O_d=\begin{bmatrix}1&0\\\cos\omega h&\sin(\omega h)/\omega\end{bmatrix},\qquad
\det\mathcal O_d=\frac{\sin\omega h}{\omega}.$$

If $h=k\pi/\omega$ for a positive integer $k$, the sampled pair is unobservable: sampling every half-period or an integer multiple misses the initial-velocity contribution. For $h=\pi/(2\omega)$, the second position sample is $v_0/\omega$ in the zero-input case, so the two samples reveal both states.

This example distinguishes exact loss of observability from sensitivity near a poorly chosen sampling interval. It is an extension for discussion, not a prerequisite for the core rank and PBH tests.

\clearpage

# Review guide and practice

## The tests in one place

For continuous-time matrices $(A,B,C)$, or with $A_d,B_d$ substituted for a discrete-time model:

| Property | Matrix test | Equivalent modal obstruction |
|---|---|---|
| Controllability | $\operatorname{rank}[B,AB,\ldots,A^{n-1}B]=n$ | A left eigenvector with $w^*B=0$ |
| Observability | $\operatorname{rank}[C;CA;\ldots;CA^{n-1}]=n$ | A right eigenvector with $Cv=0$ |

The corresponding PBH matrices are

$$\underbrace{[\lambda I-A,\ B]}_{\text{full row rank }n},\qquad
\underbrace{\begin{bmatrix}\lambda I-A\\C\end{bmatrix}}_{\text{full column rank }n}.$$

Check every distinct eigenvalue, including stable eigenvalues. These are tests of controllability and observability, not tests of stability. Use the rank form when repeated eigenvalues make an eigenvector argument awkward.

## Questions for Week 7 review

1. For $A=\operatorname{diag}(-1,-2)$ and $B=[1,0]^T$, identify the uncontrollable mode using PBH. Does its decay make the pair controllable?
2. For the same $A$ and $C=[1,1]$, determine observability. Explain why one output can reveal two states.
3. For $A=-I_2$ and $C=[1,1]$, find a nonzero invisible eigenvector. Why does measuring both state components through their sum not suffice?
4. For the force-driven mass, compare position-only and velocity-only sensing using both the observability matrix and PBH.
5. For the sampled mass with $m=h=1$, $u_0=2$, $y_0=1$, and $y_1=4$, reconstruct $x_0$ from position measurements.
6. Why can known input terms be subtracted in the observability calculation? Can a different known input distinguish initial states whose difference lies in the unobservable subspace?
7. State the dual pair associated with $(A,C)$ and explain the transpose relationship between its controllability matrix and $\mathcal O$.
8. Explain why a matrix with a repeated eigenvalue can be controllable and observable even if it is not diagonalizable. Use the mass as an example.

These questions connect Week 6 to the stability and sampled-system calculations already practiced in Homework 4. Week 7 can use them for consolidation alongside earlier modeling, linearization, and state-transition problems before the October 16 midterm.

\clearpage

## Brief answers

1. $w^*=[0,1]$ corresponds to $\lambda=-2$ and satisfies $w^*B=0$. Decay does not give the input control over that coordinate.
2. $\mathcal O=\begin{bmatrix}1&1\\-1&-2\end{bmatrix}$ has determinant $-1$. The two modes have different time dependence, so their contributions to one measured sum can be separated over time.
3. $v=[1,-1]^T$ satisfies $Av=-v$ and $Cv=0$. Both states decay at the same rate; their difference can remain hidden in the measured sum.
4. For position, $\mathcal O_q=I_2$ and $C_q[1,0]^T=1$. For velocity, $\mathcal O_v$ has rank one and $C_v[1,0]^T=0$. Absolute position is invisible to the velocity sensor.
5. $q_0=1$ and $v_0=(4-1)-2/2=2$, so $x_0=[1,2]^T$.
6. The model and inputs determine the entire forced output. Differences between two trajectories with the same input obey the homogeneous dynamics. Changing the common known input cannot reveal a genuinely unobservable initial-state difference in this LTI setting.
7. The dual pair is $(A^T,C^T)$ for real matrices. Its controllability matrix equals $\mathcal O^T$, so their ranks are equal.
8. The mass has a size-two Jordan block at zero, but $[B,AB]$ has full rank for $m>0$ and the position-sensor observability matrix is $I_2$. PBH does not require a complete eigenvector basis.
