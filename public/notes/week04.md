---
title: "MAE 6246 - Linear Systems and Control"
subtitle: "Week 4: Phase Portraits, Modal Analysis, and Stability"
author: "Fall 2026"
date: ""
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
  - \usepackage{float}
  - \usepackage{graphicx}
---

**Lecture:** Friday, September 18, 2026

**Topics:** eigenpairs and modal decomposition; real and complex modes; phase portraits and planar-system classification; Lyapunov, asymptotic, and exponential stability; eigenvalue and Jordan criteria; transient amplification

\clearpage

# Learning objectives

By the end of the lecture, you should be able to:

1. express a state trajectory as a sum of modes associated with eigenvalue-eigenvector pairs;
2. determine modal amplitudes from an initial condition;
3. connect real and complex eigenvalues to the geometry of planar trajectories;
4. interpret a vector field and a phase portrait;
5. distinguish Lyapunov stability, asymptotic stability, and exponential stability;
6. classify the stability of an LTI equilibrium using eigenvalues and Jordan structure; and
7. explain why asymptotic stability does not require monotonic decay of the state norm.

The central message is that **each eigenpair specifies a building block of the motion**. The eigenvalue describes its time dependence, the eigenvector describes its direction or shape, and the initial condition determines how much of that mode is present. The phase portrait displays their combined motion.

---

# Eigenpairs as dynamical modes

For the zero-input LTI system $\dot x=Ax$ with $x(0)=x_0$, recall from Week 3 that $x(t)=e^{At}x_0$. We now explain this trajectory by first decomposing its initial state.

## Start by decomposing the initial state

Suppose $A\in\mathbb R^{n\times n}$ has $n$ linearly independent eigenvectors $v_1,\ldots,v_n$, with corresponding eigenvalues $\lambda_1,\ldots,\lambda_n$:

$$
Av_i=\lambda_i v_i,\qquad i=1,\ldots,n.
$$

The eigenvectors form a basis. Consequently, **any initial state can be written uniquely as a linear combination of the eigenvectors**:

$$
\boxed{x_0=c_1v_1+\cdots+c_nv_n=\sum_{i=1}^n c_i v_i.}
$$

This is the starting point of modal analysis: first decompose the initial state into eigenvector components, then determine how each component evolves.

Collecting the basis vectors into $P=[v_1\ \cdots\ v_n]$, the initial-state decomposition becomes

$$
\boxed{x_0=Pc,\qquad c=P^{-1}x_0.}
$$

The coefficients $c_i$ are fixed by the initial condition. If the eigenvectors are complex, the basis and coefficients are understood over $\mathbb C$, even when $x_0$ is real. For a real matrix and a basis chosen in conjugate pairs, a real initial state requires conjugate coefficient pairs, as shown in Section 3.2.

The basis assumption is essential. If $A$ is defective, its ordinary eigenvectors do not span the whole state space; generalized eigenvectors are needed to represent an arbitrary initial state.

## Evolve each component and use superposition

Start with the initial-state component $c_i v_i$. Its trajectory is

$$
\boxed{x^{(i)}(t)=c_i e^{\lambda_i t}v_i,\qquad x^{(i)}(0)=c_i v_i.}
$$

Indeed,

$$
\dot x^{(i)}(t)=c_i\lambda_i e^{\lambda_i t}v_i
=c_i e^{\lambda_i t}Av_i=Ax^{(i)}(t).
$$

Thus each eigenvector component has its own simple exponential evolution. The elementary solution $e^{\lambda_i t}v_i$ is a **mode**, and $c_i e^{\lambda_i t}v_i$ is its contribution to this particular trajectory.

By linearity, the sum of these component trajectories is also a solution. At $t=0$, their sum is precisely the specified initial state $x_0$. Uniqueness of the initial-value problem therefore gives

$$
\boxed{
x_0=\sum_{i=1}^n c_i v_i
\quad\Longrightarrow\quad
x(t)=\sum_{i=1}^n c_i e^{\lambda_i t}v_i.
}
$$

Equivalently, propagate the initial-state expansion using the matrix exponential:

$$
x(t)=e^{At}\!\left(\sum_{i=1}^n c_i v_i\right)
=\sum_{i=1}^n c_i e^{At}v_i
=\sum_{i=1}^n c_i e^{\lambda_i t}v_i.
$$

For a real eigenpair, the line $\operatorname{span}\{v_i\}$ is invariant: a state initially on that line remains on it. A negative, zero, or positive eigenvalue makes that component decay toward the origin, remain fixed, or grow away from it, respectively.

## Connection to modal coordinates

The same decomposition can be written as a coordinate transformation. With

$$
\Lambda=\operatorname{diag}(\lambda_1,\ldots,\lambda_n),
\qquad x=Pz,
$$

we have $AP=P\Lambda$ and hence

$$
P\dot z=APz=P\Lambda z,
\qquad \boxed{\dot z=\Lambda z.}
$$

The modal coordinates satisfy

$$
\dot z_i=\lambda_i z_i,\qquad
z_i(0)=c_i,\qquad z_i(t)=c_i e^{\lambda_i t}.
$$

Thus the initial-state coefficients become time-dependent modal coordinates as the trajectory evolves. Transforming back, $x(t)=Pz(t)$ recovers the same modal sum. This also agrees with $e^{At}=Pe^{\Lambda t}P^{-1}$. For a nonzero initial time, replace $t$ in each exponential by $t-t_0$.

The three ingredients have distinct roles:

| Ingredient | Role in the trajectory |
|---|---|
| Eigenvalue $\lambda_i$ | Growth, decay, and oscillation rate |
| Eigenvector $v_i$ | Direction or relative pattern of state components |
| Coefficient $c_i$ | Amplitude and, for complex modes, phase selected by $x_0$ |

## Modes are not individual state components

If $(v_i)_j$ denotes entry $j$ of eigenvector $v_i$, then

$$
x_j(t)=\sum_{i=1}^n c_i e^{\lambda_i t}(v_i)_j.
$$

Each physical state component can contain contributions from several modes. Conversely, one mode can contribute to several state components. It is the modal coordinate $z_i$, rather than the physical coordinate $x_i$, that evolves independently.

For a general eigenvector basis, $P^{-1}\ne P^T$, and $c_i$ is not simply the Euclidean projection $v_i^T x_0$. Eigenvectors need not be orthogonal. If $v_i$ is multiplied by a nonzero scalar, its coefficient changes inversely, leaving $c_i e^{\lambda_i t}v_i$ unchanged.

The modal sum above assumes diagonalizability. Generalized eigenvectors and polynomial-exponential terms are needed when $A$ is defective.

---

## Worked example: two modes forming one state trajectory

Consider the nondimensional overdamped oscillator

$$
\ddot q+4\dot q+3q=0.
$$

With $x=[q,\dot q]^T$,

$$
\dot x=Ax,\qquad
A=\begin{bmatrix}0&1\\-3&-4\end{bmatrix},\qquad
x(0)=\begin{bmatrix}2\\-4\end{bmatrix}.
$$

### Step 1: compute the eigenpairs

The characteristic polynomial is

$$
\det(\lambda I-A)=\lambda^2+4\lambda+3
=(\lambda+1)(\lambda+3).
$$

Thus the eigenvalues are $\lambda_1=-1$ and $\lambda_2=-3$. Corresponding eigenvectors are

$$
v_1=\begin{bmatrix}1\\-1\end{bmatrix},\qquad
v_2=\begin{bmatrix}1\\-3\end{bmatrix}.
$$

Indeed, $Av_1=-v_1$ and $Av_2=-3v_2$. Therefore,

$$
P=\begin{bmatrix}1&1\\-1&-3\end{bmatrix},\qquad
P^{-1}=\begin{bmatrix}3/2&1/2\\-1/2&-1/2\end{bmatrix}.
$$

### Step 2: decompose the initial state

Because $v_1,v_2$ form a basis, write the given initial state as

$$
\begin{bmatrix}2\\-4\end{bmatrix}
=c_1\begin{bmatrix}1\\-1\end{bmatrix}
+c_2\begin{bmatrix}1\\-3\end{bmatrix}.
$$

Equating components gives $c_1+c_2=2$ and $-c_1-3c_2=-4$, so $c_1=c_2=1$. In matrix form, compute

$$
c=P^{-1}x_0
=\begin{bmatrix}3/2&1/2\\-1/2&-1/2\end{bmatrix}
\begin{bmatrix}2\\-4\end{bmatrix}
=\begin{bmatrix}1\\1\end{bmatrix}.
$$

Equivalently, $x_0=v_1+v_2$. Both modes are excited.

### Step 3: evolve each mode and add the contributions

The modal coordinates are $z_1(t)=e^{-t}$ and $z_2(t)=e^{-3t}$. Hence

$$
\boxed{
x(t)=
\underbrace{e^{-t}\begin{bmatrix}1\\-1\end{bmatrix}}_{\text{slow contribution}}
+
\underbrace{e^{-3t}\begin{bmatrix}1\\-3\end{bmatrix}}_{\text{fast contribution}}.
}
$$

The two physical state components are

$$
\boxed{q(t)=e^{-t}+e^{-3t},\qquad
\dot q(t)=-e^{-t}-3e^{-3t}.}
$$

At $t=0$, these expressions give $q(0)=2$ and $\dot q(0)=-4$. Differentiating $q(t)$ gives the second state component, as required.

Each contribution stays on its own eigenvector line. Their vector sum generally does not stay on a line because the two coefficients decay at different rates. In particular,

$$
\frac{x_2(t)}{x_1(t)}
=-\frac{1+3e^{-2t}}{1+e^{-2t}}
\longrightarrow -1\quad\text{as }t\to\infty.
$$

The trajectory therefore approaches the origin tangent to $v_1$, the slow direction. The eigenvalue $-1$ has time constant $1$, while $-3$ has time constant $1/3$.

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{figures/week04_modal_decomposition.png}
\caption{The slow and fast modal contributions lie on separate eigenvector lines. Their head-to-tail sum at a common time lies on the curved state trajectory. The time histories show that both physical state components contain both modes. The background arrows show direction only.}
\end{figure}

### How the initial condition changes the motion

For an arbitrary initial state $x_0=[a,b]^T$,

$$
c_1=\frac{3a+b}{2},\qquad c_2=-\frac{a+b}{2}.
$$

- If $x_0=v_1$, then $c_1=1$, $c_2=0$: the trajectory contains only the slow mode.
- If $x_0=v_2$, then $c_1=0$, $c_2=1$: the trajectory contains only the fast mode.
- If $x_0=[1,0]^T$, then $c_1=3/2$, $c_2=-1/2$: opposite modal contributions cancel the initial velocity.

The slowest decaying mode dominates at long times only if its coefficient is nonzero. A mode absent from the initial condition remains absent in this unforced, diagonalizable system.

---

# Real and complex modes in the phase plane

## Real eigenvalues: nodes and saddles

For two distinct real eigenvalues, the real eigenvectors provide invariant lines. In modal coordinates,

$$
z_1(t)=c_1e^{\lambda_1t},\qquad z_2(t)=c_2e^{\lambda_2t}.
$$

If both eigenvalues are negative, both modal coordinates decay: the origin is a **stable node**. If both are positive, the origin is an **unstable node**. If the signs are opposite, the origin is a **saddle**.

For example,

$$
A=\begin{bmatrix}-1&0\\0&1\end{bmatrix}
\quad\Longrightarrow\quad
x(t)=\begin{bmatrix}c_1e^{-t}\\c_2e^t\end{bmatrix}.
$$

Initial conditions on the $x_1$ axis converge to the origin. Every nonzero $x_2$ component grows. Thus an unstable equilibrium can have some convergent trajectories; stability must account for all sufficiently small perturbations.

## Complex conjugate modes and real trajectories

Let $A$ be real and suppose it has a nonreal eigenpair

$$
\lambda=\sigma+i\omega,\qquad v=p+iq,\qquad \omega>0,
$$

where $\sigma$ is the real part, $\omega>0$ is the angular frequency, and $p,q$ are real. We choose the eigenvalue with positive imaginary part; its conjugate is $\overline\lambda=\sigma-i\omega$. Taking the complex conjugate of $Av=\lambda v$ gives

$$
A\overline v=\overline\lambda\,\overline v.
$$

Thus $(\overline\lambda,\overline v)$ is also an eigenpair. Choose the two eigenvectors to be exactly $v$ and $\overline v$ so that their coefficients can be compared consistently.

**Start with the initial condition.** In this pair's real invariant plane, write

$$
x_0=c_1v+c_2\overline v.
$$

For a planar system, this represents every initial state. For a higher-dimensional diagonalizable system, apply the following argument to each conjugate pair in the full eigenvector expansion.

The state must be real, so $x_0=\overline{x_0}$. But

$$
\overline{x_0}
=\overline{c_1}\,\overline v+\overline{c_2}\,v.
$$

Comparing the two expressions gives

$$
(c_1-\overline{c_2})v
+(c_2-\overline{c_1})\overline v=0.
$$

The vectors $v$ and $\overline v$ belong to distinct eigenvalues and are linearly independent over $\mathbb C$. Uniqueness of the basis coefficients therefore requires

$$
\boxed{c_2=\overline{c_1}.}
$$

Conversely, this condition makes $x_0=c_1v+\overline{c_1v}$ real. Thus **a real initial state requires the coefficients of conjugate eigenvectors to occur in conjugate pairs**. Coefficients of real eigenvectors are real when those eigenvectors are chosen real.

**Evolve the conjugate initial-state components.** Write $c_1=c$ and $c_2=\overline c$. Superposition gives

$$
\begin{aligned}
x(t)
&=c e^{\lambda t}v+\overline c\,e^{\overline\lambda t}\overline v\\
&=c e^{\lambda t}v+\overline{c e^{\lambda t}v}.
\end{aligned}
$$

Therefore,

$$
\boxed{x(t)=2\operatorname{Re}\!\left(c e^{(\sigma+i\omega)t}v\right).}
$$

The imaginary parts cancel at every time, not only at $t=0$. The two complex modes combine into one real motion.

**Componentwise rotation in the complex plane.** To visualize the motion generated by this pair, first take $c=1/2$. For the $i$th component $v_i=p_i+iq_i$ of the eigenvector,

$$
\boxed{x_i(t)=e^{\sigma t}\operatorname{Re}\!\left(e^{i\omega t}v_i\right)
=e^{\sigma t}(p_i\cos\omega t-q_i\sin\omega t).}
$$

Multiplication by $e^{i\omega t}$ rotates $v_i$ counterclockwise in the complex plane at angular velocity $\omega$. Multiplication by $e^{\sigma t}$ scales its length, and projection onto the real axis gives the physical state component $x_i(t)$. Every component has the same angular frequency and exponential envelope, while $v_i$ determines its amplitude and phase. A zero component remains zero.

The choice $c=1/2$ gives $x(0)=\operatorname{Re}v=p$. For general $c$, the same interpretation applies to the complex number $2cv_i$ in place of $v_i$; the coefficient $c$ is determined by the initial condition. This componentwise complex-plane rotation does not determine clockwise or counterclockwise motion in the physical $(x_1,x_2)$ phase portrait.

Figure \ref{fig:complex-projection} illustrates the rotation, scaling, and projection for $v_1=1+i$ and $\lambda=-1+i$, with $c=1/2$.

\begin{figure}[p]
\centering
\includegraphics[width=\textwidth]{figures/week04_complex_rotation_projection.png}
\caption{Componentwise complex rotation and its real-valued time history. Left: at $t_*=0.5$, the gray vector is $v_1=1+i$, the blue vector is $e^{i\omega t_*}v_1$, and the green vector is $e^{\sigma t_*}e^{i\omega t_*}v_1$, with $\sigma=-1$ and $\omega=1$. The orange vector is the real-axis projection, connected to the green tip by a dashed line. Right: $x_1(t)=\operatorname{Re}(e^{(-1+i)t}(1+i))=e^{-t}(\cos t-\sin t)$. The matching orange point marks $x_1(0.5)\approx0.241$. Here $v_1$ denotes one complex component of the eigenvector.}
\label{fig:complex-projection}
\end{figure}

**Real matrix form for general initial conditions.** Write $c=(a-ib)/2$, where $a,b$ are real. Then the real initial-state expansion is

$$
x_0=cv+\overline c\,\overline v=ap+bq,
$$

and its trajectory can be written in matrix form as

$$
\boxed{
x(t)=e^{\sigma t}
\underbrace{\begin{bmatrix}p&q\end{bmatrix}}_{S}
\underbrace{\begin{bmatrix}
\cos\omega t&\sin\omega t\\
-\sin\omega t&\cos\omega t
\end{bmatrix}}_{R(-\omega t)}
\begin{bmatrix}a\\b\end{bmatrix}.
}
$$

This matrix form expresses the same motion in the real basis $S=[p\ q]$. The negative angle in $R(-\omega t)$ follows from this basis convention; it does not change the counterclockwise complex-plane rotation of each $v_i$. The map $S$ can stretch or reverse orientation, so determine the physical phase-portrait direction from the vector field. Find $a,b$ by solving $x_0=S[a,b]^T$.

The real part $\sigma$ sets the exponential envelope, and $\omega$ is the angular frequency. A complex eigenvector does not define a real invariant line; its real and imaginary parts span a real invariant plane.

- $\sigma<0$: an inward spiral, also called a stable focus.
- $\sigma>0$: an outward spiral, also called an unstable focus.
- $\sigma=0$: a center with closed elliptical orbits in the planar linear system.

**How the imaginary part produces oscillation.** Projecting a rotating complex number onto the real axis produces the sine and cosine terms in $x_i(t)$. Their angular frequency is $\omega$, their period is $T$, and their frequency in cycles per unit time is $f$:

$$
\boxed{T=\frac{2\pi}{\omega},\qquad f=\frac{\omega}{2\pi}.}
$$

Increasing $\omega$ produces faster oscillations and a shorter period. The factor $e^{\sigma t}$ makes their amplitude decay, remain constant, or grow according to the sign of $\sigma$. The full nonzero trajectory is periodic only when $\sigma=0$. For a real eigenvalue ($\omega=0$), this sinusoidal mechanism is absent.

## Connection to the damped oscillator

The Week 3 computational example used

$$
\ddot q+0.6\dot q+2q=0,\qquad
A=\begin{bmatrix}0&1\\-2&-0.6\end{bmatrix}.
$$

Its characteristic equation gives

$$
\lambda^2+0.6\lambda+2=0,
\qquad
\boxed{\lambda=-0.3\pm i\sqrt{1.91}.}
$$

Here $\sigma=-0.3$ and $\omega=\sqrt{1.91}$. The phase portrait is an inward spiral with exponential envelope $e^{-0.3t}$. At $(q,\dot q)=(1,0)$, the velocity vector is $(0,-2)$, so the motion is clockwise in the $(q,\dot q)$ plane.

Removing damping gives $\lambda=\pm i\sqrt2$. The quantity

$$
E=q^2+\frac12\dot q^2
$$

is constant because $\dot E=\dot q(2q+\ddot q)=0$. Its level sets are ellipses. The undamped oscillator stays near the origin when started nearby, but does not approach it unless started exactly there.

## Repeated and zero eigenvalues

The distinct-eigenvalue cases do not exhaust all portraits. If a planar matrix has a repeated real eigenvalue $\lambda$ and two independent eigenvectors, then $A=\lambda I$. For $\lambda<0$, all trajectories are straight rays into the origin, sometimes called a star node.

If the repeated eigenvalue has only one independent eigenvector, a Jordan block creates a $t e^{\lambda t}$ term. The resulting degenerate or improper node has different geometry. Its stability still follows the sign of $\lambda$ when $\lambda\ne0$.

A zero eigenvalue makes $A$ singular, so there is a nontrivial equilibrium subspace. For example, $A=\operatorname{diag}(-1,0)$ has solutions $x(t)=[c_1e^{-t},c_2]^T$. Trajectories approach the equilibrium line $x_1=0$, but generally do not approach the origin.

\clearpage

# Phase portraits

## Vector fields and trajectories

For a planar autonomous system,

$$
\dot x=f(x),\qquad x=\begin{bmatrix}x_1\\x_2\end{bmatrix}\in\mathbb R^2,
$$

the **vector field** assigns the velocity vector $f(x)$ to every state $x$. The solution $x(t)$ traces a curve in the state plane. Its tangent at each point is the vector $\dot x=f(x)$.

A **phase portrait** is a geometric representation of trajectories for different initial conditions, with arrows indicating increasing time. It is different from a time-history plot:

- a time history plots $x_1(t)$ or $x_2(t)$ against $t$;
- a phase-plane trajectory plots $x_2(t)$ against $x_1(t)$, with $t$ serving as a parameter.

A normalized arrow plot shows the direction of motion but suppresses its speed. Trajectories may pass through the same geometric curve at different times. Under uniqueness of solutions, two distinct autonomous trajectories cannot cross at the same state and then continue along different paths.

For a mechanical system with $x_1=q$ and $x_2=\dot q$, the relation $\dot x_1=x_2$ has an immediate geometric interpretation: trajectories move to the right in the upper half-plane and to the left in the lower half-plane.

\clearpage

## Eigenvalues and the type of equilibrium

For a real, constant $2\times2$ matrix $A$, the following table classifies the origin of $\dot x=Ax$. For nonzero distinct eigenvalues, their signs and real parts identify the basic portrait. Repeated eigenvalues require the number of independent eigenvectors as well. A zero eigenvalue makes the equilibrium nonisolated.

Here **stable** means that sufficiently small initial perturbations remain nearby; **asymptotically stable** additionally means that they converge to the origin. These definitions are developed in Section 5.

\begin{longtable}{@{}
>{\raggedright\arraybackslash}p{(\linewidth-6\tabcolsep)*\real{0.25}}
>{\raggedright\arraybackslash}p{(\linewidth-6\tabcolsep)*\real{0.25}}
>{\raggedright\arraybackslash}p{(\linewidth-6\tabcolsep)*\real{0.25}}
>{\raggedright\arraybackslash}p{(\linewidth-6\tabcolsep)*\real{0.25}}@{}}
\toprule
Eigenvalues & Additional structure & Equilibrium type & Stability of the origin \\
\midrule
\endhead
\bottomrule
\endlastfoot
Distinct real $\lambda_1,\lambda_2<0$ & Two independent eigenvectors & Stable node (sink) & Asymptotically stable \\
Distinct real $\lambda_1,\lambda_2>0$ & Two independent eigenvectors & Unstable node (source) & Unstable \\
Real $\lambda_1<0<\lambda_2$ & Two independent eigenvectors & Saddle & Unstable \\
$\sigma\pm i\omega$, $\sigma<0$, $\omega>0$ & Conjugate pair & Stable spiral (focus) & Asymptotically stable \\
$\sigma\pm i\omega$, $\sigma>0$, $\omega>0$ & Conjugate pair & Unstable spiral (focus) & Unstable \\
$\pm i\omega$, $\omega>0$ & Conjugate pair & Center & Stable, not asymptotically stable \\
$0$ and real $\lambda<0$ & Two independent eigenvectors & Line of equilibria; trajectories approach the line & Stable, not asymptotically stable \\
$0$ and real $\lambda>0$ & Two independent eigenvectors & Line of equilibria; trajectories leave the line & Unstable \\
\midrule
Repeated real $\lambda\ne0$ & Two independent eigenvectors; $A=\lambda I$ & Star node & Asymptotically stable if $\lambda<0$; unstable if $\lambda>0$ \\
Repeated real $\lambda\ne0$ & One independent eigenvector; size-two Jordan block & Improper (degenerate) node & Asymptotically stable if $\lambda<0$; unstable if $\lambda>0$ \\
$0,0$ & Two independent eigenvectors; $A=0$ & Every state is an equilibrium & Stable, not asymptotically stable \\
$0,0$ & One independent eigenvector; $A\ne0$ & Line of equilibria with shear motion & Unstable \\
\end{longtable}

The zero-eigenvalue rows describe stability of the **particular equilibrium at the origin**, not attraction to the equilibrium set. For example, $\operatorname{diag}(-1,0)$ attracts states toward a line, but generally not toward the origin. Similarly, the convergent trajectories along a saddle's stable eigenvector do not make the saddle stable.

The table applies to planar LTI dynamics. For a nonlinear system, eigenvalues of the equilibrium Jacobian can be inconclusive when they lie on the imaginary axis. Eigenvectors also affect the orientation and shape of the plotted trajectories; eigenvalues alone do not specify those details.

\begin{figure}[p]
\centering
\includegraphics[width=0.94\textwidth]{figures/week04_phase_portraits.png}
\caption{Representative planar LTI phase portraits; arrows indicate forward time. The stable and unstable node matrices are $\operatorname{diag}(-1,-2)$ and $\operatorname{diag}(1,2)$; the saddle matrix is $\operatorname{diag}(-1,1)$. The spiral and center matrices are $A_\sigma=\left[\begin{smallmatrix}\sigma&-1\\1&\sigma\end{smallmatrix}\right]$, with eigenvalues $\sigma\pm i$. The stable spiral, unstable spiral, and center use $\sigma=-0.3$, $0.3$, and $0$, respectively. These three examples rotate counterclockwise, as can be checked at $(1,0)$.}
\end{figure}

\clearpage

# Stability of an equilibrium

For an autonomous system $\dot x=f(x)$, an equilibrium $x_e$ satisfies $f(x_e)=0$. For $\dot x=Ax$, this becomes $Ax_e=0$: the origin is always an equilibrium, and it is the only one if $A$ is nonsingular. Write the trajectory starting at $x_0$ as $x(t;x_0)$ and assume solutions exist for the times under discussion.

## Lyapunov stability

The equilibrium is **stable in the sense of Lyapunov** if, for every $\varepsilon>0$, there exists $\delta>0$ such that

$$
\boxed{
\|x_0-x_e\|<\delta
\quad\Longrightarrow\quad
\|x(t;x_0)-x_e\|<\varepsilon
\quad\text{for every }t\ge0.
}
$$

An arbitrarily small prescribed neighborhood can retain every trajectory that starts in a sufficiently small neighborhood. The choice of $\delta$ may depend on $\varepsilon$, but must work for all future times and all initial states in that neighborhood.

An equilibrium is **unstable** if this condition fails. Instability does not mean that every trajectory grows, nor does stability require every distance from equilibrium to decrease monotonically.

## Attractivity and asymptotic stability

An equilibrium is **locally attractive** if there exists $r>0$ such that

$$
\|x_0-x_e\|<r
\quad\Longrightarrow\quad
\lim_{t\to\infty}x(t;x_0)=x_e.
$$

It is **asymptotically stable** if it is both Lyapunov stable and attractive. Stability concerns remaining nearby for all time; attractivity concerns eventual convergence. In general nonlinear systems these are separate requirements.

If the equilibrium is stable and attracts every initial state, it is **globally asymptotically stable**. For a finite-dimensional autonomous LTI system, asymptotic stability of the origin is global because solutions scale linearly with their initial conditions.

## Exponential stability

The origin of an LTI system is **exponentially stable** if there exist constants $M\ge1$ and $\gamma>0$ such that

$$
\boxed{\|x(t)\|\le M e^{-\gamma t}\|x_0\|
\quad\text{for every }x_0\text{ and }t\ge0.}
$$

This gives both a decay rate and a bound on possible amplification. The constant $M$ need not equal one. For finite-dimensional continuous-time LTI systems, asymptotic stability and exponential stability are equivalent.

Under a constant nonsingular coordinate change $x=Pz$, stability is unchanged: $\|x\|\le\|P\|\|z\|$ and $\|z\|\le\|P^{-1}\|\|x\|$. Numerical distances and amplification factors can change with the coordinates, but the qualitative stability classification cannot.

---

# LTI stability from eigenvalues and Jordan structure

For $\dot x=Ax$, the origin is Lyapunov stable exactly when the state-transition matrix is bounded:

$$
\boxed{\sup_{t\ge0}\|e^{At}\|<\infty.}
$$

To see the sufficient direction, suppose $\|e^{At}\|\le K$ for all $t\ge0$, with $K\ge1$. For any $\varepsilon>0$, take $\delta=\varepsilon/K$. Then $\|x(t)\|\le K\|x_0\|<\varepsilon$. Conversely, stability applied to a fixed neighborhood and linear scaling bounds the gain uniformly over all unit initial vectors.

Asymptotic stability is equivalent to $e^{At}\to0$ as $t\to\infty$. Convergence for each basis-vector initial condition gives convergence of all columns of $e^{At}$, and hence of the matrix in any finite-dimensional matrix norm.

## The eigenvalue criteria

A matrix is called **Hurwitz** when all of its eigenvalues have strictly negative real parts. For a constant finite-dimensional matrix $A$:

1. **Asymptotic stability:** the origin is asymptotically, equivalently exponentially, stable if and only if

   $$
   \boxed{\operatorname{Re}\lambda_i<0\quad\text{for every }i.}
   $$

2. **Lyapunov stability:** the origin is stable if and only if all eigenvalues have nonpositive real parts and every eigenvalue on the imaginary axis is semisimple.

3. **Instability:** the origin is unstable if at least one eigenvalue has positive real part, or an imaginary-axis eigenvalue has a Jordan block larger than size one.

An eigenvalue is **semisimple** when its geometric and algebraic multiplicities agree: there are as many independent eigenvectors for that eigenvalue as its multiplicity in the characteristic polynomial. Equivalently, all Jordan blocks associated with that eigenvalue are $1\times1$. Compare a size-one block with a size-two block:

$$
J_1(\lambda)=[\lambda],\qquad
J_2(\lambda)=\begin{bmatrix}\lambda&1\\0&\lambda\end{bmatrix}.
$$

**What "every associated Jordan block has size one" means.** Each block contains only its eigenvalue and has no superdiagonal $1$. Thus the portion of the Jordan form associated with that eigenvalue is diagonal. Repeated eigenvalues are allowed: $\operatorname{diag}(\lambda,\lambda)$ consists of two size-one blocks, whereas $J_2(\lambda)$ is one size-two block.

**For Lyapunov stability, this requirement applies only to imaginary-axis eigenvalues**, including $\lambda=0$. Blocks with $\operatorname{Re}\lambda<0$ may be larger. Only if every block of the entire Jordan form has size one is $A$ diagonalizable over $\mathbb C$; stability does not require this globally.

If the stability condition holds and at least one eigenvalue lies on the imaginary axis, the origin is stable but not asymptotically stable. Imaginary-axis modes do not decay.

## Why Jordan structure matters

Recall from Week 3 that a Jordan block $J=\lambda I+N$ of size $m$ satisfies

$$
e^{Jt}=e^{\lambda t}
\left(I+tN+\frac{t^2}{2!}N^2+\cdots+
\frac{t^{m-1}}{(m-1)!}N^{m-1}\right).
$$

Its terms have the form $t^k e^{\lambda t}$.

- If $\operatorname{Re}\lambda<0$, the exponential dominates every polynomial and each term decays.
- If $\operatorname{Re}\lambda>0$, an eigenvector mode already grows exponentially.
- If $\operatorname{Re}\lambda=0$, a size-one block gives a bounded nondecaying mode. A larger block produces an unbounded polynomial factor.

For a Hurwitz matrix, any $\gamma$ satisfying $0<\gamma<-\max_i\operatorname{Re}\lambda_i$ leaves enough exponential decay to bound all polynomial factors. Multiplication by the fixed matrices in a Jordan transformation then supplies a finite constant $M$ in the exponential-stability bound.

**Checking semisimplicity without constructing the Jordan form.** For each distinct eigenvalue $\lambda$ with $\operatorname{Re}\lambda=0$, find its algebraic multiplicity $m_\lambda$ and count its independent eigenvectors:

$$
\boxed{\dim\ker(A-\lambda I)=n-\operatorname{rank}(A-\lambda I)
\stackrel{?}{=}m_\lambda.}
$$

Equality for every imaginary-axis eigenvalue means that all its blocks are size one. A strict inequality for any of them means at least one larger block and therefore instability. For a complex eigenvalue, compute the rank and eigenspace over $\mathbb C$. A simple imaginary-axis eigenvalue automatically passes this check.

\clearpage

## A flowchart for deciding stability

Apply the following decisions to the origin of a finite-dimensional, continuous-time LTI system $\dot x=Ax$ with constant $A$. This is an exact test for the linear system; imaginary-axis eigenvalues of a nonlinear system's linearization require further analysis.

\begin{figure}[H]
\centering
\includegraphics[width=0.98\textwidth]{figures/week04_stability_flowchart.png}
\caption{Stability decisions for a continuous-time LTI system. After the two real-part tests, only eigenvalues on the imaginary axis require a Jordan-structure check.}
\end{figure}

For the last decision, use the eigenspace-dimension test in Section 6.2.

\clearpage

## Worked stability decisions

**Example 1: a positive real part.** Let

$$
A=\begin{bmatrix}1&-3\\3&1\end{bmatrix},
\qquad \lambda=1\pm3i.
$$

The first decision is **yes**: both eigenvalues have positive real part. The origin is **unstable**. Here $e^{At}=e^tR(3t)$, so $\|x(t)\|_2=e^t\|x_0\|_2$. No Jordan-structure check is needed.

**Example 2: a repeated negative eigenvalue with a size-two block.** Let

$$
A=\begin{bmatrix}-1&1\\0&-1\end{bmatrix},
\qquad e^{At}=e^{-t}\begin{bmatrix}1&t\\0&1\end{bmatrix}.
$$

The eigenvalue $-1$ has algebraic multiplicity two and only one independent eigenvector: $A$ is already a size-two Jordan block. The first decision is **no** and the second is **yes**. The origin is **globally exponentially and asymptotically stable**, because both $e^{-t}$ and $te^{-t}$ decay to zero. The superdiagonal $1$ is allowed because the eigenvalue has strictly negative real part.

**Example 3: repeated zero eigenvalues, all blocks size one.** Let

$$
A=\begin{bmatrix}0&0\\0&0\end{bmatrix},
\qquad \dim\ker A=2=m_0.
$$

The first two decisions are **no**. The last decision is **yes**: the two zero eigenvalues have two independent eigenvectors, so the Jordan form has two blocks $[0]$ and no superdiagonal $1$. Since $e^{At}=I$, $x(t)=x_0$. The origin is **stable but not asymptotically stable**; nonzero initial states remain where they started.

**Example 4: the same eigenvalues, but a size-two block.** Let

$$
A=\begin{bmatrix}0&1\\0&0\end{bmatrix},
\qquad \dim\ker A=1<2=m_0,
\qquad e^{At}=\begin{bmatrix}1&t\\0&1\end{bmatrix}.
$$

The first two decisions are again **no**, but the last decision is now **no**. The superdiagonal $1$ in this imaginary-axis block produces polynomial growth. For an arbitrarily small $x_0=[0,\eta]^T$ with $\eta\ne0$, the solution is $x(t)=[\eta t,\eta]^T$, which eventually leaves any fixed neighborhood. The origin is **unstable**.

Examples 3 and 4 have identical eigenvalues but different eigenspaces and stability. Example 2 shows why a repeated eigenvalue or a defective matrix is not automatically unstable.

---

# Stable modes can produce transient amplification

Negative-real-part eigenvalues determine eventual decay, but they do not by themselves describe the size of the transient. Revisit the non-normal matrix from the Week 3 notebook:

$$
A=\begin{bmatrix}-1&8\\0&-2\end{bmatrix},\qquad
x_0=\begin{bmatrix}0\\1\end{bmatrix}.
$$

A real matrix is **normal** when $A^TA=AA^T$. This matrix is not normal. Its eigenvectors are not orthogonal.

## Modal explanation: cancellation that changes with time

Choose

$$
\lambda_1=-1,\quad v_1=\begin{bmatrix}1\\0\end{bmatrix},\qquad
\lambda_2=-2,\quad v_2=\begin{bmatrix}-8\\1\end{bmatrix}.
$$

Since $x_0=8v_1+v_2$, the trajectory is

$$
\boxed{
x(t)=8e^{-t}v_1+e^{-2t}v_2
=\begin{bmatrix}8(e^{-t}-e^{-2t})\\e^{-2t}\end{bmatrix}.
}
$$

At $t=0$, the first components of the modal contributions cancel exactly: $8-8=0$. Both contributions subsequently decay, but at different rates, so their cancellation weakens. At $t=\log2$,

$$
x(t)=\begin{bmatrix}2\\1/4\end{bmatrix},\qquad
\|x(t)\|_2=\frac{\sqrt{65}}{4}\approx2.016>1=\|x_0\|_2.
$$

The state has grown before ultimately decaying to zero. The time $\log2$ maximizes the first component; it is used here as a convenient exact evaluation time, not as a claim about the maximum state norm.

\begin{figure}[H]
\centering
\includegraphics[width=\textwidth]{figures/week04_transient_growth.png}
\caption{Left: two individually decaying contributions to $x_1$ initially cancel. Right: their sum produces a state norm larger than its initial value before decay. The marked value at $t=\log2$ is $\sqrt{65}/4$.}
\end{figure}

## Why this does not contradict stability

Stability requires a sufficiently small initial neighborhood for each prescribed final neighborhood. It permits finite amplification. Exponential stability likewise permits $M>1$ in $\|x(t)\|\le M e^{-\gamma t}\|x_0\|$.

For a diagonalizable Hurwitz matrix with spectral abscissa $s=\max_i\operatorname{Re}\lambda_i<0$,

$$
\|e^{At}\|_2
\le\|P\|_2\|P^{-1}\|_2 e^{st}
=\kappa_2(P)e^{st}.
$$

This bound illustrates the role of the eigenvector basis, although it can be conservative and depends on eigenvector scaling. The actual maximum amplification at a fixed time is

$$
\boxed{\max_{\|x_0\|_2=1}\|x(t)\|_2
=\|e^{At}\|_2=\sigma_{\max}(e^{At}).}
$$

This connects modal analysis to the singular-value viewpoint from Week 2. Non-normality permits transient growth but does not guarantee it for every matrix or every initial state.

---

# What linear stability says about a nonlinear model

Let a continuously differentiable autonomous system have equilibrium $x_e$:

$$
\dot x=f(x),\qquad f(x_e)=0.
$$

With $\xi=x-x_e$, its local expansion is

$$
\dot\xi=A\xi+r(\xi),\qquad
A=\left.\frac{\partial f}{\partial x}\right|_{x_e},\qquad
\frac{\|r(\xi)\|}{\|\xi\|}\to0\quad\text{as }\xi\to0.
$$

The linearization gives the following local conclusions:

- if every eigenvalue of $A$ has negative real part, the nonlinear equilibrium is locally asymptotically stable;
- if at least one eigenvalue has positive real part, the nonlinear equilibrium is unstable;
- if all real parts are nonpositive and at least one is zero, the linearization alone is generally inconclusive.

For example, the scalar systems $\dot x=-x^3$ and $\dot x=x^3$ both have linearization $\dot x=0$ at the origin. The first has solutions

$$
x(t)=\frac{x_0}{\sqrt{1+2x_0^2t}},
$$

which converge to zero while remaining no farther away than their initial state. The second has solutions $x(t)=x_0/\sqrt{1-2x_0^2t}$ up to their finite escape time, and the origin is unstable. The zero linearization cannot distinguish them.

The LTI eigenvalue criteria also cannot be applied to the instantaneous eigenvalues of a general time-varying matrix $A(t)$. LTV stability depends on the state-transition matrix $\Phi(t,t_0)$, rather than on a collection of frozen-time portraits.

---

# Common mistakes

1. **Confusing a phase portrait with a time history.** Time is a parameter, rather than an axis, in the state-plane plot.
2. **Assigning one physical state component to each mode.** A mode is a vector contribution; a state component can contain several modes.
3. **Using $P^Tx_0$ for a general eigenvector basis.** Modal amplitudes are $P^{-1}x_0$; transpose and inverse agree only for a real orthogonal basis.
4. **Expecting the slow mode to dominate when its amplitude is zero.** The initial condition determines which modes are present.
5. **Treating a complex eigenvector as a real invariant line.** A conjugate pair yields real motion in a two-dimensional invariant subspace.
6. **Concluding stability from nonpositive real parts alone.** Imaginary-axis eigenvalues must also be semisimple.
7. **Calling every repeated eigenvalue defective.** Compare algebraic and geometric multiplicities.
8. **Equating stability with monotonic decay of the Euclidean norm.** Stable non-normal systems can amplify the state temporarily.
9. **Declaring a saddle stable because one trajectory converges.** Stability concerns every sufficiently small perturbation.
10. **Transferring LTI criteria directly to nonlinear or LTV systems.** The hypotheses and resulting conclusions differ.

---

# Concept checks

1. Why is the velocity vector $Ax$ tangent to a phase-plane trajectory through $x$?
2. Why can an arbitrary initial state be expanded in eigenvectors when $A$ is diagonalizable? Use this expansion and superposition to derive the modal solution.
3. For the overdamped oscillator, find the complete trajectory from $x_0=[1,0]^T$. Which modal contribution has a negative coefficient?
4. What initial states eliminate the slow mode in that example? Sketch their trajectories.
5. Why can the sum of two straight-line modal motions form a curved trajectory?
6. Classify the portrait and stability for the planar eigenvalue pairs $(-1,-3)$, $(-1,2)$, $(-0.2+i,-0.2-i)$, and $(i,-i)$.
7. Starting with $x_0=c_1v+c_2\overline v$, show why a real initial state requires $c_2=\overline{c_1}$. Why does the paired trajectory remain real for every time?
8. Compare $A=0_{2\times2}$ with $A=\begin{bmatrix}0&1\\0&0\end{bmatrix}$. Why do identical eigenvalues give different stability conclusions?
9. Explain why $t^2e^{-t}\to0$, and what this implies for a size-three Jordan block with eigenvalue $-1$.
10. In the transient-growth example, identify the components that cancel at $t=0$ and explain why that cancellation does not persist.
11. Does convergence of one initial state prove asymptotic stability? Explain using a saddle.
12. Why does a purely imaginary pair in a nonlinear equilibrium's Jacobian require further analysis?

\clearpage

# Computational verification

The following short calculation checks the modal decomposition against the matrix exponential. Solve a linear system to obtain the modal amplitudes; there is no need to form an inverse numerically.

```python
import numpy as np
from scipy.linalg import expm

A = np.array([[0., 1.], [-3., -4.]])
x0 = np.array([2., -4.])
lam, P = np.linalg.eig(A)  # eigenvectors are columns
c = np.linalg.solve(P, x0)
times = np.linspace(0., 5., 501)

# Terms[k, :, i] is the vector contribution of mode i at time k.
terms = np.array([
    P * (c * np.exp(lam * t))[None, :] for t in times
])
x_modal = terms.sum(axis=2)
x_exact = np.array([expm(A * t) @ x0 for t in times])
assert np.allclose(x_modal, x_exact)
```

The ordering and normalization of numerical eigenvectors can differ from the hand calculation, so numerical coefficients may differ as well. The complete modal contributions and their sum must agree. For a real matrix with complex eigenvalues, pair the conjugate terms to obtain a real trajectory; individual terms can be complex.

To explore the geometry, plot several initial conditions and overlay the eigenvector lines when they are real. To measure the largest gain at a given time, compute the largest singular value of `expm(A * t)`. For repeated or nearly defective eigenvalues, use analytical structure to interpret Jordan behavior and `expm` to compute trajectories reliably.

\clearpage

# Summary

For a diagonalizable LTI system, first expand the initial state in the eigenvector basis,

$$
\boxed{x_0=\sum_{i=1}^n c_i v_i,\qquad c=P^{-1}x_0.}
$$

Then evolve each component and add the resulting modes:

$$
\boxed{
x(t)=\sum_{i=1}^{n}c_i e^{\lambda_i t}v_i,
\qquad c=P^{-1}x_0,
\qquad P=[v_1\ \cdots\ v_n].
}
$$

The eigenvalue gives the time dependence, the eigenvector gives the state-space pattern, and the initial condition selects the modal amplitudes. For a real initial state, conjugate eigenvectors have conjugate coefficients; their contributions sum to $2\operatorname{Re}(c e^{\lambda t}v)$. A phase portrait shows the resulting motion geometrically.

For a finite-dimensional continuous-time LTI system:

- strictly negative real parts give global exponential and asymptotic stability;
- nonpositive real parts give Lyapunov stability only when every imaginary-axis eigenvalue is semisimple;
- a positive real part or a nontrivial imaginary-axis Jordan block gives instability.

Stable systems may exhibit transient amplification. Eigenvalues describe asymptotic behavior, while eigenvectors and the state-transition matrix reveal how modal contributions combine before that long-term behavior emerges.
