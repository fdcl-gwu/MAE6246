---
title: "MAE 6246 - Linear Systems and Control"
subtitle: "Week 5: Stability, Discrete-Time Models, Controllability, and Observability"
author: "Fall 2026"
date: "September 25, 2026"
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
  - \usepackage{needspace}
---

**Lecture:** Friday, September 25, 2026

**Topics:** equilibrium stability; LTI eigenvalue and Jordan criteria; transient amplification; exact sampled-data models; discrete-time state solutions; the concepts of controllability and observability

The stability material introduced in the Week 4 notes is developed here before we turn to the roles of actuators and sensors.

\clearpage

# Learning objectives and three questions

By the end of this lecture, you should be able to:

1. explain stability intuitively and distinguish stability from convergence;
2. classify an LTI equilibrium using eigenvalues and Jordan structure;
3. explain why attractivity, asymptotic stability, and exponential stability are equivalent for finite-dimensional LTI systems;
4. derive an exact discrete-time model when the input is held constant between samples;
5. compute a discrete-time state trajectory from its initial condition and inputs;
6. interpret controllability as the ability to move between arbitrary states; and
7. interpret observability as the ability to determine the initial state from known inputs and measured outputs.

For the state-space model

$$
\dot x=Ax+Bu,\qquad y=Cx+Du,
$$

we will distinguish three questions:

| Property | Question | Matrices that determine it |
|---|---|---|
| Internal stability | With zero input, what happens after a small initial-state disturbance? | $A$ |
| Controllability | Can the input move the state from any initial state to any desired final state? | $(A,B)$ |
| Observability | Can the initial state be determined uniquely from the input and output histories? | $(A,C)$ |

These are different properties. Stable motion need not be controllable or observable, and an unstable system can be both controllable and observable. The direct-feedthrough term $Du$ is known when $u$ is known, so it can be subtracted from the output in the observability question.

We first finish the stability analysis of $\dot x=Ax$. We then derive the discrete-time model, whose finite sums make the controllability and observability concepts especially transparent. The treatment of the latter two properties is introductory: definitions, state/output expansions, rank tests, and a mechanical example. Gramians, minimum-energy control, PBH tests, and canonical decompositions will be developed later.

\clearpage

# Stability of an equilibrium

For an autonomous system $\dot x=f(x)$, an equilibrium $x_e$ satisfies $f(x_e)=0$. For $\dot x=Ax$, this becomes $Ax_e=0$: the origin is always an equilibrium, and it is the only one if $A$ is nonsingular. Write the trajectory starting at $x_0$ as $x(t;x_0)$ and assume solutions exist for the times under discussion.

## Why study stability?

An equilibrium is useful as an operating condition only if small initial errors produce acceptable motion. A pendulum placed exactly upright is at an equilibrium, but a tiny displacement can make it move far away. The question is what happens when the initial state is near the equilibrium rather than exactly at it.

**Intuitively, stability means that a small initial disturbance produces a response that stays small for all future time.** Returning to equilibrium is a stronger requirement. Compare three familiar cases:

- An ideal undamped pendulum near its downward equilibrium stays nearby and oscillates: stable, but not attractive.
- A damped pendulum near its downward equilibrium stays nearby and settles: asymptotically stable.
- A pendulum balanced upright can move far away after an arbitrarily small disturbance: unstable.

The initial disturbance here is a perturbation of the state; we study the subsequent zero-input motion. Persistent forcing is a separate input-response question. Later, feedback will let us change the dynamics so that a desired equilibrium becomes stable.

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

An equilibrium is **locally exponentially stable** if there exist constants $M\ge1$, $\gamma>0$, and $r>0$ such that

$$
\|x(t;x_0)-x_e\|\le M e^{-\gamma t}\|x_0-x_e\|
\quad\text{for all }t\ge0\text{ whenever }\|x_0-x_e\|<r.
$$

It is **globally exponentially stable** if the same constants $M$ and $\gamma$ work for every initial state. This gives both a decay rate and a bound on possible amplification; $M$ need not equal one.

## Why these distinctions collapse for an LTI system

For a finite-dimensional continuous-time LTI system $\dot x=Ax$, the following properties of the origin are equivalent:

$$
\begin{gathered}
\text{local attractivity}
\ \Longleftrightarrow\ \text{local asymptotic stability}
\ \Longleftrightarrow\ \text{local exponential stability}\\
\Longleftrightarrow\ \text{global asymptotic stability}
\ \Longleftrightarrow\ \text{global exponential stability}\\
\Longleftrightarrow\ \operatorname{Re}\lambda_i(A)<0\quad\text{for every }i.
\end{gathered}
$$

Here attractivity requires convergence of **every** trajectory in a neighborhood, not convergence of just one trajectory or one eigendirection.

**Why local becomes global.** Linearity gives $x(t;\alpha x_0)=\alpha x(t;x_0)$. Any initial state can be scaled into the neighborhood of attraction by choosing a sufficiently small nonzero $\alpha$. Convergence of the scaled trajectory implies convergence of the original trajectory.

**Why convergence becomes exponential.** Convergence for all initial states requires every eigenvalue to have negative real part. As the next section explains, the polynomial factors from Jordan blocks can then be bounded by a decaying exponential. Thus there are constants independent of $x_0$ such that

$$
\boxed{\|x(t)\|\le M e^{-\gamma t}\|x_0\|
\quad\text{for every }x_0\text{ and }t\ge0.}
$$

This bound also proves Lyapunov stability: choose $\delta=\varepsilon/M$. **Lyapunov stability alone is weaker**; an undamped oscillator is stable without being attractive. For nonlinear or time-varying systems, the equivalences above need not hold.

Under a constant nonsingular coordinate change $x=Pz$, stability is unchanged: $\|x\|\le\|P\|\|z\|$ and $\|z\|\le\|P^{-1}\|\|x\|$. Numerical distances and amplification factors can change with the coordinates, but the qualitative stability classification cannot.

\clearpage

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

\Needspace{13\baselineskip}

## A useful shortcut for triangular matrices

The eigenvalues of an upper- or lower-triangular matrix are its diagonal entries, including repetitions, because

$$
\det(\lambda I-A)=\prod_{i=1}^n(\lambda-a_{ii}).
$$

For a real triangular matrix, all negative diagonal entries guarantee global exponential stability; any positive diagonal entry implies instability. If all diagonal entries are nonpositive and some are zero, check the Jordan structure at zero. Off-diagonal entries do not change the eigenvalues, but they can change the Jordan structure and the transient response. A nonzero off-diagonal entry in a general triangular matrix is not, by itself, proof of a nontrivial Jordan block at zero.

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
\includegraphics[width=0.98\textwidth]{figures/week05_stability_flowchart.png}
\caption{Stability decisions for a continuous-time LTI system. After the two real-part tests, only eigenvalues on the imaginary axis require a Jordan-structure check.}
\end{figure}

For the last decision, use the eigenspace-dimension test above.

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

\Needspace{12\baselineskip}

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
\includegraphics[width=\textwidth]{figures/week05_transient_growth.png}
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

\Needspace{10\baselineskip}

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

which converge to zero while remaining no farther away than their initial state. The decay is algebraic rather than exponential: this nonlinear equilibrium is globally asymptotically stable but not exponentially stable. The second has solutions $x(t)=x_0/\sqrt{1-2x_0^2t}$ up to their finite escape time, and the origin is unstable. The zero linearization cannot distinguish them.

The LTI eigenvalue criteria also cannot be applied to the instantaneous eigenvalues of a general time-varying matrix $A(t)$. LTV stability depends on the state-transition matrix $\Phi(t,t_0)$, rather than on a collection of frozen-time portraits.

\clearpage

# From continuous time to discrete time

Digital controllers compute commands at discrete instants, and sensors often report measurements at those same instants. To relate these computations to a continuous physical plant, choose a sampling period $h>0$ and define

$$
t_k=kh,\qquad x_k=x(t_k).
$$

The sample $x_k$ is the exact state at $t_k$; it does not imply that the physical state remains constant between samples. Assume a **zero-order hold** on the input:

$$
u(t)=u_k\qquad\text{for }t_k\le t<t_{k+1}.
$$

The input is piecewise constant, while the state evolves continuously. The model derived below is exact at sampling instants under this input assumption.

## Derivation from the continuous-time solution

Starting from the forced-response formula over one sampling interval,

$$
x(t_{k+1})=e^{A(t_{k+1}-t_k)}x(t_k)
+\int_{t_k}^{t_{k+1}}e^{A(t_{k+1}-\tau)}B u(\tau)\,d\tau,
$$

substitute $t_{k+1}-t_k=h$ and $u(\tau)=u_k$. With $s=t_{k+1}-\tau$, the integral becomes

$$
\int_{t_k}^{t_{k+1}}e^{A(t_{k+1}-\tau)}B\,d\tau
=\int_0^h e^{As}B\,ds.
$$

Consequently,

$$
\boxed{x_{k+1}=A_d x_k+B_d u_k,\qquad y_k=C_d x_k+D_d u_k,}
$$

where

$$
\boxed{
A_d=e^{Ah},\qquad B_d=\int_0^h e^{As}B\,ds,\qquad C_d=C,\quad D_d=D.
}
$$

Here the output is sampled just after the new command $u_k$ is applied. This convention matters when $D\ne0$; the state is continuous, but the output can jump with the input.

If $x\in\mathbb R^n$, $u\in\mathbb R^m$, and $y\in\mathbb R^p$, then $A_d\in\mathbb R^{n\times n}$, $B_d\in\mathbb R^{n\times m}$, $C_d\in\mathbb R^{p\times n}$, and $D_d\in\mathbb R^{p\times m}$.

When $A$ is invertible, the input matrix can also be written as

$$
B_d=A^{-1}(e^{Ah}-I)B.
$$

\Needspace{9\baselineskip}

The integral formula remains valid when $A$ is singular. Do not use the inverse formula in that case. A computation that works in either case is the block matrix exponential:

$$
\exp\!\left(h\begin{bmatrix}A&B\\0&0\end{bmatrix}\right)
=\begin{bmatrix}A_d&B_d\\0&I_m\end{bmatrix}.
$$

## Exact sampling versus a numerical approximation

Forward Euler gives the approximation

$$
x_{k+1}\approx (I+hA)x_k+hB u_k.
$$

This is generally different from exact zero-order-hold discretization. For example, $\dot x=-ax$, with $a>0$, has exact sampled multiplier $e^{-ah}$, which is inside the unit circle for every $h>0$. Euler gives $1-ah$, whose magnitude is less than one only when $0<ah<2$. A numerical approximation can therefore introduce instability even when the underlying plant is asymptotically stable.

## Running example: a mass driven by a force

Following the legacy notes, let $q$ denote position, $v$ velocity, $m>0$ mass, and $u$ the applied force:

$$
m\ddot q=u,\qquad
x=\begin{bmatrix}q\\v\end{bmatrix},\qquad
A=\begin{bmatrix}0&1\\0&0\end{bmatrix},\quad
B=\begin{bmatrix}0\\1/m\end{bmatrix}.
$$

Because $A^2=0$,

$$
e^{As}=I+sA=\begin{bmatrix}1&s\\0&1\end{bmatrix}.
$$

Thus

$$
\boxed{
A_d=\begin{bmatrix}1&h\\0&1\end{bmatrix},\qquad
B_d=\begin{bmatrix}h^2/(2m)\\h/m\end{bmatrix}.
}
$$

In physical coordinates,

$$
q_{k+1}=q_k+h v_k+\frac{h^2}{2m}u_k,\qquad
v_{k+1}=v_k+\frac{h}{m}u_k.
$$

The familiar constant-acceleration formulas are exactly the sampled state equations. Euler would miss the $h^2u_k/(2m)$ contribution to the position during the current interval.

\clearpage

# Solution and stability of a discrete-time LTI system

## Expand the recurrence

For $x_{k+1}=A_d x_k+B_d u_k$, the first few states are

$$
\begin{aligned}
x_1&=A_d x_0+B_d u_0,\\
x_2&=A_d^2 x_0+A_dB_d u_0+B_d u_1,\\
x_3&=A_d^3 x_0+A_d^2B_d u_0+A_dB_d u_1+B_d u_2.
\end{aligned}
$$

Continuing this pattern gives

$$
\boxed{x_k=A_d^k x_0+\sum_{j=0}^{k-1}A_d^{k-1-j}B_d u_j.}
$$

The sum is zero when $k=0$. The first term is the **zero-input response**, and the sum is the **zero-state response**. Compare this with the continuous-time formula

$$
x(t)=e^{At}x_0+\int_0^t e^{A(t-\tau)}B u(\tau)\,d\tau.
$$

Powers of $A_d$ replace the continuous-time transition matrix, and a finite sum replaces the integral. Earlier inputs have more steps over which the dynamics can act: the contribution of $u_j$ to $x_k$ is multiplied by $A_d^{k-1-j}$.

## Stability uses the unit circle

For zero input, $x_k=A_d^k x_0$. An eigenvector mode evolves as $\mu^k v$, where $\mu$ is an eigenvalue of $A_d$.

- If every $|\mu_i|<1$, the origin is globally exponentially and asymptotically stable.
- If every $|\mu_i|\le1$ and every eigenvalue on the unit circle is semisimple, the origin is Lyapunov stable.
- If any $|\mu_i|>1$, or a unit-circle eigenvalue has a nontrivial Jordan block, the origin is unstable.

The discrete exponential bound has the form $\|x_k\|\le M\rho^k\|x_0\|$ with $M\ge1$ and $0<\rho<1$. As in continuous time, attractivity and the local/global forms of asymptotic and exponential stability are equivalent for finite-dimensional LTI systems.

For the exact sampled matrix $A_d=e^{Ah}$, the eigenvalues satisfy

$$
\boxed{\mu_i=e^{\lambda_i h},\qquad |\mu_i|=e^{h\operatorname{Re}\lambda_i}.}
$$

Therefore, a Hurwitz continuous-time matrix yields an asymptotically stable exact sampled model for every $h>0$. This statement concerns zero-input dynamics; it does not imply that every sampling period preserves controllability or observability.

\Needspace{9\baselineskip}

For the force-driven mass, $A_d$ has a size-two Jordan block at $\mu=1$, and

$$
A_d^k=\begin{bmatrix}1&kh\\0&1\end{bmatrix}.
$$

A nonzero initial velocity causes unbounded position drift. Both the continuous-time and discrete-time zero-input models are unstable at the origin, even though their eigenvalues lie on the respective stability boundaries.

\Needspace{10\baselineskip}

# Controllability: can the input move the state?

## Definition and physical meaning

A linear system is **controllable** if any initial state can be transferred to any prescribed final state in finite time by a suitable input. In continuous time, the requirement is

$$
x(t_0)=x_0\quad\longrightarrow\quad x(t_f)=x_f
$$

for arbitrary $x_0$ and $x_f$ and some finite $t_f>t_0$. For a controllable continuous-time LTI system, the transfer can be achieved over any prescribed positive-duration interval when the input is unconstrained.

In discrete time, the corresponding requirement is to find a finite number of steps $N$ and an input sequence $u_0,\ldots,u_{N-1}$ that transfers $x_0$ to $x_f$.

This definition assumes the mathematical model and inputs without amplitude constraints. It asks whether a transfer is possible, not whether it can be achieved with small force, low energy, or an acceptable transient. Those questions require additional analysis.

One actuator may control several states because its influence propagates through the dynamics. In the mass example, force directly changes velocity; velocity then changes position. We do not need a separate actuator for each state variable.

## Reachable states from the discrete-time solution

At step $N$,

$$
x_N-A_d^N x_0
=\begin{bmatrix}A_d^{N-1}B_d&A_d^{N-2}B_d&\cdots&B_d\end{bmatrix}
\begin{bmatrix}u_0\\u_1\\\vdots\\u_{N-1}\end{bmatrix}.
$$

Define the finite-horizon reachability matrix and stacked input by

$$
\mathcal R_N=\begin{bmatrix}A_d^{N-1}B_d&\cdots&A_dB_d&B_d\end{bmatrix},
\qquad U_N=\begin{bmatrix}u_0\\\vdots\\u_{N-1}\end{bmatrix}.
$$

Then the transfer problem is the linear algebra problem

$$
\boxed{\mathcal R_N U_N=x_f-A_d^N x_0.}
$$

From $x_0=0$, the reachable states form $\operatorname{range}\mathcal R_N$. From a nonzero $x_0$, they form the translated set $A_d^N x_0+\operatorname{range}\mathcal R_N$. Every target is reachable in $N$ steps exactly when $\mathcal R_N$ has row rank $n$.

## The controllability matrix

It is conventional to order the blocks in the opposite direction:

$$
\boxed{\mathcal C_d=\begin{bmatrix}B_d&A_dB_d&\cdots&A_d^{n-1}B_d\end{bmatrix}.}
$$

This only reverses block columns relative to $\mathcal R_n$, so it does not change the column space or rank. The Cayley-Hamilton theorem expresses $A_d^n$ and all higher powers as combinations of lower powers. Consequently, no new directions can be added after these $n$ blocks:

$$
\boxed{(A_d,B_d)\text{ is controllable}\quad\Longleftrightarrow\quad
\operatorname{rank}\mathcal C_d=n.}
$$

The analogous continuous-time test in the legacy notes is

$$
\mathcal C=\begin{bmatrix}B&AB&\cdots&A^{n-1}B\end{bmatrix},\qquad
(A,B)\text{ controllable}\ \Longleftrightarrow\ \operatorname{rank}\mathcal C=n.
$$

Here $B$ contains the directly actuated directions; $AB,A^2B,\ldots$ describe how those directions propagate through the dynamics. A continuous-time proof using the input integral will be developed with the controllability Gramian.

## Example: one force controls position and velocity

For the mass,

$$
\mathcal C=[B\ AB]
=\begin{bmatrix}0&1/m\\1/m&0\end{bmatrix},\qquad
\det\mathcal C=-\frac{1}{m^2}\ne0.
$$

The continuous-time model is controllable. Its sampled model satisfies

$$
\mathcal C_d=[B_d\ A_dB_d]
=\begin{bmatrix}h^2/(2m)&3h^2/(2m)\\h/m&h/m\end{bmatrix},
\qquad \det\mathcal C_d=-\frac{h^3}{m^2}\ne0.
$$

It is controllable for every $h>0$. In one step from rest, the reachable states lie on the line spanned by $B_d$. In two steps, $B_d$ and $A_dB_d$ span the plane.

For a concrete transfer, use nondimensional values $m=1$, $h=1$, start at $x_0=[0,0]^T$, and seek $x_2=[1,0]^T$. The transfer equation is

$$
\begin{bmatrix}3/2&1/2\\1&1\end{bmatrix}
\begin{bmatrix}u_0\\u_1\end{bmatrix}
=\begin{bmatrix}1\\0\end{bmatrix},
$$

so $u_0=1$ and $u_1=-1$. The first command accelerates the mass and the second brings it to rest:

$$
x_1=\begin{bmatrix}1/2\\1\end{bmatrix},\qquad
x_2=\begin{bmatrix}1\\0\end{bmatrix}.
$$

\Needspace{15\baselineskip}

## A stable system can be uncontrollable

Consider

$$
A=\begin{bmatrix}-1&0\\0&-2\end{bmatrix},\qquad
B=\begin{bmatrix}1\\0\end{bmatrix}.
$$

Both unforced modes decay, but $\dot x_2=-2x_2$ is unaffected by the input. Starting from $x_2(0)=0$, no input can produce a nonzero $x_2$. Indeed,

$$
\mathcal C=\begin{bmatrix}1&-1\\0&0\end{bmatrix}
$$

has rank one. Stability concerns the unforced motion; controllability concerns what the input can change.

\Needspace{10\baselineskip}

# Observability: can measurements reveal the state?

## Definition and indistinguishable initial states

A system is **observable** if its initial state can be uniquely determined from the known input and measured output over a finite interval. Full-state measurement at one instant is not required: the dynamics can reveal an unmeasured state through its influence on later outputs.

Two distinct initial states are **indistinguishable** if they produce exactly the same output history under the same known input. If $\Delta x_0$ is their difference, linearity gives

$$
\Delta y(t)=Ce^{At}\Delta x_0
\quad\text{or}\quad
\Delta y_k=C_dA_d^k\Delta x_0.
$$

The forced responses cancel because the input histories are identical. A system is observable precisely when the only initial-state difference that produces zero output difference throughout the observation interval is $\Delta x_0=0$.

This is an ideal, noise-free uniqueness question. A state can be theoretically observable while its reconstruction is very sensitive to measurement errors.

## Expand the discrete-time output sequence

Following the derivation in the legacy notes,

$$
\begin{aligned}
y_0&=C_d x_0+D_d u_0,\\
y_1&=C_dA_d x_0+C_dB_d u_0+D_d u_1,\\
y_2&=C_dA_d^2 x_0+C_dA_dB_d u_0+C_dB_d u_1+D_d u_2.
\end{aligned}
$$

At each step, subtract the known input contribution:

$$
\widetilde y_k
=y_k-D_d u_k-\sum_{j=0}^{k-1}C_dA_d^{k-1-j}B_d u_j
=C_dA_d^k x_0.
$$

\Needspace{12\baselineskip}

Stacking the first $n$ corrected outputs yields

$$
\underbrace{\begin{bmatrix}\widetilde y_0\\\widetilde y_1\\\vdots\\\widetilde y_{n-1}\end{bmatrix}}_{\widetilde Y}
=
\underbrace{\begin{bmatrix}C_d\\C_dA_d\\\vdots\\C_dA_d^{n-1}\end{bmatrix}}_{\mathcal O_d}
x_0.
$$

Thus observability is another linear algebra question: does $\widetilde Y=\mathcal O_d x_0$ have a unique initial-state solution?

## The observability matrix

The matrix $\mathcal O_d\in\mathbb R^{pn\times n}$ is the **observability matrix**. It has full column rank exactly when the initial state is uniquely determined:

$$
\boxed{(A_d,C_d)\text{ is observable}\quad\Longleftrightarrow\quad
\operatorname{rank}\mathcal O_d=n.}
$$

If the rank is less than $n$, a nonzero $z\in\ker\mathcal O_d$ produces the same corrected outputs for $x_0$ and $x_0+z$. By Cayley-Hamilton, higher powers of $A_d$ add no independent information, so collecting more exact samples cannot resolve this structural ambiguity.

The continuous-time counterpart in the legacy notes is

$$
\mathcal O=\begin{bmatrix}C\\CA\\\vdots\\CA^{n-1}\end{bmatrix},\qquad
(A,C)\text{ observable}\ \Longleftrightarrow\ \operatorname{rank}\mathcal O=n.
$$

For the zero-input continuous-time model, $y^{(j)}(0)=CA^j x_0$ explains the appearance of these blocks. This is a theoretical interpretation; directly differentiating noisy measurements is generally a poor reconstruction method.

## Mass example: measure position

Let $y=q$, so $C_q=[1\ 0]$ and $D=0$. Then

$$
\mathcal O_q=\begin{bmatrix}C_q\\C_qA\end{bmatrix}
=\begin{bmatrix}1&0\\0&1\end{bmatrix}.
$$

Position measurements over time reveal both position and velocity. In the sampled model,

$$
\mathcal O_{d,q}=\begin{bmatrix}C_q\\C_qA_d\end{bmatrix}
=\begin{bmatrix}1&0\\1&h\end{bmatrix},
\qquad \det\mathcal O_{d,q}=h\ne0.
$$

\Needspace{12\baselineskip}

Two samples give

$$
y_0=q_0,\qquad y_1=q_0+h v_0+\frac{h^2}{2m}u_0.
$$

Therefore,

$$
\boxed{q_0=y_0,\qquad
v_0=\frac{y_1-y_0}{h}-\frac{h}{2m}u_0.}
$$

The known input contribution must be included. For $m=h=1$, $u_0=1$, $y_0=2$, and $y_1=5.5$, the initial state is $x_0=[2,3]^T$.

Although any $h>0$ gives full rank, a small $h$ amplifies measurement errors when estimating velocity from a difference divided by $h$. Observability guarantees uniqueness in exact arithmetic, not accuracy in noisy data.

\Needspace{15\baselineskip}

## Mass example: measure only velocity

Now let $y=v$, so $C_v=[0\ 1]$. Then

$$
\mathcal O_v=\begin{bmatrix}0&1\\0&0\end{bmatrix},\qquad
\mathcal O_{d,v}=\begin{bmatrix}0&1\\0&1\end{bmatrix}.
$$

Both have rank one. Two initial states with the same velocity and different positions generate identical velocity histories under the same force. Their position difference remains constant and never appears in the output. Integrating measured velocity gives position only up to the unknown initial position.

This system is controllable with the force input but unobservable with a velocity-only sensor. Changing the sensor changes observability without changing the plant dynamics or its controllability.

\Needspace{10\baselineskip}

# Connecting the three properties

## Controllability-observability duality

For real LTI matrices,

$$
\mathcal C(A^T,C^T)
=\begin{bmatrix}C^T&A^TC^T&\cdots&(A^T)^{n-1}C^T\end{bmatrix}
=\mathcal O(A,C)^T.
$$

Since a matrix and its transpose have the same rank,

$$
\boxed{(A,C)\text{ is observable}\quad\Longleftrightarrow\quad
(A^T,C^T)\text{ is controllable}.}
$$

The same statement holds with $A_d,C_d$. This is a mathematical relation between a system and its **dual**. It does not mean that controllability of $(A,B)$ implies observability of $(A,C)$ for an arbitrary sensor matrix $C$.

\Needspace{19\baselineskip}

## What each matrix tells us

For continuous-time systems, the roles can be summarized as follows; use $A_d,B_d,C_d$ and the unit-circle criterion in discrete time.

| Question | Calculation | Interpretation |
|---|---|---|
| Do all zero-input trajectories decay? | Eigenvalues of $A$ | Strictly negative real parts give global exponential stability. |
| Can inputs reach every state direction? | Column space of $\mathcal C$ | Full row rank means controllability. |
| Can outputs distinguish every initial state? | Null space of $\mathcal O$ | A trivial null space means observability. |

For the force-driven mass, zero-input position drift makes the origin unstable, force makes the system controllable, and measuring position makes the system observable. Replacing the position sensor with a velocity sensor destroys observability. None of these conclusions conflicts with the others.

A coordinate change does not alter these properties. With $x=Pz$ and nonsingular $P$,

$$
\overline A=P^{-1}AP,\quad \overline B=P^{-1}B,\quad
\overline C=CP,
$$

and

$$
\overline{\mathcal C}=P^{-1}\mathcal C,\qquad
\overline{\mathcal O}=\mathcal O P.
$$

The eigenvalues and relevant ranks are unchanged, even though numerical conditioning can change.

\clearpage

# Concept checks and practice

1. Explain the difference between "stays near equilibrium" and "returns to equilibrium." Give a stable system that is not attractive.
2. For a finite-dimensional LTI system, why does attractivity in a neighborhood imply global exponential stability? Why does convergence of a single trajectory not suffice?
3. Classify the origin for each matrix below. Which decision needs more than the signs of the eigenvalue real parts?

   $$
   A_1=\begin{bmatrix}-1&4\\0&-2\end{bmatrix},\quad
   A_2=\begin{bmatrix}0&0\\0&-1\end{bmatrix},\quad
   A_3=\begin{bmatrix}0&1\\0&0\end{bmatrix}.
   $$

4. Derive the exact sampled model for $\dot x=-2x+u$ with sampling period $h$. Compare its zero-input multiplier with forward Euler.
5. Starting from the recurrence, derive the expression for $x_3$. Explain why the earliest input has the largest power of $A_d$ multiplying its contribution.
6. For the mass with $m=h=1$, find two constant force commands that transfer $[0,0]^T$ to $[2,0]^T$ in two steps.
7. Explain geometrically why one input step cannot reach every state of the mass from rest, but two steps can.
8. For the mass with $m=h=1$, suppose $u_0=2$ and the position measurements are $y_0=1$ and $y_1=4$. Reconstruct the initial position and velocity.
9. Give two different initial states of the mass that cannot be distinguished using only velocity measurements under the same known input.
10. Does a controllable system have to be stable? Does an observable system require one sensor for each state? Use the examples in these notes to justify your answers.

\clearpage

## Brief answers

1. Stability controls distance for all future time; attraction requires convergence. An ideal undamped oscillator is stable but not attractive.
2. Scaling makes attraction global, and decay of every LTI mode forces a Hurwitz matrix, which gives an exponential bound. A saddle has convergent trajectories on its stable eigenspace but is unstable.
3. $A_1$ is globally exponentially stable. $A_2$ is stable but not attractive at the origin; its zero eigenvalue is semisimple. $A_3$ is unstable because of its size-two block at zero. Boundary eigenvalues require the additional structure check.
4. $A_d=e^{-2h}$ and $B_d=(1-e^{-2h})/2$. Euler gives $1-2h$ and $h$; its zero-input model is asymptotically stable only for $0<h<1$.
5. $x_3=A_d^3x_0+A_d^2B_du_0+A_dB_du_1+B_du_2$. The first input acts on $x_1$ and is propagated through two further updates.
6. $u_0=2$, $u_1=-2$; the intermediate state is $[1,2]^T$.
7. One step gives multiples of $B_d$; two steps give combinations of the independent columns $A_dB_d$ and $B_d$.
8. $q_0=1$ and $v_0=(4-1)-2/2=2$.
9. For example, $[0,1]^T$ and $[5,1]^T$ have identical velocity histories.
10. No to both: the force-driven mass is unstable but controllable, and its single position output makes both states observable.

# Source notes and continuation

The stability definitions, Jordan-block discussion, flowchart, worked decisions, and transient-growth figure are carried forward from the Fall 2026 Week 4 course notes. These notes add the intuitive motivation, the explicit LTI stability equivalences, and the triangular-matrix shortcut for finite-dimensional LTI systems.

The discrete-time and input/output material follows these handwritten legacy notes:

- **October 28, 2015**, *Solution of linear systems*: PDF page 1 derives the zero-order-hold model; page 2 expands the discrete-time state solution and defines controllability; page 5 introduces the force-driven mass and its controllability matrix.
- **November 4, 2015**, *Observability*: PDF page 1 defines observability, expands the output sequence, constructs the observability matrix, and establishes duality; page 2 compares position and velocity sensors for the force-driven mass.

The exact two-step force transfer, sampled position reconstruction, and practice questions adapt these legacy examples to the notation used here. Later lectures will develop controllability and observability Gramians, minimum-energy transfer, PBH tests, and system decompositions.
