---
title: "MAE 6246 - Linear Systems and Control"
subtitle: "Week 3: Solutions of Continuous-Time Linear Systems"
author: "Fall 2026"
date: ""
geometry: margin=0.78in
fontsize: 11pt
toc: true
numbersections: true
colorlinks: true
linkcolor: blue
urlcolor: blue
header-includes:
  - \usepackage{amsmath,amssymb,bm}
  - \usepackage{booktabs}
  - \usepackage{microtype}
---

**Lecture:** Friday, September 11, 2026

**Topics:** homogeneous and forced responses; zero-input and zero-state responses; matrix exponential; Laplace resolvent; fundamental matrices; state-transition matrices

---

# Learning objectives

By the end of the lecture, you should be able to:

1. derive the solution of an LTI initial-value problem;
2. separate a response into zero-input and zero-state components;
3. state and use the defining properties of the matrix exponential;
4. connect the time-domain solution with the Laplace resolvent $(sI-A)^{-1}$;
5. distinguish a fundamental matrix from the state-transition matrix;
6. state the identity, inverse, composition, and differential properties of a state-transition matrix.

The central message is that the state at time $t$ is the sum of the propagated initial condition and the accumulated effect of the input. The state-transition matrix is the object that performs both operations.

---

# Continuous-time linear systems

A linear time-varying (LTV) state model has the form

$$
\dot x(t)=A(t)x(t)+B(t)u(t),
\qquad
y(t)=C(t)x(t)+D(t)u(t),
$$

where the matrices may depend explicitly on time. A linear time-invariant (LTI) model is the special case

$$
\boxed{
\dot x(t)=Ax(t)+Bu(t),
\qquad
y(t)=Cx(t)+Du(t),
}
$$

with constant matrices.

For a specified initial time $t_0$, initial state $x(t_0)=x_0$, and input $u(t)$, the problem is to determine the unique trajectory $x(t)$ satisfying the differential equation and the initial condition.

The adjective *linear* refers to the dependence on $x$ and $u$. Time variation alone does not make a system nonlinear. Nevertheless, formulas that rely on a constant matrix $A$ cannot be transferred automatically to the LTV case.

---

# Homogeneous LTI response and the matrix exponential

First set the input to zero:

$$
\dot x=Ax,
\qquad
x(t_0)=x_0.
$$

The scalar equation $\dot z=az$ has solution $z(t)=e^{a(t-t_0)}z(t_0)$. Its matrix counterpart is

$$
\boxed{x(t)=e^{A(t-t_0)}x_0.}
$$

The matrix exponential is defined by the convergent power series

$$
\boxed{
e^{At}
=
I+At+\frac{A^2t^2}{2!}
+\frac{A^3t^3}{3!}+\cdots .
}
$$

Term-by-term differentiation gives

$$
\frac{d}{dt}e^{At}
=Ae^{At}
=e^{At}A,
$$

and therefore $x(t)=e^{A(t-t_0)}x_0$ satisfies both the differential equation and the initial condition.

## Basic properties

For a fixed matrix $A$,

$$
e^{A0}=I,
\qquad
e^{A(t_2-t_0)}
=e^{A(t_2-t_1)}e^{A(t_1-t_0)},
\qquad
\left(e^{At}\right)^{-1}=e^{-At}.
$$

In general,

$$
e^{(A+B)t}\ne e^{At}e^{Bt}.
$$

The equality holds when $A$ and $B$ commute, meaning $AB=BA$.

## Computing the matrix exponential from structure

If $A=P\Lambda P^{-1}$ is diagonalizable, then

$$
\boxed{
e^{At}=Pe^{\Lambda t}P^{-1},
\qquad
e^{\Lambda t}
=\operatorname{diag}(e^{\lambda_1t},\ldots,e^{\lambda_nt}).
}
$$

For a Jordan block $J=\lambda I+N$, where $N^m=0$,

$$
e^{Jt}
=e^{\lambda t}
\left(
I+Nt+\frac{N^2t^2}{2!}
+\cdots+
\frac{N^{m-1}t^{m-1}}{(m-1)!}
\right).
$$

Thus defective modes contain polynomial factors multiplying $e^{\lambda t}$. In numerical work, use a stable matrix-exponential routine such as **scipy.linalg.expm**; explicit Jordan-form computation is primarily a conceptual tool.

**Example: a $3\times3$ Jordan block.** Consider

$$
J=
\begin{bmatrix}
\lambda&1&0\\
0&\lambda&1\\
0&0&\lambda
\end{bmatrix}
=\lambda I+N,
\qquad
N=
\begin{bmatrix}
0&1&0\\
0&0&1\\
0&0&0
\end{bmatrix}.
$$

The nilpotent part satisfies

$$
N^2=
\begin{bmatrix}
0&0&1\\
0&0&0\\
0&0&0
\end{bmatrix},
\qquad
N^3=0.
$$

Because $\lambda I$ commutes with $N$,

$$
e^{Jt}
=e^{(\lambda I+N)t}
=e^{\lambda t}e^{Nt}.
$$

The exponential series for $N$ terminates after the quadratic term:

$$
e^{Nt}=I+tN+\frac{t^2}{2}N^2.
$$

Therefore,

$$
\boxed{
e^{Jt}
=e^{\lambda t}
\begin{bmatrix}
1&t&\dfrac{t^2}{2}\\
0&1&t\\
0&0&1
\end{bmatrix}.
}
$$

The entries on successive superdiagonals contain successively higher powers of $t$.

### Diagonalizable example from Week 2

Consider

$$
A=
\begin{bmatrix}
3&-2\\
-1&4
\end{bmatrix},
\qquad
P=
\begin{bmatrix}
2&1\\
1&-1
\end{bmatrix},
\qquad
\Lambda=
\begin{bmatrix}
2&0\\
0&5
\end{bmatrix}.
$$

As obtained in Week 2,

$$
\Lambda=P^{-1}AP,
\qquad
A=P\Lambda P^{-1},
$$

where

$$
P^{-1}
=
\frac{1}{3}
\begin{bmatrix}
1&1\\
1&-2
\end{bmatrix}.
$$

The power-series definition of the exponential gives

$$
e^{At}
=
P e^{\Lambda t}P^{-1}
=
P
\begin{bmatrix}
e^{2t}&0\\
0&e^{5t}
\end{bmatrix}
P^{-1}.
$$

Multiplying the three matrices yields

$$
\boxed{
e^{At}
=
\frac{1}{3}
\begin{bmatrix}
2e^{2t}+e^{5t}&2e^{2t}-2e^{5t}\\
e^{2t}-e^{5t}&e^{2t}+2e^{5t}
\end{bmatrix}.
}
$$

At $t=0$, this matrix is $I$. Differentiating it and evaluating at $t=0$ gives $A$, providing two quick checks of the calculation.

### Defective example from Week 2

Now consider

$$
A=
\begin{bmatrix}
3&1\\
-1&1
\end{bmatrix}.
$$

Its characteristic polynomial is $(\lambda-2)^2$. An eigenvector and a generalized eigenvector can be chosen as

$$
v_1=
\begin{bmatrix}1\\-1\end{bmatrix},
\qquad
v_2=
\begin{bmatrix}1\\0\end{bmatrix},
$$

because

$$
(A-2I)v_1=0,
\qquad
(A-2I)v_2=v_1.
$$

With

$$
P=
\begin{bmatrix}
1&1\\
-1&0
\end{bmatrix},
\qquad
P^{-1}=
\begin{bmatrix}
0&-1\\
1&1
\end{bmatrix},
$$

the Jordan form is

$$
J=P^{-1}AP
=
\begin{bmatrix}
2&1\\
0&2
\end{bmatrix}
=2I+N,
\qquad
N^2=0.
$$

Therefore,

$$
e^{Jt}
=e^{2t}e^{Nt}
=e^{2t}(I+tN)
=e^{2t}
\begin{bmatrix}
1&t\\
0&1
\end{bmatrix},
$$

and

$$
\boxed{
e^{At}
=P e^{Jt}P^{-1}
=e^{2t}
\begin{bmatrix}
1+t&t\\
-t&1-t
\end{bmatrix}.
}
$$

Equivalently, $A=2I+(A-2I)$ and $(A-2I)^2=0$, so the same result follows directly from

$$
e^{At}=e^{2t}\bigl[I+t(A-2I)\bigr].
$$

---

# Forced LTI response

Return to

$$
\dot x=Ax+Bu.
$$

Multiply by $e^{-At}$ and use the product rule:

$$
\frac{d}{dt}\left(e^{-At}x(t)\right)
=e^{-At}\bigl(\dot x-Ax\bigr)
=e^{-At}Bu(t).
$$

Integrating from $t_0$ to $t$ gives

$$
e^{-At}x(t)-e^{-At_0}x(t_0)
=
\int_{t_0}^{t}e^{-A\tau}Bu(\tau)\,d\tau.
$$

Multiplying by $e^{At}$ yields the variation-of-constants formula

$$
\boxed{
x(t)
=
e^{A(t-t_0)}x_0
+
\int_{t_0}^{t}
e^{A(t-\tau)}Bu(\tau)\,d\tau .
}
$$

The two terms have different physical meanings.

## Zero-input and zero-state responses

The **zero-input response** is caused only by the initial condition:

$$
\boxed{
x_{\mathrm{zi}}(t)=e^{A(t-t_0)}x_0.
}
$$

The **zero-state response** is caused only by the input:

$$
\boxed{
x_{\mathrm{zs}}(t)
=
\int_{t_0}^{t}e^{A(t-\tau)}Bu(\tau)\,d\tau.
}
$$

By linearity,

$$
\boxed{x(t)=x_{\mathrm{zi}}(t)+x_{\mathrm{zs}}(t).}
$$

The output follows directly:

$$
y(t)
=
Ce^{A(t-t_0)}x_0
+
\int_{t_0}^{t}Ce^{A(t-\tau)}Bu(\tau)\,d\tau
+Du(t).
$$

The kernel

$$
G(t)=Ce^{At}B,\qquad t\ge0,
$$

is the strictly proper part of the impulse response. The term $Du(t)$ is the direct feedthrough.

---

# Laplace-transform and resolvent derivation

Let $t_0=0$ and denote the unilateral Laplace transforms by $X(s)$ and $U(s)$. Since

$$
\mathcal L\{\dot x\}=sX(s)-x(0),
$$

the state equation gives

$$
sX(s)-x(0)=AX(s)+BU(s).
$$

Therefore,

$$
(sI-A)X(s)=x(0)+BU(s),
$$

and

$$
\boxed{
X(s)
=(sI-A)^{-1}x(0)
+(sI-A)^{-1}BU(s).
}
$$

The matrix

$$
\boxed{R(s;A)=(sI-A)^{-1}}
$$

is the **resolvent** of $A$. Its poles occur at the eigenvalues of $A$ because

$$
\det(sI-A)=0
$$

at those values. The identity

$$
\mathcal L\{e^{At}\}=(sI-A)^{-1}
$$

connects the time-domain matrix exponential to the frequency-domain resolvent.

## Convolution theorem

Let

$$
F(s)=\mathcal L\{f(t)\},
\qquad
G(s)=\mathcal L\{g(t)\}.
$$

The convolution theorem states that

$$
\boxed{
\mathcal L\{(f*g)(t)\}=F(s)G(s),
}
$$

where

$$
\boxed{
(f*g)(t)
=\int_0^t f(t-\tau)g(\tau)\,d\tau.
}
$$

Equivalently,

$$
\boxed{
\mathcal L^{-1}\{F(s)G(s)\}
=\int_0^t f(t-\tau)g(\tau)\,d\tau.
}
$$

Apply this identity to the forced term in $X(s)$. Since

$$
\mathcal L^{-1}\{(sI-A)^{-1}B\}=e^{At}B,
$$

we obtain

$$
\boxed{
\mathcal L^{-1}\{(sI-A)^{-1}BU(s)\}
=\int_0^t e^{A(t-\tau)}Bu(\tau)\,d\tau.
}
$$

Thus, the product in the Laplace domain becomes the zero-state convolution integral in the time domain.

For zero initial condition, the transfer matrix is

$$
\boxed{G(s)=C(sI-A)^{-1}B+D.}
$$

The transfer matrix describes input-output behavior, whereas the state-transition solution retains the internal state and the effect of the initial condition.

---

# Fundamental matrices for homogeneous LTV systems

Consider the homogeneous LTV system

$$
\dot x=A(t)x.
$$

The set of its solutions is a linear space: any linear combination of solutions is again a solution. Choose $n$ linearly independent solutions

$$
\psi_1(t),\ldots,\psi_n(t)
$$

and place them into the columns of

$$
\Psi(t)
=
\begin{bmatrix}
\psi_1(t)&\cdots&\psi_n(t)
\end{bmatrix}.
$$

Then

$$
\boxed{\dot\Psi(t)=A(t)\Psi(t).}
$$

If $\Psi(t)$ is nonsingular, it is a **fundamental matrix**. A fundamental matrix is not unique: multiplying it on the right by any constant nonsingular matrix produces another fundamental matrix.

---

# State-transition matrix

Given any fundamental matrix $\Psi(t)$, define

$$
\boxed{
\Phi(t,t_0)=\Psi(t)\Psi^{-1}(t_0).
}
$$

Although $\Psi$ is not unique, $\Phi(t,t_0)$ is unique for a given $A(t)$. It maps a state at $t_0$ to the corresponding homogeneous state at $t$:

$$
\boxed{x(t)=\Phi(t,t_0)x(t_0).}
$$

## Defining properties

The state-transition matrix satisfies

$$
\boxed{\Phi(t_0,t_0)=I}
$$

and the composition property

$$
\boxed{
\Phi(t_2,t_0)
=
\Phi(t_2,t_1)\Phi(t_1,t_0).
}
$$

Consequently,

$$
\boxed{
\Phi^{-1}(t,t_0)=\Phi(t_0,t).
}
$$

Its derivatives with respect to the first and second time arguments are

$$
\boxed{
\frac{\partial}{\partial t}\Phi(t,t_0)
=A(t)\Phi(t,t_0),
}
$$

$$
\boxed{
\frac{\partial}{\partial t_0}\Phi(t,t_0)
=-\Phi(t,t_0)A(t_0).
}
$$

## Worked LTV example

Consider the homogeneous time-varying system

$$
\dot x=A(t)x,
\qquad
A(t)=
\begin{bmatrix}
0&0\\
t&0
\end{bmatrix}.
$$

Its component equations are

$$
\dot x_1=0,
\qquad
\dot x_2=t x_1.
$$

Because $x_1$ is constant, integration gives

$$
x_1(t)=c_1,
\qquad
x_2(t)=\frac{1}{2}c_1t^2+c_2.
$$

Two linearly independent solutions are

$$
\psi_1(t)=
\begin{bmatrix}0\\1\end{bmatrix},
\qquad
\psi_2(t)=
\begin{bmatrix}2\\t^2\end{bmatrix}.
$$

Placing them in the columns produces the fundamental matrix

$$
\Psi(t)=
\begin{bmatrix}
0&2\\
1&t^2
\end{bmatrix}.
$$

Its determinant is $-2$, so it is nonsingular for every $t$. At the initial time $t_0$,

$$
\Psi^{-1}(t_0)
=
\begin{bmatrix}
-\dfrac{t_0^2}{2}&1\\[2mm]
\dfrac{1}{2}&0
\end{bmatrix}.
$$

Therefore,

$$
\begin{aligned}
\Phi(t,t_0)
&=\Psi(t)\Psi^{-1}(t_0)\\[1mm]
&=
\begin{bmatrix}
0&2\\
1&t^2
\end{bmatrix}
\begin{bmatrix}
-\dfrac{t_0^2}{2}&1\\[2mm]
\dfrac{1}{2}&0
\end{bmatrix}\\[1mm]
&=
\boxed{
\begin{bmatrix}
1&0\\[1mm]
\dfrac{t^2-t_0^2}{2}&1
\end{bmatrix}}.
\end{aligned}
$$

The result satisfies the identity property,

$$
\Phi(t_0,t_0)=I,
$$

and its time derivative is

$$
\frac{\partial}{\partial t}\Phi(t,t_0)
=
\begin{bmatrix}
0&0\\
t&0
\end{bmatrix}
=A(t)\Phi(t,t_0).
$$

The lower-left entry also makes composition transparent:

$$
\frac{t_2^2-t_0^2}{2}
=
\frac{t_2^2-t_1^2}{2}
+
\frac{t_1^2-t_0^2}{2},
$$

which implies

$$
\Phi(t_2,t_0)
=\Phi(t_2,t_1)\Phi(t_1,t_0).
$$

## Forced LTV solution

For

$$
\dot x=A(t)x+B(t)u(t),
$$

variation of constants gives

$$
\boxed{
x(t)
=
\Phi(t,t_0)x_0
+
\int_{t_0}^{t}
\Phi(t,\tau)B(\tau)u(\tau)\,d\tau.
}
$$

This is the same initial-condition-plus-input structure as the LTI formula.

## LTI specialization

When $A(t)=A$ is constant,

$$
\boxed{\Phi(t,t_0)=e^{A(t-t_0)}.}
$$

The dependence reduces from two separate times to the elapsed time $t-t_0$. This time-shift property is a consequence of time invariance.

---

# Constant-input example

For a nonsingular $A$ and constant input $u(t)=u_0$,

$$
x(t)=e^{At}x_0+\int_0^t e^{A(t-\tau)}Bu_0\,d\tau.
$$

Changing variables and integrating gives

$$
x(t)
=e^{At}x_0+A^{-1}(e^{At}-I)Bu_0.
$$

The equilibrium associated with $u_0$ is

$$
x^\star=-A^{-1}Bu_0,
$$

so the solution can be written compactly as

$$
\boxed{
x(t)=x^\star+e^{At}\bigl(x_0-x^\star\bigr).
}
$$

This expression says that the displacement from equilibrium evolves according to the homogeneous dynamics.

---

# Common mistakes

1. **Dropping the initial condition in the Laplace transform.**\
   The correct identity is $\mathcal L\{\dot x\}=sX(s)-x(0)$.

2. **Reversing the kernel in the forced response.**\
   The input applied at time $\tau$ propagates to time $t$ through $e^{A(t-\tau)}$ or $\Phi(t,\tau)$.

3. **Treating $e^{A(t-\tau)}$ as a scalar exponential.**\
   Matrix multiplication order matters.

4. **Assuming $e^{A+B}=e^Ae^B$ without checking commutativity.**

5. **Calling every fundamental matrix the state-transition matrix.**\
   A fundamental matrix is not unique. The normalized combination $\Psi(t)\Psi^{-1}(t_0)$ is the unique state-transition matrix.

6. **Using $e^{\int A(t)\,dt}$ for a general LTV system.**\
   This shortcut is generally invalid when $A(t_1)$ and $A(t_2)$ do not commute.

---

# Concept checks

1. Why does $e^{A(t-t_0)}x_0$ satisfy the initial condition?
2. Which part of the response remains when $u=0$?
3. Which part remains when $x(t_0)=0$?
4. Why does the resolvent become singular at an eigenvalue of $A$?
5. If $\Psi(t)$ is a fundamental matrix, why is $\Psi(t)M$ also fundamental for every constant nonsingular $M$?
6. Use composition to show that $\Phi(t,t_0)^{-1}=\Phi(t_0,t)$.
7. What additional condition permits $e^{(A+B)t}=e^{At}e^{Bt}$?

---

# Computational companion

The Week 3 notebook illustrates:

- homogeneous motion computed with **scipy.linalg.expm** and **solve_ivp**;
- zero-input and zero-state decomposition;
- forced response computed by numerical integration, convolution, and **scipy.signal.lsim**;
- identity, inverse, and composition properties of state-transition matrices;
- numerical construction of an LTV state-transition matrix; and
- transient amplification in a stable non-normal system.

\clearpage

# Summary

For an LTI system,

$$
\boxed{
x(t)
=
e^{A(t-t_0)}x_0
+
\int_{t_0}^{t}e^{A(t-\tau)}Bu(\tau)\,d\tau.
}
$$

For an LTV system,

$$
\boxed{
x(t)
=
\Phi(t,t_0)x_0
+
\int_{t_0}^{t}\Phi(t,\tau)B(\tau)u(\tau)\,d\tau.
}
$$

The matrix exponential is the LTI state-transition matrix, and its Laplace transform is the resolvent $(sI-A)^{-1}$. These formulas separate initial-condition effects from input effects and provide the foundation for controllability, observability, feedback, and estimation.
