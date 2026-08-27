# MAE 6246 - Linear Systems and Control

## Week 1: State-Space Models and Linearization

**Lecture:** Friday, August 28, 2026  
**Topics:** state, input, and output; nonlinear and linear state-space models; equilibria; Jacobian linearization; vector spaces, span, linear independence, basis, and coordinates

---

## 1. Learning objectives

By the end of this week, you should be able to:

1. identify a valid state for a dynamical system;
2. write a nonlinear model in first-order state-space form;
3. distinguish nonlinear, linear time-varying, and linear time-invariant models;
4. test whether a candidate operating point is an equilibrium;
5. derive a local linear model using Jacobian matrices;
6. interpret state vectors as elements of a vector space; and
7. determine whether vectors span a space, are linearly independent, or form a basis.

The pendulum will be our running example. It is simple enough to derive by hand, but it already exposes an essential idea: **one nonlinear system can have different linear models near different operating points.**

---

## 2. Why state space?

In a classical input-output description, a system is represented by a differential equation or, for an LTI system with zero initial conditions, a transfer function

$$
G(s)=\frac{Y(s)}{U(s)}.
$$

This description is valuable, but it emphasizes the relation between an input and an output. A state-space description also records the system's internal condition.

State-space models are especially useful when:

- the system has multiple inputs or multiple outputs;
- initial conditions matter;
- the model is nonlinear or time varying;
- internal variables must be estimated;
- feedback will use several physical variables; or
- controllability, observability, optimal control, or state estimation is of interest.

### 2.1 Input, state, and output

Consider a dynamical system with

$$
u(t)\in\mathbb{R}^m,\qquad
x(t)\in\mathbb{R}^n,\qquad
y(t)\in\mathbb{R}^p.
$$

- $u(t)$ is the **input**: an externally applied command or disturbance.
- $x(t)$ is the **state**: the information needed at time $t$ to determine the future behavior once the future input is known.
- $y(t)$ is the **output**: the quantity measured or otherwise selected for observation.

More precisely, $x(t_0)$ is a state if $x(t_0)$ together with $u(t)$ for $t\ge t_0$ uniquely determines the system response for $t\ge t_0$.

The state is not necessarily the output. For example, a position sensor may report angle while angular velocity remains an unmeasured state.

### 2.2 General nonlinear state-space model

A continuous-time nonlinear system can be written as

$$
\dot{x}(t)=f\bigl(t,x(t),u(t)\bigr),
\qquad
y(t)=g\bigl(t,x(t),u(t)\bigr).
$$

The state equation determines the evolution of $x$. The output equation determines what is reported or measured.

Many physical models begin as second-order equations. A state-space model rewrites them as a set of coupled first-order equations. This is not an approximation; it is a change of representation.

---

## 3. Running example: torque-driven pendulum

Consider a point mass $m$ attached to a massless rod of length $\ell$. Let $\theta=0$ denote the downward-hanging position, and let $u$ be the torque applied at the pivot. The equation of motion is

$$
m\ell^2\ddot{\theta}=-mg\ell\sin\theta+u.
$$

Choose the state

$$
x=
\begin{bmatrix}
x_1\\x_2
\end{bmatrix}
=
\begin{bmatrix}
\theta\\\dot{\theta}
\end{bmatrix}.
$$

Then

$$
\dot{x}=
f(x,u)
=
\begin{bmatrix}
x_2\\
-\dfrac{g}{\ell}\sin x_1+\dfrac{1}{m\ell^2}u
\end{bmatrix}.
$$

If only the angle is measured, the output equation is

$$
y=x_1.
$$

This model is nonlinear because $\sin x_1$ is a nonlinear function of the state.

> **Modeling note.** A different definition of positive angle or positive torque changes some signs in the equations. The equations must be internally consistent; the physical convention is a choice.

### 3.1 State is not unique

The choice $x=[\theta,\dot\theta]^T$ is physically natural, but it is not the only valid state. Any invertible coordinate transformation of a valid state gives another valid state. We will formalize this statement when we study change of basis and similarity transformations.

---

## 4. Linear systems and superposition

A system is linear when it satisfies **superposition**. If initial condition $\phi_1$ and input $u_1$ produce state $x_1$ and output $y_1$, while $\phi_2$ and $u_2$ produce $x_2$ and $y_2$, then

$$
\alpha\phi_1+\beta\phi_2,
\qquad
\alpha u_1+\beta u_2
$$

must produce

$$
\alpha x_1+\beta x_2,
\qquad
\alpha y_1+\beta y_2
$$

for every pair of scalars $\alpha,\beta$.

Superposition contains two familiar properties:

- **homogeneity:** scaling the initial condition and input scales the response;
- **additivity:** the response to a sum is the sum of the responses.

The pendulum does not satisfy superposition because, in general,

$$
\sin(\alpha\theta_1+\beta\theta_2)
\ne
\alpha\sin\theta_1+\beta\sin\theta_2.
$$

### 4.1 Linear time-varying model

The general continuous-time linear state-space model is

$$
\dot{x}(t)=A(t)x(t)+B(t)u(t),
\qquad
y(t)=C(t)x(t)+D(t)u(t),
$$

where

$$
A(t)\in\mathbb{R}^{n\times n},\quad
B(t)\in\mathbb{R}^{n\times m},\quad
C(t)\in\mathbb{R}^{p\times n},\quad
D(t)\in\mathbb{R}^{p\times m}.
$$

The matrices may depend on time, but they do not depend on $x$ or $u$.

### 4.2 Linear time-invariant model

If the matrices are constant, the model is linear time invariant (LTI):

$$
\boxed{
\dot{x}=Ax+Bu,
\qquad
y=Cx+Du.
}
$$

The matrix roles are:

| Matrix | Role |
|---|---|
| $A$ | autonomous state dynamics |
| $B$ | how the input enters the state dynamics |
| $C$ | how the state appears in the output |
| $D$ | direct input-to-output feedthrough |

A nonlinear system is not globally converted into an LTI system merely by writing it in matrix notation. Linearization gives a **local approximation**, which is the subject of the next sections.

---

## 5. Equilibria and operating points

For an autonomous system with zero input,

$$
\dot{x}=f(x,0),
$$

a point $x^\star$ is an equilibrium when a trajectory starting at $x^\star$ remains there:

$$
x(t_0)=x^\star
\quad\Longrightarrow\quad
x(t)=x^\star\quad\text{for all }t\ge t_0.
$$

Equivalently,

$$
\boxed{f(x^\star,0)=0.}
$$

More generally, a constant pair $(x^\star,u^\star)$ is an equilibrium operating point when

$$
\boxed{f(x^\star,u^\star)=0.}
$$

The corresponding steady output is

$$
y^\star=g(x^\star,u^\star).
$$

### 5.1 Pendulum equilibria

With $u^\star=0$, the equilibrium conditions are

$$
x_2^\star=0,
\qquad
\sin x_1^\star=0.
$$

Thus the equilibria repeat every $\pi$ radians. Two physically distinct cases are

$$
x_d^\star=
\begin{bmatrix}0\\0\end{bmatrix}
\quad\text{(downward)},
\qquad
x_u^\star=
\begin{bmatrix}\pi\\0\end{bmatrix}
\quad\text{(upright)}.
$$

The same nonlinear equation governs both, but their nearby behavior is very different.

---

## 6. Linearization about an equilibrium

Suppose

$$
\dot{x}=f(x,u),
\qquad
y=g(x,u),
$$

and $(x^\star,u^\star)$ is an equilibrium. Define perturbation variables

$$
\delta x=x-x^\star,
\qquad
\delta u=u-u^\star,
\qquad
\delta y=y-y^\star.
$$

A first-order Taylor expansion gives

$$
f(x^\star+\delta x,u^\star+\delta u)
\approx
f(x^\star,u^\star)
+
\left.\frac{\partial f}{\partial x}\right|_\star\delta x
+
\left.\frac{\partial f}{\partial u}\right|_\star\delta u.
$$

Because $f(x^\star,u^\star)=0$, the perturbation dynamics are

$$
\delta\dot{x}=A\delta x+B\delta u,
\qquad
\delta y=C\delta x+D\delta u,
$$

where

$$
\boxed{
A=\left.\frac{\partial f}{\partial x}\right|_{(x^\star,u^\star)},
\qquad
B=\left.\frac{\partial f}{\partial u}\right|_{(x^\star,u^\star)},
}
$$

$$
\boxed{
C=\left.\frac{\partial g}{\partial x}\right|_{(x^\star,u^\star)},
\qquad
D=\left.\frac{\partial g}{\partial u}\right|_{(x^\star,u^\star)}.
}
$$

The symbol $|_\star$ means that the Jacobian is evaluated at the selected operating point.

### 6.1 Why perturbation variables matter

The linear state is $\delta x$, not necessarily the original physical state $x$. Near the upright pendulum, for example,

$$
\delta x_1=\theta-\pi,
$$

so $\delta x_1=0$ means the pendulum is upright, not downward.

Dropping the $\delta$ notation after the operating point has been clearly stated is common, but the coordinate shift must not be forgotten.

### 6.2 Accuracy of the approximation

The neglected Taylor terms are second order and higher. Therefore, the approximation improves as $(\delta x,\delta u)$ becomes smaller. Linearization is not a statement that the nonlinear model has become linear everywhere.

---

## 7. Pendulum linearizations

For the pendulum,

$$
f(x,u)=
\begin{bmatrix}
x_2\\
-\dfrac{g}{\ell}\sin x_1+\dfrac{1}{m\ell^2}u
\end{bmatrix}.
$$

Its Jacobians are

$$
\frac{\partial f}{\partial x}
=
\begin{bmatrix}
0&1\\
-\dfrac{g}{\ell}\cos x_1&0
\end{bmatrix},
\qquad
\frac{\partial f}{\partial u}
=
\begin{bmatrix}
0\\
\dfrac{1}{m\ell^2}
\end{bmatrix}.
$$

For the output $y=x_1$,

$$
C=\begin{bmatrix}1&0\end{bmatrix},
\qquad
D=0.
$$

### 7.1 Downward equilibrium

At $x_d^\star=[0,0]^T$,

$$
A_d=
\begin{bmatrix}
0&1\\
-\dfrac{g}{\ell}&0
\end{bmatrix},
\qquad
B_d=
\begin{bmatrix}
0\\
\dfrac{1}{m\ell^2}
\end{bmatrix}.
$$

Equivalently, the small perturbation angle satisfies

$$
\delta\ddot{\theta}+\frac{g}{\ell}\delta\theta
=
\frac{1}{m\ell^2}\delta u.
$$

The gravitational term points back toward the equilibrium, so it is locally restoring.

### 7.2 Upright equilibrium

At $x_u^\star=[\pi,0]^T$, since $\cos\pi=-1$,

$$
A_u=
\begin{bmatrix}
0&1\\
+\dfrac{g}{\ell}&0
\end{bmatrix},
\qquad
B_u=
\begin{bmatrix}
0\\
\dfrac{1}{m\ell^2}
\end{bmatrix}.
$$

Now

$$
\delta\ddot{\theta}-\frac{g}{\ell}\delta\theta
=
\frac{1}{m\ell^2}\delta u.
$$

The gravitational term pushes a small displacement farther away. This is the operating point that will later motivate feedback stabilization.

### 7.3 One nonlinear model, two local models

The matrices $A_d$ and $A_u$ differ only in one sign, but that sign changes the qualitative local behavior. A linear model must always be accompanied by its operating point and coordinate definition.

---

## 8. Vector spaces: the language of state

The state vector $x\in\mathbb{R}^n$ is more than a column of numbers. It is an element of a vector space. This viewpoint lets us discuss coordinate systems, reachable directions, unobservable directions, and modal decompositions later in the course.

### 8.1 Vector space

A vector space $X$ over a scalar field $\mathbb{F}$ is a set equipped with vector addition and scalar multiplication satisfying the familiar closure, associativity, commutativity, distributivity, identity, and inverse properties.

In this course, the main examples are $\mathbb{R}^n$ and $\mathbb{C}^n$. Unless stated otherwise, physical state vectors are real.

Not every set is a vector space. For example, the set $\{0,1\}$ with ordinary addition is not closed because $1+1=2\notin\{0,1\}$.

### 8.2 Linear combinations and span

A vector $v$ is a linear combination of $v_1,\ldots,v_k$ if

$$
v=\alpha_1v_1+\cdots+\alpha_kv_k
$$

for some scalars $\alpha_i$. The span is the set of all such combinations:

$$
\operatorname{span}\{v_1,\ldots,v_k\}
=
\left\{\sum_{i=1}^k\alpha_i v_i:\alpha_i\in\mathbb{F}\right\}.
$$

Geometrically, a nonzero vector spans a line through the origin; two non-collinear vectors in $\mathbb{R}^2$ span the plane.

### 8.3 Linear independence

Vectors $v_1,\ldots,v_k$ are linearly independent if

$$
\alpha_1v_1+\cdots+\alpha_kv_k=0
$$

implies

$$
\alpha_1=\cdots=\alpha_k=0.
$$

If a nonzero choice of coefficients produces zero, the vectors are linearly dependent.

Example:

$$
v_1=\begin{bmatrix}1\\0\end{bmatrix},
\qquad
v_2=\begin{bmatrix}1\\-1\end{bmatrix}
$$

are independent. In contrast,

$$
v_1=\begin{bmatrix}1\\0\end{bmatrix},
\qquad
v_2=\begin{bmatrix}-1\\0\end{bmatrix}
$$

are dependent because $v_1+v_2=0$.

### 8.4 Basis and coordinates

A basis of an $n$-dimensional vector space is a set of $n$ linearly independent vectors that spans the space. Every vector then has a unique coordinate representation.

Using

$$
e_1=\begin{bmatrix}1\\0\end{bmatrix},
\qquad
e_2=\begin{bmatrix}1\\-1\end{bmatrix},
$$

write

$$
x=\begin{bmatrix}x_1\\x_2\end{bmatrix}
=\alpha_1e_1+\alpha_2e_2
=
\begin{bmatrix}
\alpha_1+\alpha_2\\-\alpha_2
\end{bmatrix}.
$$

Therefore,

$$
\alpha_1=x_1+x_2,
\qquad
\alpha_2=-x_2.
$$

The physical vector has not changed; only its coordinates have changed. Week 2 will develop this idea using coordinate-transformation matrices and similarity transformations.

---

## 9. Python example: nonlinear pendulum versus local models

The complete executable example is provided separately:

**[Open the Week 1 pendulum notebook in Google Colab](https://colab.research.google.com/github/fdcl-gwu/MAE6246/blob/main/notebooks/week01_pendulum.ipynb)**

The notebook uses NumPy, SciPy, and Matplotlib to compare the exact nonlinear model with both local linearizations. The following sections summarize the experiments and the conclusions to draw from them.

### 9.1 Downward equilibrium

The first experiment compares initial angles of $5^\circ$, $30^\circ$, and $90^\circ$ with zero initial angular velocity and zero applied torque.

Expected observation:

- At $5^\circ$, the curves nearly coincide.
- At $30^\circ$, a phase error becomes visible.
- At $90^\circ$, the linear model is qualitatively suggestive but quantitatively poor.

The familiar approximation $\sin\theta\approx\theta$ is accurate only near zero. Its leading error is cubic:

$$
\sin\theta-\theta\approx-\frac{\theta^3}{6}.
$$

### 9.2 Upright equilibrium

The second experiment uses perturbation coordinates about $\theta=\pi$ and compares initial perturbations of $0.5^\circ$, $2^\circ$, and $8^\circ$.

The perturbations initially agree, but both move away from the upright equilibrium. Eventually the local approximation fails because the state is no longer close to the point where the Jacobian was evaluated.

### 9.3 Quantifying local-model error

The final experiment sweeps the initial angle from $0.1^\circ$ to $60^\circ$ and plots the maximum angle error over three seconds on logarithmic axes.

There is no universal angle at which the model suddenly becomes invalid. Validity depends on the acceptable error, the time interval, the input, and the question being asked.

---

## 10. Common mistakes

1. **Confusing the state with the output.** A sensor may reveal only part of the state.
2. **Calling an affine equation linear.** An equation such as $\dot{x}=Ax+b$ does not satisfy superposition unless $b=0$. It may become linear in perturbation coordinates about an equilibrium.
3. **Finding an equilibrium from only part of $f(x,u)=0$.** Every component must vanish.
4. **Evaluating the Jacobian before choosing an operating point.** Different equilibria can produce different matrices.
5. **Using $x$ when the linear model actually describes $\delta x=x-x^\star$.** This is especially dangerous at a nonzero equilibrium.
6. **Treating linearization as a global identity.** It is a first-order local approximation.
7. **Checking only the number of vectors when claiming a basis.** The vectors must also be linearly independent and span the space.

---

## 11. Concept checks

1. A cart has position $q$ and velocity $\dot q$, but only $q$ is measured. Propose $x$, $u$, and $y$.
2. Does $\dot{x}=Ax+b$ satisfy superposition when $b\ne0$? Show your answer using the zero input and zero initial condition.
3. Find all zero-input equilibria of the pendulum over $0\le\theta<2\pi$.
4. Re-derive $A_d$ and $A_u$ directly from $\sin\delta\theta\approx\delta\theta$ and $\sin(\pi+\delta\theta)\approx-\delta\theta$.
5. Explain why $[1,0]^T$ and $[2,0]^T$ do not form a basis of $\mathbb{R}^2$.
6. Express $x=[3,-2]^T$ in the basis $e_1=[1,0]^T$, $e_2=[1,-1]^T$.
7. In the Python experiment, choose a numerical error tolerance and estimate the largest acceptable initial angle for the downward linearization over three seconds.

---

## 12. Summary

- A state contains the information required to predict future behavior when the future input is known.
- A nonlinear state-space model has the form $\dot{x}=f(t,x,u)$, $y=g(t,x,u)$.
- An LTI model has the form $\dot{x}=Ax+Bu$, $y=Cx+Du$.
- An equilibrium operating point satisfies $f(x^\star,u^\star)=0$.
- Jacobian linearization produces a first-order model in perturbation coordinates.
- The pendulum has locally different downward and upright linearizations.
- Span, linear independence, basis, and coordinates provide the language used to describe state space and its important subspaces.

---

## 13. Further reading

- Review the original handwritten lecture note dated September 2, 2015 for the legacy development of state, superposition, pendulum linearization, and vector-space fundamentals.
- In Week 2, we will develop change of basis, linear operators, eigenstructure, singular value decomposition, and the matrix exponential.
