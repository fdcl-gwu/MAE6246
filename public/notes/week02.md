# MAE 6246 - Linear Systems and Control

## Week 2: Linear Operators and Computational Linear Algebra

**Lecture:** Friday, September 4, 2026  
**Topics:** linear spaces, span, independence, and basis; linear operators and matrix representations; change of basis and similarity; range, null space, and rank; eigenstructure and modal coordinates; repeated eigenvalues and generalized eigenvectors; singular value decomposition and conditioning

---

## 1. Learning objectives

By the end of the lecture, you should be able to:

1. explain why $\mathbb R^n$ is a linear space and identify a basis for it;
2. represent a linear operator as a matrix after bases have been selected;
3. transform vectors and state-space models between coordinate systems;
4. interpret the range, null space, rank, and nullity of a matrix;
5. determine when a linear system $Ax=b$ has no solution, one solution, or infinitely many solutions;
6. interpret eigenvectors as invariant directions and use an eigenvector basis to obtain modal coordinates;
7. distinguish a diagonalizable matrix from a defective matrix; and
8. interpret singular values as directional gains and use them to recognize rank deficiency and poor conditioning.

The central message is that a matrix is not merely an array of coefficients. It is the coordinate representation of a linear map. A good coordinate system can expose structure that is hidden in the original variables.

---

## 2. Linear spaces and bases: a brief review

A **linear space**, or vector space, is a set whose elements can be added and multiplied by scalars while remaining in the set. The operations must obey the familiar rules of vector arithmetic, including associativity, commutativity of addition, distributivity, the existence of a zero vector, and the existence of additive inverses.

The set

$$
\mathbb R^n=
\left\{
\begin{bmatrix}x_1&\cdots&x_n\end{bmatrix}^T:x_i\in\mathbb R
\right\}
$$

is a linear space over $\mathbb R$. To see the essential reason, take any $x,y\in\mathbb R^n$ and any scalars $\alpha,\beta\in\mathbb R$. Every component of

$$
\alpha x+\beta y
$$

is still a real number, so

$$
\boxed{\alpha x+\beta y\in\mathbb R^n.}
$$

Thus $\mathbb R^n$ is closed under linear combinations. It also contains the zero vector and $-x$ for every $x$, and its componentwise operations inherit the usual arithmetic rules of real numbers.

Vectors $v_1,\ldots,v_k$ **span** a space if every vector in the space can be written as their linear combination. They are **linearly independent** if

$$
\alpha_1v_1+\cdots+\alpha_kv_k=0
$$

implies $\alpha_1=\cdots=\alpha_k=0$. A **basis** is a set of vectors that both spans the space and is linearly independent. Consequently, every vector has a unique coordinate representation in a basis.

The standard basis of $\mathbb R^n$ is

$$
e_1=\begin{bmatrix}1\\0\\\vdots\\0\end{bmatrix},
\quad
e_2=\begin{bmatrix}0\\1\\\vdots\\0\end{bmatrix},
\quad\ldots\quad,
e_n=\begin{bmatrix}0\\0\\\vdots\\1\end{bmatrix}.
$$

A subset $W\subseteq\mathbb R^n$ is a subspace if it is closed under linear combinations. The range, null space, and eigenspaces introduced below are important examples of subspaces.

---

## 3. Linear operators and matrix representations

Let $X$ and $Y$ be vector spaces over the same scalar field. A map

$$
L:X\rightarrow Y
$$

is a **linear operator** if

$$
L(\alpha x_1+\beta x_2)
=
\alpha L(x_1)+\beta L(x_2)
$$

for all $x_1,x_2\in X$ and all scalars $\alpha,\beta$.

Linearity combines additivity and homogeneity:

$$
L(x_1+x_2)=L(x_1)+L(x_2),
\qquad
L(\alpha x)=\alpha L(x).
$$

Rotations, projections, differentiation, integration, and multiplication by a fixed matrix are linear. A translation,

$$
L(x)=Ax+b,\qquad b\ne0,
$$

is affine rather than linear because $L(0)=b\ne0$.

### 3.1 Constructing a matrix representation

Choose an input basis

$$
\mathcal B_X=\{v_1,\ldots,v_n\}
$$

and an output basis

$$
\mathcal B_Y=\{w_1,\ldots,w_m\}.
$$

The matrix representation $A\in\mathbb F^{m\times n}$ is defined by

$$
[L(x)]_{\mathcal B_Y}=A[x]_{\mathcal B_X}.
$$

Its $j$th column is the coordinate representation of the image of the $j$th input basis vector:

$$
\boxed{A_{:j}=[L(v_j)]_{\mathcal B_Y}.}
$$

This column rule connects the abstract operator to its matrix representation.

### 3.2 Example: rotation in the standard basis

Let $L:\mathbb R^2\rightarrow\mathbb R^2$ rotate a vector counterclockwise by $\theta$. In the standard basis,

$$
L(e_1)=
\begin{bmatrix}
\cos\theta\\ \sin\theta
\end{bmatrix},
\qquad
L(e_2)=
\begin{bmatrix}
-\sin\theta\\ \cos\theta
\end{bmatrix}.
$$

Therefore,

$$
[L]_{\mathcal E}=
\begin{bmatrix}
\cos\theta&-\sin\theta\\
\sin\theta&\cos\theta
\end{bmatrix}.
$$

The columns are the images of the selected input basis vectors.

---

## 4. Coordinates, change of basis, and similarity

Let $\mathcal B=\{v_1,\ldots,v_n\}$ be a basis of $\mathbb R^n$, and put the basis vectors into the columns of

$$
P=\begin{bmatrix}v_1&\cdots&v_n\end{bmatrix}.
$$

If $z=[x]_{\mathcal B}$ contains the coordinates of the physical vector $x$ in the new basis, then

$$
\boxed{x=Pz},
\qquad
\boxed{z=P^{-1}x}.
$$

The vector has not changed. Only its coordinate description has changed.

### 4.1 Similarity transformation

Suppose the operator is represented by $A$ in the original coordinates. Write $x=Pz$ and $y=P\zeta$ in $y=Ax$. Then

$$
P\zeta=APz,
$$

and hence

$$
\boxed{\zeta=\bar A z,\qquad \bar A=P^{-1}AP.}
$$

The matrices $A$ and $\bar A$ are **similar**: they describe the same operator in different bases. Similarity preserves the characteristic polynomial, eigenvalues, determinant, trace, and rank.

### 4.2 State-space coordinate transformation

For the LTI model

$$
\dot x=Ax+Bu,
\qquad
y=Cx+Du,
$$

introduce $x=Pz$. Since $P$ is constant, $\dot x=P\dot z$, and

$$
\boxed{
\dot z=P^{-1}APz+P^{-1}Bu,
\qquad
y=CPz+Du.
}
$$

Thus,

$$
\boxed{
\bar A=P^{-1}AP,
\quad
\bar B=P^{-1}B,
\quad
\bar C=CP,
\quad
\bar D=D.
}
$$

These transformations will be used repeatedly in controllability, observability, feedback, and estimation.

---

## 5. Range, null space, rank, and linear equations

Let $A\in\mathbb R^{m\times n}$ define a map from $\mathbb R^n$ to $\mathbb R^m$.

### 5.1 Range and null space

The **range**, or column space, is

$$
\mathcal R(A)=\{Ax:x\in\mathbb R^n\}.
$$

It is the set of all outputs that the operator can produce and equals the span of the columns of $A$.

The **null space**, or kernel, is

$$
\mathcal N(A)=\{x\in\mathbb R^n:Ax=0\}.
$$

A nonzero vector in $\mathcal N(A)$ is an input direction that the operator cannot distinguish from zero.

### 5.2 Rank-nullity theorem

The rank is the dimension of the range, and the nullity is the dimension of the null space:

$$
\operatorname{rank}(A)=\dim\mathcal R(A),
\qquad
\operatorname{nullity}(A)=\dim\mathcal N(A).
$$

For a matrix with $n$ columns,

$$
\boxed{\operatorname{rank}(A)+\operatorname{nullity}(A)=n.}
$$

### 5.3 What the subspaces tell us about $Ax=b$

The equation $Ax=b$ is consistent exactly when $b\in\mathcal R(A)$. If $x_p$ is one particular solution, every solution has the form

$$
\boxed{x=x_p+x_n,\qquad x_n\in\mathcal N(A).}
$$

Therefore:

- no solution exists if $b\notin\mathcal R(A)$;
- the solution is unique if $b\in\mathcal R(A)$ and $\mathcal N(A)=\{0\}$; and
- infinitely many solutions exist if $b\in\mathcal R(A)$ and $\mathcal N(A)$ contains a nonzero vector.

This geometric viewpoint will later help us interpret reachable states, unobservable directions, and redundant measurements.

---

## 6. Eigenvalues, eigenvectors, and modal coordinates

A nonzero vector $v$ is an eigenvector of a square matrix $A$ with eigenvalue $\lambda$ if

$$
\boxed{Av=\lambda v.}
$$

The direction spanned by $v$ is invariant under $A$: applying $A$ changes its scale, and possibly its orientation or phase, but not its direction.

Since $(A-\lambda I)v=0$, a nonzero eigenvector exists only if $A-\lambda I$ is singular. Therefore,

$$
\boxed{\det(\lambda I-A)=0.}
$$

### 6.1 Algebraic and geometric multiplicity

The number of times $\lambda_i$ occurs as a root of the characteristic polynomial is its **algebraic multiplicity**. Its eigenspace is

$$
\mathcal E_{\lambda_i}=\mathcal N(A-\lambda_iI),
$$

and the dimension of this eigenspace is its **geometric multiplicity**. Always,

$$
1\le
\text{geometric multiplicity}
\le
\text{algebraic multiplicity}.
$$

### 6.2 Diagonalization

If $A$ has $n$ linearly independent eigenvectors, collect them as columns of

$$
P=\begin{bmatrix}v_1&\cdots&v_n\end{bmatrix}.
$$

Then

$$
AP=P\Lambda,
\qquad
\Lambda=\operatorname{diag}(\lambda_1,\ldots,\lambda_n),
$$

so

$$
\boxed{P^{-1}AP=\Lambda,\qquad A=P\Lambda P^{-1}.}
$$

Distinct eigenvalues guarantee independent eigenvectors. Repeated eigenvalues require us to count the independent eigenvectors.

### 6.3 Modal coordinates

For the autonomous system $\dot x=Ax$, use $x=Pz$. If $P$ is an eigenvector matrix, then

$$
\dot z=\Lambda z,
\qquad
\dot z_i=\lambda_i z_i.
$$

The modal coordinates evolve independently. This is the principal reason eigenvectors are useful in dynamic systems.

---

## 7. Running example: two coupled masses

Consider two identical unit masses connected by identical springs. A nondimensional stiffness matrix is

$$
K=
\begin{bmatrix}
2&-1\\
-1&2
\end{bmatrix},
$$

and the undamped free motion satisfies $\ddot q+Kq=0$.

The normalized eigenvectors can be chosen as

$$
v_1=\frac{1}{\sqrt2}
\begin{bmatrix}1\\1\end{bmatrix},
\qquad
v_2=\frac{1}{\sqrt2}
\begin{bmatrix}1\\-1\end{bmatrix}.
$$

The first is an in-phase mode and the second is an out-of-phase mode. Put them into $P=\begin{bmatrix}v_1&v_2\end{bmatrix}$. Because $K$ is symmetric, $P$ can be chosen orthogonal, so $P^{-1}=P^T$. The modal stiffness matrix is

$$
P^TKP=
\begin{bmatrix}
1&0\\
0&3
\end{bmatrix}.
$$

With $q=P\eta$, the equations become

$$
\ddot\eta_1+\eta_1=0,
\qquad
\ddot\eta_2+3\eta_2=0.
$$

Thus the coupled physical motion is the combination of two independent modal oscillations with natural frequencies $\omega_1=1$ and $\omega_2=\sqrt3$.

---

## 8. Defective matrices and generalized eigenvectors

Not every square matrix can be diagonalized. Consider

$$
J=
\begin{bmatrix}
\lambda&1\\
0&\lambda
\end{bmatrix}.
$$

The eigenvalue $\lambda$ has algebraic multiplicity two, but the eigenspace is one dimensional. The matrix has only one independent eigenvector and is called **defective**.

Let $v_1$ be an eigenvector, so $(A-\lambda I)v_1=0$. A generalized eigenvector $v_2$ satisfies

$$
\boxed{(A-\lambda I)v_2=v_1.}
$$

The ordered vectors $v_1,v_2$ form a Jordan chain. In this basis,

$$
Av_1=\lambda v_1,
\qquad
Av_2=v_1+\lambda v_2.
$$

For a longer chain, $(A-\lambda I)v_{k+1}=v_k$. Jordan form explains why defective systems contain polynomial factors multiplying exponential behavior. For the $2\times2$ block,

$$
e^{Jt}=e^{\lambda t}
\begin{bmatrix}
1&t\\
0&1
\end{bmatrix}.
$$

We use Jordan form primarily as a conceptual tool. It is very sensitive to roundoff and is generally not constructed from measured or floating-point matrices. The companion notebook constructs known Jordan blocks and explores their structure computationally.

---

## 9. Singular value decomposition and conditioning

Every matrix $A\in\mathbb R^{m\times n}$ has a singular value decomposition:

$$
\boxed{A=U\Sigma V^T.}
$$

The columns of $U$ and $V$ are orthonormal, and the singular values satisfy $\sigma_1\ge\sigma_2\ge\cdots\ge0$. The relation

$$
Av_i=\sigma_i u_i
$$

shows how the operator acts on special orthogonal input directions. Geometrically:

1. $V^T$ rotates or reflects the input coordinates;
2. $\Sigma$ stretches or compresses the coordinate directions; and
3. $U$ rotates or reflects the result into the output coordinates.

Consequently, a unit sphere is mapped to an ellipsoid whose semiaxis lengths are the singular values.

### 9.1 Rank and fundamental subspaces from the SVD

The number of nonzero singular values is the rank. The right singular vectors associated with zero singular values span $\mathcal N(A)$, while the left singular vectors associated with nonzero singular values span $\mathcal R(A)$.

Unlike an eigenvalue decomposition, the SVD applies to rectangular matrices and always exists.

### 9.2 Condition number

For a full-rank square matrix,

$$
\boxed{\kappa_2(A)=\frac{\sigma_{\max}(A)}{\sigma_{\min}(A)}.}
$$

A large condition number means that some directions are amplified much more strongly than others. Recovering information along a weakly amplified direction can magnify measurement errors and roundoff.

Consider

$$
H_\epsilon=
\begin{bmatrix}
1&1\\
1&1+\epsilon
\end{bmatrix}.
$$

For nonzero $\epsilon$, this matrix is invertible. As $\epsilon\rightarrow0$, however, its rows become nearly identical, its smallest singular value approaches zero, and its condition number becomes large. Invertibility alone therefore does not guarantee a reliable inverse calculation.

### 9.3 Eigenvalue decomposition versus SVD

| Question | Eigenvalue decomposition | Singular value decomposition |
|---|---|---|
| Matrix type | square | square or rectangular |
| Form | $A=P\Lambda P^{-1}$, when diagonalizable | $A=U\Sigma V^T$, always |
| Main directions | invariant directions | orthogonal input and output directions |
| Main use here | modes and coordinate decoupling | rank, amplification, and conditioning |

These decompositions answer different questions and should not be treated as interchangeable.

---

## 10. Computational companion

The lecture concepts are illustrated in the separate notebook:

**[Open the Week 2 computational linear algebra notebook in Google Colab](https://colab.research.google.com/github/fdcl-gwu/MAE6246/blob/main/notebooks/week02_computational_linear_algebra.ipynb)**

The notebook:

1. transforms a coupled mechanical model into modal coordinates;
2. compares coupled physical motion with independent modal motion;
3. identifies rank and null-space directions with the SVD;
4. visualizes the unit-circle-to-ellipse geometry of a nearly singular map;
5. demonstrates noise amplification as the condition number grows; and
6. constructs higher-order Jordan blocks and verifies their matrix exponentials.

---

## 11. Common mistakes

1. **Confusing a vector with its coordinates.** The physical vector is unchanged when its coordinate column changes.
2. **Writing $z=Px$ when the columns of $P$ are the new basis vectors.** With this convention, $x=Pz$ and $z=P^{-1}x$.
3. **Using $P^T$ instead of $P^{-1}$ for a general basis.** They are equal only for an orthogonal matrix.
4. **Transforming $A$ but not $B$ and $C$.** A state-coordinate transformation must be applied consistently.
5. **Assuming repeated eigenvalues imply diagonalizability.** Count the independent eigenvectors.
6. **Treating a numerically tiny singular value as exactly zero without specifying a tolerance.** Numerical rank depends on scale and precision.
7. **Using the determinant to judge conditioning.** Singular values reveal directional sensitivity more clearly.
8. **Trying to compute Jordan form from noisy data.** Small perturbations can change the apparent Jordan structure.

---

## 12. Concept checks

1. Why does $\alpha x+\beta y\in\mathbb R^n$ for all $x,y\in\mathbb R^n$ and $\alpha,\beta\in\mathbb R$?
2. What is the difference between a spanning set and a basis?
3. If $P=[v_1\ \cdots\ v_n]$ and $x=Pz$, what do the entries of $z$ represent?
4. Derive $\bar A$, $\bar B$, and $\bar C$ under the state-coordinate change $x=Pz$.
5. For

   $$
   A=
   \begin{bmatrix}
   1&2&3\\
   2&4&6
   \end{bmatrix},
   $$

   determine the rank and nullity. Is every $b\in\mathbb R^2$ in $\mathcal R(A)$?
6. Why are eigenvectors belonging to distinct eigenvalues linearly independent?
7. Interpret the two eigenvectors of the coupled stiffness matrix physically.
8. How do algebraic and geometric multiplicity reveal whether a repeated eigenvalue is defective?
9. If $(A-\lambda I)v_2=v_1$, explain how $A$ acts on $v_2$.
10. Why can the SVD be applied to a rectangular matrix while diagonalization cannot?
11. What physical or numerical difficulty is indicated by a very small singular value?
12. Why might an invertible matrix still produce an unreliable solution of $Ax=b$?

---

## 13. Summary

- $\mathbb R^n$ is closed under linear combinations and is a linear space over $\mathbb R$.
- A basis is a linearly independent spanning set and gives every vector unique coordinates.
- A matrix represents a linear operator after input and output bases are selected.
- If $x=Pz$, an operator transforms according to $\bar A=P^{-1}AP$.
- Range describes attainable outputs; null space describes input directions mapped to zero.
- Rank-nullity connects the dimensions of these two subspaces.
- Eigenvectors are invariant directions, and an eigenvector basis produces modal coordinates.
- A defective matrix lacks enough eigenvectors and requires generalized eigenvectors.
- The SVD always exists and exposes rank, directional amplification, and conditioning.
- Coordinate transformations and matrix decompositions prepare us to analyze LTI dynamics beginning in Week 3.
