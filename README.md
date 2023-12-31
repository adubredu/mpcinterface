# mpcinterface
Given an MPC problem of the form

$$
\begin{align}
min_{x_{1:H},u_{1:H-1}} ~\sum_{k=1}^{H-1}\left(\frac{1}{2}x_k^\top Q_k x_k + \frac{1}{2}u_k^\top R_k u_k\right) + x_H^\top P_H x_H \\
s.t.~~ x_{k+1} = A_k x_k + B_k u_k \\
x_{min} \leq x_k \leq x_{max} \\
u_{min} \leq u_k \leq u_{max} \\
\end{align} 
$$

this package re-formulates the MPC problem into a canonical Quadratic Program of the form 

$$
\begin{align}
min_z \frac{1}{2} z^\top \bar{P} z + \bar{q}^\top z \\ 
s.t.~~ \bar{l} \leq \bar{A}z \leq \bar{u} \\
\end{align}
$$

where $$z = \[x_1, u_1, x_2, u_2, \dots, u_{H-1}, x_H\] ^\top $$

More specifically, this package returns the $\bar{P}, \bar{q}, \bar{l}, \bar{u}$ and $\bar{A}$ matrices for your QP MPC problem. These matrices can then be directly passed to off-the-shelf QP solvers like [OSQP](https://github.com/google/osqp-cpp).

## Installation
This package is compiled using the [Bazel build system](https://bazel.build/about). Install Bazel by following the installation instructions [here](https://bazel.build/install) 

The only other dependency of this package is [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). This package however automatically downloads its own copy of Eigen during compilation so it doesn't matter whether or not you already have Eigen installed on your local system.

## Usage
See the [test](test) files for usage examples.