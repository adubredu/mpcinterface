# mpcinterface
Given an MPC problem of the form

$$
\begin{align}
min_{x_{1:H},u_{1:H-1}} ~\sum_{k=1}^{H-1}\frac{1}{2}x_k^\top Q_k x_k + \frac{1}{2}u_k^\top R_k u_k ~~+~~ x_H^\top P_H x_H \\
s.t.~~ x_{k+1} = A_k x_k + B_k u_k \\
x_{min} \leq x_k \leq x_{max} \\
u_{min} \leq u_k \leq u_{max} \\

\end{align} 
$$
