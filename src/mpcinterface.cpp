#include "mpcinterface.h"

MPCInterface::MPCInterface (int H, int n, int m) {
    // Initialize horizon length
    this->H = H;

    // Initialize state dimension
    this->n = n;

    // Initialize input dimension
    this->m = m;

    // Initialize state cost matrix
    this->Q = Eigen::VectorXd::Zero(n, n);

    // Initialize input cost matrix
    this->R = Eigen::VectorXd::Zero(m, m);

    // Initialize terminal state cost matrix
    this->P = Eigen::VectorXd::Zero(n, n);

    // Initialize state lower bound
    this->x_min = Eigen::VectorXd::Zero(n);

    // Initialize state upper bound
    this->x_max = Eigen::VectorXd::Zero(n);

    // Initialize input lower bound
    this->u_min = Eigen::VectorXd::Zero(m);

    // Initialize input upper bound
    this->u_max = Eigen::VectorXd::Zero(m);

    // Initialize state dynamics matrix
    this->A = Eigen::MatrixXd::Zero(n, n);

    // Initialize input dynamics matrix
    this->B = Eigen::MatrixXd::Zero(n, m);

    // Initialize state reference
    this->x_ref = Eigen::VectorXd::Zero(n);

    // Initialize input reference
    this->u_ref = Eigen::VectorXd::Zero(m);
}

MPCInterface::~MPCInterface() {}

MPCInterface::CanonicalForm MPCInterface::getCanonicalForm() {

    // Create P_bar
    Eigen::MatrixXd P_bar((H * n + H * m), (H * n + H * m));
    P_bar.setZero();  // Initialize to zeros
    for (int i = 0; i < H; i++) {
        P_bar.block(i * n, i * n, n, n) = Q;
        P_bar.block(H * n + i * m, H * n + i * m, m, m) = R;
    }
    P_bar.block(H * n, H * n, n, n) = P;

    // Create q_bar
    Eigen::VectorXd q_bar((H * (n + m)));
    q_bar.setZero();  // Initialize to zeros
    for (int i = 0; i < H; i++) {
        q_bar.segment(i * n, n) = -2 * Q * x_ref.col(i);
        q_bar.segment(H * n + i * m, m) = -2 * R * u_ref.col(i);
    }

    // Construct A_bar with additional rows for constraints
    Eigen::MatrixXd A_bar(2 * H * (n + m) + H * n, H * (n + m));
    A_bar.setZero();  // Initialize to zeros

    // Dynamics
    for (int i = 0; i < H; ++i) {
        A_bar.block(i * n, i * n, n, n) = A;
        A_bar.block(i * n, H * n + i * m, n, m) = B;
    }

    // State constraints
    for (int i = 0; i < H; ++i) {
        A_bar.block(H * n + 2 * i * n, i * n, n, n) = Eigen::MatrixXd::Identity(n, n);        // x_min <= x
        A_bar.block(H * n + (2 * i + 1) * n, i * n, n, n) = -Eigen::MatrixXd::Identity(n, n); // x <= x_max
    }

    // Input constraints
    for (int i = 0; i < H; ++i) {
        A_bar.block(2 * H * n + 2 * i * m, H * n + i * m, m, m) = Eigen::MatrixXd::Identity(m, m);        // u_min <= u
        A_bar.block(2 * H * n + (2 * i + 1) * m, H * n + i * m, m, m) = -Eigen::MatrixXd::Identity(m, m); // u <= u_max
    }

    // Construct l_bar and u_bar
    Eigen::VectorXd l_bar(2 * H * (n + m));
    Eigen::VectorXd u_bar(2 * H * (n + m));
    l_bar.setZero();  // Initialize to zeros
    u_bar.setZero();  // Initialize to zeros

    for (int i = 0; i < H; ++i) {
        l_bar.segment(2 * i * n, n) = x_min;
        l_bar.segment(2 * i * n + n, n) = -x_max;
        l_bar.segment(2 * H * n + 2 * i * m, m) = u_min;
        l_bar.segment(2 * H * n + 2 * i * m + m, m) = -u_max;

        u_bar.segment(2 * i * n, n) = x_max;
        u_bar.segment(2 * i * n + n, n) = -x_min;
        u_bar.segment(2 * H * n + 2 * i * m, m) = u_max;
        u_bar.segment(2 * H * n + 2 * i * m + m, m) = -u_min;
    }

    // Construct CanonicalForm struct
    CanonicalForm canonicalForm({P_bar, q_bar, A_bar, l_bar, u_bar});

    return canonicalForm;
}