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
    
}