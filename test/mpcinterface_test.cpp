#include <Eigen/Core>
#include "mpcinterface.h"
#include "gtest/gtest.h"

TEST(MPCInterfaceTest, TestInitializedMatrixDimensions) {
    // Initialize horizon length
    int H = 5;

    // Initialize state dimension
    int n = 3;

    // Initialize input dimension
    int m = 2;

    // Initialize MPCInterface object
    MPCInterface mpc(H, n, m);

    EXPECT_EQ(mpc.Q.rows(), n);
    EXPECT_EQ(mpc.Q.cols(), n);
    EXPECT_EQ(mpc.R.rows(), m);
    EXPECT_EQ(mpc.R.cols(), m);
    EXPECT_EQ(mpc.P.rows(), n);
    EXPECT_EQ(mpc.P.cols(), n);
    EXPECT_EQ(mpc.x_min.size(), n);
    EXPECT_EQ(mpc.x_max.size(), n);
    EXPECT_EQ(mpc.u_min.size(), m);
    EXPECT_EQ(mpc.u_max.size(), m);
    EXPECT_EQ(mpc.A.rows(), n);
    EXPECT_EQ(mpc.A.cols(), n);
    EXPECT_EQ(mpc.B.rows(), n);
    EXPECT_EQ(mpc.B.cols(), m);
    EXPECT_EQ(mpc.x_ref.cols(), H+1);
    EXPECT_EQ(mpc.x_ref.rows(), n);
    EXPECT_EQ(mpc.u_ref.cols(), H+1);
    EXPECT_EQ(mpc.u_ref.rows(), m);
}

TEST(MPCInterfaceTest, TestCanonicalMatrixDimensions) {
    // Initialize horizon length
    int H = 5;

    // Initialize state dimension
    int n = 3;

    // Initialize input dimension
    int m = 2;

    // Initialize MPCInterface object
    MPCInterface mpc(H, n, m);

    // Generate Canonical Form
    MPCInterface::CanonicalForm canonical_form = mpc.getCanonicalForm();

    EXPECT_EQ(canonical_form.P_bar.rows(), H * (n + m) + n);
    EXPECT_EQ(canonical_form.P_bar.cols(), H * (n + m) + n);
    EXPECT_EQ(canonical_form.q_bar.size(), H * (n + m) + n); 
    EXPECT_EQ(canonical_form.A_bar.rows(), 2 * H * (n + m) + H * n);
    EXPECT_EQ(canonical_form.A_bar.cols(), H * (n + m));
    EXPECT_EQ(canonical_form.l_bar.size(), 2 * H * (n + m)); 
    EXPECT_EQ(canonical_form.u_bar.size(), 2 * H * (n + m));
}

TEST(MPCInterfaceTest, TestCanonicalForm) {
    int H = 2;
    int n = 2;
    int m = 1;

    MPCInterface mpc(H, n, m);
    mpc.Q = Eigen::MatrixXd::Identity(n, n);
    mpc.R = Eigen::MatrixXd::Identity(m, m);
    mpc.P = 2*Eigen::MatrixXd::Identity(n, n);
    mpc.A = Eigen::MatrixXd::Identity(n, n);
    mpc.A(0, 1) = 1;
    mpc.B(0, 0) = 0;
    mpc.B(1, 0) = 1;
    mpc.x_ref(0, 0) = 1;
    mpc.x_ref(1, 0) = 2;
    mpc.x_ref(0, 1) = 2;
    mpc.x_ref(1,1) = 3;
    mpc.x_ref(0, 2) = 3;
    mpc.x_ref(1,2) = 4;
    mpc.u_ref(0, 0) = 1;
    mpc.u_ref(0, 1) = 0;
    mpc.x_min(0) = -10;
    mpc.x_min(1) = -10;
    mpc.x_max(0) = 10;
    mpc.x_max(1) = 10;
    mpc.u_min(0) = -2;
    mpc.u_max(0) = 2;

    MPCInterface::CanonicalForm canonical_form = mpc.getCanonicalForm();
    Eigen::MatrixXd expected_P_bar(8, 8);
    expected_P_bar << 1, 0, 0, 0, 0, 0, 0, 0,
                      0, 1, 0, 0, 0, 0, 0, 0,
                      0, 0, 1, 0, 0, 0, 0, 0,
                      0, 0, 0, 1, 0, 0, 0, 0,
                      0, 0, 0, 0, 1, 0, 0, 0,
                      0, 0, 0, 0, 0, 1, 0, 0,
                      0, 0, 0, 0, 0, 0, 2, 0,
                      0, 0, 0, 0, 0, 0, 0, 2;
    EXPECT_EQ(canonical_form.P_bar, expected_P_bar);

    Eigen::VectorXd expected_q_bar(8);
    expected_q_bar << -1, -2, -2, -3, -1, 0, -6, -8;
    EXPECT_EQ(canonical_form.q_bar, 2*expected_q_bar);

    Eigen::MatrixXd expected_A_bar(16, 6);
    expected_A_bar << 1, 1, 0, 0, 0, 0,
                      0, 1, 1, 0, 0, 0,
                      0, 0, 0, 1, 1, 0,
                      0, 0, 0, 0, 1, 1,
                      1, 0, 0, 0, 0, 0,
                      -1, 0, 0, 0, 0, 0,
                      0, 1, 0, 0, 0, 0,
                      0, -1, 0, 0, 0, 0,
                      0, 0, 1, 0, 0, 0,
                      0, 0, -1, 0, 0, 0,
                      0, 0, 0, 1, 0, 0,
                      0, 0, 0, -1, 0, 0,
                      0, 0, 0, 0, 1, 0,
                      0, 0, 0, 0, -1, 0,
                      0, 0, 0, 0, 0, 1,
                      0, 0, 0, 0, 0, -1;
    EXPECT_EQ(canonical_form.A_bar, expected_A_bar);

}