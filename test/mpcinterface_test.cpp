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
    EXPECT_EQ(mpc.x_ref.cols(), H);
    EXPECT_EQ(mpc.x_ref.rows(), n);
    EXPECT_EQ(mpc.u_ref.cols(), H);
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

    EXPECT_EQ(canonical_form.P_bar.rows(), H * (n + m));
    EXPECT_EQ(canonical_form.P_bar.cols(), H * (n + m));
    EXPECT_EQ(canonical_form.q_bar.size(), H * (n + m)); 
    EXPECT_EQ(canonical_form.A_bar.rows(), 2 * H * (n + m) + H * n);
    EXPECT_EQ(canonical_form.A_bar.cols(), H * (n + m));
    EXPECT_EQ(canonical_form.l_bar.size(), 2 * H * (n + m)); 
    EXPECT_EQ(canonical_form.u_bar.size(), 2 * H * (n + m));
}

TEST(MPCInterfaceTest, TestCanonicalForm) {
    
}