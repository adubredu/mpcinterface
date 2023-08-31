#include <Eigen/Core>

class MPCInterface
{
public:
    // Constructor
    // Inputs:
    // Horizon length H (int) 
    // State dimension n (int)
    // Input dimension m (int)
    MPCInterface(int H, int n, int m);

    // Destructor
    ~MPCInterface();

    // Inputs:
    // State cost matrix
    Eigen::VectorXd Q;

    // Input cost matrix
    Eigen::VectorXd R;

    // Terminal state cost matrix
    Eigen::VectorXd P;

    // State lower bound
    Eigen::VectorXd x_min;

    // State upper bound
    Eigen::VectorXd x_max;

    // Input lower bound
    Eigen::VectorXd u_min;

    // Input upper bound
    Eigen::VectorXd u_max;

    // State dynamics matrix
    Eigen::MatrixXd A;

    // Input dynamics matrix
    Eigen::MatrixXd B;
    
    // State reference
    Eigen::VectorXd x_ref;

    // Input reference
    Eigen::VectorXd u_ref;

    // Outputs:
    struct CanonicalForm {
        // Objective Matrix
        Eigen::MatrixXd P_bar;

        // Objective Vector
        Eigen::VectorXd q_bar;

        // Constraint Matrix
        Eigen::MatrixXd A_bar;

        // Lower bound vector
        Eigen::VectorXd l_bar;

        // Upper bound vector
        Eigen::VectorXd u_bar;
    };

    // Get the canonical form of the MPC problem
    CanonicalForm getCanonicalForm();

private:
    // Horizon length
    int H;

    // State dimension
    int n;

    // Input dimension
    int m;




};