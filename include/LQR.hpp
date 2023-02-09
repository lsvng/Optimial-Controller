/**
 * @file LQR.hpp
 * @author duckstarr
 * @brief Infinite-horizon Linear Quadratic Regulator
 * 
 */

#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>

namespace Optimal_Controller
{

class LQR
{
  public:
    LQR(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    ~LQR() {};

    Eigen::MatrixXd getVelocity(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E);

  private:
    /**
     * @brief An infinite-horizon LQR controller with penalties on the state feedback and input signal (Q and R, respectively)
     * @brief LQR gain is solved by searching for stable poles of the system through the Hamiltonian matrix
     * 
     * @param A System dynamics matrix
     * @param B Input matrix
     * @param E State error
     * @return Eigen::MatrixXd LQR gain, K
     */
    void computeHamiltonian(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

    void computeRiccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E);
    void computeVelocity(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E);

  private:
    // Declare generic MAT for LQR computations
    Eigen::MatrixXd Q;            // Weight on the systems state
    Eigen::MatrixXd R;            // Weight on control input
    Eigen::MatrixXd I;            // Identity matrix
    Eigen::MatrixXd P;            // Riccati matrix
    Eigen::MatrixXcd eigenVec;    // Eigenvectors
    Eigen::MatrixXd cmd_vel;      // Command velocity
    Eigen::MatrixXd controlInput; // Control input
};

} // namespace controller

#endif /* LQR_HPP */
