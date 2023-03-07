/**
 * @file LQR.hpp
 * @author duckstarr
 * @brief Infinite-horizon Linear Quadratic Regulator
 * 
 */

#ifndef LQR_HPP
#define LQR_HPP

#include <Eigen/Dense>
#include <StateSpaceModel.hpp>

namespace Optimal_Controller
{

class LQR
{
  public:
    /**
     * @brief Construct a new LQR object
     * 
     * @param Q Weight on the systems state
     * @param R Weight on control input
     */
    LQR(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

    /**
     * @brief Destroy the LQR object
     * 
     */
    ~LQR();

    /**
     * @brief Disable copy constructor
     * 
     * @param iLQR 
     */
    LQR(LQR& iLQR) = delete;

    /**
     * @brief Disable assignment constructor
     * 
     * @param iLQR 
     */
    void operator=(LQR& iLQR) = delete;

    /**
     * @brief Get the control input
     * 
     * @param A System dynamics matrix
     * @param B Input matrix
     * @param C Output matrix
     * @param E State error
     * @return Eigen::MatrixXd 
     */
    Eigen::MatrixXd getControlInput(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& E);

  private:
    /**
     * @brief Compute Hamiltionian
     * @brief An infinite-horizon LQR controller with penalties on the state feedback and input signal (Q and R, respectively)
     * @brief LQR gain is solved by searching for stable poles of the system through the Hamiltonian matrix
     * 
     * @param A System dynamics matrix
     * @param B Input matrix
     * @return Eigen::MatrixXd LQR gain, K
     */
    void computeHamiltonian(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

    /**
     * @brief Solve the Riccati Equation
     * 
     * @param A System dynamics matrix
     * @param B Input matrix
     * @param E State error 
     */
    void computeRiccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E);
    
    /**
     * @brief Compute velocity
     * 
     * @param A System dynamics matrix
     * @param B Input matrix
     * @param E State error 
     */
    void computeVelocity(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E);

  private:
    Eigen::MatrixXd Q;            // Weight on the systems state
    Eigen::MatrixXd R;            // Weight on control input
    Eigen::MatrixXd I;            // Identity matrix
    Eigen::MatrixXd P;            // Riccati matrix
    Eigen::MatrixXcd eigenVec;    // Eigenvectors
    Eigen::MatrixXd controlInput; // Control input

    StateSpaceModel* statespace;  // State Space Model object
};

} // namespace controller

#endif /* LQR_HPP */
