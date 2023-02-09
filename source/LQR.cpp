/**
 * @file LQR.cpp
 * @author duckstarr
 * @brief Infinite-horizon Linear Quadratic Regulator
 * 
 */

#include <LQR.hpp>
#include <Eigen/Dense>

using namespace Optimal_Controller;

LQR::LQR(
    const Eigen::MatrixXd& Q, 
    const Eigen::MatrixXd& R
    ):
    Q(Q),
    R(R)
{
    P.setZero(Q.rows(), Q.cols());
    cmd_vel.setZero(1, R.rows()); // [translation rate, rotation rate]
}

Eigen::MatrixXd LQR::getVelocity(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E)
{
  this->computeHamiltonian(A, B);
  this->computeRiccati(A, B, E);
  this->computeVelocity(A, B, E);

  return cmd_vel;
}

void LQR::computeHamiltonian(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
  // Set Hamilton matrix.
  Eigen::MatrixXd Ham;
  Ham = Eigen::MatrixXd::Zero(2 * A.rows(), 2 * A.rows());
  Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

  // Get eigenvalues and eigenvectors from Hamilton matrix.
  Eigen::EigenSolver<Eigen::MatrixXd> eigen(Ham);

  // Form a 2nxn matrix whos columns from a basis of the corresponding subspace.
  eigenVec = Eigen::MatrixXcd::Zero(2 * A.rows(), A.rows());

  // Iterate over the Hamilton matrix and extract a stable Eigenvector.
  int j = 0;
  for(unsigned int i = 0; i < 2 * A.rows(); ++i)
  {
    // Get the negative real part: a stable value in the Laplace domain.
    if(eigen.eigenvalues()[i].real() < 0)
    {
      eigenVec.col(j) = eigen.eigenvectors().block(0, i, 2 * A.rows(), 1);
      ++j;
    }
  }
}

void LQR::computeRiccati(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E)
{
  Eigen::MatrixXd K;
  Eigen::MatrixXcd U11, U21;

  U11 = eigenVec.block(0, 0, A.rows(), A.rows());
  U21 = eigenVec.block(A.rows(), 0, A.rows(), A.rows());
  P = (U21 * U11.inverse()).real();

  // Update LQR gain.
  K = R.inverse() * B.transpose() * P;
  controlInput = K * E;
}

void LQR::computeVelocity(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E)
{
  cmd_vel << 
    sqrt(pow(controlInput(0, 0), 2.0) + pow(controlInput(0, 1), 2.0) + pow(controlInput(0, 2), 2.0)),
    sqrt(pow(controlInput(1, 0), 2.0) + pow(controlInput(1, 1), 2.0) + pow(controlInput(1, 2), 2.0));
}
