/**
 * @file MPC.cpp
 * @author duckstarr
 * @brief Finite-horizon Model Predictive Controller
 * 
 */

#include <MPC.hpp>
#include <Eigen/Dense>
#include <iostream>

using namespace Optimal_Controller;

MPC::MPC(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const double S, const double tolarance) 
  : Q(Q)
  , R(R)
  , saturation(S)
  , tolarance(tolarance)
  , converged(false)
{
  Ad.setZero(Q.rows(), Q.cols());
  I.setIdentity(Q.rows(), Q.cols());
  cmd_vel.setZero(1, R.rows()); // [translation rate, rotation rate]
}

MPC::~MPC()
{
}

Eigen::MatrixXd MPC::getVelocity(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& E, double dt)
{
  this->discretization(A, B, dt);
  this->computeRiccati(E, dt);
  this->computeVelocity();

  return cmd_vel;
}

void MPC::discretization(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double dt)
{
  Ad = (I + A * dt * 0.5) * (I - A * dt * 0.5).inverse();
  Bd = B * dt;
}

void MPC::computeRiccati(const Eigen::MatrixXd& E, double dt)
{
  Eigen::MatrixXd K;
  Eigen::MatrixXd P_new;

  // A Cost-to-go matrix P evolving backwards in time from Q is defined as,
  P = Q;

  // Iterating over a finite-horizon Algebraic Riccati Equation to a steady-state solution; that is, P_new - P ~= 0 + tolarance
  for(unsigned int i = 0; i < numIterations; i++)
  {
    P_new = Ad.transpose() * P * Ad - (Ad.transpose() * P * Bd) * (R + Bd.transpose() * P * Bd).inverse() * (Bd.transpose() * P * Ad) + Q;

    if(fabs((P_new - P).maxCoeff()) < tolarance)
    {
      converged = true;
      break;
    }

    // Update Riccati solution
    P = P_new;
  }

    K = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    controlInput = K * E;
}

void MPC::computeVelocity()
{
  if (converged)
  {    
    // Compute magnitude of velocity vector
    cmd_vel << 
      sqrt(pow(controlInput(0, 0), 2.0) + pow(controlInput(0, 1), 2.0) + pow(controlInput(0, 2), 2.0)),
      sqrt(pow(controlInput(1, 0), 2.0) + pow(controlInput(1, 1), 2.0) + pow(controlInput(1, 2), 2.0));

    // Saturate output signal
    if(cmd_vel(0, 0) > saturation) cmd_vel(0, 0)  = saturation;
    if(cmd_vel(0, 0) < -saturation) cmd_vel(0, 0) = -saturation;
    if(cmd_vel(0, 1) > saturation) cmd_vel(0, 1)  = saturation;
    if(cmd_vel(0, 1) < -saturation) cmd_vel(0, 1) = -saturation;
  }
  else
  {
    std::cout << "MPC controller did not converge. Resetting cmd_vel..." << std::endl;
    cmd_vel.setZero();
  }
}
