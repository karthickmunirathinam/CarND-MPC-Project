#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>

class MPC
{
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state,
                            const Eigen::VectorXd &coeffs);
  // This is the length from front to CoG that has a similar radius.
  const double Lf = 2.67;
};

#endif // MPC_H
