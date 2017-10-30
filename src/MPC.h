#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  /**
   * Solve the model given an initial state and polynomial coefficients of reference trajectory.
   * @param[in]   state         initial state {x, y, psi, v, cte, epsi}
   * @param[in]   coeffs        polynomial coefficients of reference trajectory y=p(x)
   * @param[out]  delta         control value for steering angle
   * @param[out]  a             control value for acceleration
   * @param[out]  x_trajectory  trajectory component x
   * @param[out]  y_trajectory  trajectory component y
   * @return success of operation
   */
  bool Solve
    (
      const Eigen::VectorXd &state,
      const Eigen::VectorXd &coeffs,
      double &delta,
      double &a,
      vector<double> &x_trajectory,
      vector<double> &y_trajectory
    );
};

#endif /* MPC_H */
