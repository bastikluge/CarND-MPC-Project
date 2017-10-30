#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
public:
  // Set the timestep length and duration
  static const size_t N;
  static const double dt;

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  static const double Lf;

  // Both the reference cross track and orientation errors are 0.
  // The reference velocity is set to 40 mph.
  static const double ref_v;

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
