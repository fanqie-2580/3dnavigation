/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*/
#ifndef MPC_CRITICS__COLLISION_MODEL_H_
#define MPC_CRITICS__COLLISION_MODEL_H_

#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

class CollisionModel : public ScoringModel
{
public:
  CollisionModel();
  virtual double scoreTrajectory(base_trajectory::Trajectory &traj);

protected:
  virtual void onInitialize();

  double collision_check_radius_;
  double inscribed_shrink_x_;
  double inscribed_shrink_y_;
  double inscribed_shrink_z_;
  double soft_margin_x_;
  double soft_margin_y_;
  double soft_margin_z_;
  double soft_margin_weight_;
  bool verbose_reject_log_;
  int ignore_first_n_points_;
};

}  // namespace mpc_critics

#endif  // MPC_CRITICS__COLLISION_MODEL_H_
