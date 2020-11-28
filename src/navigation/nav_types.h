#ifndef NAV_TYPES_CS393R_HH
#define NAV_TYPES_CS393R_HH

namespace navigation{

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float distance_to_goal;
  float cost;
  Eigen::Vector2f fpl_end;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  Eigen::Vector2f end_point; 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Obstacle{
  Eigen::Vector2f loc;
  double timestamp;
};

}

#endif
