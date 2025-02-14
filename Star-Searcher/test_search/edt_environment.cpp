#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

namespace fast_planner {

void EDTEnvironment::init() {
}

void EDTEnvironment::setMap(shared_ptr<SDFMap>& map) {
  this->sdf_map_ = map;
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time, double& dist,
                                         Eigen::Vector3d& grad) {
  dist = sdf_map_->getDistWithGrad(pos, grad);
}

double EDTEnvironment::evaluateCoarseEDT(const Eigen::Vector3d& pos, double time) {
  return sdf_map_->getDistance(pos);
}

}  // namespace fast_planner