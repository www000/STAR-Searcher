#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <utility>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class SDFMap;

class EDTEnvironment {
public:
  typedef shared_ptr<EDTEnvironment> Ptr;

  EDTEnvironment(/* args */) {}
  ~EDTEnvironment() {}

  shared_ptr<SDFMap> sdf_map_;

  void init();
  void setMap(shared_ptr<SDFMap> &map);
  void evaluateEDTWithGrad(const Eigen::Vector3d &pos, double time,
                           double &dist, Eigen::Vector3d &grad);
  double evaluateCoarseEDT(const Eigen::Vector3d &pos, double time);
};

} // namespace fast_planner

#endif