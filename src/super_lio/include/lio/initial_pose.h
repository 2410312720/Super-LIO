#ifndef SUPER_LIO_INITIAL_POSE_H_
#define SUPER_LIO_INITIAL_POSE_H_

#include "basic/Manifold.h"

namespace LI2Sup {

// Optional rigid transform from Super-LIO's default local world frame into an
// externally defined world frame. Keeping this separate from relocation's
// initial guess lets normal mapping and future map-based relocation coexist.
struct InitialPoseOptions {
  bool enabled = false;
  BASIC::SE3 world_from_lio;
};

BASIC::SE3 applyInitialPoseAlignment(
    const BASIC::SE3& local_pose,
    const InitialPoseOptions& options);

}  // namespace LI2Sup

#endif  // SUPER_LIO_INITIAL_POSE_H_
