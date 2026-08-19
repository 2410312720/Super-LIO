#include "lio/initial_pose.h"

namespace LI2Sup {

BASIC::SE3 applyInitialPoseAlignment(
    const BASIC::SE3& local_pose,
    const InitialPoseOptions& options) {
  if (!options.enabled) {
    return local_pose;
  }
  return options.world_from_lio * local_pose;
}

}  // namespace LI2Sup
