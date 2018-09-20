/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SAMPLE_CARTO_CORE_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_H_
#define SAMPLE_CARTO_CORE_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "rigid_transform.h"

namespace sample_carto {
namespace core {
namespace optimization {


template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<T>& quaternion) {
  Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
  // We choose the quaternion with positive 'w', i.e., the one with a smaller
  // angle that represents this orientation.
  if (normalized_quaternion.w() < 0.) {
    // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
    normalized_quaternion.w() *= T(-1.);
    normalized_quaternion.x() *= T(-1.);
    normalized_quaternion.y() *= T(-1.);
    normalized_quaternion.z() *= T(-1.);
  }
  // We convert the normalized_quaternion into a vector along the rotation axis
  // with length of the rotation angle.
  const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                normalized_quaternion.w());
  constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
  const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
  return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                scale * normalized_quaternion.y(),
                                scale * normalized_quaternion.z());
}

template <typename T>
static std::array<T, 6> ComputeUnscaledError(
    const transform::Rigid3d& relative_pose, const T* const start_rotation,
    const T* const start_translation, const T* const end_rotation,
    const T* const end_translation) {
  const Eigen::Quaternion<T> R_i_inverse(start_rotation[0], -start_rotation[1],
                                         -start_rotation[2],
                                         -start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> delta(end_translation[0] - start_translation[0],
                                     end_translation[1] - start_translation[1],
                                     end_translation[2] - start_translation[2]);
  const Eigen::Matrix<T, 3, 1> h_translation = R_i_inverse * delta;

  const Eigen::Quaternion<T> h_rotation_inverse =
      Eigen::Quaternion<T>(end_rotation[0], -end_rotation[1], -end_rotation[2],
                           -end_rotation[3]) *
      Eigen::Quaternion<T>(start_rotation[0], start_rotation[1],
                           start_rotation[2], start_rotation[3]);

  const Eigen::Matrix<T, 3, 1> angle_axis_difference =
      RotationQuaternionToAngleAxisVector(
          h_rotation_inverse * relative_pose.rotation().cast<T>());

  return {{T(relative_pose.translation().x()) - h_translation[0],
           T(relative_pose.translation().y()) - h_translation[1],
           T(relative_pose.translation().z()) - h_translation[2],
           angle_axis_difference[0], angle_axis_difference[1],
           angle_axis_difference[2]}};
}

template <typename T>
std::array<T, 6> ScaleError(const std::array<T, 6>& error,
                            double translation_weight, double rotation_weight) {
  // clang-format off
  return {{
      error[0] * translation_weight,
      error[1] * translation_weight,
      error[2] * translation_weight,
      error[3] * rotation_weight,
      error[4] * rotation_weight,
      error[5] * rotation_weight
  }};
  // clang-format on
}




}  // namespace optimization
}  // namespace core
}  // namespace sample_carto

//#include "src/core/pose_graph/optimization/cost_helpers_impl.h"

#endif  // SAMPLE_CARTO_CORE_OPTIMIZATION_COST_FUNCTIONS_COST_HELPERS_H_
