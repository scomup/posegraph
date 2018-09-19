/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef SAMPLE_CARTO_CORE_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_
#define SAMPLE_CARTO_CORE_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_

#include <array>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "math.h"
#include "rigid_transform.h"
#include "transform.h"
#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "constraints.h"
#include "cost_helpers.h"

#include "ceres/ceres.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"


namespace sample_carto {
namespace core {
namespace optimization {

class SpaCostFunction3D {
 public:
  static ceres::CostFunction* CreateAutoDiffCostFunction(
      const Constraint::Pose& pose) {
    return new ceres::AutoDiffCostFunction<
        SpaCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
        3 /* translation variables */, 4 /* rotation variables */,
        3 /* translation variables */>(new SpaCostFunction3D(pose));
  }

  template <typename T>
  bool operator()(const T* const c_i_rotation, const T* const c_i_translation,
                  const T* const c_j_rotation, const T* const c_j_translation,
                  T* const e) const {
    const std::array<T, 6> error = ScaleError(
        ComputeUnscaledError(pose_.zbar_ij, c_i_rotation, c_i_translation,
                             c_j_rotation, c_j_translation),
        pose_.translation_weight, pose_.rotation_weight);
    std::copy(std::begin(error), std::end(error), e);
    return true;
  }

 private:
  explicit SpaCostFunction3D(const Constraint::Pose& pose)
      : pose_(pose) {}

  const Constraint::Pose pose_;
};

}  // namespace optimization
}  // namespace core
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_CORE_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_H_