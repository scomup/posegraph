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

#ifndef SAMPLE_CARTO_CORE_OPTIMIZATION_CERES_POSE_H_
#define SAMPLE_CARTO_CORE_OPTIMIZATION_CERES_POSE_H_

#include <array>
#include <memory>

#include "Eigen/Core"
#include "rigid_transform.h"
#include "ceres/ceres.h"

namespace sample_carto {
namespace core {
namespace optimization {

class CeresPose {
 public:
  CeresPose(
      const transform::Rigid3d& rigid,
      std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
      std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
      ceres::Problem* problem);

  const transform::Rigid3d ToRigid() const;

  double* translation() { return data_->translation.data(); }
  const double* translation() const { return data_->translation.data(); }

  double* rotation() { return data_->rotation.data(); }
  const double* rotation() const { return data_->rotation.data(); }

  struct Data {
    std::array<double, 3> translation;
    // Rotation quaternion as (w, x, y, z).
    std::array<double, 4> rotation;
  };

  Data& data() { return *data_; }

 private:
  std::shared_ptr<Data> data_;
};

CeresPose::Data FromPose(const transform::Rigid3d& pose);

} //namespace samaple_carto
} //namespace core
} //namespace optimization

#endif  // SAMPLE_CARTO_CORE_OPTIMIZATION_CERES_POSE_H_