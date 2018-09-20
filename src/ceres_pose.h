
#ifndef CERES_POSE_H_
#define CERES_POSE_H_

#include <array>
#include <memory>

#include "rigid_transform.h"

class CeresPose {
 public:
   CeresPose(const sample_carto::transform::Rigid3d &pose)
   {
       Data d{{{pose.translation().x(), pose.translation().y(),
                   pose.translation().z()}},
                 {{pose.rotation().w(), pose.rotation().x(),
                   pose.rotation().y(), pose.rotation().z()}}};
       data_ = std::make_shared<Data>(d);
   }

   struct Data
   {
       std::array<double, 3> translation;// Rotation quaternion as (w, x, y, z).
       std::array<double, 4> rotation;
  };

  Data& data() { return *data_; }
  const sample_carto::transform::Rigid3d ToRigid() {return sample_carto::transform::Rigid3d::FromArrays(data_->rotation, data_->translation);};

 private:
  std::shared_ptr<Data> data_;
};

#endif  // SAMPLE_CARTO_CORE_OPTIMIZATION_CERES_POSE_H_
