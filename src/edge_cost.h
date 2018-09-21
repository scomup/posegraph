#ifndef EDGE_COST_H_
#define EDGE_COST_H_

#include "edge.h"
#include "rigid_transform.h"


class EdgeCost
{
  public:
    EdgeCost(const Edge::Pose &pose)
        : pose_(pose) {}

    template <typename T>
    Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
        const Eigen::Quaternion<T> &quaternion) const
    {
        Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
        // We choose the quaternion with positive 'w', i.e., the one with a smaller
        // angle that represents this orientation.
        if (normalized_quaternion.w() < 0.)
        {
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
        constexpr double kCutoffAngle = 1e-7; // We linearize below this angle.
        const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
        return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                      scale * normalized_quaternion.y(),
                                      scale * normalized_quaternion.z());
    }

    template <typename T>
    bool operator()(const T *const start_rotation, const T *const start_translation,
                    const T *const end_rotation, const T *const end_translation,
                    T *const error) const
    {

        auto relative_pose = pose_.zbar_ij;
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
            RotationQuaternionToAngleAxisVector(h_rotation_inverse * relative_pose.rotation().cast<T>());

        error[0] = T(relative_pose.translation().x()) - h_translation[0];
        error[1] = T(relative_pose.translation().y()) - h_translation[1];
        error[2] = T(relative_pose.translation().z()) - h_translation[2];
        error[3] = angle_axis_difference[0];
        error[4] = angle_axis_difference[1];
        error[5] = angle_axis_difference[2];

        error[0] = error[0] * pose_.translation_weight;
        error[1] = error[1] * pose_.translation_weight;
        error[2] = error[2] * pose_.translation_weight;
        error[3] = error[3] * pose_.rotation_weight;
        error[4] = error[4] * pose_.rotation_weight;
        error[5] = error[5] * pose_.rotation_weight;

        return true;
    }

    const Edge::Pose pose_;
};

#endif
