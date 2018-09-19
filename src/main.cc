#include<iostream>
#include<fstream>

#include "rigid_transform.h"
#include "constraints.h"
#include "spa_cost_function_3d.h"
#include "csv.h"
#include "ceres_pose.h"
#include "make_unique.h"

using namespace sample_carto;

std::vector<transform::Rigid3d> readNodes(std::string filename)
{
    std::ifstream file(filename);
    CSVRow row;

    std::vector<transform::Rigid3d> nodes;
    while (file >> row)
    {
        double x = std::stod(row[0]);
        double y = std::stod(row[1]);
        double z = std::stod(row[2]);
        double yaw = std::stod(row[3]);
        nodes.emplace_back(Eigen::Vector3d(x, y, z), transform::RollPitchYaw(0, 0, yaw));
    }
    return nodes;
}

int main()
{
    auto nodes = readNodes("node0.csv");
    std::vector<core::Constraint> constraints;
    for ( auto i = 0; i < (int)nodes.size() - 1; i++){
        transform::Rigid3d& current_node = nodes[i];
        transform::Rigid3d& next_node = nodes[i+1];
        transform::Rigid3d transfom_cur_to_nxt = current_node.inverse() * next_node;
        constraints.push_back(core::Constraint{i, i+1,
                                               core::Constraint::Pose{transfom_cur_to_nxt, 1, 10},
                                               core::Constraint::Tag::INTER_SUBMAP});
    }
    auto transfom_first_to_last = transform::Rigid3d {Eigen::Vector3d(0, 0, 0), transform::RollPitchYaw(0, 0, 0)};
    constraints.push_back(core::Constraint{0, (int)nodes.size() - 1,
                                           core::Constraint::Pose{transfom_first_to_last, 1, 10},
                                           core::Constraint::Tag::INTRA_SUBMAP});

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    std::map<int, core::optimization::CeresPose> C_nodes;

    for (auto i = 0; i < (int)nodes.size() ; i++)
    {
        C_nodes.emplace(
            i,
            core::optimization::CeresPose(nodes[i], nullptr,
                      common::make_unique<ceres::QuaternionParameterization>(),
                      &problem));
    }

    for (const auto &constraint : constraints)
    {
        problem.AddResidualBlock(
            core::optimization::SpaCostFunction3D::CreateAutoDiffCostFunction(constraint.pose),
            // Only loop closure constraints should have a loss function.
            constraint.tag == core::Constraint::INTER_SUBMAP
                ? new ceres::HuberLoss(100)
                : nullptr /* loss function */,
            C_nodes.at(constraint.id0).rotation(),
            C_nodes.at(constraint.id0).translation(),
            C_nodes.at(constraint.id1).rotation(),
            C_nodes.at(constraint.id1).translation());
    }
      // Solve.
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 10;
  options.num_threads = 1;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::ofstream outputfile("node1.csv");

  for (auto node : C_nodes){
      auto new_node =  node.second.ToRigid();
      //nodes[i] = C_nodes[i].ToRigid();
      std::cout<<"before:"<< nodes[node.first].translation()<<std::endl <<"after:"<< new_node.translation()<<std::endl<<std::endl;
      outputfile << new_node.translation().x()<<", "<< new_node.translation().y()<<", "<< new_node.translation().z()<<"\n";
  }
  outputfile.close();

  

}