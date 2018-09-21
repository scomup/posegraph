#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "rigid_transform.h"
#include "edge.h"
#include "edge_cost.h"
#include "g2o_io.h"
#include "ceres_pose.h"
#include "viewer/viewer.h"


using namespace sample_carto;


int main(int argc, char** argv)
{

    std::string filename;
    if (argc >= 2)
    {
        filename = argv[1];
    }
    else
    {
        std::cout << "No input file!" << std::endl;
        return 0;
    }

    G2OFile g2odata(filename);
    auto nodes = g2odata.nodes();
    auto edges = g2odata.edges();

    //Create edges for two adjacent edges.
    for (auto i = 0; i < (int)nodes.size() - 1; i++)
    {
        transform::Rigid3d &current_node = nodes[i];
        transform::Rigid3d &next_node = nodes[i + 1];
        transform::Rigid3d transfom_cur_to_nxt = current_node.inverse() * next_node;
        edges.push_back(Edge{i, i + 1,
                                               Edge::Pose{transfom_cur_to_nxt, 1, 10},
                                               Edge::Tag::NORMAL});
    }

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    std::map<int, CeresPose*> c_nodes;

    for (auto i = 0; i < (int)nodes.size(); i++)
    {
        
        ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();
        CeresPose* ceres_pose =  new CeresPose(nodes[i]);
        c_nodes.emplace(i, ceres_pose);
        problem.AddParameterBlock(c_nodes[i]->data().translation.data(), 3,
                                  nullptr);
        problem.AddParameterBlock(c_nodes[i]->data().rotation.data(), 4,
                                  local_parameterization);
    }

    for (const auto &edge : edges)
    {

        ceres::CostFunction *cost =
            new ceres::AutoDiffCostFunction<EdgeCost, 6, 4, 3, 4, 3>(
                new EdgeCost(edge.pose));
        problem.AddResidualBlock(
            cost,
            edge.tag == Edge::LOOP_CLOSING
                ? new ceres::HuberLoss(100)
                : nullptr,
            c_nodes[edge.i]->data().rotation.data(),
            c_nodes[edge.i]->data().translation.data(),
            c_nodes[edge.j]->data().rotation.data(),
            c_nodes[edge.j]->data().translation.data());
    }
    // Solve.
    ceres::Solver::Options options;
    options.use_nonmonotonic_steps = true;
    options.max_num_iterations = 10;
    options.num_threads = 1;

    ceres::Solver::Summary summary;

    std::cout<<"Number of nodes:"<<nodes.size()<<"\n";
    std::cout<<"Number of edges:"<<edges.size()<<"\n";
    std::cout<<"Try to solve...\n";

    ceres::Solve(options, &problem, &summary);
    std::cout<<"Oh yeah! we did it!\n";
    std::map<int, sample_carto::transform::Rigid3d> new_nodes;

    for (auto node : c_nodes)
    {
        auto new_node =  node.second->ToRigid();
        //std::cout << new_node << "\n";
        new_nodes[node.first] = new_node;
    }

    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);
    viewer->UpdateNodes(nodes, 0);
    viewer->UpdateNodes(new_nodes, 1);
    viewer->UpdateEdges(edges);
    viewer_thread.join();
}
