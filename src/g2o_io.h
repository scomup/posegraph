#ifndef G2O_IO_H_
#define G2O_IO_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <sstream>
#include <map>

#include "edge.h"

class G2OFile
{
  public:
    std::vector<std::string> split(const std::string &str, char sep)
    {
        std::vector<std::string> v;
        std::stringstream ss(str);
        std::string buffer;
        while (std::getline(ss, buffer, sep))
        {
            v.push_back(buffer);
        }
        v.erase(
            std::remove_if(
                v.begin(), v.end(),
                [](std::string str) { return str == std::string(""); }),
            v.end());
        return v;
    }

    G2OFile(std::string filename)
    {
        std::ifstream infile(filename);

        if (infile.fail())
        {
            std::cout << "Failed to open file: " << filename << std::endl;
            exit(0);
        }
        std::string line;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            auto v = split(line, ' ');
            if (v[0] == "VERTEX_SE2")
            {
                int id = std::stod(v[1]);
                double x = std::stod(v[2]);
                double y = std::stod(v[3]);
                double z = 0;
                double yaw = std::stod(v[4]);
                nodes_[id] = sample_carto::transform::Rigid3d(Eigen::Vector3d(x, y, z),
                                                              sample_carto::transform::RollPitchYaw(0, 0, yaw));
            }
            else if (v[0] == "VERTEX_SE3:QUAT")
            {
                int id = std::stod(v[1]);
                double x = std::stod(v[2]);
                double y = std::stod(v[3]);
                double z = std::stod(v[4]);
                double qx = std::stod(v[5]);
                double qy = std::stod(v[6]);
                double qz = std::stod(v[7]);
                double qw = std::stod(v[8]);

                nodes_[id] = sample_carto::transform::Rigid3d(Eigen::Vector3d(x, y, z),
                                                              Eigen::Quaterniond(qw, qx, qy, qz));
            }

            else if (v[0] == "EDGE_SE2")
            {
                int id_i = std::stod(v[1]);
                int id_j = std::stod(v[2]);
                double x = std::stod(v[3]);
                double y = std::stod(v[4]);
                double z = 0;
                double yaw = std::stod(v[5]);
                sample_carto::transform::Rigid3d transfom_ij(Eigen::Vector3d(x, y, z),
                                                             sample_carto::transform::RollPitchYaw(0, 0, yaw));
                edges_.push_back(Edge{id_i, id_j,
                                            Edge::Pose{transfom_ij, 1, 10},
                                            Edge::Tag::LOOP_CLOSING});
            }
            else if (v[0] == "EDGE_SE3:QUAT")
            {
                int id_i = std::stod(v[1]);
                int id_j = std::stod(v[2]);
                double x = std::stod(v[3]);
                double y = std::stod(v[4]);
                double z = std::stod(v[5]);;
                double qx = std::stod(v[6]);
                double qy = std::stod(v[7]);
                double qz = std::stod(v[8]);
                double qw = std::stod(v[9]);
                sample_carto::transform::Rigid3d transfom_ij(Eigen::Vector3d(x, y, z),
                                                             Eigen::Quaterniond(qw, qx, qy, qz));
                edges_.push_back(Edge{id_i, id_j,
                                            Edge::Pose{transfom_ij, 1, 5},
                                            Edge::Tag::LOOP_CLOSING});
            }
            else
            {
                std::cout << "undefined: " << v[0] << "\n";
                exit(0);
            }
        }
    }

    std::map<int, sample_carto::transform::Rigid3d>& nodes(){return nodes_;}
    std::vector<Edge>& edges(){return edges_;}

  private:
    std::map<int, sample_carto::transform::Rigid3d> nodes_;
    std::vector<Edge> edges_;
};

#endif // G2O_IO_H_


