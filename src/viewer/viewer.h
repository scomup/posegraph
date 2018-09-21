/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VIEWER_H
#define VIEWER_H

#include "../edge.h"
#include <iostream>
#include <memory>
#include <mutex>


class Viewer
{
public:
    Viewer();

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    bool isFinished();

    bool isStopped();

    void UpdateNodes(std::map<int, sample_carto::transform::Rigid3d>& nodes, int id);
    void UpdateEdges(std::vector<Edge>& edges);


    void SetFinish();


    

private:
    void DrawNodes(int mode);
    void DrawEdges(int mode);



    double image_width_;
    double image_height_;
    double view_point_x_;
    double view_point_y_;
    double view_point_z_;
    double view_point_f_;

    bool finish_;

    std::mutex mutex_frame_;
    std::map<int, sample_carto::transform::Rigid3d> nodes0_;
    std::map<int, sample_carto::transform::Rigid3d> nodes1_;
    std::vector<Edge> edges_;

};

#endif // VIEWER_H
	

