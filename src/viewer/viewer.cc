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

#include "viewer.h"
#include <pangolin/pangolin.h>
#include "myhandler.h"

#include "draw_object_helper.h"
#include "frustum_culling.h"

#include <iostream>
#include <chrono>
#include <thread>


Viewer::Viewer()
{

    image_width_ = 640;
    image_height_ = 480;
    view_point_x_ = 0;
    view_point_y_ = 0;
    view_point_z_ = 100;
    view_point_f_ = 2000;
    finish_ = false;
}

void Viewer::Run()
{
    pangolin::CreateWindowAndBind("pose graph viewer",1024,768);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuShowOptResult("menu.Show result",false,true);



    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,view_point_f_,view_point_f_,512,389,0.1,1000),
                pangolin::ModelViewLookAt(view_point_x_,view_point_y_,view_point_z_, 0,0,0,-0.1,1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::MyHandler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();



    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    
   //auto lastTime = std::chrono::system_clock::now();

    while( !pangolin::ShouldQuit() && !finish_)
    {
        //auto currentTime = std::chrono::system_clock::now();
        //auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
        //printf("%d ms\n", msec);
        //lastTime = currentTime;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        d_cam.Activate(s_cam);
        
        pangolin::glDrawAxis(1);
        DrawGrid(200,1);
        if(!menuShowOptResult){
            DrawNodes(0);
            DrawEdges(0);
        }
        else{
            DrawNodes(1);
            DrawEdges(1);

        }

        pangolin::FinishFrame();
    }
    SetFinish();
}

void Viewer::UpdateNodes(std::map<int, sample_carto::transform::Rigid3d> &nodes, int id)
{
    std::unique_lock<std::mutex> lock(mutex_frame_);
    if (id == 0)
        nodes0_ = nodes;
    else
        nodes1_ = nodes;
}


void Viewer::UpdateEdges(std::vector<Edge>& edges)
{
    std::unique_lock<std::mutex> lock(mutex_frame_);
    edges_ = edges;
}

void Viewer::DrawEdges(int mode)
{
    std::map<int, sample_carto::transform::Rigid3d>* nodes;
    if(!mode){
        nodes = &nodes0_;
    }
    else{
        nodes = &nodes1_;
    }
    for (const auto &edge : edges_)
    {
        if (edge.tag == Edge::Tag::LOOP_CLOSING)
        {
            const int id_i = edge.i;
            const int id_j = edge.j;
            //glLineWidth(3);
            glColor3f(0.0f, 1.0f, 0.0f);
            pangolin::glDrawLine((*nodes)[id_i].translation().x(),
                                 (*nodes)[id_i].translation().y(),
                                 (*nodes)[id_i].translation().z(),
                                 (*nodes)[id_j].translation().x(),
                                 (*nodes)[id_j].translation().y(),
                                 (*nodes)[id_j].translation().z());
            //drawSphere(constraint_pose.translation().x(),constraint_pose.translation().y(),constraint_pose.translation().z(),0.5);
            //std::cout<<"Constraint from "<<constraint_pose.translation()<<" to "<<node_pose.translation()<<std::endl;
        }
    }
}



void Viewer::DrawNodes(int mode)
{
    std::unique_lock<std::mutex> lock(mutex_frame_);
    std::map<int, sample_carto::transform::Rigid3d>* nodes;
    if(!mode){
        glColor3f(0,0,1);
        nodes = &nodes0_;
    }
    else{
        glColor3f(1,0,0);
        nodes = &nodes1_;
    }
    glLineWidth(1);

    glBegin(GL_LINE_STRIP);
    for (auto it = nodes->begin(); it != nodes->end(); ++it)
    {
        double x = it->second.translation().x();
        double y = it->second.translation().y();
        double z = it->second.translation().z();
        glVertex3f(x, y, z);
    }
    glEnd();

    for (auto it = nodes->begin(); it != nodes->end(); ++it)
    {
        double x = it->second.translation().x();
        double y = it->second.translation().y();
        double z = it->second.translation().z();
        drawSphere(x,y,z,0.02);
    }
}

void Viewer::SetFinish()
{
    std::unique_lock<std::mutex> lock(mutex_frame_);
    finish_ = true;
}

bool Viewer::isFinished()
{
    std::unique_lock<std::mutex> lock(mutex_frame_);
    return finish_;
}

