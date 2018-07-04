/// \author	Antoine Dozois <dozois.a@gmail.com>
/// \copyright Copyright (c) 2018 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <tensorflow/core/public/session.h>
#include "tensorflow/core/platform/env.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "integration/TensorflowObjectDetection.h"

int main() {
    std::string graph_path = "/home/walle/Workspaces/ros_sonia_ws/src/deep_detector/models/ssd_mobilenet_v11_coco/frozen_inference_graph.pb";
    std::string image_path = "/home/walle/PS/test.jpg";

    tensorflow::Session* session= nullptr;
    tensorflow::Status status = tensorflow::NewSession(tensorflow::SessionOptions(), &session);

    if (!status.ok()) {
        std::cout << "Can't create session_. Error: " << status.ToString() << "\n";
        return 1;
    } else {
        std::cout << "Session successfully created" << std::endl;
    }

    tensorflow::GraphDef graph;
    status = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_path, &graph);

    if (!status.ok()) {
        std::cout << "Can't create graph_. Error: " << status.ToString() << "\n";
        return 1;
    } else {
        std::cout << "Graph successfully created" << std::endl;
    }


    return 0;
}



