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
#include "ros/ros.h"
#include "proc_deep_detector.h"
#include "integration/TensorflowModel.h"


int main(int argc, char **argv) {
    std::string graph_path = "/home/walle/Documents/frozen_inference_graph.pb";
    std::string label = "/home/walle/Documents/label_map.pbtxt";
    std::string input_node = "image_tensor:0";
    std::vector<std::string> output_node = {"detection_boxes:0", "detection_scores:0", "detection_classes:0", "num_detections:0"};

    ros::init(argc, argv, "proc_deep_detector");
    ros::NodeHandle nh("~");

    proc_deep_detector::DeepNetwork net(nh, graph_path, label, input_node, output_node,
                                        TensorflowModel::ModelType::DETECTION);

    while (ros::ok()){
        usleep(20000);
        ros::spinOnce();
    }

    return 0;
}



