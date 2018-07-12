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

#ifndef PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H
#define PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H

#include <ros/ros.h>
#include "integration/TensorflowModel.h"
#include <image_transport/image_transport.h>
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <proc_deep_detector/DetectionArray.h>


namespace proc_deep_detector{

using namespace proc_deep_detector;
using namespace TensorflowModel;

class DeepNetwork {
public:
    DeepNetwork() = delete;
    ~DeepNetwork() = default;
    DeepNetwork(const ros::NodeHandle &nh);
    DeepNetwork(const ros::NodeHandle &nh, const std::string &model_path, const std::string &label_path,
                    const std::string &input_node, const std::vector<std::string> &output_node,
                    ModelType type);

private:
    ros::NodeHandle nh_;
    std::shared_ptr<TensorflowModel::TensorflowModel> model_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_subscriber_;
    ros::Publisher bbox_publisher_;
    DetectionArray objects_;
    cv::Mat img_;
    std::mutex img_lock_;

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
};

}


#endif //PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H
