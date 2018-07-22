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

#include "proc_deep_detector.h"
#include "integration/TensorflowObjectDetection.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace proc_deep_detector;

DeepNetwork::DeepNetwork(const ros::NodeHandle &nh, const std::string &config_file) :
        config_{YAML::LoadFile(config_file)},
        nh_{nh},
        model_{nullptr},
        it_{nh}{
        std::string model_path = config_["model_path"].as<std::string>();
        std::string label_path = config_["label_path"].as<std::string>();
        std::string input_node = config_["input_node"].as<std::string>();
        std::vector<std::string> output_node = config_["output_node"].as<std::vector<std::string>>();
        subscriber_name_ = config_["image_subscriber"].as<std::string>();
        ModelType type = TensorflowModel::GetModelType(config_["type"].as<std::string>());
        float threshold = config_["threshold"].as<float>();
    if (type == ModelType::DETECTION) {
        ROS_INFO("Loading model");
        model_.reset(new TensorflowObjectDetection(model_path, label_path, input_node, output_node, threshold));
        ROS_INFO("Model loaded");
        bbox_publisher_ = nh_.advertise<DetectionArray>("/proc_deep_detector/bounding_box", 10);
        image_subscriber_ = it_.subscribe(subscriber_name_, 10, &DeepNetwork::ImageCallback, this);
        ROS_INFO("Starting detection");
    }
}

DeepNetwork::DeepNetwork(const ros::NodeHandle &nh, const std::string &model_path, const std::string &label_path,
                         const std::string &input_node, const std::vector<std::string> &output_node,
                         const std::string &image_subscriber, ModelType type) :
        nh_{nh},
        model_{nullptr},
        it_{nh},
        subscriber_name_{image_subscriber}{
    if (type == ModelType::DETECTION) {
        ROS_INFO("Loading model");
        model_.reset(new TensorflowObjectDetection(model_path, label_path, input_node, output_node, 0.5));
        ROS_INFO("Model loaded");
        bbox_publisher_ = nh_.advertise<DetectionArray>("/proc_deep_detector/bounding_box", 10);
        image_subscriber_ = it_.subscribe(subscriber_name_, 10, &DeepNetwork::ImageCallback, this);
        ROS_INFO("Starting detection");
    }
}

void DeepNetwork::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    img_lock_.lock();
    try {
        cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(img_);
        cv::cvtColor(img_, img_, cv::COLOR_BGR2RGB);
        std::shared_ptr<TensorflowObjectDetection> detection = std::static_pointer_cast<TensorflowObjectDetection>(
                model_);
        detection->AddImage(img_);
        detection->Run();
        objects_.detected_object = detection->GetPredictions();
        bbox_publisher_.publish(objects_);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("%s Could not convert from '%s' to 'bgr8'.", "Topic: ", subscriber_name_.c_str(),
                  msg->encoding.c_str());
    }
    img_lock_.unlock();
}


