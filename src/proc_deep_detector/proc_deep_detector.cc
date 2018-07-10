//
// Created by walle on 7/5/18.
//

#include "proc_deep_detector.h"
#include "integration/TensorflowObjectDetection.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace proc_deep_detector;

DeepNetwork::DeepNetwork(const ros::NodeHandle &nh):
    nh_{nh},
    model_{nullptr},
    it_{nh}{

}

DeepNetwork::DeepNetwork(const ros::NodeHandle &nh, const std::string &model_path, const std::string &input_node,
                         const std::vector<std::string> &output_node, ModelType type) :
    nh_{nh},
    model_{nullptr},
    it_{nh}{
    if (type == ModelType::DETECTION){
        model_ .reset(new TensorflowObjectDetection(model_path, input_node, output_node));
        subscriber_ = it_.subscribe("/provider_vision/Front_GigE", 10, &DeepNetwork::ImageCallback, this);
    }
}

void DeepNetwork::ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    img_lock_.lock();
    try
    {
        cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(img_);
        std::shared_ptr<TensorflowObjectDetection> detection = std::static_pointer_cast<TensorflowObjectDetection>(model_);
        detection->AddImage(img_);
        detection->Run();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("%s Could not convert from '%s' to 'bgr8'.", "Topic: /provider_vision/FrontGigE", msg->encoding.c_str());
    }
    img_lock_.unlock();
}
