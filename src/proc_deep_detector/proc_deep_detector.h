//
// Created by walle on 7/5/18.
//

#ifndef PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H
#define PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H

#include <ros/ros.h>
#include "integration/TensorflowModel.h"
#include <image_transport/image_transport.h>
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>


namespace proc_deep_detector{

using namespace proc_deep_detector;
using namespace TensorflowModel;

class DeepNetwork {
public:
    DeepNetwork() = delete;
    ~DeepNetwork() = default;
    DeepNetwork(const ros::NodeHandle &nh);
    DeepNetwork(const ros::NodeHandle &nh, const std::string &model_path, const std::string &input_node,
                const std::vector<std::string> &output_node, ModelType type);

private:
    ros::NodeHandle nh_;
    std::shared_ptr<TensorflowModel::TensorflowModel> model_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber subscriber_;
    cv::Mat img_;
    std::mutex img_lock_;

    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
};

}


#endif //PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H
