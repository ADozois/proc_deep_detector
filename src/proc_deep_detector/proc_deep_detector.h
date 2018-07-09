//
// Created by walle on 7/5/18.
//

#ifndef PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H
#define PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H

#include <ros/ros.h>
#include "integration/TensorflowModel.h"
namespace proc_deep_detector{

using namespace proc_deep_detector;

class DeepNetwork {
public:
    DeepNetwork() = delete;
    ~DeepNetwork() = default;
    DeepNetwork(const ros::NodeHandle &nh);
    DeepNetwork(const ros::NodeHandle &nh, TensorflowModel *model);

private:
    ros::NodeHandle nh_;
    TensorflowModel * model_;
};

}


#endif //PROC_DEEP_DETECTOR_PROC_DEEP_DETECTOR_H
