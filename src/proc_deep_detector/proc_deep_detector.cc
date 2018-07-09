//
// Created by walle on 7/5/18.
//

#include "proc_deep_detector.h"

using namespace proc_deep_detector;

DeepNetwork::DeepNetwork(const ros::NodeHandle &nh):
    nh_{nh},
    model_{nullptr}{

}

DeepNetwork::DeepNetwork(const ros::NodeHandle &nh, TensorflowModel *model):
    nh_{nh},
    model_{model}{

}