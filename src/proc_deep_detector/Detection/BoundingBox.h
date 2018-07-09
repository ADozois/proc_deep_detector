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

#ifndef PROC_DEEP_DETECTOR_BOUNDINGBOX_H
#define PROC_DEEP_DETECTOR_BOUNDINGBOX_H

#include <geometry_msgs/Pose2D.h>


struct BoundingBox{
    geometry_msgs::Pose2D center_;
    double width_;
    double height_;

    void TransferPose(geometry_msgs::Pose2D * src, geometry_msgs::Pose2D * dst){
        dst->x = src->x;
        dst->y = src->y;
        dst->theta= src->theta;
    }

    void TransferBoundingBox(BoundingBox * src){
        TransferPose(&src->center_, &center_);
        height_ = src->height_;
        width_ = src->width_;
    }
};



#endif //PROC_DEEP_DETECTOR_BOUNDINGBOX_H
