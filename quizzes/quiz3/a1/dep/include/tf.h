#ifndef TF_H
#define TF_H

#include "types.h"

namespace tf {

    geometry_msgs::Quaternion yawToQuaternion(double yaw);

    double quaternionToYaw(geometry_msgs::Quaternion q);
}

#endif // TF_H
