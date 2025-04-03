#include "rangeprocessing.h"
#include <algorithm>
#include <numeric>

namespace range_processing {


    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    bool detectHeightOfObject(sensor_msgs::Range rangeScan, geometry_msgs::Pose pose, double& height)
    {
        if (rangeScan.range >= rangeScan.min_range && rangeScan.range <= rangeScan.max_range){
            height = pose.position.z - rangeScan.range;
            return true;
        }
        else return false;
    }


    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    bool detectPerson(sensor_msgs::Range rangeScan, geometry_msgs::Pose pose, double personHeight, geometry_msgs::Point& location)
    {
        double height;
        if (detectHeightOfObject(rangeScan, pose, height)){
            location.x = pose.position.x;
            location.y = pose.position.y;
            location.z = personHeight;
            return true;
        }
        
        else
            return false;
    }
}
