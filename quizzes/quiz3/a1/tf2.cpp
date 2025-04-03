#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations

namespace tf2 {

    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
        Point p;

        // Convert to local Cartesian cooridnates
        double localX = rangeBearing.range * cos(rangeBearing.bearing);
        double localY = rangeBearing.range * sin(rangeBearing.bearing);

        // Convert to global Cartesian cooridnates
        double yaw = tf::quaternionToYaw(aircraft.orientation);

        double dx = localX * cos(-yaw) + localY * sin(-yaw);
        double dy = -localX * sin(-yaw) + localY * cos(-yaw);

        p.x = dx + aircraft.position.x;
        p.y = dy + aircraft.position.y;

        return p;
    }

    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft)
    {
        RangeBearingStamped rbstamped = {0,0,0};

        // Convert to local Cartesian coordinates
        double yaw = tf::quaternionToYaw(aircraft.orientation);

        double dx = globalEnemy.x - aircraft.position.x;
        double dy = globalEnemy.y - aircraft.position.y;

        double localX = dx * cos(yaw) + dy * sin(yaw);
        double localY = -dx * sin(yaw) + dy * cos(yaw);

        // Convert to local polar coordinate
        rbstamped.range = std::sqrt(std::pow(localX,2) + std::pow(localY,2));
        rbstamped.bearing = atan2(localY,localX);   

        return rbstamped;
    }


    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      if (theta > M_PI){
          theta = -( (2* M_PI) - theta);
      }

      return theta;
    }

}
