/*! @file
 *
 *  @brief Collection of simple types used by the simulator class.
 *  Types are now related to ROS messages, by name and composition.
 *
 *  Students will to use this header and structures to receive data from the simulator.
 *
 *  @author Alen Alempijevic
 *  @date 10-09-2021
*/
#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cmath> // For abs

static const double float_comparison_tolerence =0.1;

namespace geometry_msgs {

    struct Point{
        double x; /*!< x coordinate within global map (m) */
        double y; /*!< y coordinate within global map (m) */
        double z; /*!< z coordinate within global map (m) */
        bool operator== (const Point &p1){
          return ((std::abs(this->x - p1.x)<float_comparison_tolerence) &&
                  (std::abs(this->y - p1.y)<float_comparison_tolerence) &&
                  (std::abs(this->z - p1.z)<float_comparison_tolerence));
        }

    };

    struct Quaternion{
      double x;
      double y;
      double z;
      double w;
    };

    struct Pose {
      Point position;
      Quaternion orientation;
    };


    struct RangeBearingStamped { /*!< Contains a timestamped range/bearing reading */
      double range;             /*!< The range (distance reading) in metres */
      double bearing;           /*!< The bearing (bearing reading) in radians */
      long timestamp;           /*!< Timestamp (milliseconds) */
    };

    struct RangeVelocityStamped { /*!< Contains a timestamped range/velocity reading */
      double range;               /*!< The range (distance reading) in metres */
      double velocity;            /*!< Linear velocity (metres/second) */
      long timestamp;             /*!< Timestamp (milliseconds) */
    };
}
#endif // TYPES_H
