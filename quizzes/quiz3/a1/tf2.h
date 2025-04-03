#ifndef TF2_H
#define TF2_H

#include "types.h"
#include <vector>

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;

namespace tf2 {

    /**
     * @brief Compute x,y location of enemy in the reference frame of base station (which is at 0,0)
     * @param rangeBearing - (polar coordinates) of enemy, obtained from the Pose of aircraft suppplied
     * @param aircraft - Pose where the readings were obtained, this pose in relative to map origin (base station)
     * @return location of enemy relative to map origin (base station)
     */

    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft);

    /**
     * @brief compute the range and bearig (polar coordinates) of enemy, relative to aicrraft pose suppplied
     * @param globalEnemy - x,y location of enemy in the reference frame of (base station)
     * @param aircraft - Pose relative to map origin (base station)
     * @return rangeBearing - (polar coordinates of bogie) relative to the aircraft Pose suppplied
     */

    RangeBearingStamped global2local(Point globalEnemy, Pose aircraft);

    /**
     * @brief
     * Normalises angle between -PI to PI, can only handle angles between -2PI to 4PI
     * @param angle
     * @return normalised angle
     */
    double normaliseAngle(double theta);
}

#endif // TF2_H
