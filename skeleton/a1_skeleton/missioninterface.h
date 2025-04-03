#ifndef MISSIONINTERFACE_H
#define MISSIONINTERFACE_H

#include <vector>
#include "controllerinterface.h"

/**
 * @brief Specifies the required interface for your Mission class
 * must inherit from it. <b> You MUST NOT edit this file </b>.
 * 
 */

/*!
 *  \brief     Mision Interface Class
 *  \details
 *  Specifies the required interface for your Mission
 *  must inherit from it. <b> You MUST NOT edit this file </b>.
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2022-03-02
 *  \pre       none
 *  \bug       none reported as of 2022-03-02
 *  \warning   students MUST NOT change this class (the header file)
 */

namespace  mission{

    typedef enum {
      TIME, /*!< Selection of Mission Objective based on Time */
      DISTANCE /*!< Selection of Mission Objective based on Distance */
    } Objective; /*!< Platform Types */

}


class MissionInterface
{
public:
    MissionInterface(){};

    /**
     * @brief Accepts the container of goals.
     *
     * @param goals
     */
    virtual void setGoals(std::vector<pfms::geometry_msgs::Point*> goals) = 0;

    /**
     * @brief Runs the mission
      * @return bool indicating mission complete (false if mission not possible OR aborted because it
      * can not be completed )
     */
    virtual bool runMission() = 0;

    /**
     * @brief Set mission objective
     */
    virtual void setMissionObjective(mission::Objective objective) = 0;

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the total distance travelled by the corresponding platform
     * from the time of starting the program.
     *
     * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
     *
     */
    virtual std::vector<double> getDistanceTravelled() = 0;

    /**
     * @brief Returns a vector of same size as number of controllers (platforms).
     * The values in the vector correspond to the time the corresponding platfore has been moving
     * from the time the program started. Moving means the platform was not stationary.
     *
     * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
     *
     */
    virtual std::vector<double> getTimeMoving() = 0;

    /**
     * @brief Returns a vector of same size as number of goals. The values in the vector
     * correspond to the platform number that is completing the goal
     *
     * @return vector of unsigned int's corresponds to platform number completing the goal
     *
     * @sa grabAndFuseData
     */
    virtual std::vector<unsigned int> getPlatformGoalAssociation() = 0;


};

#endif // MISSIONINTERFACE_H
