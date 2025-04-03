#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"


class Mission: public MissionInterface
{
public:

  /**
   * @brief Constructor
   * 
   * Receive controller pointer as an argument and use it to initialize the corresponding member variable
   * Also set default value for the mission objective  as BASIC   
   *  
   * @param controllers A vector of pointer to ControllerInterface object 
  */
  Mission(std::vector<ControllerInterface*> controllers);


  /**
   * @brief Accepts the container of goals and the corresponding platform
   *
   * Do additional goal allocation according the objective of the mission, since different objectives requires
   * different ways to store and distribut goals
   * 
   * Conduct graph search accordingly if the objective is either ADVANCED or SUPER
   * 
   * Calculate the total distance that the platforms have to travel given the goal vector processed in the above part
   * 
   * @param goals Vector of Points 
   * @param platform Platform type
   */
  virtual void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform);


  /**
   * @brief Runs the mission, non blocking call
   * 
   * @return bool indicating mission can be completed (false if mission not possible)
  */
  virtual bool run();


/**
 * @brief Status of the mission
 * 
 * Returns mission completion status (indicating percentage of completion of task) by each platform @sa setGoals
 * 
 * @return Vector with each element of vector corresponding to a platform. The value is percent of completed distance of entire mission for the 
 * corresponding platform value between 0-100.
 */
  virtual std::vector<unsigned int> status(void);


  /**
   * @brief Set mission objective
   * 
   * Also clears the goal vector of Ackerman and Quadcopter
   */
  virtual void setMissionObjective(mission::Objective objective);


  /**
   * @brief Returns a vector of same size as number of controllers (platforms).
   * The values in the vector correspond to the total distance travelled by the corresponding platform
   * from the time of starting the program.
   *
   * @return std::vector<double> - each element is distance travelled for each platform [m]
   *
   */
  virtual std::vector<double> getDistanceTravelled();


  /**
   * @brief Returns a vector of same size as number of controllers (platforms).
   * The values in the vector correspond to the time the corresponding platfore has been moving
   * from the time the program started. Moving means the platform was not stationary.
   *
   * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
   */
  virtual std::vector<double> getTimeMoving();

  /**
   * @brief Returns a vector of same size as the total number of goals. The values in the vector
   * are a pair, corresponding to the controller that is completing the goal and the goal number
   * (the goal number is derived from the order of all goals supplied)
   *
   * @return vector of pair of int as per brief
   */
  virtual std::vector<std::pair<int, int>> getPlatformGoalAssociation();



private:
  std::vector<ControllerInterface*> controllers_; 

  std::vector<pfms::geometry_msgs::Point> goalsAckerman_;
  std::vector<pfms::geometry_msgs::Point> goalsQuadcopter_;
  
  std::vector<pfms::geometry_msgs::Point> goalsSuper_;

  mission::Objective missionObjective_;

  double totalDistanceAckerman_ = -1;
  double totalDistanceQuadcopter_ = -1;

  std::vector<std::pair<int, int>> platformAssociation_;

  double minDistAckerman_;
  double minDistQuadcopter_;

  std::vector<int> bestOrderAckerman_;
  std::vector<int> bestOrderQuadcopter_; 

  double minCost_;

  // A flag indicating whether the set of goals is reachable, updated by checkOriginToDestination
  bool reachable_ = true;
};

#endif // RANGERFUSION_H
