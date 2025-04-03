#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pipes.h"

class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @sa ControllerInterface and @sa MissionInterface for more information
    */
    Mission(std::vector<ControllerInterface*> controllers);

    ~Mission();

    void setGoals(std::vector<pfms::geometry_msgs::Point*> goals);

    bool runMission();

    void setMissionObjective(mission::Objective objective);

    std::vector<double> getDistanceTravelled();

    std::vector<double> getTimeMoving();

    std::vector<unsigned int> getPlatformGoalAssociation();

private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point*> goals_; //!< A private copy of goals
  std::vector<bool> goalsReachable_; //!< A private copy of goals
  std::vector<unsigned int> association_;//!< Allocation of platforms to goals

  Pipes* pipes_;
  unsigned int goalNum_;
  mission::Objective objective_;
  bool missionOK_;
};

#endif // RANGERFUSION_H
