#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"

class Mission: public MissionInterface
{
public:
    /**
    The Default constructor
    @sa ControllerInterface and @sa MissionInterface for more information
    */
  Mission(std::vector<ControllerInterface*> controllers);

  void setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
  bool runMission() override;
  void setMissionObjective(mission::Objective objective) override;
  std::vector<double> getDistanceTravelled() override;
  std::vector<double> getTimeMoving() override;
  std::vector<unsigned int> getPlatformGoalAssociation() override;


private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point> goals_; //!< A private copy of goals
  mission::Objective objective_;
  std::vector<double> distance_travelled_;
  std::vector<double> time_moving_;
  std::vector<unsigned int> platform_goal_association_;

};

#endif // MISSION_H
