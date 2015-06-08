#ifndef HIGH_LEVEL_ACTIONS_H_
#define HIGH_LEVEL_ACTIONS_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread/mutex.hpp>
#include <carl_dynamixel/LookAtFrame.h>
#include <carl_moveit/ArmAction.h>
#include <carl_moveit/ObtainObjectAction.h>
#include <carl_navigation/MoveCarlAction.h>
#include <wpi_jaco_msgs/HomeArmAction.h>

#define NUM_JACO_JOINTS 6
#define MAX_HOME_ATTEMPTS 3

class SequentialTasks
{

public:

  /**
  * \brief Constructor
  */
  SequentialTasks();

  /**
  * \brief Perform a sequence that collects and packs all of the objects needed to pack a school bag
  *
  * Note that this task is designed to show a use case for object replacement.
  */
  void packSchoolBag();

  void checkSurface(int location, std::string surfaceLink);

  void obtainObject(int location, std::string object, std::string surfaceLink);

private:
  ros::NodeHandle n;

  ros::ServiceClient lookAtFrameClient;

  actionlib::SimpleActionClient<carl_moveit::ArmAction> armClient;
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> armHomeClient;
  actionlib::SimpleActionClient<carl_moveit::ObtainObjectAction> obtainObjectClient;
  actionlib::SimpleActionClient<carl_navigation::MoveCarlAction> moveCarlClient;
};

#endif
