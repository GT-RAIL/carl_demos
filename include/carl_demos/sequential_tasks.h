#ifndef HIGH_LEVEL_ACTIONS_H_
#define HIGH_LEVEL_ACTIONS_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread/mutex.hpp>
#include <carl_dynamixel/LookAtFrame.h>
#include <carl_demos/ObtainObjectAction.h>
#include <rail_manipulation_msgs/ArmAction.h>
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

  void makeFruitBasket();

  void checkSurface(int location, std::string surfaceLink);

  /**
   * \brief Navigate to a surface, pick up an object, and store it on the robot
   *
   * \param location navigation location, pass -1 for no navigation
   * \param object name of the object to pick up
   * \param surfaceLink name of the surface link to look at (only used if navigation is specified)
   */
  void obtainObject(int location, std::string object, std::string surfaceLink);

private:
  ros::NodeHandle n;

  ros::ServiceClient lookAtFrameClient;

  actionlib::SimpleActionClient<rail_manipulation_msgs::ArmAction> armClient;
  actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> armHomeClient;
  actionlib::SimpleActionClient<carl_demos::ObtainObjectAction> obtainObjectClient;
  actionlib::SimpleActionClient<carl_navigation::MoveCarlAction> moveCarlClient;
};

#endif
