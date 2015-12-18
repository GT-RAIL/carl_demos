#include <carl_demos/sequential_tasks.h>

using namespace std;

SequentialTasks::SequentialTasks() :
    armClient("carl_moveit_wrapper/common_actions/arm_action"),
    armHomeClient("jaco_arm/home_arm"),
    obtainObjectClient("carl_demos/obtain_object"),
    moveCarlClient("move_carl")
{
  lookAtFrameClient = n.serviceClient<carl_dynamixel::LookAtFrame>("/asus_controller/look_at_frame");

  ROS_INFO("Waiting for action servers...");
  armClient.waitForServer();
  obtainObjectClient.waitForServer();
  moveCarlClient.waitForServer();
  armHomeClient.waitForServer();
  ROS_INFO("Initialized.");
}

void SequentialTasks::makeFruitBasket()
{
  ROS_INFO("*************************************************");
  ROS_INFO("Apple");
  ROS_INFO("*************************************************");
  obtainObject(-1, "apple", "kitchen_table_surface_link");

  ROS_INFO("*************************************************");
  ROS_INFO("Orange");
  ROS_INFO("*************************************************");
  obtainObject(-1, "orange", "kitchen_table_surface_link");

  ROS_INFO("*************************************************");
  ROS_INFO("Banana");
  ROS_INFO("*************************************************");
  obtainObject(-1, "banana", "kitchen_table_surface_link");
}

void SequentialTasks::packSchoolBag()
{
  /*
  //notebook
  obtainObject(carl_navigation::MoveCarlGoal::COFFEE_TABLE, "notebook", "coffee_table_surface_link");

  //look for glue
  checkSurface(carl_navigation::MoveCarlGoal::KITCHEN_TABLE, "kitchen_table_surface_link");

  //get feedback
  carl_navigation::MoveCarlGoal moveGoal;
  moveGoal.location = carl_navigation::MoveCarlGoal::INTERACTION_2;
  moveCarlClient.sendGoal(moveGoal);
  moveCarlClient.waitForResult(ros::Duration(60.0));
  //look at chair
  carl_dynamixel::LookAtFrame lookSrv;
  lookSrv.request.frame = "lazy_chair_2_surface_link";
  if (!lookAtFrameClient.call(lookSrv))
  {
    ROS_INFO("Could not call look at frame client!");
    return;
  }
  */


  //tape
  obtainObject(carl_navigation::MoveCarlGoal::KITCHEN_TABLE, "tape", "kitchen_table_surface_link");

  //look for pencil
  checkSurface(carl_navigation::MoveCarlGoal::COFFEE_TABLE, "coffee_table_surface_link");

  //get feedback
  carl_navigation::MoveCarlGoal moveGoal;
  moveGoal.location = carl_navigation::MoveCarlGoal::INTERACTION_1;
  moveCarlClient.sendGoal(moveGoal);
  moveCarlClient.waitForResult(ros::Duration(60.0));
  //look at chair
  carl_dynamixel::LookAtFrame lookSrv;
  lookSrv.request.frame = "lazy_chair_2_surface_link";
  if (!lookAtFrameClient.call(lookSrv))
  {
    ROS_INFO("Could not call look at frame client!");
    return;
  }


  /*
  //pens
  obtainObject(carl_navigation::MoveCarlGoal::COFFEE_TABLE, "pens", "coffee_table_surface_link");

  //ruler
  obtainObject(carl_navigation::MoveCarlGoal::KITCHEN_TABLE, "ruler", "kitchen_table_surface_link");

  //scissors
  obtainObject(carl_navigation::MoveCarlGoal::KITCHEN_TABLE, "scissors", "kitchen_table_surface_link");

  //look for apple
  checkSurface(carl_navigation::MoveCarlGoal::FRIDGE, "kitchen_counter_right_surface_link");

  //get feedback
  carl_navigation::MoveCarlGoal moveGoal;
  moveGoal.location = carl_navigation::MoveCarlGoal::INTERACTION_2;
  moveCarlClient.sendGoal(moveGoal);
  moveCarlClient.waitForResult(ros::Duration(60.0));
  //look at chair
  carl_dynamixel::LookAtFrame lookSrv;
  lookSrv.request.frame = "lazy_chair_2_surface_link";
  if (!lookAtFrameClient.call(lookSrv))
  {
    ROS_INFO("Could not call look at frame client!");
    return;
  }
  */

  /*
  //orange
  obtainObject(carl_navigation::MoveCarlGoal::FRIDGE, "kitchen_counter_right_surface_link");

  //return bag
  carl_navigation::MoveCarlGoal moveGoal;
  moveGoal.location = carl_navigation::MoveCarlGoal::INTERACTION_2;
  moveCarlClient.sendGoal(moveGoal);
  moveCarlClient.waitForResult(ros::Duration(60.0));
  */

  ROS_INFO("Finished.");
}

void SequentialTasks::checkSurface(int location, std::string surfaceLink)
{
  ROS_INFO("Navigating to new location...");
  //navigate to given location
  carl_navigation::MoveCarlGoal moveGoal;
  moveGoal.location = location;
  moveCarlClient.sendGoal(moveGoal);
  moveCarlClient.waitForResult(ros::Duration(60.0));
  ROS_INFO("Navigation complete.");

  ROS_INFO("Looking at surface...");
  //look at surface
  carl_dynamixel::LookAtFrame lookSrv;
  lookSrv.request.frame = surfaceLink;
  if (!lookAtFrameClient.call(lookSrv))
  {
    ROS_INFO("Could not call look at frame client!");
    return;
  }
  ROS_INFO("Looking at surface complete.");

  ROS_INFO("Retracting arm...");
  //retract arm
  rail_manipulation_msgs::ArmGoal retractGoal;
  retractGoal.action = rail_manipulation_msgs::ArmGoal::RETRACT;
  armClient.sendGoal(retractGoal);
  armClient.waitForResult(ros::Duration(20.0));
  bool success = armClient.getResult()->success;
  if (!success)
  {
    ROS_INFO("Could not ready arm after storing object.");
    return;
  }
  ROS_INFO("Arm ready.");

  ros::Duration(5.0).sleep();
}

void SequentialTasks::obtainObject(int location, string object, string surfaceLink)
{
  if (location != -1)
  {
    ROS_INFO("Navigating to new location...");
    //navigate to given location
    carl_navigation::MoveCarlGoal moveGoal;
    moveGoal.location = location;
    moveCarlClient.sendGoal(moveGoal);
    moveCarlClient.waitForResult(ros::Duration(60.0));
    ROS_INFO("Navigation complete.");

    ROS_INFO("Looking at surface...");
    //look at surface
    carl_dynamixel::LookAtFrame lookSrv;
    lookSrv.request.frame = surfaceLink;
    if (!lookAtFrameClient.call(lookSrv)) {
      ROS_INFO("Could not call look at frame client!");
      return;
    }
    ROS_INFO("Looking at surface complete.");
  }

  ROS_INFO("Obtaining object...");
  //obtain given object
  carl_demos::ObtainObjectGoal obtainGoal;
  obtainGoal.lift = true;
  obtainGoal.verify = false;
  obtainGoal.object_name = object;
  obtainObjectClient.sendGoal(obtainGoal);
  obtainObjectClient.waitForResult(ros::Duration(60.0));
  carl_demos::ObtainObjectResultConstPtr obtainResult = obtainObjectClient.getResult();
  if (!obtainResult->success)
  {
    ROS_INFO("Failed to obtain requested object.");
    return;
  }
  ROS_INFO("Object obtained.");

  ROS_INFO("Homing arm...");
  //ready arm
  wpi_jaco_msgs::HomeArmGoal homeGoal;
  homeGoal.retract = false;
  armHomeClient.sendGoal(homeGoal);
  armHomeClient.waitForResult(ros::Duration(20.0));
  bool success = armHomeClient.getResult()->success;
  if (!success)
  {
    ROS_INFO("Could not ready arm after storing object.");
    return;
  }
  ROS_INFO("Arm ready.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sequential_tasks");

  SequentialTasks st;

  st.makeFruitBasket();

  ROS_INFO("Action teminated.");

  return EXIT_SUCCESS;
}
