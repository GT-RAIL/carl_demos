#include <carl_demos/sequential_tasks.h>

using namespace std;

SequentialTasks::SequentialTasks() :
    armClient("carl_moveit_wrapper/common_actions/arm_action"),
    obtainObjectClient("carl_moveit_wrapper/high_level_actions/obtain_object"),
    moveCarlClient("move_carl")
{
  lookAtFrameClient = n.serviceClient<carl_dynamixel::LookAtFrame>("/asus_controller/look_at_frame");
}

void SequentialTasks::packSchoolBag()
{
  obtainObject(carl_navigation::MoveCarlGoal::KITCHEN_TABLE, "tape");
}

void SequentialTasks::obtainObject(int location, string object)
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
  lookSrv.request.frame = "kitchen_table_surface_link";
  if (!lookAtFrameClient.call(lookSrv))
  {
    ROS_INFO("Could not call look at frame client!");
    return;
  }
  ROS_INFO("Looking at surface complete.");

  ROS_INFO("Obtaining object...");
  //obtain given object
  carl_moveit::ObtainObjectGoal obtainGoal;
  obtainGoal.lift = true;
  obtainGoal.verify = false;
  obtainGoal.object_name = object;
  obtainObjectClient.sendGoal(obtainGoal);
  obtainObjectClient.waitForResult(ros::Duration(60.0));
  carl_moveit::ObtainObjectResultConstPtr obtainResult = obtainObjectClient.getResult();
  if (!obtainResult->success)
  {
    ROS_INFO("Failed to obtain requested object.");
    return;
  }
  ROS_INFO("Object obtained.");

  ROS_INFO("Readying arm...");
  //ready arm
  carl_moveit::ArmGoal readyGoal;
  readyGoal.action = carl_moveit::ArmGoal::READY;
  armClient.sendGoal(readyGoal);
  bool completed = armClient.waitForResult(ros::Duration(20.0));
  bool succeeded = (armClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  bool success = armClient.getResult()->success;
  if (!completed || !succeeded || !success)
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

  st.packSchoolBag();

  ROS_INFO("Action teminated.");

  return EXIT_SUCCESS;
}
