#include <carl_demos/specific_actions.h>

using namespace std;

SpecificActions::SpecificActions() :
    armClient("carl_moveit_wrapper/common_actions/arm_action"),
    pickupClient("carl_moveit_wrapper/common_actions/pickup"),
    storeClient("carl_moveit_wrapper/common_actions/store"),
    obtainObjectServer(n, "carl_demos/obtain_object", boost::bind(&SpecificActions::executeObtainObject, this, _1), false)
{
  recognizedObjectsCounter = 0;

  recognizedObjectsSubscriber = n.subscribe("/object_recognition_listener/recognized_objects", 1, &SpecificActions::recognizedObjectsCallback, this);

  segmentClient = n.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");

  //start action server
  obtainObjectServer.start();
}

void SpecificActions::executeObtainObject(const carl_demos::ObtainObjectGoalConstPtr &goal)
{
  carl_demos::ObtainObjectFeedback feedback;
  carl_demos::ObtainObjectResult result;
  result.success = false;

  //retract arm
  rail_manipulation_msgs::ArmGoal retractGoal;
  retractGoal.action = rail_manipulation_msgs::ArmGoal::RETRACT;
  armClient.sendGoal(retractGoal);
  bool completed = armClient.waitForResult(ros::Duration(20.0));
  bool succeeded = (armClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  bool success = armClient.getResult()->success;
  if (!completed || !succeeded || !success)
  {
    ROS_INFO("Could not retract arm for segmentation.");
    obtainObjectServer.setSucceeded(result, "Could not retract arm during search.");
    return;
  }

  //perform recognition
  feedback.message = "Attempting to segment the surface.";
  obtainObjectServer.publishFeedback(feedback);
  std_srvs::Empty segment;
  recognizedObjectsCounter = 0;
  if (!segmentClient.call(segment))
  {
    ROS_INFO("Could not call segment service.");
    obtainObjectServer.setSucceeded(result, "Could not call segment service.");
    return;
  }

  //spin and wait
  feedback.message = "Waiting for recognition results.";
  ROS_INFO("Waiting on recognition...");
  obtainObjectServer.publishFeedback(feedback);
  bool finished = false;
  while (!finished)
  {
    {
      boost::mutex::scoped_lock lock(recognizedObjectsMutex);
      finished = recognizedObjectsCounter == 2;
    }
  }

  //pickup the specified object
  string objectName = boost::to_upper_copy(goal->object_name);
  bool pickupSucceeded = false;
  ROS_INFO("Looking for object %s...", objectName.c_str());
  for (unsigned int i = 0; i < recognizedObjects.objects.size(); i ++)
  {
    if (recognizedObjects.objects[i].name == objectName)
    {
      ROS_INFO("Found object! Attempting pickup...");
      rail_manipulation_msgs::PickupGoal pickupGoal;
      pickupGoal.lift = goal->lift;
      pickupGoal.verify = goal->verify;

      for (unsigned int j = 0; j < recognizedObjects.objects[i].grasps.size(); j ++)
      {
        ROS_INFO("ATTEMPTING PICKUP WITH GRASP %d", j);
        pickupGoal.pose = recognizedObjects.objects[i].grasps[j].grasp_pose;
        pickupClient.sendGoal(pickupGoal);
        pickupClient.waitForResult(ros::Duration(30.0));

        rail_manipulation_msgs::PickupResultConstPtr pickupResult = pickupClient.getResult();
        if (!pickupResult->success)
        {
          ROS_INFO("PICKUP FAILED, moving on to a new grasp...");
        }
        else
        {
          ROS_INFO("PICKUP SUCCEEDED");
          pickupSucceeded = true;
          break;
        }
      }

      if (pickupSucceeded)
        break;
    }
  }

  if (!pickupSucceeded)
  {
    ROS_INFO("Could not find or pickup the specified object.");
    obtainObjectServer.setSucceeded(result, "Could not find or pickup the specified object.");
    return;
  }

  //Store object on robot
  rail_manipulation_msgs::StoreGoal storeGoal;
  storeClient.sendGoal(storeGoal);
  storeClient.waitForResult(ros::Duration(30.0));
  success = storeClient.getResult()->success;
  if (!success)
  {
    ROS_INFO("Could not store object.");
    obtainObjectServer.setSucceeded(result, "Could not store object.");
    return;
  }

  ROS_INFO("Finished obtaining object successfully!");
  result.success = true;
  obtainObjectServer.setSucceeded(result);
}

void SpecificActions::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  boost::mutex::scoped_lock lock(recognizedObjectsMutex);

  recognizedObjects = objects;
  recognizedObjectsCounter++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "specific_actions");

  SpecificActions sa;

  ros::spin();
}
