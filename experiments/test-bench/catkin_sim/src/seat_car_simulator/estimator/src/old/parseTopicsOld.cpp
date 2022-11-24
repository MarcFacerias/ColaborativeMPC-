#define _GLIBCXX_USE_C99 1
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include <sstream>
#include <deque>
#include <string>
#include <sstream>

// GLOBAL VARS
std::deque<geometry_msgs::Pose> pose_landmark;
std::deque<int>  id_landmark;
bool publish_flag = true;

// Alternativa
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

void GetPoseCallback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{

  for (int j = 0; j< msg.markers.size(); j++){

    pose_landmark.push_back(msg.markers[j].pose.pose);
    id_landmark.push_back(msg.markers[j].id);

  }

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ParseMessage");

  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("Corrected_Pose", 1000);

  ros::Rate loop_rate(10);

  ros::Subscriber sub = n.subscribe("/ar_pose_marker", 1000, GetPoseCallback);

  while (ros::ok())
  {
    ar_track_alvar_msgs::AlvarMarkers msg_lnd_corrected;
    ar_track_alvar_msgs::AlvarMarker lnd_corrected;


    for (int i = 0; i < pose_landmark.size(); i++ ){

        std::stringstream ss_tag;
        std::stringstream ss_landmark;
        std::string str;
        str = ToString(id_landmark[i]);

        ss_tag << "landmark_" << str[1] << "_tag_link" << str[2];
        ss_landmark << "landmark_" << str[1] << "_link";

        geometry_msgs::TransformStamped cam_tag;
        geometry_msgs::TransformStamped tag_center;
        geometry_msgs::TransformStamped cam_center;

        try{
          tag_center = tfBuffer.lookupTransform(ss_landmark.str(), ss_tag.str(),
                                   ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        tf2::Quaternion q1( tag_center.transform.rotation.x,
                           tag_center.transform.rotation.y,
                           tag_center.transform.rotation.z,
                           tag_center.transform.rotation.w
            );
        tf2::Vector3 r1( tag_center.transform.translation.x,
                        tag_center.transform.translation.y,
                        tag_center.transform.translation.z
            );
        tf2::Transform tag_center_tf( q1,r1);

        tf2::Quaternion q2( pose_landmark[i].rotation.x,
                           pose_landmark[i].rotation.y,
                           pose_landmark[i].rotation.z,
                           pose_landmark[i].rotation.w
            );
        tf2::Vector3 r2( pose_landmark[i].translation.x,
                        pose_landmark[i].translation.y,
                        pose_landmark[i].translation.z
            );
        tf2::Transform cam_tag_tf( q2,r2);

        tf2::Transform cam_center_tf = cam_tag_tf.inverse() * tag_center_tf;

        lnd_corrected.id = str[1] - '0';
        msg_lnd_corrected.markers.push_back(lnd_corrected);
        pose_landmark.pop_front();
        id_landmark.pop_front();

    }


    pose_pub.publish(msg_lnd_corrected);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
