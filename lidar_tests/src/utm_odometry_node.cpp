/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <optional>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace gps_common;

static const double EXTRAPOLATION_STEP = 0.2; // seconds

class GPSTranslator
{
public:
  GPSTranslator(ros::NodeHandle& nh)
  {
    ros::NodeHandle ph("~");

    ph.param<std::string>("frame_id", frame_id, "");
    ph.param<std::string>("child_frame_id", child_frame_id, "");
    ph.param<double>("rot_covariance", rot_cov, 99999.0);
    ph.param<bool>("append_zone", append_zone, false);
    bool extrapolate_poses;
    ph.param<bool>("extrapolate_poses", extrapolate_poses, false);
    ROS_ASSERT_MSG(ph.getParam("position_offsets", position_offsets), "Failed to load position offsets");

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

    fix_sub_ = nh.subscribe("fix", 10, &GPSTranslator::callback, this);

    if(extrapolate_poses)
    {
      pose_extrapolation_timer_ = nh.createTimer(ros::Duration(EXTRAPOLATION_STEP),
                                                 [this](const ros::TimerEvent& event){
        if(!last_tf_msg_)
        {
          return;
        }

        ros::Time current_time = ros::Time::now();
        ros::Duration time_since_last_pose = current_time - last_tf_msg_->header.stamp;
        Eigen::Vector3d displacement = time_since_last_pose.toSec() * velocity_;
        Eigen::Isometry3d extrapolated_pose = Eigen::Translation3d(displacement) *
            tf2::transformToEigen(last_tf_msg_.value());


        // broadcasting to tf
        geometry_msgs::TransformStamped tf_msg = tf2::eigenToTransform(extrapolated_pose);
        tf_msg.header = last_tf_msg_->header;
        tf_msg.header.stamp = current_time;
        tf_msg.child_frame_id = child_frame_id;

        tf_broadcaster->sendTransform(tf_msg);
      });
    }
  }

  ~GPSTranslator()
  {
  }

  void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
      ROS_DEBUG_THROTTLE(60,"No fix.");
      return;
    }

    if (fix->header.stamp == ros::Time(0)) {
      return;
    }

    double northing, easting;
    std::string zone;

    LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if (odom_pub) {
      nav_msgs::Odometry odom;
      odom.header.stamp = fix->header.stamp;

      if (frame_id.empty()) {
        if(append_zone) {
          odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
        } else {
          odom.header.frame_id = fix->header.frame_id;
        }
      } else {
        if(append_zone) {
          odom.header.frame_id = frame_id + "/utm_" + zone;
        } else {
          odom.header.frame_id = frame_id;
        }
      }

      odom.child_frame_id = child_frame_id;

      odom.pose.pose.position.x = easting - position_offsets[0];
      odom.pose.pose.position.y = northing - position_offsets[1];
      odom.pose.pose.position.z = fix->altitude - position_offsets[2];

      odom.pose.pose.orientation.x = 0;
      odom.pose.pose.orientation.y = 0;
      odom.pose.pose.orientation.z = 0;
      odom.pose.pose.orientation.w = 1;

      // Use ENU covariance to build XYZRPY covariance
      boost::array<double, 36> covariance = {{
        fix->position_covariance[0],
        fix->position_covariance[1],
        fix->position_covariance[2],
        0, 0, 0,
        fix->position_covariance[3],
        fix->position_covariance[4],
        fix->position_covariance[5],
        0, 0, 0,
        fix->position_covariance[6],
        fix->position_covariance[7],
        fix->position_covariance[8],
        0, 0, 0,
        0, 0, 0, rot_cov, 0, 0,
        0, 0, 0, 0, rot_cov, 0,
        0, 0, 0, 0, 0, rot_cov
      }};

      odom.pose.covariance = covariance;

      odom_pub.publish(odom);

      // broadcasting to tf
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header = odom.header;
      tf_msg.header.stamp = ros::Time::now();
      const auto& pos = odom.pose.pose.position;
      tf_msg.transform.translation.x = pos.x;
      tf_msg.transform.translation.y = pos.y;
      tf_msg.transform.translation.z = pos.z;
      tf_msg.transform.rotation = odom.pose.pose.orientation;
      tf_msg.child_frame_id = odom.child_frame_id;

      tf_broadcaster->sendTransform(tf_msg);

      if(!last_tf_msg_)
      {
        last_tf_msg_ = tf_msg;
        return;
      }

      Eigen::Isometry3d current_pose = tf2::transformToEigen(tf_msg);
      Eigen::Isometry3d prev_pose = tf2::transformToEigen(last_tf_msg_.value());
      ros::Duration dt = tf_msg.header.stamp - last_tf_msg_->header.stamp;
      last_tf_msg_ = tf_msg;
      velocity_ = current_pose.translation() - prev_pose.translation();
      velocity_ = velocity_/dt.toSec();
    }
  }

private:

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  ros::Publisher odom_pub;
  ros::Subscriber fix_sub_;
  std::string frame_id, child_frame_id;
  double rot_cov;
  bool append_zone = false;
  std::vector<double> position_offsets;

  std::optional<geometry_msgs::TransformStamped> last_tf_msg_;
  ros::Timer pose_extrapolation_timer_;
  Eigen::Vector3d velocity_;
};



int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle nh;
  GPSTranslator gps_translator(nh);
  ros::spin();
}

