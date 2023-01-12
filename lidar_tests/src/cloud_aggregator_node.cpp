#include <optional>

#include <glog/logging.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>

static const std::string CLOUD_TOPIC = "cloud_input";
static const std::string AGGREGATE_CLOUD_TOPIC = "aggregate_cloud";
static const int WAIT_TRANSFORM_PAUSE = 0.01;

using CloudT = pcl::PointCloud<pcl::PointXYZ>;

Eigen::Isometry3d Interpolate(Eigen::Isometry3d& t1, Eigen::Isometry3d& t2, double alpha)
{
  using namespace Eigen;
  Quaterniond rot1(t1.linear());
  Quaterniond rot2(t2.linear());
  Vector3d trans1(t1.translation());
  Vector3d trans2(t2.translation());
  Isometry3d result = Isometry3d::Identity();
  result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
  result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();
  return result;
}

std::optional<geometry_msgs::TransformStamped> GetTransform(tf2_ros::Buffer& tf_buffer, const std::string target_frame_id,
                                             const std::string child_frame_id,
                                             const ros::Time& requested_time){
  geometry_msgs::TransformStamped tf_msg;
  bool success = false;
  for(std::size_t i = 0; i < 4 && ros::ok(); i++)
  {
    try
    {
      tf_msg = tf_buffer.lookupTransform(target_frame_id, child_frame_id, requested_time, ros::Duration(0.05));
      success = true;
      break;
    }
    catch(tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM(ex.what());
      continue;
    }
  }
  if(success)
  {
    return tf_msg;
  }
  ROS_ERROR_STREAM("Failed to get transform from " <<target_frame_id << " to " << child_frame_id);
  return std::nullopt;
}

template <class P>
P LoadParameter(ros::NodeHandle& nh, const std::string& param_name)
{
  P param_val;
  CHECK(nh.getParam(param_name, param_val)) <<"Failed to get parameter " << param_name;
  return param_val;
}

template <class P, int size_>
std::array<P, size_> LoadArrayParam(ros::NodeHandle& nh, const std::string& param_name)
{
  std::vector<P> vals;
  CHECK(nh.getParam(param_name, vals)) << "Failed to get array parameter " << param_name;
  CHECK(vals.size() == size_) << "Array size from parameter "<< param_name << " != " << size_;
  std::array<P, size_> vals_arr;
  std::copy_n(vals.begin(), vals.size(), vals_arr.begin());
  return vals_arr;
}

class CloudAggregator
{
public:

  struct CloudWithPose
  {
    sensor_msgs::PointCloud2 cloud_msg;
    Eigen::Isometry3d pose;
  };

  CloudAggregator(ros::NodeHandle& nh):
    tf_listener_(tf_buffer_),
    child_frame_id_("gps"),
    target_frame_id_("odom"),
    offset_transform_(Eigen::Isometry3d::Identity()),
    do_plane_adjustment_(false),
    align_clouds_(false),
    max_cloud_count_(100),
    min_distance_(1.0),
    voxel_leaf_size_(0.05),
    cloud_count_(0),
    previous_processed_cloud_(boost::make_shared<CloudT>()),
    aggregate_cloud_(boost::make_shared<CloudT>()),
    previous_cloud_pose_(std::nullopt)
  {
    using namespace Eigen;
    // loading parameters
    ros::NodeHandle ph("~");
    ph.param("max_cloud_count", max_cloud_count_, 500);
    ph.param("voxel_leaf_size", voxel_leaf_size_, 0.05);
    ph.param("do_plane_adjustment", do_plane_adjustment_, false);
    ph.param("align_clouds", align_clouds_, false);
    min_distance_ = LoadParameter<double>(ph, "min_distance");
    child_frame_id_ = LoadParameter<std::string>(ph, "child_frame_id");
    target_frame_id_ = LoadParameter<std::string>(ph, "target_frame_id");
    min_box_bounds_ = LoadArrayParam<double, 3>(ph, "min_box_bounds");
    max_box_bounds_ = LoadArrayParam<double, 3>(ph, "max_box_bounds");

    std::vector<double> offset_transform_vals = LoadParameter< std::vector<double> >(ph, "offset_transform");
    CHECK(offset_transform_vals.size() == 6) <<" Offset transform array requires 6 entries, only "<<
        offset_transform_vals.size()<<" were passed";
    std::array<double,6> temp_vals;
    std::copy_n(offset_transform_vals.begin(), offset_transform_vals.size(), temp_vals.begin());
    auto [x, y, z, rx, ry, rz] = temp_vals;
    offset_transform_ = Translation3d(x, y, z) *
        AngleAxisd(DEG2RAD(rz), Vector3d::UnitZ()) *
        AngleAxisd(DEG2RAD(ry), Vector3d::UnitY()) *
        AngleAxisd(DEG2RAD(rx), Vector3d::UnitX());

    // Initializing ROS interfaces
    cloud_subscriber_ = nh.subscribe(CLOUD_TOPIC, 1, &CloudAggregator::CloudMsgCallback, this);
    aggregate_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(AGGREGATE_CLOUD_TOPIC,1);
    aggregation_timer_ = nh.createTimer(ros::Duration(0.2), &CloudAggregator::AggregationCalback, this);
    publish_cloud_timer_ = nh.createTimer(ros::Duration(2.0),[this](const ros::TimerEvent&){
      PublishAggregateCloud();
    });

    application_start_time_ = ros::Time::now();

    ROS_INFO("Cloud aggregator ready");
  }

  ~CloudAggregator()
  {

  }

private:

  void CloudMsgCallback(const sensor_msgs::PointCloud2& msg)
  {
    if(!first_msg_time_)
    {
      first_msg_time_ = msg.header.stamp;
    }
    if(cloud_count_ > max_cloud_count_)
    {
      return;
    }

    // getting clouds transform
    ros::Duration(WAIT_TRANSFORM_PAUSE).sleep();
    ros::Time transform_time = application_start_time_ + (msg.header.stamp - first_msg_time_.value());
    std::optional<geometry_msgs::TransformStamped> tf_msg = GetTransform(tf_buffer_, target_frame_id_,
                                                                         msg.header.frame_id,
                                                                         transform_time);
    if(!tf_msg)
    {
      return;
    }
    Eigen::Isometry3d current_pose = tf2::transformToEigen(tf_msg.value());
    CloudWithPose cloud_entry = {.cloud_msg = msg, .pose = current_pose};

    if(!previous_cloud_pose_)
    {
      clouds_list_.emplace_back(std::move(cloud_entry));
      previous_cloud_pose_ = current_pose;
      return;
    }

    // check distance from previous cloud
    const Eigen::Isometry3d& previous_pose = previous_cloud_pose_.value();
    Eigen::Vector3d prev_position = previous_pose.translation();
    Eigen::Vector3d current_position = current_pose.translation();

    double dist = (current_position - prev_position).norm();
    ROS_INFO("Distance %f, required min distance %f", dist, min_distance_);
    if(dist < min_distance_)
    {
      return;
    }

    previous_cloud_pose_ = current_pose;

    clouds_list_.emplace_back(std::move(cloud_entry));
    ROS_INFO_STREAM("Stored cloud, total cloud count " << clouds_list_.size());
  }

  void ApplyPlaneRectification(CloudT::Ptr cloud)
  {
    pcl::ModelCoefficients::Ptr coeff = boost::make_shared<pcl::ModelCoefficients>();
    pcl::PointIndices::Ptr inliers = boost::make_shared<pcl::PointIndices>();

    pcl::SACSegmentation<CloudT::PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setMaxIterations(200);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    if(inliers->indices.empty())
    {
      ROS_ERROR("Plane estimation failed");
      return;
    }

    Eigen::Vector3d plane_normal(coeff->values[0], coeff->values[1], coeff->values[2]);
    double angle = std::acos(plane_normal.dot(Eigen::Vector3d::UnitZ())/(plane_normal.norm()));
    Eigen::Vector3d rot_axis = plane_normal.cross(Eigen::Vector3d::UnitZ()).normalized();
    ROS_INFO_STREAM("Angle Axes values: " << angle <<" x: " << rot_axis.x() << " y: " << rot_axis.y() << " z: " << rot_axis.z());

    // computing centroid
    Eigen::Vector4d centroid_vals;
    pcl::compute3DCentroid(*cloud, centroid_vals);
    Eigen::Vector3d centroid(centroid_vals.x(), centroid_vals.y(), centroid_vals.z());

    Eigen::Affine3d cloud_transform = Eigen::Translation3d( centroid.x(), centroid.y(), 0.0) * Eigen::AngleAxisd(angle, rot_axis)
        * Eigen::Translation3d(-1.0 * centroid)*Eigen::Quaterniond::Identity();
     pcl::transformPointCloud(*cloud, *cloud, cloud_transform, true);
  }

  void AlignCloudNDT(CloudT::ConstPtr target_cloud, CloudT::Ptr input_cloud)
  {
    pcl::NormalDistributionsTransform<CloudT::PointType, CloudT::PointType> ndt_alg;
    ndt_alg.setTransformationEpsilon (0.01); // Setting minimum transformation difference for termination condition.
    ndt_alg.setStepSize(0.1); // Setting maximum step size for More-Thuente line search.
    ndt_alg.setResolution (1.0); //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt_alg.setMaximumIterations (10); // Setting max number of registration iterations.
    ndt_alg.setInputSource(input_cloud);
    ndt_alg.setInputTarget(target_cloud);


    CloudT::Ptr aligned_cloud = boost::make_shared<CloudT>();
    ndt_alg.align(*aligned_cloud, Eigen::Isometry3f::Identity().matrix());
    if(!ndt_alg.hasConverged())
    {
      ROS_ERROR("NDT did not converge");
      return;
    }
    ROS_INFO_STREAM("NDT converged with fitness score " << ndt_alg.getFitnessScore());
    Eigen::Isometry3f alignment_transform(ndt_alg.getFinalTransformation());
    input_cloud->clear();
    pcl::copyPointCloud(*aligned_cloud, *input_cloud);
  }

  void AlignCloudICP(CloudT::ConstPtr target_cloud, CloudT::Ptr input_cloud)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaximumIterations (100); // Setting max number of registration iterations.

    CloudT::Ptr aligned_cloud = boost::make_shared<CloudT>();
    icp.align(*aligned_cloud, Eigen::Isometry3f::Identity().matrix());
    if(!icp.hasConverged())
    {
      ROS_ERROR("ICP did not converge");
      return;
    }
    ROS_INFO_STREAM("ICP converged with fitness score " << icp.getFitnessScore());
    Eigen::Isometry3f alignment_transform(icp.getFinalTransformation());
    input_cloud->clear();
    pcl::copyPointCloud(*aligned_cloud, *input_cloud);
  }

  void AggregationCalback(const ros::TimerEvent&)
  {
    ros::Time start_time = ros::Time::now();
    if(cloud_count_ > max_cloud_count_)
    {
      ROS_INFO_STREAM_ONCE("Max cloud count "<< max_cloud_count_ << " reached, aggregation stopped");
      return;
    }

    if(clouds_list_.size() <2)
    {
      return;
    }

    CloudT temp_aggregate_cloud;
    for(std::size_t i = 0; i < clouds_list_.size(); i++)
    {
      if(cloud_count_ > max_cloud_count_)
      {
        break;
      }
      Eigen::Isometry3d current_pose = clouds_list_[i].pose * offset_transform_;
      CloudT::Ptr raw_cloud = boost::make_shared<CloudT>();
      CloudT::Ptr transformed_cloud = boost::make_shared<CloudT>();
      pcl::fromROSMsg(clouds_list_[i].cloud_msg, *raw_cloud);

      // applying cloud filters
      ApplyCloudFilters(*raw_cloud);

      // transforming
      pcl::transformPointCloud(*raw_cloud, *transformed_cloud, Eigen::Affine3d(current_pose), true);

      // forcing cloud to ground plane
      if(do_plane_adjustment_)
      {
        ApplyPlaneRectification(transformed_cloud);
      }

      if(align_clouds_ && !previous_processed_cloud_->empty())
      {
        AlignCloudICP(previous_processed_cloud_, transformed_cloud);
      }
      previous_processed_cloud_->clear();
      pcl::copyPointCloud(*transformed_cloud, *previous_processed_cloud_);

      temp_aggregate_cloud += *transformed_cloud;
      cloud_count_++;

    }
    clouds_list_.clear();
    *aggregate_cloud_ += temp_aggregate_cloud;
    //ApplyPlaneRectification(aggregate_cloud_);
    ros::Duration processing_duration = ros::Time::now() - start_time;
    ROS_INFO_STREAM("Aggregation processing took " << processing_duration.toSec() <<" seconds");
  }

  void ApplyCloudFilters(CloudT& cloud)
  {
    CloudT::Ptr filtered_cloud = cloud.makeShared();

    // voxel grid
    if(voxel_leaf_size_ > std::numeric_limits<double>::epsilon())
    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_grid.setInputCloud(filtered_cloud);
      voxel_grid.filter(*filtered_cloud);
    }

    // crop box
    pcl::CropBox<pcl::PointXYZ> crop_box(true);
    auto [min_x, min_y, min_z] = min_box_bounds_;
    auto [max_x, max_y, max_z] = max_box_bounds_;
    crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
    crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
    crop_box.setInputCloud(filtered_cloud);
    crop_box.filter(*filtered_cloud);

    // copy results
    pcl::copyPointCloud(*filtered_cloud, cloud);
  }

  void PublishAggregateCloud()
  {
    sensor_msgs::PointCloud2 aggregate_cloud_msg;
    pcl::toROSMsg(*aggregate_cloud_, aggregate_cloud_msg);
    aggregate_cloud_msg.header.frame_id = target_frame_id_;
    aggregate_cloud_msg.header.stamp = ros::Time::now();
    aggregate_cloud_publisher_.publish(aggregate_cloud_msg);
  }

  // ros interfaces
  ros::Timer aggregation_timer_;
  ros::Timer publish_cloud_timer_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher aggregate_cloud_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ros parameters
  std::string child_frame_id_;
  std::string target_frame_id_;
  Eigen::Isometry3d offset_transform_;
  bool do_plane_adjustment_;
  bool align_clouds_;
  int max_cloud_count_;
  double min_distance_; // minimum distance from the previous cloud pose required to store new cloud
  double voxel_leaf_size_;
  std::array<double, 3> min_box_bounds_;
  std::array<double, 3> max_box_bounds_;

  // intermediate members
  std::optional<geometry_msgs::TransformStamped> previous_transform_msg_;
  CloudT::Ptr previous_processed_cloud_;
  std::optional<Eigen::Isometry3d> previous_cloud_pose_;
  std::vector<CloudWithPose> clouds_list_;
  std::size_t cloud_count_;
  ros::Time application_start_time_;
  std::optional<ros::Time> first_msg_time_;

  // output
  CloudT::Ptr aggregate_cloud_;

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_aggregator");
  ros::NodeHandle nh;
  CloudAggregator cloud_aggregator(nh);
  ros::spin();

  return 0;
}
