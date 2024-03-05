#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>

#include "eigen_points_types.h"
#include "m-detector/dynamic_object_filter.h"

std::shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();

std::string points_topic, odom_topic;
std::string out_folder, out_folder_origin;
double lidar_end_time = 0;
int cur_frame = 0;

std::deque<M3D> buffer_rots;
std::deque<V3D> buffer_poss;
std::deque<double> buffer_times;
std::deque<boost::shared_ptr<PointCloudXYZI>> buffer_pcs;
std::vector<double> lidar_base_transform(3, 0.0);
std::vector<double> lidar_base_rotation(3, 0.0);

ros::Publisher pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std;

void OdomCallback(const nav_msgs::Odometry& cur_odom) {
  Eigen::Quaterniond cur_q;
  geometry_msgs::Quaternion tmp_q;
  tmp_q = cur_odom.pose.pose.orientation;
  tf::quaternionMsgToEigen(tmp_q, cur_q);
  cur_rot = cur_q.matrix();
  cur_pos << cur_odom.pose.pose.position.x, cur_odom.pose.pose.position.y,
      cur_odom.pose.pose.position.z;
  buffer_rots.push_back(cur_rot);
  buffer_poss.push_back(cur_pos);
  lidar_end_time = cur_odom.header.stamp.toSec();
  buffer_times.push_back(lidar_end_time);
}

void PointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg_in) {
  boost::shared_ptr<PointCloudXYZI> feats_undistort(new PointCloudXYZI());
  boost::shared_ptr<PointCloudXYZI> points_aft_trans(new PointCloudXYZI());
  pcl::fromROSMsg(*msg_in, *feats_undistort);
  //激光坐标与odom不一致时需要转换
  Eigen::Matrix4d points_trans_mat = Eigen::Matrix4d::Identity();
  M3D points_rot = Eigen::Matrix3d::Identity();
  V3D points_pose = Eigen::Vector3d::Zero();
  points_pose << lidar_base_transform[0], lidar_base_transform[1],
      lidar_base_transform[2];
  points_rot =
      Eigen::AngleAxisd(lidar_base_rotation[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(lidar_base_rotation[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(lidar_base_rotation[0], Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  points_trans_mat.block<3, 3>(0, 0) = points_rot;
  points_trans_mat.block<3, 1>(0, 3) = points_pose;
  pcl::transformPointCloud(*feats_undistort, *points_aft_trans,
                           points_trans_mat);

  buffer_pcs.push_back(points_aft_trans);
}

void TimerCallback(const ros::TimerEvent& e) {
  if (buffer_pcs.size() > 0 && buffer_poss.size() > 0 &&
      buffer_rots.size() > 0 && buffer_times.size() > 0) {
    boost::shared_ptr<PointCloudXYZI> cur_pc = buffer_pcs.at(0);
    buffer_pcs.pop_front();
    auto cur_rot = buffer_rots.at(0);
    buffer_rots.pop_front();
    auto cur_pos = buffer_poss.at(0);
    buffer_poss.pop_front();
    auto cur_time = buffer_times.at(0);
    buffer_times.pop_front();
    std::string file_name = out_folder;
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << cur_frame;
    file_name += ss.str();
    file_name.append(".label");
    std::string file_name_origin = out_folder_origin;
    std::stringstream sss;
    sss << std::setw(6) << std::setfill('0') << cur_frame;
    file_name_origin += sss.str();
    file_name_origin.append(".label");

    if (file_name.length() > 15 || file_name_origin.length() > 15)
      DynObjFilt->set_path(file_name, file_name_origin);

    DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
    DynObjFilt->publish_dyn(pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std,
                            cur_time);
    cur_frame++;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynfilter_odom");
  ros::NodeHandle nh;
  nh.param<std::string>("dyn_obj/points_topic", points_topic, "");
  nh.param<std::string>("dyn_obj/odom_topic", odom_topic, "");
  nh.param<std::string>("dyn_obj/out_file", out_folder, "");
  nh.param<std::string>("dyn_obj/out_file_origin", out_folder_origin, "");
  nh.param<std::vector<double>>("dyn_obj/lidar_base_transform", lidar_base_transform,
                                lidar_base_transform);
  nh.param<std::vector<double>>("dyn_obj/lidar_base_rotation", lidar_base_rotation,
                                lidar_base_rotation);

  DynObjFilt->init(nh);
  /*** ROS subscribe and publisher initialization ***/
  pub_pcl_dyn_extend =
      nh.advertise<sensor_msgs::PointCloud2>("/m_detector/frame_out", 10000);
  pub_pcl_dyn =
      nh.advertise<sensor_msgs::PointCloud2>("/m_detector/point_out", 100000);
  pub_pcl_std =
      nh.advertise<sensor_msgs::PointCloud2>("/m_detector/std_points", 100000);
  ros::Subscriber sub_pcl = nh.subscribe(points_topic, 200000, PointsCallback);
  ros::Subscriber sub_odom = nh.subscribe(odom_topic, 200000, OdomCallback);
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), TimerCallback);

  ros::spin();
  return 0;
}
