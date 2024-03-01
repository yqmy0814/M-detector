#ifndef DYN_OBJ_FLT_H
#define DYN_OBJ_FLT_H

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <algorithm>
#include <execution>
#include <mutex>
#include <string>

#include "m-detector/dynamic_object_cluster.h"
#include "m-detector/depth_map.h"
#include "parallel_q.h"

class DynObjFilter {
 public:
  DynObjFilter(){};
  DynObjFilter(float windows_dur, float hor_resolution, float ver_resolution)
      : depth_map_duration_(windows_dur),
        horizontal_resolution_(hor_resolution),
        vertical_resolution_(ver_resolution){};
  ~DynObjFilter(){};

  void init(ros::NodeHandle &nh);
  void filter(PointCloudXYZI::Ptr feats_undistort, const M3D &rot_end,
              const V3D &pos_end, const double &scan_end_time);
  void publish_dyn(const ros::Publisher &pub_point_out,
                   const ros::Publisher &pub_frame_out,
                   const ros::Publisher &pub_steady_points,
                   const double &scan_end_time);
  void set_path(std::string file_path, std::string file_path_origin);

 private:
  void Points2Buffer(std::vector<point_soph *> &points,
                     std::vector<int> &index_vector);
  void Buffer2DepthMap(double cur_time);
  void SphericalProjection(point_soph &p, int depth_index, const M3D &rot,
                           const V3D &transl, point_soph &p_spherical);
  bool Case1(point_soph &p);
  bool Case1Enter(const point_soph &p, const DepthMap &map_info);
  bool Case1MapConsistencyCheck(point_soph &p, const DepthMap &map_info,
                                bool interp);
  float DepthInterpolationStatic(point_soph &p, int map_index,
                                 const DepthMap2D &depth_map);
  bool Case2(point_soph &p);
  bool Case2Enter(point_soph &p, const DepthMap &map_info);
  bool Case2MapConsistencyCheck(point_soph &p, const DepthMap &map_info,
                                bool interp);
  float DepthInterpolationAll(point_soph &p, int map_index,
                              const DepthMap2D &depth_map);
  bool Case2DepthConsistencyCheck(const point_soph &p,
                                  const DepthMap &map_info);
  bool Case2SearchPointOccludingP(point_soph &p, const DepthMap &map_info);
  bool Case2IsOccluded(const point_soph &p, const point_soph &p_occ);
  bool Case2VelCheck(float v1, float v2, double delta_t);
  bool InvalidPointCheck(const V3D &body, const int intensity);
  bool SelfPointCheck(const V3D &body, const dyn_obj_flg dyn);
  bool Case3(point_soph &p);
  bool Case3Enter(point_soph &p, const DepthMap &map_info);
  bool Case3MapConsistencyCheck(point_soph &p, const DepthMap &map_info,
                                bool interp);
  bool Case3SearchPointOccludedbyP(point_soph &p, const DepthMap &map_info);
  bool Case3IsOccluding(const point_soph &p, const point_soph &p_occ);
  bool Case3VelCheck(float v1, float v2, double delta_t);
  bool Case3DepthConsistencyCheck(const point_soph &p,
                                  const DepthMap &map_info);
  bool CheckVerFoV(const point_soph &p, const DepthMap &map_info);
  void SearchNeighborPixel(const point_soph &p, const DepthMap &map_info,
                           float &max_depth, float &min_depth);

  //外部参数
  // 数据集编号
  int dataset_ = 0;
  // 深度图像像素尺寸
  float horizontal_resolution_ = 0.02f, vertical_resolution_ = 0.02f;
  // 保存的深度地图总时长
  double depth_map_duration_ = 0.2;
  // 保存的深度地图与实时激光帧之间的时间延迟
  double map_buffer_delay_ = 0.1;
  // 构建深度地图的点缓存大小
  int buffer_size_ = 300000;
  // 保存的深度地图最大数量
  int max_depth_map_num_ = 5;
  // 一个深度地图像素的最大点量
  int max_pixel_points_ = 50;
  // 静止点为几帧之和
  int static_buffer_size_ = 5;
  // 体素滤波分辨率
  float voxel_filter_size_ = 0.1;
  // 载体点云的范围
  float self_x_front_ = 2.5, self_x_back_ = -1.5, self_y_left_ = 1.6,
        self_y_right_ = -1.6;
  // 激光雷达每帧的时长
  double lidar_scan_period_ = 0.1;
  // 激光雷达的扫描范围
  float lidar_fov_up_ = 2.0, lidar_fov_down_ = -23, lidar_fov_left_ = 180,
        lidar_fov_right_ = -180;
  // 激光雷达盲区
  float lidar_blind_distance_ = 0.3;
  // 激光雷达每一帧点的数量
  int points_num_perframe_ = 200000;
  // 输出点云坐标系
  std::string frame_id_ = "camera_init";
  // 时间日志名
  std::string time_file_;
  // 是否开启debug模式
  bool debug_on_ = false;

  // Case1深度判断阈值
  float case1_enter_threshold_min_ = 2.0, case1_enter_threshold_max_ = 0.5;
  // Case1地图连续性判断阈值
  float case1_depth_consistency_threshold_ = 0.5f,
        case1_hor_consistency_threshold_ = 0.02f,
        case1_ver_consistency_threshold_ = 0.01f;
  // Case1地图连续性搜索范围因子
  float case1_hor_consist_factor_ = 0.2, case1_ver_consist_factor_ = 0.2;

  // Case1判定成功阈值
  int case1_success_threshold_ = 3;
  // Case1地图连续性判定是否开启插值
  bool case1_interp_on_ = false;
  bool case3_interp_on_ = false;
  // Case1插值阈值
  float case1_interp_threshold_ = 1.0f;
  // Case1插值进行线性变换的阈值
  float case1_interp_start_depth_ = 30;
  // Case1搜索像素深度最值的范围
  int neighbor_pixel_range_ = 1;

  // Case2速度阈值
  float case2_velocity_threshold_ = 0.5;
  // Case2加速度阈值
  float case2_acc_threshold_ = 1.0;
  // Case2地图连续性判断阈值
  float case2_depth_consistency_threshold_ = 0.15f,
        case2_hor_consistency_threshold_ = 0.02f,
        case2_ver_consistency_threshold_ = 0.01f;
  // Case2地图连续性判定是否开启插值
  bool case2_interp_on_ = false;
  // Case2遮挡判定阈值
  float case2_depth_occlusion_threshold_ = 0.15f,
        case2_hor_occlusion_threshold_ = 0.02f,
        case2_ver_occlusion_threshold_ = 0.01f;
  // Case2深度连续性判定阈值
  float case2_depth_depth_consistency_threshold_ = 0.15f,
        case2_depth_depth_consistency_threshold_max_ = 0.15f,
        case2_hor_depth_consistency_threshold_ = 0.02f,
        case2_ver_depth_consistency_threshold_ = 0.01f;
  // Case2深度连续性判定深度阈值参数
  float case2_k_depth_ = 0.005;
  // Case2判定成功阈值
  int case2_occluded_times_threshold_ = 3;
  // Case2插值阈值
  float case2_interp_threshold_ = 1.0f;

  // Case3速度阈值
  float case3_velocity_threshold_ = 0.5;
  // Case3加速度阈值
  float case3_acc_threshold_ = 1.0;
  // Case3判定成功阈值
  int case3_occlusion_times_threshold_ = 3;
  // Case3遮挡判定阈值
  float case3_depth_occlusion_threshold_ = 0.15f,
        case3_hor_occlusion_threshold_ = 0.02f,
        case3_ver_occlusion_threshold_ = 0.01f;
  // Case3地图连续性判断阈值
  float case3_depth_consistency_threshold_ = 0.15f,
        case3_hor_consistency_threshold_ = 0.02f,
        case3_ver_consistency_threshold_ = 0.01f;
  // Case3深度连续性判定阈值
  float case3_depth_depth_consistency_threshold_ = 0.15f,
        case3_depth_depth_consistency_threshold_max_ = 0.15f,
        case3_hor_depth_consistency_threshold_ = 0.02f,
        case3_ver_depth_consistency_threshold_ = 0.01f;
  // Case3深度连续性判定深度阈值参数
  float case3_k_depth_ = 0.005;
  // Case3插值阈值
  float case3_interp_threshold_ = 1.0f;

  // 插值搜索范围阈值
  float interp_hor_threshold_ = 0.01f, interp_ver_threshold_ = 0.01f;
  // 插值线性变换系数
  float interp_kp_ = 0.1, interp_kd_ = 1.0;

  // 聚类开关
  bool cluster_coupled_ = false, cluster_future_ = false;

  //变量
  //深度像素下标二维投影至一维参数
  int hor_num_ = MAX_1D, ver_num_ = MAX_1D_HALF;
  // 插值搜索范围
  int interp_hor_num_ = 0, interp_ver_num_ = 0;
  // Case1地图连续性搜索范围
  int map_cons_hor_num1_ = 0, map_cons_ver_num1_ = 0;
  // Case2地图连续性搜索范围
  int map_cons_hor_num2_ = 0, map_cons_ver_num2_ = 0;
  // Case3地图连续性搜索范围
  int map_cons_hor_num3_ = 0, map_cons_ver_num3_ = 0;
  // Case2遮挡搜索范围
  int occ_hor_num2_ = 0, occ_ver_num2_ = 0;
  // Case3遮挡搜索范围
  int occ_hor_num3_ = 0, occ_ver_num3_ = 0;
  // Case2深度连续性搜索范围
  int depth_cons_hor_num2_ = 0, depth_cons_ver_num2_ = 0;
  // Case3深度连续性搜索范围
  int depth_cons_hor_num3_ = 0, depth_cons_ver_num3_ = 0;
  // kitti数据集放大参数
  int enlarge_distort_ = 4;
  // 历史深度地图
  std::deque<DepthMap::Ptr> depth_map_list_;
  // 构建深度地图的点缓存
  PARALLEL_Q<point_soph *> points_map_buffer_;
  // 保存当前帧球面点的缓存
  std::vector<point_soph *> point_soph_pointers_;
  // 当前帧下标
  int cur_point_soph_pointers_ = 0;
  // 每个激光帧预分配内存大小
  int max_pointers_num_ = 0;
  // 已处理的帧总数
  int frame_num_for_rec_ = 0;
  // 历史点云缓存
  std::deque<PointCloudXYZI::Ptr> history_pointcloud_list_;
  // 静止点云
  PointCloudXYZI::Ptr static_point_cloud_;
  // 历史静止点云
  PointCloudXYZI::Ptr history_static_point_cloud_;
  // 聚类后的静态点云
  PointCloudXYZI::Ptr static_point_cloud_cluster_;
  // 静态点云缓存
  std::deque<PointCloudXYZI::Ptr> static_point_cloud_buffer_;
  // 获得静态点云的次数
  int static_point_cloud_times_ = 0;
  // 动态点云
  PointCloudXYZI::Ptr dynamic_point_cloud_;
  // 世界系下的动态点云
  PointCloudXYZI::Ptr dynamic_point_cloud_world_;
  // 聚类后的动态点云
  PointCloudXYZI::Ptr dynamic_point_cloud_cluster_;
  // 聚类器
  DynObjCluster cluster_;
  // 聚类前点的状态判断
  std::vector<int> status_tag_origin_;
  // 聚类后点的状态判断
  std::vector<int> status_tag_cluster_;
  // 当前深度地图下标
  int map_index_ = 0;
  // Case Enter计数
  int case1_num_ = 0, case2_num_ = 0, case3_num_ = 0;
  // 时长记录
  double time_total_ = 0.0, time_total_average_ = 0.0;
  // 时长计算计数
  int timer_index_ = 0;
  // 激光扫描范围在深度图像上的映射
  int pixel_fov_up_, pixel_fov_down_, pixel_fov_left_, pixel_fov_right_;
  // 下标偏置
  std::vector<int> pos_offset_;
  //时间日志流
  std::ofstream time_out_;
  // 是否设置日志路径
  bool is_set_path = false;
  // 聚类后日志文件名
  std::string out_file_;
  // 聚类前日志文件名
  std::string out_file_origin_;
};

#endif
