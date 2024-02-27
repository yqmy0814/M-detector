#include <iostream>
#include <random>
#include <vector>
// #include <algorithm>
// #include <chrono>
// #include <execution>

#include "m-detector/DynObjFilter.h"

#define PI_MATH (3.14159f)

void DynObjFilter::init(ros::NodeHandle &nh) {
  nh.param<double>("dyn_obj/map_buffer_delay", map_buffer_delay_, 0.1);
  nh.param<int>("dyn_obj/buffer_size", buffer_size_, 300000);
  nh.param<int>("dyn_obj/points_num_perframe", points_num_perframe_, 150000);
  nh.param<double>("dyn_obj/depth_map_duration", depth_map_duration_, 0.2);
  nh.param<int>("dyn_obj/max_depth_map_num", max_depth_map_num_, 5);
  nh.param<int>("dyn_obj/max_pixel_points", max_pixel_points_, 50);
  nh.param<double>("dyn_obj/lidar_scan_period", lidar_scan_period_, 0.1);
  nh.param<int>("dyn_obj/dataset", dataset_, 0);
  nh.param<float>("dyn_obj/self_x_front", self_x_front_, 0.15f);
  nh.param<float>("dyn_obj/self_x_back", self_x_back_, 0.15f);
  nh.param<float>("dyn_obj/self_y_left", self_y_left_, 0.15f);
  nh.param<float>("dyn_obj/self_y_right", self_y_right_, 0.5f);
  nh.param<float>("dyn_obj/lidar_blind_distance", lidar_blind_distance_, 0.15f);
  nh.param<float>("dyn_obj/lidar_fov_up", lidar_fov_up_, 0.15f);
  nh.param<float>("dyn_obj/lidar_fov_down", lidar_fov_down_, 0.15f);
  nh.param<float>("dyn_obj/lidar_fov_left", lidar_fov_left_, 180.0f);
  nh.param<float>("dyn_obj/lidar_fov_right", lidar_fov_right_, -180.0f);
  nh.param<float>("dyn_obj/horizontal_resolution", horizontal_resolution_,
                  0.0025f);
  nh.param<float>("dyn_obj/vertical_resolution", vertical_resolution_, 0.0025f);
  nh.param<std::string>("dyn_obj/frame_id", frame_id_, "camera_init");
  nh.param<std::string>("dyn_obj/time_file", time_file_, "");

  // Case 1相关参数
  nh.param<float>("dyn_obj/case1_enter_threshold_min",
                  case1_enter_threshold_min_, 0.15f);
  nh.param<float>("dyn_obj/case1_enter_threshold_max",
                  case1_enter_threshold_max_, 0.15f);
  nh.param<float>("dyn_obj/case1_depth_consistency_threshold",
                  case1_depth_consistency_threshold_, 0.5f);
  nh.param<float>("dyn_obj/case1_hor_consistency_threshold",
                  case1_hor_consistency_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case1_ver_consistency_threshold",
                  case1_ver_consistency_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case1_hor_consist_factor", case1_hor_consist_factor_,
                  0.2f);
  nh.param<float>("dyn_obj/case1_ver_consist_factor", case1_ver_consist_factor_,
                  0.1f);
  nh.param<int>("dyn_obj/case1_success_threshold", case1_success_threshold_, 3);
  nh.param<bool>("dyn_obj/case1_interp_on", case1_interp_on_, false);
  nh.param<int>("dyn_obj/neighbor_pixel_range", neighbor_pixel_range_, 1);

  // Case 2相关参数
  nh.param<float>("dyn_obj/case2_velocity_threshold", case2_velocity_threshold_,
                  0.5f);
  nh.param<float>("dyn_obj/case2_acc_threshold", case2_acc_threshold_, 1.0f);
  nh.param<float>("dyn_obj/case2_depth_consistency_threshold",
                  case2_depth_consistency_threshold_, 0.15f);
  nh.param<float>("dyn_obj/case2_hor_consistency_threshold",
                  case2_hor_consistency_threshold_, 0.02f);
  nh.param<float>("dyn_obj/case2_ver_consistency_threshold",
                  case2_ver_consistency_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case2_depth_occlusion_threshold",
                  case2_depth_occlusion_threshold_, 0.15f);
  nh.param<float>("dyn_obj/case2_hor_occlusion_threshold",
                  case2_hor_occlusion_threshold_, 0.02f);
  nh.param<float>("dyn_obj/case2_ver_occlusion_threshold",
                  case2_ver_occlusion_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case2_depth_depth_consistency_threshold",
                  case2_depth_depth_consistency_threshold_, 0.5f);
  nh.param<float>("dyn_obj/case2_depth_depth_consist_threshold_max",
                  case2_depth_depth_consistency_threshold_max_, 0.5f);
  nh.param<float>("dyn_obj/case2_hor_depth_consist_threshold",
                  case2_hor_depth_consistency_threshold_, 0.02f);
  nh.param<float>("dyn_obj/case2_ver_depth_consist_threshold",
                  case2_ver_depth_consistency_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case2_k_depth", case2_k_depth_, 0.005f);
  nh.param<int>("dyn_obj/case2_occluded_times_threshold",
                case2_occluded_times_threshold_, 3);
  nh.param<bool>("dyn_obj/case2_interp_on", case2_interp_on_, false);

  // Case 3相关参数
  nh.param<float>("dyn_obj/case3_velocity_threshold", case3_velocity_threshold_,
                  0.5f);
  nh.param<float>("dyn_obj/case3_acc_threshold", case3_acc_threshold_, 1.0f);
  nh.param<float>("dyn_obj/case3_depth_consistency_threshold",
                  case3_depth_consistency_threshold_, 0.15f);
  nh.param<float>("dyn_obj/case3_hor_consistency_threshold",
                  case3_hor_consistency_threshold_, 0.02f);
  nh.param<float>("dyn_obj/case3_ver_consistency_threshold",
                  case3_ver_consistency_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case3_depth_occlusion_threshold",
                  case3_depth_occlusion_threshold_, 0.15f);
  nh.param<float>("dyn_obj/case3_hor_occlusion_threshold",
                  case3_hor_occlusion_threshold_, 0.02f);
  nh.param<float>("dyn_obj/case3_ver_occlusion_threshold",
                  case3_ver_occlusion_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case3_depth_depth_consistency_threshold",
                  case3_depth_depth_consistency_threshold_, 0.5f);
  nh.param<float>("dyn_obj/case3_depth_depth_consistency_threshold_max",
                  case3_depth_depth_consistency_threshold_max_, 0.5f);
  nh.param<float>("dyn_obj/case3_hor_depth_consistency_threshold",
                  case3_hor_depth_consistency_threshold_, 0.02f);
  nh.param<float>("dyn_obj/case3_ver_depth_consistency_threshold",
                  case3_ver_depth_consistency_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case3_k_depth", case3_k_depth_, 0.005f);
  nh.param<int>("dyn_obj/case3_occlusion_times_threshold",
                case3_occlusion_times_threshold_, 3);
  nh.param<bool>("dyn_obj/case3_interp_on", case3_interp_on_, false);

  // 插值参数
  nh.param<float>("dyn_obj/interp_hor_threshold", interp_hor_threshold_, 0.01f);
  nh.param<float>("dyn_obj/interp_ver_threshold", interp_ver_threshold_, 0.01f);
  nh.param<float>("dyn_obj/case1_interp_threshold", case1_interp_threshold_,
                  1.0f);
  nh.param<float>("dyn_obj/case1_interp_start_depth", case1_interp_start_depth_,
                  20.0f);
  nh.param<float>("dyn_obj/interp_kp", interp_kp_, 0.1f);
  nh.param<float>("dyn_obj/interp_kd", interp_kd_, 1.0f);
  nh.param<float>("dyn_obj/case2_interp_threshold", case2_interp_threshold_,
                  0.15f);
  nh.param<float>("dyn_obj/case3_interp_threshold", case3_interp_threshold_,
                  0.15f);
  nh.param<bool>("dyn_obj/debug_on", debug_on_, true);
  nh.param<int>("dyn_obj/static_buffer_size", static_buffer_size_, 5);
  nh.param<float>("dyn_obj/voxel_filter_size", voxel_filter_size_, 0.1f);

  // 聚类参数
  nh.param<bool>("dyn_obj/cluster_coupled", cluster_coupled_, false);
  nh.param<bool>("dyn_obj/cluster_future", cluster_future_, false);
  nh.param<int>("dyn_obj/cluster_extend_pixel", cluster_.cluster_extend_pixel,
                2);
  nh.param<int>("dyn_obj/cluster_min_pixel_number",
                cluster_.cluster_min_pixel_number, 4);
  nh.param<float>("dyn_obj/cluster_thrustable_thresold",
                  cluster_.thrustable_thresold, 0.3f);
  nh.param<float>("dyn_obj/cluster_Voxel_revolusion", cluster_.Voxel_revolusion,
                  0.3f);
  nh.param<bool>("dyn_obj/cluster_debug_en", cluster_.debug_on, false);
  nh.param<std::string>("dyn_obj/cluster_out_file", cluster_.out_file, "");

  if (history_pointcloud_list_.size() == 0) {
    PointCloudXYZI::Ptr first_frame(new PointCloudXYZI());
    first_frame->reserve(400000);
    history_pointcloud_list_.push_back(first_frame);
    history_static_point_cloud_ = PointCloudXYZI::Ptr(new PointCloudXYZI());
    static_point_cloud_ = PointCloudXYZI::Ptr(new PointCloudXYZI());
    dynamic_point_cloud_ = PointCloudXYZI::Ptr(new PointCloudXYZI());
    dynamic_point_cloud_world_ = PointCloudXYZI::Ptr(new PointCloudXYZI());
    int xy_ind[3] = {-1, 1};
    for (int ind_hor = 0; ind_hor < 2 * hor_num_ + 1; ind_hor++) {
      for (int ind_ver = 0; ind_ver < 2 * ver_num_ + 1; ind_ver++) {
        pos_offset_.push_back(
            ((ind_hor) / 2 + ind_hor % 2) * xy_ind[ind_hor % 2] * MAX_1D_HALF +
            ((ind_ver) / 2 + ind_ver % 2) * xy_ind[ind_ver % 2]);
      }
    }
  }
  map_cons_hor_num1_ =
      ceil(case1_hor_consistency_threshold_ / horizontal_resolution_);
  map_cons_ver_num1_ =
      ceil(case1_ver_consistency_threshold_ / vertical_resolution_);
  interp_hor_num_ = ceil(interp_hor_threshold_ / horizontal_resolution_);
  interp_ver_num_ = ceil(interp_ver_threshold_ / vertical_resolution_);
  map_cons_hor_num2_ =
      ceil(case2_hor_consistency_threshold_ / horizontal_resolution_);
  map_cons_ver_num2_ =
      ceil(case2_ver_consistency_threshold_ / vertical_resolution_);
  occ_hor_num2_ = ceil(case2_hor_occlusion_threshold_ / horizontal_resolution_);
  occ_ver_num2_ = ceil(case2_ver_occlusion_threshold_ / vertical_resolution_);
  depth_cons_hor_num2_ =
      ceil(case2_hor_depth_consistency_threshold_ / horizontal_resolution_);
  depth_cons_ver_num2_ =
      ceil(case2_ver_depth_consistency_threshold_ / vertical_resolution_);
  map_cons_hor_num3_ =
      ceil(case3_hor_consistency_threshold_ / horizontal_resolution_);
  map_cons_ver_num3_ =
      ceil(case3_ver_consistency_threshold_ / vertical_resolution_);
  occ_hor_num3_ = ceil(case3_hor_occlusion_threshold_ / horizontal_resolution_);
  occ_ver_num3_ = ceil(case3_ver_occlusion_threshold_ / vertical_resolution_);
  depth_cons_hor_num3_ =
      ceil(case3_hor_depth_consistency_threshold_ / horizontal_resolution_);
  depth_cons_ver_num3_ =
      ceil(case3_ver_depth_consistency_threshold_ / vertical_resolution_);
  points_map_buffer_.init(buffer_size_);

  pixel_fov_up_ = floor((lidar_fov_up_ / 180.0 * PI_MATH + 0.5 * PI_MATH) /
                        vertical_resolution_);
  pixel_fov_down_ = floor((lidar_fov_down_ / 180.0 * PI_MATH + 0.5 * PI_MATH) /
                          vertical_resolution_);
  pixel_fov_left_ = floor((lidar_fov_left_ / 180.0 * PI_MATH + PI_MATH) /
                          horizontal_resolution_);
  pixel_fov_right_ = floor((lidar_fov_right_ / 180.0 * PI_MATH + PI_MATH) /
                           horizontal_resolution_);
  max_pointers_num_ =
      round((max_depth_map_num_ * depth_map_duration_ + map_buffer_delay_) /
            lidar_scan_period_) +
      1;
  point_soph_pointers_.reserve(max_pointers_num_);
  for (int i = 0; i < max_pointers_num_; i++) {
    point_soph *p = new point_soph[points_num_perframe_];
    point_soph_pointers_.push_back(p);
  }
  if (time_file_ != "") {
    time_out_.open(time_file_, std::ios::out);
  }
  cluster_.Init();
}

void DynObjFilter::filter(PointCloudXYZI::Ptr feats_undistort,
                          const M3D &rot_end, const V3D &pos_end,
                          const double &scan_end_time) {
  double t00 = omp_get_wtime();
  time_total_ = 0.0;

  int num_build = 0, num_search_0 = 0, num_research = 0;
  if (feats_undistort == NULL) return;
  int size = feats_undistort->points.size();
  if (debug_on_) {
    history_static_point_cloud_.reset(new PointCloudXYZI());
    history_static_point_cloud_->reserve(20 * size);
  }
  status_tag_origin_.clear();
  status_tag_origin_.reserve(size);
  status_tag_origin_.resize(size);
  status_tag_cluster_.clear();
  status_tag_cluster_.reserve(size);
  status_tag_cluster_.resize(size);
  dynamic_point_cloud_.reset(new PointCloudXYZI());
  dynamic_point_cloud_->reserve(size);
  dynamic_point_cloud_world_.reset(new PointCloudXYZI());
  dynamic_point_cloud_world_->reserve(size);
  static_point_cloud_.reset(new PointCloudXYZI());
  static_point_cloud_->reserve(size);
  dynamic_point_cloud_cluster_.reset(new PointCloudXYZI());
  dynamic_point_cloud_cluster_->reserve(size);
  static_point_cloud_cluster_.reset(new PointCloudXYZI());
  static_point_cloud_cluster_->reserve(size);
  std::ofstream out;
  std::ofstream out_origin;
  bool is_rec = false;
  bool is_rec_origin = false;
  if (is_set_path) {
    out.open(out_file_, std::ios::out | std::ios::binary);
    out_origin.open(out_file_origin_, std::ios::out | std::ios::binary);
    if (out.is_open()) {
      is_rec = true;
    }
    if (out_origin.is_open()) {
      is_rec_origin = true;
    }
  }

  double t0 = omp_get_wtime();
  double time_case1 = 0, time_case2 = 0, time_case3 = 0;
  pcl::PointCloud<PointType> raw_points_world;
  raw_points_world.reserve(size);
  raw_points_world.resize(size);
  std::vector<int> index(size);
  for (int i = 0; i < size; i++) {
    index[i] = i;
  }
  std::vector<point_soph *> points;
  points.reserve(size);
  points.resize(size);
  point_soph *p_soph = point_soph_pointers_[cur_point_soph_pointers_];
  if (time_file_ != "") time_out_ << size << " ";  // rec computation time
  // std::for_each(std::execution::seq, index.begin(),
  // index.end(), [&](const int &i)
  std::for_each(
      std::execution::par, index.begin(), index.end(), [&](const int &i) {
        p_soph[i].reset();
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y,
                   feats_undistort->points[i].z);
        V3D p_global(rot_end * (p_body) + pos_end);
        p_soph[i].global = p_global;
        p_soph[i].status = STATIC;
        p_soph[i].rot = rot_end.transpose();
        p_soph[i].trans = pos_end;
        p_soph[i].time = scan_end_time;
        p_soph[i].local = p_body;
        p_soph[i].intensity = feats_undistort->points[i].intensity;
        if (dataset_ == 0 && fabs(p_soph[i].intensity - 666) < 10E-4) {
          p_soph[i].is_distort = true;
        }
        if (InvalidPointCheck(p_body, p_soph[i].intensity)) {
          p_soph[i].status = INVALID;
          status_tag_origin_[i] = 0;
          status_tag_cluster_[i] = -1;
        } else if (SelfPointCheck(p_body, p_soph[i].status)) {
          p_soph[i].status = INVALID;
          status_tag_origin_[i] = 0;
        } else if (Case1(p_soph[i])) {
          p_soph[i].status = CASE1;
          status_tag_origin_[i] = 1;
        } else if (Case2(p_soph[i])) {
          p_soph[i].status = CASE2;
          status_tag_origin_[i] = 1;
        } else if (Case3(p_soph[i])) {
          p_soph[i].status = CASE3;
          status_tag_origin_[i] = 1;
        } else {
          status_tag_origin_[i] = 0;
        }
        points[i] = &p_soph[i];
      });

  if (time_file_ != "")
    time_out_ << omp_get_wtime() - t0 << " ";  // rec computation time

  for (int i = 0; i < size; i++) {
    PointType po;
    po.x = points[i]->local[0];
    po.y = points[i]->local[1];
    po.z = points[i]->local[2];
    po.intensity = points[i]->intensity;
    PointType po_w;
    po_w.x = points[i]->global[0];
    po_w.y = points[i]->global[1];
    po_w.z = points[i]->global[2];
    raw_points_world[i] = po;
    switch (points[i]->status) {
      case CASE1:
        dynamic_point_cloud_->push_back(po);
        dynamic_point_cloud_world_->push_back(po_w);
        break;
      case CASE2:
        dynamic_point_cloud_->push_back(po);
        dynamic_point_cloud_world_->push_back(po_w);
        break;
      case CASE3:
        dynamic_point_cloud_->push_back(po);
        dynamic_point_cloud_world_->push_back(po_w);
        break;
      default:
        static_point_cloud_->push_back(po_w);
    }
  }
  int num_1 = 0, num_2 = 0, num_3 = 0, num_inval = 0, num_neag = 0;
  double clus_before = omp_get_wtime();  // rec computation time
  std_msgs::Header header_clus;
  header_clus.stamp = ros::Time().fromSec(scan_end_time);
  header_clus.frame_id = frame_id_;

  if (cluster_coupled_ || cluster_future_) {
    cluster_.Clusterprocess(status_tag_cluster_, *dynamic_point_cloud_,
                            raw_points_world, header_clus, rot_end, pos_end);
    for (int i = 0; i < size; i++) {
      PointType po;
      po.x = points[i]->global(0);
      po.y = points[i]->global(1);
      po.z = points[i]->global(2);
      switch (points[i]->status) {
        case CASE1:
          if (status_tag_cluster_[i] == 0) {
            points[i]->status = STATIC;
            points[i]->occu_times = -1;
            points[i]->is_occu_times = -1;
            po.intensity = (int)(points[i]->local.norm() * 10) + 10;
            static_point_cloud_cluster_->push_back(po);
            num_neag += 1;
          } else  // case1
          {
            dynamic_point_cloud_cluster_->push_back(po);

            po.intensity = (int)(points[i]->local.norm() * 10) + 10;

            num_1 += 1;
          }
          break;
        case CASE2:
          if (status_tag_cluster_[i] == 0) {
            points[i]->status = STATIC;
            points[i]->occu_times = -1;
            points[i]->is_occu_times = -1;
            po.intensity = (int)(points[i]->local.norm() * 10) + 10;
            static_point_cloud_cluster_->push_back(po);
            num_neag += 1;
          } else {
            dynamic_point_cloud_cluster_->push_back(po);

            po.intensity = (int)(points[i]->local.norm() * 10) + 10;

            num_2 += 1;
          }
          break;
        case CASE3:
          if (status_tag_cluster_[i] == 0) {
            points[i]->status = STATIC;
            points[i]->occu_times = -1;
            points[i]->is_occu_times = -1;
            po.intensity = (int)(points[i]->local.norm() * 10) + 10;
            static_point_cloud_cluster_->push_back(po);
            num_neag += 1;
          } else {
            dynamic_point_cloud_cluster_->push_back(po);

            po.intensity = (int)(points[i]->local.norm() * 10) + 10;

            num_3 += 1;
          }
          break;
        case STATIC:
          if (status_tag_cluster_[i] == 1) {
            points[i]->status = CASE1;
            points[i]->occu_times = -1;
            points[i]->is_occu_times = -1;
            dynamic_point_cloud_cluster_->push_back(po);

            po.intensity = (int)(points[i]->local.norm() * 10) + 10;

            num_1 += 1;
          } else {
            po.intensity = (int)(points[i]->local.norm() * 10) + 10;
            static_point_cloud_cluster_->push_back(po);
            num_neag += 1;
          }
          break;
        default:  // invalid
          num_inval += 1;
          break;
      }
    }
  }

  if (time_file_ != "")
    time_out_ << omp_get_wtime() - clus_before << " ";  // rec computation time
  double t3 = omp_get_wtime();

  Points2Buffer(points, index);

  double t4 = omp_get_wtime();
  if (time_file_ != "")
    time_out_ << omp_get_wtime() - t3 << " ";  // rec computation time

  Buffer2DepthMap(scan_end_time);

  if (time_file_ != "")
    time_out_ << omp_get_wtime() - t3 << std::endl;  // rec computation time

  if (cluster_coupled_) {
    for (int i = 0; i < size; i++) {
      if (status_tag_cluster_[i] == 1) {
        if (is_rec) {
          int tmp = 251;
          out.write((char *)&tmp, sizeof(int));
        }
      } else {
        if (is_rec) {
          int tmp = 9;
          out.write((char *)&tmp, sizeof(int));
        }
      }

      if (status_tag_origin_[i] == 1 || status_tag_origin_[i] == 2) {
        if (is_rec_origin) {
          int tmp = 251;
          out_origin.write((char *)&tmp, sizeof(int));
        }
      } else {
        if (is_rec_origin) {
          int tmp = 9;
          out_origin.write((char *)&tmp, sizeof(int));
        }
      }
    }
  } else {
    for (int i = 0; i < size; i++) {
      if (status_tag_origin_[i] == 1 || status_tag_origin_[i] == 2) {
        if (is_rec) {
          int tmp = 251;
          out.write((char *)&tmp, sizeof(int));
        }
      } else {
        if (is_rec) {
          int tmp = 9;
          out.write((char *)&tmp, sizeof(int));
        }
      }
    }
  }

  frame_num_for_rec_++;
  cur_point_soph_pointers_ = (cur_point_soph_pointers_ + 1) % max_pointers_num_;
  if (is_rec) out.close();
  time_total_ = omp_get_wtime() - t00;
  timer_index_++;
  time_total_average_ =
      time_total_average_ * (timer_index_ - 1) / timer_index_ +
      time_total_ / timer_index_;
}

void DynObjFilter::Points2Buffer(std::vector<point_soph *> &points,
                                 std::vector<int> &index_vector) {
  int cur_tail = points_map_buffer_.tail;
  points_map_buffer_.push_parallel_prepare(points.size());
  std::for_each(std::execution::par, index_vector.begin(), index_vector.end(),
                [&](const int &i) {
                  points_map_buffer_.push_parallel(points[i], cur_tail + i);
                });
}

void DynObjFilter::Buffer2DepthMap(double cur_time) {
  int len = points_map_buffer_.size();
  double total_0 = 0.0;
  double total_1 = 0.0;
  double total_2 = 0.0;
  double total_3 = 0.0;
  double t = 0.0;
  int max_point = 0;
  for (int k = 0; k < len; k++) {
    point_soph *point = points_map_buffer_.front();
    if ((cur_time - point->time) >=
        map_buffer_delay_ - lidar_scan_period_ / 2.0) {
      if (depth_map_list_.size() == 0) {
        if (depth_map_list_.size() < max_depth_map_num_) {
          map_index_++;
          DepthMap::Ptr new_map_pointer(
              new DepthMap(point->rot, point->trans, point->time, map_index_));
          depth_map_list_.push_back(new_map_pointer);
        } else {
          points_map_buffer_.pop();
          continue;
        }
      } else if ((point->time - depth_map_list_.back()->time) >=
                 depth_map_duration_ - lidar_scan_period_ / 2.0) {
        map_index_++;
        if (depth_map_list_.size() == max_depth_map_num_) {
          depth_map_list_.front()->Reset(point->rot, point->trans, point->time,
                                         map_index_);
          DepthMap::Ptr new_map_pointer = depth_map_list_.front();
          depth_map_list_.pop_front();
          depth_map_list_.push_back(new_map_pointer);
        } else if (depth_map_list_.size() < max_depth_map_num_) {
          DepthMap::Ptr new_map_pointer(
              new DepthMap(point->rot, point->trans, point->time, map_index_));
          depth_map_list_.push_back(new_map_pointer);
        }
      }

      switch (point->status) {
        if (depth_map_list_.back()->depth_map.size() <= point->position)
        case STATIC:
          SphericalProjection(*point, depth_map_list_.back()->map_index,
                              depth_map_list_.back()->project_R,
                              depth_map_list_.back()->project_T, *point);
        if (depth_map_list_.back()->depth_map[point->position].size() <
            max_pixel_points_) {
          depth_map_list_.back()->depth_map[point->position].push_back(point);
          if (point->vec(2) >
              depth_map_list_.back()->max_depth_all[point->position]) {
            depth_map_list_.back()->max_depth_all[point->position] =
                point->vec(2);
            depth_map_list_.back()->max_depth_index_all[point->position] =
                depth_map_list_.back()->depth_map[point->position].size() - 1;
          }
          if (point->vec(2) <
                  depth_map_list_.back()->min_depth_all[point->position] ||
              depth_map_list_.back()->min_depth_all[point->position] < 10E-5) {
            depth_map_list_.back()->min_depth_all[point->position] =
                point->vec(2);
            depth_map_list_.back()->min_depth_index_all[point->position] =
                depth_map_list_.back()->depth_map[point->position].size() - 1;
          }
          if (point->vec(2) <
                  depth_map_list_.back()->min_depth_static[point->position] ||
              depth_map_list_.back()->min_depth_static[point->position] <
                  10E-5) {
            depth_map_list_.back()->min_depth_static[point->position] =
                point->vec(2);
          }
          if (point->vec(2) >
              depth_map_list_.back()->max_depth_static[point->position]) {
            depth_map_list_.back()->max_depth_static[point->position] =
                point->vec(2);
          }
        }
        break;
        case CASE1:

        case CASE2:

        case CASE3:
          SphericalProjection(*point, depth_map_list_.back()->map_index,
                              depth_map_list_.back()->project_R,
                              depth_map_list_.back()->project_T, *point);
          if (depth_map_list_.back()->depth_map[point->position].size() <
              max_pixel_points_) {
            depth_map_list_.back()->depth_map[point->position].push_back(point);
            if (point->vec(2) >
                depth_map_list_.back()->max_depth_all[point->position]) {
              depth_map_list_.back()->max_depth_all[point->position] =
                  point->vec(2);
              depth_map_list_.back()->max_depth_index_all[point->position] =
                  depth_map_list_.back()->depth_map[point->position].size() - 1;
            }
            if (point->vec(2) <
                    depth_map_list_.back()->min_depth_all[point->position] ||
                depth_map_list_.back()->min_depth_all[point->position] <
                    10E-5) {
              depth_map_list_.back()->min_depth_all[point->position] =
                  point->vec(2);
              depth_map_list_.back()->min_depth_index_all[point->position] =
                  depth_map_list_.back()->depth_map[point->position].size() - 1;
            }
          }
          break;
        default:
          break;
      }
      points_map_buffer_.pop();
    } else {
      break;
    }
  }
  if (debug_on_) {
    for (int i = 0; i < depth_map_list_.size(); i++) {
      for (int j = 0; j < depth_map_list_[i]->depth_map.size(); j++) {
        for (int k = 0; k < depth_map_list_[i]->depth_map[j].size(); k++) {
          PointType po;
          point_soph *point = depth_map_list_[i]->depth_map[j][k];
          po.x = point->global(0);
          po.y = point->global(1);
          po.z = point->global(2);
          po.intensity = point->local(2);
          if (point->status == STATIC)
            history_static_point_cloud_->push_back(po);
        }
      }
    }
  }
}

void DynObjFilter::SphericalProjection(point_soph &p, int depth_map_index,
                                       const M3D &rot, const V3D &transl,
                                       point_soph &p_spherical) {
  if (fabs(p.last_vecs.at(depth_map_index % HASH_PRIM)[2]) > 10E-5) {
    p_spherical.vec = p.last_vecs.at(depth_map_index % HASH_PRIM);
    p_spherical.hor_ind = p.last_positions.at(depth_map_index % HASH_PRIM)[0];
    p_spherical.ver_ind = p.last_positions.at(depth_map_index % HASH_PRIM)[1];
    p_spherical.position = p.last_positions.at(depth_map_index % HASH_PRIM)[2];
  } else {
    V3D p_proj(rot * (p.global - transl));
    p_spherical.GetVec(p_proj, horizontal_resolution_, vertical_resolution_);
    p.last_vecs.at(depth_map_index % HASH_PRIM) = p_spherical.vec;
    p.last_positions.at(depth_map_index % HASH_PRIM)[0] = p_spherical.hor_ind;
    p.last_positions.at(depth_map_index % HASH_PRIM)[1] = p_spherical.ver_ind;
    p.last_positions.at(depth_map_index % HASH_PRIM)[2] = p_spherical.position;
  }
}

bool DynObjFilter::InvalidPointCheck(const V3D &body, const int intensity) {
  if ((pow(body(0), 2) + pow(body(1), 2) + pow(body(2), 2)) <
          lidar_blind_distance_ * lidar_blind_distance_ ||
      (dataset_ == 1 && fabs(body(0)) < 0.1 && fabs(body(1)) < 1.0) &&
          fabs(body(2)) < 0.1) {
    return true;
  } else {
    return false;
  }
}

bool DynObjFilter::SelfPointCheck(const V3D &body, const dyn_obj_flg dyn) {
  if (dataset_ == 0) {
    if ((body(0) > -1.2 && body(0) < -0.4 && body(1) > -1.7 && body(1) < -1.0 &&
         body(2) > -0.65 && body(2) < -0.4) ||
        (body(0) > -1.75 && body(0) < -0.85 && body(1) > 1.0 && body(1) < 1.6 &&
         body(2) > -0.75 && body(2) < -0.40) ||
        (body(0) > 1.4 && body(0) < 1.7 && body(1) > -1.3 && body(1) < -0.9 &&
         body(2) > -0.8 && body(2) < -0.6) ||
        (body(0) > 2.45 && body(0) < 2.6 && body(1) > -0.6 && body(1) < -0.45 &&
         body(2) > -1.0 && body(2) < -0.9) ||
        (body(0) > 2.45 && body(0) < 2.6 && body(1) > 0.45 && body(1) < 0.6 &&
         body(2) > -1.0 && body(2) < -0.9)) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool DynObjFilter::CheckVerFoV(const point_soph &p, const DepthMap &map_info) {
  bool ver_up = false, ver_down = false;
  for (int i = p.ver_ind; i >= pixel_fov_down_; i--) {
    int cur_pos = p.hor_ind * MAX_1D_HALF + i;
    if (map_info.depth_map[cur_pos].size() > 0) {
      ver_down = true;
      break;
    }
  }
  for (int i = p.ver_ind; i <= pixel_fov_up_; i++) {
    int cur_pos = p.hor_ind * MAX_1D_HALF + i;
    if (map_info.depth_map[cur_pos].size() > 0) {
      ver_up = true;
      break;
    }
  }
  if (ver_up && ver_down) {
    return false;
  } else {
    return true;
  }
}

void DynObjFilter::SearchNeighborPixel(const point_soph &p,
                                       const DepthMap &map_info,
                                       float &max_depth, float &min_depth) {
  int n = neighbor_pixel_range_;
  for (int i = -n; i <= n; i++) {
    for (int j = -n; j <= n; j++) {
      int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
      if (cur_pos < MAX_2D_N && cur_pos >= 0 &&
          map_info.depth_map[cur_pos].size() > 0) {
        float cur_max_depth = map_info.max_depth_static[cur_pos];
        float cur_min_depth = map_info.min_depth_static[cur_pos];
        if (min_depth > 10E-5)
          min_depth = std::min(cur_min_depth, min_depth);
        else
          min_depth = cur_min_depth;
        if (max_depth > 10E-5)
          max_depth = std::max(cur_max_depth, max_depth);
        else
          max_depth = cur_max_depth;
      }
    }
  }
}

bool DynObjFilter::Case1(point_soph &p) {
  int depth_map_num = depth_map_list_.size();
  int occluded_map = depth_map_num;
  for (int i = depth_map_num - 1; i >= 0; i--) {
    SphericalProjection(p, depth_map_list_[i]->map_index,
                        depth_map_list_[i]->project_R,
                        depth_map_list_[i]->project_T, p);
    if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF ||
        p.vec(2) < 0.0f || p.position < 0 || p.position >= MAX_2D_N) {
      p.status = INVALID;
      continue;
    }
    if (Case1Enter(p, *depth_map_list_[i])) {
      if (Case1FalseRejection(p, *depth_map_list_[i])) {
        occluded_map -= 1;
      }
    } else {
      occluded_map -= 1;
    }
    if (occluded_map < case1_success_threshold_) {
      return false;
    }
    if (occluded_map - (i) >= case1_success_threshold_) {
      return true;
    }
  }
  if (occluded_map >= case1_success_threshold_) {
    return true;
  }
  return false;
}

bool DynObjFilter::Case1Enter(const point_soph &p, const DepthMap &map_info) {
  float max_depth = 0, min_depth = 0;
  float max_depth_all = 0, min_depth_all = 0;
  if (map_info.depth_map[p.position].size() > 0) {
    max_depth = map_info.max_depth_static[p.position];
    min_depth = map_info.min_depth_static[p.position];
  } else {
    if (p.ver_ind <= pixel_fov_up_ && p.ver_ind > pixel_fov_down_ &&
        p.hor_ind <= pixel_fov_left_ && p.hor_ind >= pixel_fov_right_ &&
        CheckVerFoV(p, map_info)) {
      SearchNeighborPixel(p, map_info, max_depth, min_depth);
    }
  }
  float cur_min = case1_enter_threshold_min_;
  float cur_max = case1_enter_threshold_max_;
  if (dataset_ == 0 && p.is_distort) {
    cur_min = enlarge_distort_ * cur_min;
    cur_max = enlarge_distort_ * cur_max;
  }
  if (p.vec(2) < min_depth - cur_max ||
      (min_depth < p.vec(2) - cur_min && max_depth > p.vec(2) + cur_max)) {
    case1_num_++;
    return true;
  }
  return false;
}

bool DynObjFilter::Case1FalseRejection(point_soph &p,
                                       const DepthMap &map_info) {
  return Case1MapConsistencyCheck(p, map_info, case1_interp_on_);
}

bool DynObjFilter::Case1MapConsistencyCheck(point_soph &p,
                                            const DepthMap &map_info,
                                            bool interp) {
  float hor_half = std::max(
      case1_hor_consist_factor_ / (std::max(p.vec(2), lidar_blind_distance_)),
      case1_hor_consistency_threshold_);
  float ver_half = std::max(
      case1_ver_consist_factor_ / (std::max(p.vec(2), lidar_blind_distance_)),
      case1_ver_consistency_threshold_);
  float cur_map_cons_depth_thr1 = case1_depth_consistency_threshold_;
  float cur_map_cons_min_thr1 = case1_enter_threshold_min_;
  float cur_map_cons_max_thr1 = case1_enter_threshold_max_;
  if (dataset_ == 0 && p.is_distort) {
    cur_map_cons_depth_thr1 = enlarge_distort_ * cur_map_cons_depth_thr1;
    cur_map_cons_min_thr1 = enlarge_distort_ * cur_map_cons_min_thr1;
    cur_map_cons_max_thr1 = enlarge_distort_ * cur_map_cons_max_thr1;
  }
  int cur_map_cons_hor_num1 = ceil(hor_half / horizontal_resolution_);
  int cur_map_cons_ver_num1 = ceil(ver_half / vertical_resolution_);
  int num = 0;
  for (int ind_hor = -cur_map_cons_hor_num1; ind_hor <= cur_map_cons_hor_num1;
       ind_hor++) {
    for (int ind_ver = -cur_map_cons_ver_num1; ind_ver <= cur_map_cons_ver_num1;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      if (map_info.max_depth_static[pos_new] <
              p.vec(2) - cur_map_cons_min_thr1 ||
          map_info.min_depth_static[pos_new] >
              p.vec(2) + cur_map_cons_max_thr1) {
        continue;
      }
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (point->status == STATIC &&
            (fabs(p.vec(2) - point->vec(2)) < cur_map_cons_depth_thr1 ||
             ((p.vec(2) - point->vec(2)) > cur_map_cons_depth_thr1 &&
              (p.vec(2) - point->vec(2)) < cur_map_cons_min_thr1)) &&
            fabs(p.vec(0) - point->vec(0)) < hor_half &&
            fabs(p.vec(1) - point->vec(1)) < ver_half) {
          return true;
        }
      }
    }
  }
  if (interp && (p.local(0) < self_x_back_ || p.local(0) > self_x_front_ ||
                 p.local(1) > self_y_left_ || p.local(1) < self_y_right_)) {
    float depth_static =
        DepthInterpolationStatic(p, map_info.map_index, map_info.depth_map);
    float cur_interp = case1_interp_threshold_;
    if (p.vec(2) > case1_interp_start_depth_)
      cur_interp +=
          ((p.vec(2) - case1_interp_start_depth_) * interp_kp_ + interp_kd_);
    if (dataset_ == 0) {
      if (fabs(depth_static + 1) < 10E-5 || fabs(depth_static + 2) < 10E-5) {
        return false;
      } else {
        if (fabs(depth_static - p.vec(2)) < cur_interp) {
          return true;
        }
      }
    } else {
      if (fabs(depth_static + 1) < 10E-5 || fabs(depth_static + 2) < 10E-5) {
        return false;
      } else {
        if (fabs(depth_static - p.vec(2)) < cur_interp) {
          return true;
        }
      }
    }
  }
  return false;
}

float DynObjFilter::DepthInterpolationStatic(point_soph &p, int map_index,
                                             const DepthMap2D &depth_map) {
  if (fabs(p.last_depth_interps.at(
          map_index - depth_map_list_.front()->map_index)) > 10E-4) {
    float depth_cal =
        p.last_depth_interps.at(map_index - depth_map_list_.front()->map_index);
    return depth_cal;
  }
  V3F p_1 = V3F::Zero();
  V3F p_2 = V3F::Zero();
  V3F p_3 = V3F::Zero();
  std::vector<V3F> p_neighbors;
  int all_num = 0, static_num = 0;
  for (int ind_hor = -interp_hor_num_; ind_hor <= interp_hor_num_; ind_hor++) {
    for (int ind_ver = -interp_ver_num_; ind_ver <= interp_ver_num_;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel = depth_map[pos_new];
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (fabs(point->time - p.time) < lidar_scan_period_) {
          continue;
        }
        float hor_minus = point->vec(0) - p.vec(0);
        float ver_minus = point->vec(1) - p.vec(1);
        if (fabs(hor_minus) < interp_hor_threshold_ &&
            fabs(ver_minus) < interp_ver_threshold_) {
          all_num++;
          if (point->status == STATIC) {
            static_num++;
          }
          if (point->status == STATIC) {
            p_neighbors.push_back(point->vec);
            if (p_1(2) < 0.000001 ||
                fabs(hor_minus) + fabs(ver_minus) <
                    fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1))) {
              p_1 = point->vec;
            }
          }
        }
      }
    }
  }
  if (p_1(2) < 10E-5) {
    p.last_depth_interps.at(map_index - depth_map_list_.front()->map_index) =
        -1;
    return -1;
  }
  int cur_size = p_neighbors.size();
  for (int t_i = 0; t_i < cur_size - 2; t_i++) {
    p_1 = p_neighbors[t_i];
    p_2 = V3F::Zero();
    p_3 = V3F::Zero();
    float min_fabs = 2 * (interp_hor_threshold_ + interp_ver_threshold_);
    float x = p.vec(0) - p_1(0);
    float y = p.vec(1) - p_1(1);
    float alpha = 0, beta = 0;
    for (int i = t_i + 1; i < cur_size - 1; i++) {
      if (fabs(p_neighbors[i](0) - p.vec(0)) +
              fabs(p_neighbors[i](1) - p.vec(1)) <
          min_fabs) {
        p_2 = p_neighbors[i];
        float single_fabs = fabs(p_neighbors[i](0) - p.vec(0)) +
                            fabs(p_neighbors[i](1) - p.vec(1));
        if (single_fabs >= min_fabs) continue;
        for (int ii = i + 1; ii < cur_size; ii++) {
          float cur_fabs = fabs(p_neighbors[i](0) - p.vec(0)) +
                           fabs(p_neighbors[i](1) - p.vec(1)) +
                           fabs(p_neighbors[ii](0) - p.vec(0)) +
                           fabs(p_neighbors[ii](1) - p.vec(1));
          if (cur_fabs < min_fabs) {
            float x1 = p_neighbors[i](0) - p_1(0);
            float x2 = p_neighbors[ii](0) - p_1(0);
            float y1 = p_neighbors[i](1) - p_1(1);
            float y2 = p_neighbors[ii](1) - p_1(1);
            float lower = x1 * y2 - x2 * y1;
            if (fabs(lower) > 10E-5) {
              alpha = (x * y2 - y * x2) / lower;
              beta = -(x * y1 - y * x1) / lower;
              if (alpha > 0 && alpha < 1 && beta > 0 && beta < 1 &&
                  (alpha + beta) > 0 && (alpha + beta) < 1) {
                p_3 = p_neighbors[ii];
                min_fabs = cur_fabs;
              }
            }
          }
        }
      }
    }
    if (p_2(2) < 10E-5 || p_3(2) < 10E-5) {
      continue;
    }
    float depth_cal =
        (1 - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
    p.last_depth_interps.at(map_index - depth_map_list_.front()->map_index) =
        depth_cal;
    return depth_cal;
  }
  if (static_num > 0 && cur_size < all_num / 2) {
    p.last_depth_interps.at(map_index - depth_map_list_.front()->map_index) =
        -2;
    return -2;
  } else {
    p.last_depth_interps.at(map_index - depth_map_list_.front()->map_index) =
        -2;
    return -2;
  }
}  // return -1 denotes no point, -2 denotes no trianguolar but with points

bool DynObjFilter::Case2(point_soph &p) {
  if (dataset_ == 0 && p.is_distort) return false;
  int first_i = depth_map_list_.size();
  first_i -= 1;
  if (first_i < 0) return false;
  point_soph p_spherical = p;
  SphericalProjection(p, depth_map_list_[first_i]->map_index,
                      depth_map_list_[first_i]->project_R,
                      depth_map_list_[first_i]->project_T, p_spherical);
  if (fabs(p_spherical.hor_ind) >= MAX_1D ||
      fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f ||
      p_spherical.position < 0 || p_spherical.position >= MAX_2D_N) {
    p.status = INVALID;
    return false;
  }
  int cur_occ_times = 0;
  if (Case2Enter(p_spherical, *depth_map_list_[first_i])) {
    if (!Case2MapConsistencyCheck(p_spherical, *depth_map_list_[first_i],
                                  case2_interp_on_)) {
      double ti = 0;
      float vi = 0;
      float min_hor = case2_hor_occlusion_threshold_,
            min_ver = case2_ver_occlusion_threshold_;
      bool map_cons = true;
      for (int ind_hor = -occ_hor_num2_; ind_hor <= occ_hor_num2_; ind_hor++) {
        for (int ind_ver = -occ_ver_num2_; ind_ver <= occ_ver_num2_;
             ind_ver++) {
          int pos_new =
              ((p_spherical.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
              ((p_spherical.ver_ind + ind_ver) % MAX_1D_HALF);
          if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
          const std::vector<point_soph *> &points_in_pixel =
              depth_map_list_[first_i]->depth_map[pos_new];
          if (depth_map_list_[first_i]->min_depth_all[pos_new] >
              p_spherical.vec(2)) {
            continue;
          }
          for (int k = 0; k < points_in_pixel.size() && map_cons; k++) {
            const point_soph *p_occ = points_in_pixel[k];
            if (Case2IsOccluded(p_spherical, *p_occ) &&
                Case2DepthConsistencyCheck(*p_occ, *depth_map_list_[first_i])) {
              cur_occ_times = 1;
              if (cur_occ_times >= case2_occluded_times_threshold_) break;
              ti = (p_occ->time + p.time) / 2;
              vi =
                  (p_spherical.vec(2) - p_occ->vec(2)) / (p.time - p_occ->time);
              p.occu_index[0] = depth_map_list_[first_i]->map_index;
              p.occu_index[1] = pos_new;
              p.occu_index[2] = k;
              p.occ_vec = p_spherical.vec;
              p.occu_times = cur_occ_times;
              point_soph p0 = p;
              point_soph p1 = *points_in_pixel[k];
              int i = depth_map_list_.size();
              i = i - 2;
              V3D t1, t2;
              t1.setZero();
              t2.setZero();
              while (i >= 0) {
                if (p1.occu_index[0] == -1 ||
                    p1.occu_index[0] < depth_map_list_.front()->map_index) {
                  SphericalProjection(p1, depth_map_list_[i]->map_index,
                                      depth_map_list_[i]->project_R,
                                      depth_map_list_[i]->project_T, p1);
                  if (Case2SearchPointOccludingP(p1, *depth_map_list_[i])) {
                    p1.occ_vec = p1.vec;
                  } else {
                    break;
                  }
                }
                i = p1.occu_index[0] - depth_map_list_.front()->map_index;
                point_soph *p2 =
                    depth_map_list_[i]
                        ->depth_map[p1.occu_index[1]][p1.occu_index[2]];
                SphericalProjection(p, depth_map_list_[i]->map_index,
                                    depth_map_list_[i]->project_R,
                                    depth_map_list_[i]->project_T, p);
                if (Case2MapConsistencyCheck(p, *depth_map_list_[i],
                                             case2_interp_on_)) {
                  map_cons = false;
                  break;
                }
                float vc = (p1.occ_vec(2) - p2->vec(2)) / (p1.time - p2->time);
                double tc = (p2->time + p1.time) / 2;
                if (Case2IsOccluded(p, *p2) &&
                    Case2DepthConsistencyCheck(*p2, *depth_map_list_[i]) &&
                    Case2VelCheck(vi, vc, ti - tc)) {
                  cur_occ_times += 1;
                  if (cur_occ_times >= case2_occluded_times_threshold_) {
                    p.occu_times = cur_occ_times;
                    return true;
                  }
                  t2 = p2->global;
                  p1 = *p2;
                  vi = vc;
                  ti = tc;
                } else {
                  break;
                }
                i--;
              }
            }
            if (cur_occ_times >= case2_occluded_times_threshold_) break;
          }
          if (cur_occ_times >= case2_occluded_times_threshold_) break;
        }
        if (cur_occ_times >= case2_occluded_times_threshold_) break;
      }
    }
  }
  if (cur_occ_times >= case2_occluded_times_threshold_) {
    p.occu_times = cur_occ_times;
    return true;
  }
  return false;
}

bool DynObjFilter::Case2Enter(point_soph &p, const DepthMap &map_info) {
  if (p.status != STATIC) {
    return false;
  }
  float max_depth = 0;
  float depth_thr2_final = case2_depth_occlusion_threshold_;
  if (map_info.depth_map[p.position].size() > 0) {
    const point_soph *max_point =
        map_info
            .depth_map[p.position][map_info.max_depth_index_all[p.position]];
    max_depth = max_point->vec(2);
    float delta_t = (p.time - max_point->time);
    depth_thr2_final =
        std::min(depth_thr2_final, case2_velocity_threshold_ * delta_t);
  }
  if (p.vec(2) > max_depth + depth_thr2_final) {
    case2_num_++;
    return true;
  } else {
    return false;
  }
}

bool DynObjFilter::Case2MapConsistencyCheck(point_soph &p,
                                            const DepthMap &map_info,
                                            bool interp) {
  float cur_hor = case2_hor_consistency_threshold_;
  float cur_ver = case2_ver_consistency_threshold_;
  float cur_depth = case2_depth_consistency_threshold_;
  for (int ind_hor = -map_cons_hor_num2_; ind_hor <= map_cons_hor_num2_;
       ind_hor++) {
    for (int ind_ver = -map_cons_ver_num2_; ind_ver <= map_cons_ver_num2_;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth &&
          map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth) {
        continue;
      }
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (point->status == STATIC &&
            fabs(p.time - point->time) > lidar_scan_period_ &&
            fabs(p.vec(2) - point->vec(2)) < cur_depth &&
            fabs(p.vec(0) - point->vec(0)) < case2_hor_consistency_threshold_ &&
            fabs(p.vec(1) - point->vec(1)) < case2_ver_consistency_threshold_) {
          return true;
        }
      }
    }
  }
  if (interp && (p.local(0) < self_x_back_ || p.local(0) > self_x_front_ ||
                 p.local(1) > self_y_left_ || p.local(1) < self_y_right_)) {
    float cur_interp =
        case2_interp_threshold_ *
        (depth_map_list_.back()->map_index - map_info.map_index + 1);
    float depth_all =
        DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
    if (fabs(p.vec(2) - depth_all) < cur_interp) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool DynObjFilter::Case2SearchPointOccludingP(point_soph &p,
                                              const DepthMap &map_info) {
  for (int ind_hor = -occ_hor_num2_; ind_hor <= occ_hor_num2_; ind_hor++) {
    for (int ind_ver = -occ_ver_num2_; ind_ver <= occ_ver_num2_; ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      if (map_info.min_depth_all[pos_new] > p.vec(2)) {
        continue;
      }
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *p_cond = points_in_pixel[j];
        if (Case2IsOccluded(p, *p_cond) &&
            Case2DepthConsistencyCheck(*p_cond, map_info)) {
          p.occu_index[0] = map_info.map_index;
          p.occu_index[1] = pos_new;
          p.occu_index[2] = j;
          p.occ_vec = p.vec;
          return true;
        }
      }
    }
  }
  return false;
}

bool DynObjFilter::Case2IsOccluded(const point_soph &p,
                                   const point_soph &p_occ) {
  if ((dataset_ == 0 && p_occ.is_distort) || (dataset_ == 0 && p.is_distort) ||
      p_occ.status == INVALID)
    return false;
  if ((p.local(0) > self_x_back_ && p.local(0) < self_x_front_ &&
       p.local(1) < self_y_left_ && p.local(1) > self_y_right_) ||
      (p_occ.local(0) > self_x_back_ && p_occ.local(0) < self_x_front_ &&
       p_occ.local(1) < self_y_left_ && p_occ.local(1) > self_y_right_)) {
    return false;
  }
  float delta_t = p.time - p_occ.time;
  float cur_occ_hor = case2_hor_occlusion_threshold_;
  float cur_occ_ver = case2_ver_occlusion_threshold_;
  if (delta_t > 0) {
    float depth_thr2_final = std::min(case2_depth_occlusion_threshold_,
                                      case2_velocity_threshold_ * delta_t);
    if (p.vec(2) > p_occ.vec(2) + depth_thr2_final &&
        fabs(p.vec(0) - p_occ.vec(0)) < cur_occ_hor &&
        fabs(p.vec(1) - p_occ.vec(1)) < cur_occ_ver) {
      return true;
    }
  }
  return false;
}

float DynObjFilter::DepthInterpolationAll(point_soph &p, int map_index,
                                          const DepthMap2D &depth_map) {
  V3F p_1 = V3F::Zero();
  V3F p_2 = V3F::Zero();
  V3F p_3 = V3F::Zero();
  std::vector<V3F> p_neighbors;
  int all_num = 0;
  for (int ind_hor = -interp_hor_num_; ind_hor <= interp_hor_num_; ind_hor++) {
    for (int ind_ver = -interp_ver_num_; ind_ver <= interp_ver_num_;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel = depth_map[pos_new];
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (fabs(point->time - p.time) < lidar_scan_period_) {
          continue;
        }
        float hor_minus = point->vec(0) - p.vec(0);
        float ver_minus = point->vec(1) - p.vec(1);
        if (fabs(hor_minus) < interp_hor_threshold_ &&
            fabs(ver_minus) < interp_ver_threshold_) {
          all_num++;
          p_neighbors.push_back(point->vec);
          if (p_1(2) < 0.000001 ||
              fabs(hor_minus) + fabs(ver_minus) <
                  fabs(p_1(0) - p.vec(0)) + fabs(p_1(1) - p.vec(1))) {
            p_1 = point->vec;
          }
        }
      }
    }
  }
  int cur_size = p_neighbors.size();
  if (p_1(2) < 10E-5 || cur_size < 3) {
    return -1;
  }
  for (int t_i = 0; t_i < cur_size - 2; t_i++) {
    p_1 = p_neighbors[t_i];
    p_2 = V3F::Zero();
    p_3 = V3F::Zero();
    float min_fabs = 2 * (interp_hor_threshold_ + interp_ver_threshold_);
    float x = p.vec(0) - p_1(0);
    float y = p.vec(1) - p_1(1);
    float alpha = 0, beta = 0;
    for (int i = t_i + 1; i < cur_size - 1; i++) {
      if (fabs(p_neighbors[i](0) - p.vec(0)) +
              fabs(p_neighbors[i](1) - p.vec(1)) <
          min_fabs) {
        p_2 = p_neighbors[i];
        float single_fabs = fabs(p_neighbors[i](0) - p.vec(0)) +
                            fabs(p_neighbors[i](1) - p.vec(1));
        if (single_fabs >= min_fabs) continue;
        for (int ii = i + 1; ii < cur_size; ii++) {
          float cur_fabs = fabs(p_neighbors[i](0) - p.vec(0)) +
                           fabs(p_neighbors[i](1) - p.vec(1)) +
                           fabs(p_neighbors[ii](0) - p.vec(0)) +
                           fabs(p_neighbors[ii](1) - p.vec(1));
          if (cur_fabs < min_fabs) {
            float x1 = p_neighbors[i](0) - p_1(0);
            float x2 = p_neighbors[ii](0) - p_1(0);
            float y1 = p_neighbors[i](1) - p_1(1);
            float y2 = p_neighbors[ii](1) - p_1(1);
            float lower = x1 * y2 - x2 * y1;
            if (fabs(lower) > 10E-5) {
              alpha = (x * y2 - y * x2) / lower;
              beta = -(x * y1 - y * x1) / lower;
              if (alpha > 0 && alpha < 1 && beta > 0 && beta < 1 &&
                  (alpha + beta) > 0 && (alpha + beta) < 1) {
                p_3 = p_neighbors[ii];
                min_fabs = cur_fabs;
              }
            }
          }
        }
      }
    }
    if (p_2(2) < 10E-5 || p_3(2) < 10E-5) {
      continue;
    }
    float depth_cal =
        (1 - alpha - beta) * p_1(2) + alpha * p_2(2) + beta * p_3(2);
    return depth_cal;
  }
  return -2;
}  // -1 denotes no points, -2 denotes no triangular > 1000 denotes gauss
   // interpolation

bool DynObjFilter::Case2DepthConsistencyCheck(const point_soph &p,
                                              const DepthMap &map_info) {
  float all_minus = 0;
  int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;
  for (int ind_hor = -depth_cons_hor_num2_; ind_hor <= depth_cons_hor_num2_;
       ind_hor++) {
    for (int ind_ver = -depth_cons_ver_num2_; ind_ver <= depth_cons_ver_num2_;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (fabs(point->time - p.time) < lidar_scan_period_ &&
            fabs(point->vec(0) - p.vec(0)) <
                case2_hor_depth_consistency_threshold_ &&
            fabs(point->vec(1) - p.vec(1)) <
                case2_ver_depth_consistency_threshold_) {
          all_num++;
          if (point->status == STATIC) {
            float cur_minus = p.vec(2) - point->vec(2);
            if (fabs(cur_minus) <
                case2_depth_depth_consistency_threshold_max_) {
              num++;
              all_minus += fabs(point->vec(2) - p.vec(2));
            } else if (cur_minus > 0) {
              smaller_num++;
            } else {
              greater_num++;
            }
          }
        }
      }
    }
  }
  if (all_num > 0) {
    if (num > 1) {
      float cur_depth_thr = std::max(case2_depth_depth_consistency_threshold_,
                                     case2_k_depth_ * p.vec(2));
      if (all_minus / (num - 1) > cur_depth_thr) {
        return false;
      }
    }
    if (greater_num == 0 || smaller_num == 0) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool DynObjFilter::Case2VelCheck(float v1, float v2, double delta_t) {
  if (fabs(v1 - v2) < delta_t * case2_acc_threshold_) {
    return true;
  }
  return false;
}

bool DynObjFilter::Case3VelCheck(float v1, float v2, double delta_t) {
  if (fabs(v1 - v2) < delta_t * case3_acc_threshold_) {
    return true;
  }
  return false;
}

bool DynObjFilter::Case3(point_soph &p) {
  if (dataset_ == 0 && p.is_distort) return false;
  int first_i = depth_map_list_.size();
  first_i -= 1;
  if (first_i < 0) return false;
  point_soph p_spherical = p;
  SphericalProjection(p, depth_map_list_[first_i]->map_index,
                      depth_map_list_[first_i]->project_R,
                      depth_map_list_[first_i]->project_T, p_spherical);
  if (fabs(p_spherical.hor_ind) >= MAX_1D ||
      fabs(p_spherical.ver_ind) >= MAX_1D_HALF || p_spherical.vec(2) < 0.0f ||
      p_spherical.position < 0 || p_spherical.position >= MAX_2D_N) {
    p.status = INVALID;
    return false;
  }
  int cur_occ_times = 0;
  if (Case3Enter(p_spherical, *depth_map_list_[first_i])) {
    if (!Case3MapConsistencyCheck(p_spherical, *depth_map_list_[first_i],
                                  case3_interp_on_)) {
      double ti = 0;
      float vi = 0;
      float min_hor = case3_hor_occlusion_threshold_,
            min_ver = case3_ver_occlusion_threshold_;
      bool map_cons = true;
      for (int ind_hor = -occ_hor_num3_; ind_hor <= occ_hor_num3_; ind_hor++) {
        for (int ind_ver = -occ_ver_num3_; ind_ver <= occ_ver_num3_;
             ind_ver++) {
          int pos_new =
              ((p_spherical.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
              ((p_spherical.ver_ind + ind_ver) % MAX_1D_HALF);
          if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
          const std::vector<point_soph *> &points_in_pixel =
              depth_map_list_[first_i]->depth_map[pos_new];
          if (depth_map_list_[first_i]->max_depth_all[pos_new] <
              p_spherical.vec(2)) {
            continue;
          }
          for (int k = 0; k < points_in_pixel.size() && map_cons; k++) {
            const point_soph *p_occ = points_in_pixel[k];
            if (Case3IsOccluding(p_spherical, *p_occ) &&
                Case3DepthConsistencyCheck(*p_occ, *depth_map_list_[first_i])) {
              cur_occ_times = 1;
              ti = (p_occ->time + p.time) / 2;
              vi =
                  (p_occ->vec(2) - p_spherical.vec(2)) / (p.time - p_occ->time);
              p.is_occu_index[0] = depth_map_list_[first_i]->map_index;
              p.is_occu_index[1] = pos_new;
              p.is_occu_index[2] = k;
              p.is_occ_vec = p_spherical.vec;
              p.is_occu_times = cur_occ_times;
              point_soph p0 = p;
              point_soph p1 = *points_in_pixel[k];
              int i = depth_map_list_.size();
              i = i - 2;
              while (i >= 0) {
                if (p1.is_occu_index[0] == -1 ||
                    p1.is_occu_index[0] < depth_map_list_.front()->map_index) {
                  SphericalProjection(p1, depth_map_list_[i]->map_index,
                                      depth_map_list_[i]->project_R,
                                      depth_map_list_[i]->project_T, p1);
                  if (Case3SearchPointOccludedbyP(p1, *depth_map_list_[i])) {
                    p1.is_occ_vec = p1.vec;
                  } else {
                    break;
                  }
                }
                i = p1.is_occu_index[0] - depth_map_list_.front()->map_index;
                point_soph *p2 =
                    depth_map_list_[i]
                        ->depth_map[p1.is_occu_index[1]][p1.is_occu_index[2]];
                SphericalProjection(p, depth_map_list_[i]->map_index,
                                    depth_map_list_[i]->project_R,
                                    depth_map_list_[i]->project_T, p);
                if (Case3MapConsistencyCheck(p, *depth_map_list_[i],
                                             case3_interp_on_)) {
                  map_cons = false;
                  break;
                }
                float vc =
                    -(p1.is_occ_vec(2) - p2->vec(2)) / (p1.time - p2->time);
                double tc = (p2->time + p1.time) / 2;
                if (Case3IsOccluding(p, *p2) &&
                    Case3DepthConsistencyCheck(*p2, *depth_map_list_[i]) &&
                    Case3VelCheck(vi, vc, ti - tc)) {
                  cur_occ_times += 1;
                  if (cur_occ_times >= case3_occlusion_times_threshold_) {
                    p.is_occu_times = cur_occ_times;
                    return true;
                  }
                  p1 = *p2;
                  vi = vc;
                  ti = tc;
                } else {
                  break;
                }
                i--;
              }
            }
            if (cur_occ_times >= case3_occlusion_times_threshold_) break;
          }
          if (cur_occ_times >= case3_occlusion_times_threshold_) break;
        }
        if (cur_occ_times >= case3_occlusion_times_threshold_) break;
      }
    }
  }
  if (cur_occ_times >= case3_occlusion_times_threshold_) {
    p.is_occu_times = cur_occ_times;
    return true;
  }
  return false;
}

bool DynObjFilter::Case3Enter(point_soph &p, const DepthMap &map_info) {
  if (p.status != STATIC) {
    return false;
  }
  float min_depth = 0;
  float depth_thr3_final =

      case3_depth_occlusion_threshold_;
  if (map_info.depth_map[p.position].size() > 0) {
    const point_soph *min_point =
        map_info
            .depth_map[p.position][map_info.min_depth_index_all[p.position]];
    min_depth = min_point->vec(2);
    float delta_t = (p.time - min_point->time);
    depth_thr3_final =
        std::min(depth_thr3_final, case3_velocity_threshold_ * delta_t);
  }
  if (dataset_ == 0 && p.is_distort) {
    depth_thr3_final = enlarge_distort_ * depth_thr3_final;
  }
  if (p.vec(2) < min_depth - depth_thr3_final) {
    case3_num_++;
    return true;
  } else {
    return false;
  }
}

bool DynObjFilter::Case3MapConsistencyCheck(point_soph &p,
                                            const DepthMap &map_info,
                                            bool interp) {
  float cur_v_min = case3_velocity_threshold_;
  float cur_hor = case3_hor_consistency_threshold_;
  float cur_ver = case3_ver_consistency_threshold_;
  float cur_depth =

      case3_depth_consistency_threshold_;
  if (dataset_ == 0 && p.is_distort) cur_v_min = enlarge_distort_ * cur_v_min;
  for (int ind_hor = -map_cons_hor_num3_; ind_hor <= map_cons_hor_num3_;
       ind_hor++) {
    for (int ind_ver = -map_cons_ver_num3_; ind_ver <= map_cons_ver_num3_;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      if (map_info.max_depth_all[pos_new] > p.vec(2) + cur_depth &&
          map_info.min_depth_all[pos_new] < p.vec(2) - cur_depth) {
        continue;
      }
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (point->status == STATIC &&
            fabs(p.time - point->time) > lidar_scan_period_ &&
            (point->vec(2) - p.vec(2)) < cur_depth && \ 
                    fabs(p.vec(0) - point->vec(0)) < cur_hor &&
            fabs(p.vec(1) - point->vec(1)) < cur_ver) {
          return true;
        }
      }
    }
  }
  if (interp && (p.local(0) < self_x_back_ || p.local(0) > self_x_front_ ||
                 p.local(1) > self_y_left_ || p.local(1) < self_y_right_)) {
    float cur_interp =
        case3_interp_threshold_ *
        (depth_map_list_.back()->map_index - map_info.map_index + 1);
    float depth_all =
        DepthInterpolationAll(p, map_info.map_index, map_info.depth_map);
    if (fabs(p.vec(2) - depth_all) < cur_interp) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

bool DynObjFilter::Case3SearchPointOccludedbyP(point_soph &p,
                                               const DepthMap &map_info) {
  for (int ind_hor = -occ_hor_num3_; ind_hor <= occ_hor_num3_; ind_hor++) {
    for (int ind_ver = -occ_ver_num3_; ind_ver <= occ_ver_num3_; ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      if (map_info.min_depth_all[pos_new] > p.vec(2)) {
        continue;
      }
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *p_cond = points_in_pixel[j];
        if (Case3IsOccluding(p, *p_cond) &&
            Case3DepthConsistencyCheck(*p_cond, map_info)) {
          p.is_occu_index[0] = map_info.map_index;
          p.is_occu_index[1] = pos_new;
          p.is_occu_index[2] = j;
          p.occ_vec = p.vec;
          return true;
        }
      }
    }
  }
  return false;
}

bool DynObjFilter::Case3IsOccluding(const point_soph &p,
                                    const point_soph &p_occ) {
  if ((dataset_ == 0 && p_occ.is_distort) || (dataset_ == 0 && p.is_distort) ||
      p_occ.status == INVALID)
    return false;
  if ((p.local(0) > self_x_back_ && p.local(0) < self_x_front_ &&
       p.local(1) < self_y_left_ && p.local(1) > self_y_right_) ||
      (p_occ.local(0) > self_x_back_ && p_occ.local(0) < self_x_front_ &&
       p_occ.local(1) < self_y_left_ && p_occ.local(1) > self_y_right_)) {
    return false;
  }
  float delta_t = p.time - p_occ.time;
  if (delta_t > 0) {
    float depth_thr3_final = std::min(case3_depth_consistency_threshold_,
                                      case3_velocity_threshold_ * delta_t);
    if (dataset_ == 0 && p.is_distort)
      depth_thr3_final = enlarge_distort_ * depth_thr3_final;
    if (p_occ.vec(2) > p.vec(2) + depth_thr3_final &&
        fabs(p.vec(0) - p_occ.vec(0)) < case3_hor_occlusion_threshold_ &&
        fabs(p.vec(1) - p_occ.vec(1)) < case3_ver_occlusion_threshold_) {
      return true;
    }
  }
  return false;
}

bool DynObjFilter::Case3DepthConsistencyCheck(const point_soph &p,
                                              const DepthMap &map_info) {
  float all_minus = 0;
  int num = 0, smaller_num = 0, all_num = 0, greater_num = 0;  //
  for (int ind_hor = -depth_cons_hor_num3_; ind_hor <= depth_cons_hor_num3_;
       ind_hor++) {
    for (int ind_ver = -depth_cons_ver_num3_; ind_ver <= depth_cons_ver_num3_;
         ind_ver++) {
      int pos_new = ((p.hor_ind + ind_hor) % MAX_1D) * MAX_1D_HALF +
                    ((p.ver_ind + ind_ver) % MAX_1D_HALF);
      if (pos_new < 0 || pos_new >= MAX_2D_N) continue;
      const std::vector<point_soph *> &points_in_pixel =
          map_info.depth_map[pos_new];
      for (int j = 0; j < points_in_pixel.size(); j++) {
        const point_soph *point = points_in_pixel[j];
        if (fabs(point->time - p.time) < lidar_scan_period_ &&
            fabs(point->vec(0) - p.vec(0)) <
                case3_hor_depth_consistency_threshold_ &&
            fabs(point->vec(1) - p.vec(1)) <
                case3_ver_depth_consistency_threshold_) {
          all_num++;
          if (point->status == STATIC) {
            float cur_minus = p.vec(2) - point->vec(2);
            if (fabs(cur_minus) <
                case3_depth_depth_consistency_threshold_max_) {
              num++;
              all_minus += fabs(point->vec(2) - p.vec(2));
            } else if (cur_minus > 0) {
              smaller_num++;
            } else {
              greater_num++;
            }
          }
        }
      }
    }
  }
  if (all_num > 0) {
    if (num > 1) {
      float cur_depth_thr = std::max(case3_depth_depth_consistency_threshold_,
                                     case3_k_depth_ * p.vec(2));
      if (all_minus / (num - 1) > cur_depth_thr) {
        return false;
      }
    }
    if (greater_num == 0 || smaller_num == 0) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void DynObjFilter::publish_dyn(const ros::Publisher &pub_point_out,
                               const ros::Publisher &pub_frame_out,
                               const ros::Publisher &pub_steady_points,
                               const double &scan_end_time) {
  if (cluster_coupled_)  // pubLaserCloudEffect pub_pcl_dyn_extend
                         // pubLaserCloudEffect_depth
  {
    std::cout << "Found Dynamic Objects, numbers: "
              << dynamic_point_cloud_cluster_->points.size()
              << " Total time: " << time_total_
              << " Average total time: " << time_total_average_ << std::endl;
  } else {
    std::cout << "Found Dynamic Objects, numbers: "
              << dynamic_point_cloud_->points.size()
              << " Total time: " << time_total_
              << " Average total time: " << time_total_average_ << std::endl;
  }
  std::cout << "case1 num: " << case1_num_ << " case2 num: " << case2_num_
            << " case3 num: " << case3_num_ << std::endl;

  case1_num_ = 0;
  case2_num_ = 0;
  case3_num_ = 0;
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*dynamic_point_cloud_world_, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time().fromSec(scan_end_time);
  laserCloudFullRes3.header.frame_id = frame_id_;
  pub_point_out.publish(laserCloudFullRes3);
  if (cluster_coupled_ || cluster_future_) {
    sensor_msgs::PointCloud2 laserCloudFullRes4;
    pcl::toROSMsg(*dynamic_point_cloud_cluster_, laserCloudFullRes4);
    laserCloudFullRes4.header.stamp = ros::Time().fromSec(scan_end_time);
    laserCloudFullRes4.header.frame_id = frame_id_;
    pub_frame_out.publish(laserCloudFullRes4);
  }
  sensor_msgs::PointCloud2 laserCloudFullRes2;
  PointCloudXYZI::Ptr laserCloudSteadObj_pub(new PointCloudXYZI);
  if (cluster_coupled_) {
    if (static_point_cloud_times_ < static_buffer_size_) {
      static_point_cloud_times_++;
      static_point_cloud_buffer_.push_back(static_point_cloud_cluster_);
      for (int i = 0; i < static_point_cloud_buffer_.size(); i++) {
        *laserCloudSteadObj_pub += *static_point_cloud_buffer_[i];
      }
    } else {
      static_point_cloud_buffer_.pop_front();
      static_point_cloud_buffer_.push_back(static_point_cloud_cluster_);
      for (int i = 0; i < static_point_cloud_buffer_.size(); i++) {
        *laserCloudSteadObj_pub += *static_point_cloud_buffer_[i];
      }
    }
    pcl::VoxelGrid<PointType> downSizeFiltermap;
    downSizeFiltermap.setLeafSize(voxel_filter_size_, voxel_filter_size_,
                                  voxel_filter_size_);
    downSizeFiltermap.setInputCloud(laserCloudSteadObj_pub);
    PointCloudXYZI laserCloudSteadObj_down;
    downSizeFiltermap.filter(laserCloudSteadObj_down);
    pcl::toROSMsg(laserCloudSteadObj_down, laserCloudFullRes2);
  } else {
    std::cout << "Found Steady Objects, numbers: "
              << static_point_cloud_->points.size() << std::endl;
    if (static_point_cloud_times_ < static_buffer_size_) {
      static_point_cloud_times_++;
      static_point_cloud_buffer_.push_back(static_point_cloud_);
      for (int i = 0; i < static_point_cloud_buffer_.size(); i++) {
        *laserCloudSteadObj_pub += *static_point_cloud_buffer_[i];
      }
    } else {
      static_point_cloud_buffer_.pop_front();
      static_point_cloud_buffer_.push_back(static_point_cloud_);
      for (int i = 0; i < static_point_cloud_buffer_.size(); i++) {
        *laserCloudSteadObj_pub += *static_point_cloud_buffer_[i];
      }
    }
    pcl::toROSMsg(*laserCloudSteadObj_pub, laserCloudFullRes2);
  }
  laserCloudFullRes2.header.stamp = ros::Time().fromSec(scan_end_time);
  laserCloudFullRes2.header.frame_id = frame_id_;
  pub_steady_points.publish(laserCloudFullRes2);
}

void DynObjFilter::set_path(std::string file_path,
                            std::string file_path_origin) {
  is_set_path = true;
  out_file_ = file_path;
  out_file_origin_ = file_path_origin;
}
