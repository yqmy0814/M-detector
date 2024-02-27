#ifndef DEPTH_MAP_H
#define DEPTH_MAP_H

#include "eigen_points_types.h"

/*** For dynamic object filtering ***/
#define PI_MATH (3.141593f)
#define HASH_P 116101
#define MAX_N 100000
#define MAX_2D_N \
  (564393)  // 1317755(1317755) //(50086) 39489912 // MAX_1D * MAX_1D_HALF
#define MAX_1D (1257)      //(2095) //(317)  // 2*pi/ hor_resolution 0.005
#define MAX_1D_HALF (449)  // 3142() //158 31416 pi / ver_resolution 0.007
#define DEPTH_WIDTH (80 * 10)
#define COE_SMALL 1000
#define MAP_NUM 17    // 30
#define HASH_PRIM 19  // 37

enum dyn_obj_flg { STATIC, CASE1, CASE2, CASE3, SELF, UNCERTAIN, INVALID };

struct point_soph {
  int hor_ind;
  V3F vec;
  int ver_ind;
  int position;
  int occu_times;
  int is_occu_times;
  Eigen::Vector3i occu_index;
  Eigen::Vector3i is_occu_index;
  double time;
  V3F occ_vec;
  V3F is_occ_vec;
  M3D rot;
  V3D trans;
  dyn_obj_flg status;
  V3D global;
  V3D local;
  V3F cur_vec;
  float intensity;
  bool is_distort;
  V3D last_closest;
  std::array<float, MAP_NUM> last_depth_interps = {};
  std::array<V3F, HASH_PRIM> last_vecs = {};
  std::array<Eigen::Vector3i, HASH_PRIM> last_positions = {};
  typedef boost::shared_ptr<point_soph> Ptr;

  point_soph(V3D &point, float &horizontal_resolution,
             float &vertical_resolution) {
    vec(2) = float(point.norm());
    vec(0) = atan2f(float(point(1)), float(point(0)));
    vec(1) = atan2f(float(point(2)),
                    sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
    hor_ind = floor((vec(0) + PI_MATH) / horizontal_resolution);
    ver_ind = floor((vec(1) + 0.5 * PI_MATH) / vertical_resolution);
    position = hor_ind * MAX_1D_HALF + ver_ind;
    time = -1;
    occu_times = is_occu_times = 0;
    occu_index = -1 * Eigen::Vector3i::Ones();
    is_occu_index = -1 * Eigen::Vector3i::Ones();
    occ_vec.setZero();
    is_occ_vec.setZero();
    trans.setZero();
    global.setZero();
    rot.setOnes();
    last_depth_interps.fill(0.0);
    last_vecs.fill(V3F::Zero());
    last_positions.fill(Eigen::Vector3i::Zero());
    is_distort = false;
    cur_vec.setZero();
    local.setZero();
    last_closest.setZero();
  };
  point_soph() {
    vec.setZero();
    hor_ind = ver_ind = position = occu_times = is_occu_times = 0;
    time = -1;
    occu_index = -1 * Eigen::Vector3i::Ones();
    is_occu_index = -1 * Eigen::Vector3i::Ones();
    occ_vec.setZero();
    is_occ_vec.setZero();
    trans.setZero();
    global.setZero();
    rot.setOnes();
    last_depth_interps.fill(0.0);
    last_vecs.fill(V3F::Zero());
    last_positions.fill(Eigen::Vector3i::Zero());
    is_distort = false;
    cur_vec.setZero();
    local.setZero();
    last_closest.setZero();
  };
  point_soph(V3F s, int ind1, int ind2, int pos) {
    vec = s;
    hor_ind = ind1;
    ver_ind = ind2;
    position = pos;
    occu_times = is_occu_times = 0;
    time = -1;
    occu_index = -1 * Eigen::Vector3i::Ones();
    is_occu_index = -1 * Eigen::Vector3i::Ones();
    occ_vec.setZero();
    is_occ_vec.setZero();
    trans.setZero();
    global.setZero();
    rot.setOnes();
    last_depth_interps.fill(0.0);
    last_vecs.fill(V3F::Zero());
    last_positions.fill(Eigen::Vector3i::Zero());
    is_distort = false;
    cur_vec.setZero();
    local.setZero();
    last_closest.setZero();
  };
  point_soph(const point_soph &cur) {
    vec = cur.vec;
    hor_ind = cur.hor_ind;
    ver_ind = cur.ver_ind;
    position = cur.position;
    time = cur.time;
    occu_times = cur.occu_times;
    is_occu_times = cur.is_occu_times;
    occu_index = cur.occu_index;
    is_occu_index = cur.is_occu_index;
    occ_vec = cur.occ_vec;
    is_occ_vec = cur.is_occ_vec;
    trans = cur.trans;
    global = cur.global;
    rot = cur.rot;
    status = cur.status;
    last_depth_interps = cur.last_depth_interps;
    last_vecs = cur.last_vecs;
    last_positions = cur.last_positions;
    local = cur.local;
    is_distort = cur.is_distort;
    cur_vec = cur.cur_vec;
    last_closest = cur.last_closest;
  };

  ~point_soph(){};

  void GetVec(V3D &point, float &horizontal_resolution,
              float &vertical_resolution) {
    vec(2) = float(point.norm());
    vec(0) = atan2f(float(point(1)), float(point(0)));
    vec(1) = atan2f(float(point(2)),
                    sqrt(pow(float(point(0)), 2) + pow(float(point(1)), 2)));
    hor_ind = floor((vec(0) + PI_MATH) / horizontal_resolution);
    ver_ind = floor((vec(1) + 0.5 * PI_MATH) / vertical_resolution);
    position = hor_ind * MAX_1D_HALF + ver_ind;
  };

  void reset() {
    occu_times = is_occu_times = 0;
    occu_index = -1 * Eigen::Vector3i::Ones();
    is_occu_index = -1 * Eigen::Vector3i::Ones();
    occ_vec.setZero();
    is_occ_vec.setZero();
    last_closest.setZero();
    last_depth_interps.fill(0.0);
    last_vecs.fill(V3F::Zero());
    last_positions.fill(Eigen::Vector3i::Zero());
    is_distort = false;
  };
};

typedef std::vector<std::vector<point_soph *>> DepthMap2D;

class DepthMap {
 public:
  DepthMap2D depth_map;
  double time;
  int map_index;
  M3D project_R;
  V3D project_T;
  std::vector<point_soph::Ptr> point_sopth_pointer;
  int point_sopth_pointer_count = 0;
  float *min_depth_static = nullptr;
  float *min_depth_all = nullptr;
  float *max_depth_all = nullptr;
  float *max_depth_static = nullptr;
  int *max_depth_index_all = nullptr;
  int *min_depth_index_all = nullptr;
  std::vector<int> index_vector;
  typedef boost::shared_ptr<DepthMap> Ptr;

  DepthMap() {
    printf("build depth map2\n");
    depth_map.assign(MAX_2D_N, std::vector<point_soph *>());

    time = 0.;
    project_R.setIdentity(3, 3);
    project_T.setZero(3, 1);

    min_depth_static = new float[MAX_2D_N];
    min_depth_all = new float[MAX_2D_N];
    max_depth_all = new float[MAX_2D_N];
    max_depth_static = new float[MAX_2D_N];
    std::fill_n(min_depth_static, MAX_2D_N, 0.0);
    std::fill_n(min_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_static, MAX_2D_N, 0.0);
    max_depth_index_all = new int[MAX_2D_N];
    min_depth_index_all = new int[MAX_2D_N];
    std::fill_n(min_depth_index_all, MAX_2D_N, -1);
    std::fill_n(max_depth_index_all, MAX_2D_N, -1);
    map_index = -1;
    index_vector.assign(MAX_2D_N, 0);
    for (int i = 0; i < MAX_2D_N; i++) {
      index_vector[i] = i;
    }
  }

  DepthMap(M3D rot, V3D transl, double cur_time, int frame) {
    depth_map.assign(MAX_2D_N, std::vector<point_soph *>());
    time = cur_time;
    project_R = rot;
    project_T = transl;
    min_depth_static = new float[MAX_2D_N];
    min_depth_all = new float[MAX_2D_N];
    max_depth_all = new float[MAX_2D_N];
    max_depth_static = new float[MAX_2D_N];
    std::fill_n(min_depth_static, MAX_2D_N, 0.0);
    std::fill_n(min_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_static, MAX_2D_N, 0.0);
    max_depth_index_all = new int[MAX_2D_N];
    min_depth_index_all = new int[MAX_2D_N];
    std::fill_n(min_depth_index_all, MAX_2D_N, -1);
    std::fill_n(max_depth_index_all, MAX_2D_N, -1);
    map_index = frame;
    index_vector.assign(MAX_2D_N, 0);
    for (int i = 0; i < MAX_2D_N; i++) {
      index_vector[i] = i;
    }
  }

  DepthMap(const DepthMap &cur) {
    depth_map = cur.depth_map;
    time = cur.time;
    project_R = cur.project_R;
    project_T = cur.project_T;
    point_sopth_pointer = cur.point_sopth_pointer;
    min_depth_static = new float[MAX_2D_N];
    min_depth_all = new float[MAX_2D_N];
    max_depth_all = new float[MAX_2D_N];
    max_depth_static = new float[MAX_2D_N];
    std::fill_n(min_depth_static, MAX_2D_N, 0.0);
    std::fill_n(min_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_static, MAX_2D_N, 0.0);
    max_depth_index_all = new int[MAX_2D_N];
    min_depth_index_all = new int[MAX_2D_N];
    map_index = cur.map_index;
    for (int i = 0; i < MAX_2D_N; i++) {
      min_depth_static[i] = cur.min_depth_static[i];
      max_depth_static[i] = cur.max_depth_static[i];
      min_depth_all[i] = cur.min_depth_all[i];
      max_depth_all[i] = cur.max_depth_all[i];
      max_depth_index_all[i] = cur.max_depth_index_all[i];
      min_depth_index_all[i] = cur.min_depth_index_all[i];
    }
    index_vector.assign(MAX_2D_N, 0);
    for (int i = 0; i < MAX_2D_N; i++) {
      index_vector[i] = i;
    }
  }
  ~DepthMap() {
    if (min_depth_static != nullptr) delete[] min_depth_static;
    if (min_depth_all != nullptr) delete[] min_depth_all;
    if (max_depth_all != nullptr) delete[] max_depth_all;
    if (max_depth_static != nullptr) delete[] max_depth_static;
    if (max_depth_index_all != nullptr) delete[] max_depth_index_all;
    if (min_depth_index_all != nullptr) delete[] min_depth_index_all;
  }

  void Reset(M3D rot, V3D transl, double cur_time, int frame) {
    time = cur_time;
    project_R = rot;
    project_T = transl;
    map_index = frame;
    double t = omp_get_wtime();
    std::for_each(std::execution::par, index_vector.begin(), index_vector.end(),
                  [&](const int &i) { depth_map[i].clear(); });
    std::fill_n(min_depth_static, MAX_2D_N, 0.0);
    std::fill_n(min_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_all, MAX_2D_N, 0.0);
    std::fill_n(max_depth_static, MAX_2D_N, 0.0);
    std::fill_n(max_depth_index_all, MAX_2D_N, -1);
    std::fill_n(min_depth_index_all, MAX_2D_N, -1);
  }
};
#endif