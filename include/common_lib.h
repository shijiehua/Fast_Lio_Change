#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#include <iostream>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <deque>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace Eigen;

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define SKEW_SYM_MATRX(v) 0.0,  -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#define G_m_s2 (9.81)         // Gravaty const in GuangDong/China  9.81
#define DIM_STATE (18)  // 状态参数总维度
#define DIM_Q (12)  // 状态参数总维度

typedef pcl::PointXYZINormal PointType;  
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;
typedef Matrix<double, DIM_STATE, DIM_STATE> MddD;
typedef Matrix<double, DIM_Q, DIM_Q> MdvD;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

// 每一帧对齐的数据包
struct MeasureGroup     // Lidar data and imu dates for the curent process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    double last_lio_update_time;
    double point_time;
    PointCloudXYZI::Ptr lidar;
    deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct Pose6D
{
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p,
                const Matrix<T, 3, 3> &R)
{
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  // Map<M3D>(rot_kp.rot, 3,3) = R;
  return move(rot_kp);
}


#endif