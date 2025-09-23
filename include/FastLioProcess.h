#ifndef FastLio_Process_H
#define FastLio_Process_H
#include <ros/ros.h>   // 通过导入其他 .h 包含这个就可以不导入
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>  // 图像
#include <opencv2/opencv.hpp>

#include <condition_variable>   // 用于信号
#include <pcl/filters/voxel_grid.h>  // 用于下采样
#include <pcl/io/pcd_io.h> // 用于保存文件
#include <pcl_conversions/pcl_conversions.h>  // 用于将pcl点云转为ros发布信息
// #include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <omp.h>  //  OpenMP 的头文件 用于多线程操作

#include <string>
#include <common_lib.h>
#include <csignal> // 用于监听信号
#include <state.h>
#include <ikd-Tree/ikd_Tree.h>
#include <fstream>  // 用于输出文件
#include <filesystem>
namespace fs = std::filesystem;

#define MAX_INI_COUNT (10)
#define LASER_POINT_COV     (0.001)
#define NUM_MATCH_POINTS    (5)  // 匹配点数

class FastLioProcess{
public:
    FastLioProcess(ros::NodeHandle &nh); // 构造函数
    ~FastLioProcess();  // 析构函数

    // 从launch 中读取参数
    bool path_en=true;  // 是否发布路径
    //设置是否发布激光雷达数据，是否发布稠密数据，是否发布激光雷达数据的身体数据
    bool  scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
    int NUM_MAX_ITERATIONS=4;
    string map_file_path=""; // 地图保存路径
    string lid_topic="/livox/lidar";
    string imu_topic="/livox/imu";
    double time_diff_lidar_to_imu=0.0;
    double filter_size_surf_min=0.5;  // 单帧点云降采样时的体素大小
    double filter_size_map_min =0.5;  // 二叉树地图的网格大小
    double cube_len = 200;  // 地图的局部区域的长度
    double fov_deg = 180;  // 视场角  通常用于描述传感器（可能是激光雷达等）的视野范围角度。
    double gyr_cov =0.1;  // IMU陀螺仪的协方差
    double acc_cov =0.1; // IMU加速度计的协方差
    double b_gyr_cov =0.1; // IMU陀螺仪偏置的协方差
    double b_acc_cov =0.1; // IMU加速度计偏置的协方差
    double blind =0.01;   // 点云过滤的距离范围内的点
    int N_SCANS=16; // 激光雷达扫描的线数
    int point_filter_num=2; // 采样间隔，即每隔point_filter_num个点取1个点
    bool pcd_save_en=false;  // 是否将点云地图保存到PCD文件
    int pcd_save_interval=-1; // 隔多少帧保存一帧点云 ， -1 表示不保存
    vector<double> extrinT; // 雷达转imu的平移关系
    vector<double> extrinR; // 雷达转imu的旋转关系矩阵
    bool imu_en =false;
    bool lidar_en =false;

    // 定义接收器和发布器
    ros::Subscriber sub_pcl;
    ros::Subscriber sub_imu;
    // 定义回调函数
    sensor_msgs::Imu::Ptr imu_tmp;
    void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
    void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
    void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
    void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void livox_avia_process(const livox_ros_driver::CustomMsg::ConstPtr &msg,PointCloudXYZI::Ptr &pcl_out);
    PointCloudXYZI pl_full, pl_corn, pl_surf;  // 全部点、边缘点、平面点
    

    // 记录最后回调的信息时间
    double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;   //设置雷达时间戳，imu时间戳
    // 回调buffer
    deque<double>                     lidar_time_buffer;  // 激光雷达数据时间戳缓存队列
    deque<PointCloudXYZI::Ptr>        lidar_buffer;   //记录特征提取或间隔采样后的lidar（特征）数据
    deque<cv::Mat> img_buffer;
    deque<sensor_msgs::Imu::ConstPtr> imu_buffer;   // IMU数据缓存队列

    // 数据同步时所用参数
    double last_lio_update_time = -1.0, lidar_end_time = 0.0,lidar_beg_time = 0.0;;
    bool flg_first_scan=true, lidar_pushed = false;
    struct MeasureGroup Measures;

    //################################ 主体程序所需的变量和函数
    // 主函数运行
    void run();
    
    //数据同步

    bool sync_packages(MeasureGroup &meas);
    double first_lidar_time;
    PointCloudXYZI::Ptr lidar{new PointCloudXYZI()}; // 当前的原始雷达点云
    // 全局状态
    StatesGroup state;  
    // 前向传播和反向传播
    void processImu();
    void IMU_init();
    bool imu_need_init_ =true,imu_init_first_frame=true;
    int init_iter_num =1;
    V3D cov_acc= V3D(0.1, 0.1, 0.1);    //加速度测量协方差
    V3D cov_gyr= V3D(0.1, 0.1, 0.1);    //角速度测量协方差
    V3D cov_bias_gyr= V3D(0.0001, 0.0001, 0.0001);     //角速度测量协方差偏置
    V3D cov_bias_acc= V3D(0.0001, 0.0001, 0.0001);     //加速度测量协方差偏置
    V3D mean_acc=V3D(0, 0, -1.0);  //加速度均值,用于计算方差
    V3D mean_gyr= V3D(0, 0, 0);
    sensor_msgs::ImuConstPtr last_imu;   // 上一帧数据的最后一个imu
    void UndistortPcl();  // 去畸变
    Matrix<double, DIM_Q, DIM_Q>  Q; //测量误差协方差
    vector<Pose6D> IMUpose;
    
    // 下采样
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    
    // 这里只能声明，不能小括号初始化
    PointCloudXYZI::Ptr pcl_undistort{new PointCloudXYZI()};  //去畸变的特征点云 (雷达坐标系)
    PointCloudXYZI::Ptr pcl_undistort_world{new PointCloudXYZI()}; //畸变纠正后降采样的单帧点云，(世界坐标系)
    PointCloudXYZI::Ptr feats_down_body{new PointCloudXYZI()};  //降采样的去畸变的特征点云 (雷达坐标系)
    PointCloudXYZI::Ptr feats_down_world{new PointCloudXYZI()};  //降采样的去畸变的特征点云 (世界坐标系)
    PointCloudXYZI::Ptr pcl_org_world{new PointCloudXYZI()};
    PointCloudXYZI::Ptr normvec{new PointCloudXYZI()};  // 存储匹配成功的点对应平面的法向量和店面距离  (世界坐标系)
    PointCloudXYZI::Ptr pcl_match_body{new PointCloudXYZI(100000, 1)};   // 存储匹配成功的点  (雷达坐标系)
    PointCloudXYZI::Ptr corr_normvect{new PointCloudXYZI(100000, 1)};  // 存储匹配成功的点对应平面的法向量和店面距离  (世界坐标系)

    vector<PointVector>  Nearest_Points;   // 每个点的最近点 序列  是两重列表
    


    V3D pos_lid; // 雷达在世界坐标系下的位置
    int pcl_undistort_size;  // 去畸变后的点云数量
    int feats_down_size; //记录滤波后的点云数量


    // 地图管理
    // KD_TREE<PointType> ikdtree;  // 创建KD树对象   为什么只能创建再全局 ？？？？？？？？？？？？？？？？？？
    void lasermap_fov_segment(); // 动态调整局部地图
    void points_cache_collect(); //得到被剔除的点
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;  // 是否已经初始化地图
    vector<BoxPointType> cub_needrm;
    // kdtree_size_st为ikd-tree获得的节点数，kdtree_size_end为ikd-tree结束时的节点数，add_point_size为添加点的数量，kdtree_delete_counter为删除点的数量
    int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
    // kdtree_incremental_time为kdtree建立时间，kdtree_search_time为kdtree搜索时间，kdtree_delete_time为kdtree删除时间;
    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    const float MOV_THRESHOLD = 1.5f;  //设置的当前雷达系中心到各个地图边缘的权重
    float DET_RANGE = 300.0f;   //设置的当前雷达系中心到各个地图边缘的距离  传感器能够有效探测到目标的最远距离
    void map_incremental(); // 在kd树中添加新的点云

    // 状态优化
    void state_estimation(); 
    void match_points();
    int effct_feat_num = 0;
    bool flg_EKF_converged=true; // 为true，表示收敛
    bool match_valid = true; // 点云中是否有足量点云匹配平面成功
    int max_iterations=5;
    bool   point_selected_surf[100000] = {0};  // 对应位置表示点是否为平面特征点
    float res_last[100000] = {0.0};  //残差绝对值， 是一个指标但在代码中没用
    double R = 0.001;  // 代表雷达点的测量协方差  每个点的协方差是一样的
    double limit[DIM_STATE] = {0.001}; // 优化状态的限制
    
    // 工具函数
    void pointBodyToWorld(PointType const * const pi, PointType * const po);
    
    // 发布数据
    ros::Publisher pubOdomAftMapped; // 位姿发布器
    nav_msgs::Odometry odomAftMapped;  // 发布的位姿格式
    void publish_odometry();
    ros::Publisher pubPath;  // 发布里程计总的路径，topic名字为/path   发布机器人的路径，用于rviz建图
    nav_msgs::Path path;  // 发布的位置格式 是个列表
    geometry_msgs::PoseStamped msg_body_pose; //是一个列表，存储机器人每个运动节点的时间，姿态（偏移量t）（旋转）
    void publish_path(const ros::Publisher pubPath);
    template<typename T> void set_posestamp(T & out);
    ros::Publisher pubLaserCloudFull;
    void publish_frame_world();

    // 迭代后
    V3D euler_cur;
    geometry_msgs::Quaternion geoQuat;   //四元数

    // 保存点云
    std::ofstream fout_cloud;
    void save();
    int save_i=0; // 统计帧数
    string save_root;

    int frame_index =0;

    // 线程锁
    // std::mutex mtx_buffer, mtx_buffer_imu_prop;  //mtx_buffer_imu_prop 该参数还未知
    // std::condition_variable sig_buffer;  // 用于唤醒所有线程
    // // 用于用于监听信号
    // void SigHandle(int sig);
    // bool flg_exit = false;
};
// ######################################全局变量
//  线程锁
extern std::mutex mtx_buffer, mtx_buffer_imu_prop;  //mtx_buffer_imu_prop 该参数还未知
extern std::condition_variable sig_buffer;  // 用于唤醒所有线程
// 用于用于监听信号
void SigHandle(int sig);
extern bool flg_exit;

// 用于创建文件夹
bool createDirectoryIfNotExists(const std::string& path);


template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {   
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);  //超定方程组的最小二乘解: 
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;  // 有一个点的误差较大就说明不是平面
        }
    }
    return true;
}

// 计算欧拉角
template <typename T> Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular)
  {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  }
  else
  {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

float calc_dist(PointType p1, PointType p2);  // 计算点的距离

#endif