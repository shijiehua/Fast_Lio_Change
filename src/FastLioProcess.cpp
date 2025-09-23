#include <FastLioProcess.h>
// ######################################全局变量
KD_TREE<PointType> ikdtree;  // 创建KD树对象  为什么只能创建再全局  ？？？？？？？？？？？
//  线程锁
std::mutex mtx_buffer, mtx_buffer_imu_prop;  //mtx_buffer_imu_prop 该参数还未知
std::condition_variable sig_buffer;  // 用于唤醒所有线程
// 用于用于监听信号
// void SigHandle(int sig);
bool flg_exit = false;

FastLioProcess::FastLioProcess(ros::NodeHandle &nh)
    : extrinT(3, 0.0),  // 初始化平移向量（3个元素，值为0.0）
      extrinR(9, 0.0)   // 初始化旋转矩阵（9个元素，值为0.0）
{   
    // 从参数服务器读取参数值赋给变量（包括launch文件和launch读取的yaml文件中的参数）
    nh.param<bool>("publish/path_en",path_en, true);  
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);   // 是否发布当前正在扫描的点云的topic
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);    // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic，
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);  // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic，需要该变量和上一个变量同时为true才发布
    nh.param<int>("max_iteration",max_iterations,4);  // 卡尔曼滤波的最大迭代次数
    nh.param<string>("map_file_path",map_file_path,"");  // 地图保存路径
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");  // 激光雷达点云topic名称
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");   // IMU的topic名称
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);  // 单帧点云降采样时的体素大小
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);   // VoxelGrid降采样时的体素大小
    nh.param<double>("cube_side_length",cube_len,200);    // 地图的局部区域的长度（FastLio2论文中有解释
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);  // 激光雷达的最大探测范围
    nh.param<double>("mapping/fov_degree",fov_deg,180);  // 视场角  通常用于描述传感器（可能是激光雷达等）的视野范围角度。
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);  // IMU陀螺仪的协方差
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);   // IMU加速度计的协方差
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);  // IMU陀螺仪偏置的协方差
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);   // IMU加速度计偏置的协方差
    nh.param<double>("preprocess/blind", blind, 0.01);  // 最小距离阈值，即过滤掉0～blind范围内的点云
    nh.param<int>("preprocess/scan_line", N_SCANS, 16);   // 激光雷达扫描的线数（livox avia为6线）
    nh.param<int>("point_filter_num", point_filter_num, 2);   // 采样间隔，即每隔point_filter_num个点取1个点
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);   // 是否将点云地图保存到PCD文件
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());  // 外参平移向量  IMU 和 LiDAR 之间的相对平移关系
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());  // 外参旋转矩阵  IMU 和 LiDAR 之间的旋转关系矩阵
    nh.param<bool>("common/lidar_en", lidar_en, false);
    nh.param<bool>("common/imu_en", imu_en, false);

    // 节点接收器和发布器的定义和初始化
    sub_pcl =nh.subscribe(lid_topic, 200000, &FastLioProcess::livox_pcl_cbk,this);
    sub_imu = nh.subscribe(imu_topic, 200000, &FastLioProcess::imu_cbk,this);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/Odometry", 100000);  // 发布当前里程计信息，topic名字为/Odometry  ，发布机器人位姿的，用于rviz建图
    pubPath = nh.advertise<nav_msgs::Path> ("/path", 100000);  // 发布里程计总的路径，topic名字为/path   发布机器人的路径，用于rviz建图
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);  // 发布当前正在扫描的点云，topic名字为/cloud_registered
    // 设置 state 参数  外参
    state.offset_R_L_I<<MAT_FROM_ARRAY(extrinR);
    state.offset_T_L_I<<VEC_FROM_ARRAY(extrinT);
    cov_bias_gyr = {b_gyr_cov,b_gyr_cov,b_gyr_cov};
    cov_bias_acc = {b_acc_cov,b_acc_cov,b_acc_cov};
    // 设置点云下采样
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    // 保存点云
    save_root = string(string(ROOT_DIR) + "PCD/");
    createDirectoryIfNotExists(save_root);
    string save_scan_path = string(string(ROOT_DIR) + "PCD/scan.txt");
    fs::remove(save_scan_path);
    fout_cloud.open(save_scan_path, ios::app); 
}

FastLioProcess::~FastLioProcess(){}  // 析构函数未实现

void FastLioProcess::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
    if (!imu_en) return;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));   // 单帧imu的数据
    //去除无效数据
    // if (msg->linear_acceleration.x==0 && msg->linear_acceleration.y==0 && msg->linear_acceleration.z==0){
    //     cout<<"0值，跳过imu时间："<<msg_in->header.stamp.toSec()<<endl;
    //   return;
    // } 
    // if (imu_tmp != nullptr && msg->linear_acceleration.x==imu_tmp->linear_acceleration.x && msg->linear_acceleration.y==imu_tmp->linear_acceleration.y && msg->linear_acceleration.z==imu_tmp->linear_acceleration.z){
    //     cout<<"重复，跳过imu时间："<<msg_in->header.stamp.toSec()<<endl;
    //   return;
    // }
    imu_tmp = msg;
    double timestamp = msg->header.stamp.toSec(); // IMU时间戳
    // cout << "imu时间戳："<< timestamp<<endl;
    mtx_buffer.lock();
    // 如果当前IMU的时间戳小于上一个时刻IMU的时间戳，则IMU数据有误，将IMU数据缓存队列清空
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
// 除了AVIA类型之外的雷达点云回调函数，将数据引入到buffer当中
void FastLioProcess::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{ // 传感器信息获取
    mtx_buffer.lock(); //加锁
    // 如果当前扫描的激光雷达数据的时间戳比上一次扫描的激光雷达数据的时间戳早，需要将激光雷达数据缓存队列清空
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_time_buffer.clear();
    }
    // cout<<"该点云时间戳："<<msg->header.stamp.toSec()<<endl;
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    // 进行点云预处理 ##########################################
    // 根据情况自己编写
    // p_pre->process(msg, ptr);  // p_pre 是前处理对象的指针
    //  ##########################################
    // cout <<"点云大小："<<ptr->size()<<endl;
    lidar_buffer.push_back(ptr);  //点云预处理  降采样
    lidar_time_buffer.push_back(msg->header.stamp.toSec()); //将时间放入缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec();  //记录最后一个时间
    mtx_buffer.unlock();
    sig_buffer.notify_all();  // 唤醒所有线程
}
void FastLioProcess::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)  // msg 里面是一次激光扫描的雷达数据 
{  // 专门针对livox雷达 
    mtx_buffer.lock(); // 互斥锁
    if (msg->header.stamp.toSec() < last_timestamp_lidar)    
    { // 如果当前扫描的激光雷达数据的时间戳比上一次扫描的激光雷达数据的时间戳早，需要将激光雷达数据缓存队列清空
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_time_buffer.clear();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();  // 这里为啥是雷达时间? 这一个雷达点云的开始时间 单位为秒 而offset_time单位为ms
    // cout <<"当前帧雷达head时间" <<last_timestamp_lidar<<endl;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    // 对激光雷达数据进行预处理（特征提取或者降采样），其中p_pre是Preprocess类的智能指针
    livox_avia_process(msg, ptr);
    lidar_buffer.push_back(ptr); //点云预处理  降采样
    lidar_time_buffer.push_back(last_timestamp_lidar);
    mtx_buffer.unlock();
    sig_buffer.notify_all();   // 唤醒所有线程
}
void FastLioProcess::livox_avia_process(const livox_ros_driver::CustomMsg::ConstPtr &msg,PointCloudXYZI::Ptr &pcl_out)
{
    pl_surf.clear(); // 清除之前的平面点云缓存
    pl_corn.clear(); // 清除之前的角点云缓存
    pl_full.clear(); // 清除之前的全点云缓存
    double t1 = omp_get_wtime(); // 后面没用到  获取当前时间。用于优化线程的
    int plsize = msg->point_num;  // 这一批中的点云总个数
    // cout<<"plsie: "<<plsize<<endl;

    pl_corn.reserve(plsize);  // 分配空间
    pl_surf.reserve(plsize);  // 分配空间
    pl_full.resize(plsize);   // 分配空间

    uint valid_num = 0;  // 有效的点云数
    // 特征提取（FastLIO2默认不进行特征提取）
    // 分别对每个点云进行处理
    size_t count_out=0;
    size_t count_near=0;
    for(uint i=1; i<plsize; i++)
    {  // 只取线数在0~N_SCANS内并且回波次序为0或者1的点云
        if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num ++;  // 有效的点云数
            if (valid_num % point_filter_num == 0)  // 等间隔降采样  // 这里应该做一个判断 如果点数少了则不进行下采样
            {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
                // 只有当当前点和上一点的间距足够大（>1e-7），并且在最小距离阈值之外，才将当前点认为是有用的点，加入到pl_surf队列中
                if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z <= (blind * blind)){
                count_out+=1;
                }
                if ((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
                && (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                && (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)){
                count_near+=1;
                }

                if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
                    || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
                    || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
                    && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
                {
                pl_surf.push_back(pl_full[i]);
                }
            }
        }  
    }
    *pcl_out = pl_surf;
}
bool FastLioProcess::sync_packages(MeasureGroup &meas){
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }//如果缓存队列中没有数据，则返回false
    // ######################################### 这部分容易出现反复读取数据 
    if(!lidar_pushed){
        if (flg_first_scan) last_lio_update_time=lidar_time_buffer.front();
            lidar = lidar_buffer.front();
            lidar_beg_time = lidar_time_buffer.front();
            lidar_end_time = lidar_beg_time + lidar->points.back().curvature / double(1000);
            lidar_pushed = true;
    }
    
    // #########################################
    // 防止IMU数据没有覆盖雷达时间
    if (last_timestamp_imu <= lidar_end_time)
    {   
        return false;
    }

    // 收集imu数据
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    // 防止雷达前面没有imu
    if (lidar_end_time < imu_time)  
    {   
        lidar_time_buffer.pop_front();
        lidar_buffer.pop_front();
        lidar_pushed = false;
        return false;
    }
    meas.imu.clear();
     // 拿出lidar_beg_time到lidar_end_time之间的所有IMU数据
    while ((!imu_buffer.empty()) && (imu_time <= lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    //##########################
    // 是否对点云时间进行处理 （也可以放在反向传播中进行处理）
    //##########################
    meas.lidar = lidar;
    meas.lidar_beg_time = lidar_beg_time;
    meas.lidar_end_time = lidar_end_time;
    meas.last_lio_update_time = last_lio_update_time;
    last_lio_update_time = lidar_end_time;
    lidar_pushed = false;  // 等这一批数据整理完后再 返回初始值
    lidar_buffer.pop_front();
    lidar_time_buffer.pop_front();
    return true;
}
//按下ctrl+c后唤醒所有线程
void SigHandle(int sig)
{  //  会唤醒所有等待队列中阻塞的线程 线程被唤醒后，会通过轮询方式获得锁，获得锁前也一直处理运行状态，不会被再次阻塞。
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}
void FastLioProcess::run(){
    // 用于设置信号处理函数 可以自定义处理（SIGINT） Ctrl + C 中断信号的行为
    signal(SIGINT, SigHandle); 
    //控制循环频率  在后续的代码中，可以使用 rate.sleep() 函数来使程序休眠一段时间，以达到控制循环频率的目的
    ros::Rate rate(5000); 
    bool status = ros::ok(); //ROS 节点是否被关闭、是否接收到关闭信号
    // 保存位姿差
    string save_imu = string(string(ROOT_DIR) + "PCD/imu_pos.txt");
    fs::remove(save_imu);
    std::ofstream fout_imu_pos;
    fout_imu_pos.open(save_imu, ios::app); 

    string save_state = string(string(ROOT_DIR) + "PCD/state_pos.txt");
    fs::remove(save_state);
    std::ofstream fout_state_pos;
    fout_state_pos.open(save_state, ios::app); 

    while (status){
        // 如果监听到信号则推出循环
        if (flg_exit) break; 
        ros::spinOnce(); 
        if(sync_packages(Measures)) // 将这次的雷达数据以及对应的时间区间的imu数据存入Measures中
        {   
            if (flg_first_scan) // 没看出来这一步的意义
            {
                first_lidar_time = Measures.lidar_beg_time;  // 刚开始默认为0
                cout <<"first_lidar_time: " <<first_lidar_time <<endl;
                flg_first_scan = false;
                continue;
            }
            // 这里前向传播和去畸变操作
            processImu(); 
            // 保存前向传播后的位姿
            if(fout_imu_pos.is_open()){
                fout_imu_pos << std::fixed << std::setprecision(3) << state.pos[0] << " " << state.pos[1]
                        << " " << state.pos[2] << " " << state.rot.coeff(0)<< " " << state.rot.coeff(1) << " " << state.rot.coeff(2)<< " " 
                        <<state.rot.coeff(3)<<" " <<state.rot.coeff(4)<<" " <<state.rot.coeff(5)<<" " <<state.rot.coeff(6)<<" "<<state.rot.coeff(7)
                        <<" " <<state.rot.coeff(8)<<endl;
            }

            int pcl_undistort_size = pcl_undistort->points.size();
            if (pcl_undistort->empty() || (pcl_undistort == NULL))
            {   cout<<"pcl_undistort为空"<<endl;
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            pos_lid = state.pos + state.rot * state.offset_T_L_I;
            lasermap_fov_segment(); // 动态调整局部地图

            downSizeFilterSurf.setInputCloud(pcl_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();  //记录滤波后的点云数量
            // 对于第一帧点云开始构建kd树
            if(ikdtree.Root_Node == nullptr){
                if(feats_down_size > 5)
                {   
                    // 设置ikd tree的降采样参数
                    ikdtree.set_downsample_param(filter_size_map_min);  // 0.5
                    feats_down_world->resize(feats_down_size);  //点云对象将包含 feats_down_size 个点。
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    // 组织ikd tree
                    ikdtree.Build(feats_down_world->points); //构建ikd树
                }
                // 是否保存单帧
                if(false){
                    string root_(string(string(ROOT_DIR) + "PCD/split"));
                    string root_org_(string(string(ROOT_DIR) + "PCD/split_org")); 
                    createDirectoryIfNotExists(root_);
                    createDirectoryIfNotExists(root_org_);
                    string all_points_dir(string(string(ROOT_DIR) + "PCD/split/scans_") + to_string(save_i) + string(".pcd"));
                    pcl::PCDWriter pcd_writer;
                    pcl_undistort_world->resize(pcl_undistort_size);
                    for(int i = 0; i < pcl_undistort_size; i++)
                    {
                        pointBodyToWorld(&(pcl_undistort->points[i]), &(pcl_undistort_world->points[i]));
                    }
                    cout << "init current scan saved to " << all_points_dir<<endl;
                    pcd_writer.writeBinary(all_points_dir, *pcl_undistort_world);


                    string all_points_dir_(string(string(ROOT_DIR) + "PCD/split_org/scans_") + to_string(save_i) + string(".pcd"));
                    pcl::PCDWriter pcd_writer_;
                    cout << "init current scan saved to " << all_points_dir_<<endl;
                    pcl_org_world->resize(Measures.lidar->size());
                    for(int i = 0; i < Measures.lidar->size(); i++)
                    {
                        pointBodyToWorld(&(Measures.lidar->points[i]), &(pcl_org_world->points[i]));
                    }
                    pcd_writer_.writeBinary(all_points_dir_, *pcl_org_world);

                    save_i+=1;
                }
                continue;
            }
            // ################################ 这部分没用
            int featsFromMapNum = ikdtree.validnum(); // 获取 树的有效点树
            kdtree_size_st = ikdtree.size();  // 获取树的大小
            // ################################
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            } // 在这里跳过了
            // 重置一些属性  迭代时需要
            normvec->resize(feats_down_size);  // 特诊点匹配上的点存在这里面
            feats_down_world->resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            state_estimation();  // 迭代优化  ####################################################
            // 保存优化后的位姿
            if(fout_state_pos.is_open()){
                fout_state_pos << std::fixed << std::setprecision(3) << state.pos[0] << " " << state.pos[1]
                        << " " << state.pos[2] << " " << state.rot.coeff(0)<< " " << state.rot.coeff(1) << " " << state.rot.coeff(2)<< " " 
                        <<state.rot.coeff(3)<<" " <<state.rot.coeff(4)<<" " <<state.rot.coeff(5)<<" " <<state.rot.coeff(6)<<" "<<state.rot.coeff(7)
                        <<" " <<state.rot.coeff(8)<<endl;
            }
            
            // 点云转换
            pcl_undistort_world->resize(pcl_undistort_size);
            for(int i = 0; i < pcl_undistort_size; i++)
            {   
                pointBodyToWorld(&(pcl_undistort->points[i]), &(pcl_undistort_world->points[i]));
            }
            feats_down_world->resize(feats_down_size);
            for(int i = 0; i < feats_down_size; i++)
            {
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            }
            // 发布数据  
            euler_cur = RotMtoEuler(state.rot);
            geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
            pos_lid = state.pos + state.rot * state.offset_T_L_I; 
            publish_odometry();  // 发布位姿 
            // 发布数据
            publish_frame_world();
            // 在kd树中添加新的点云，并进行降采样 重建树
            map_incremental();
            // 保存数据
            save();
            frame_index++;
        }
        status = ros::ok();
        rate.sleep();
    }

}

// 前向传播和反向传播
void FastLioProcess::processImu(){
    if(Measures.imu.empty()) {
        cout << "imu为空"<<endl;
        return;};  // 拿到的当前帧的imu测量为空，则直接返回
    ROS_ASSERT(Measures.lidar != nullptr);  
    if (imu_need_init_)
    {   
        IMU_init();
        if (init_iter_num > MAX_INI_COUNT){
            imu_need_init_ = false;
            ROS_INFO("IMU Initial Done");
        }
        cov_acc = V3D(acc_cov, acc_cov, acc_cov);
        cov_gyr = V3D(gyr_cov, gyr_cov, gyr_cov);
        cov_bias_gyr = V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov);
        cov_bias_acc =V3D(b_acc_cov, b_acc_cov, b_acc_cov);
        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;  
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;  
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
        return;
    }
    UndistortPcl();
}
void FastLioProcess::IMU_init(){
    V3D cur_acc, cur_gyr;
    if (imu_init_first_frame){
        init_iter_num = 1;
        imu_init_first_frame=false;
        const auto &imu_acc = Measures.imu.front()->linear_acceleration;
        const auto &gyr_acc = Measures.imu.front()->angular_velocity;  
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;  //加速度测量作为初始化均值
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;  //角速度测量作为初始化均值 
    }
    //计算方差
    for (const auto &imu : Measures.imu)  //拿到所有的imu帧
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        //根据当前帧和均值差作为均值的更新
        mean_acc      += (cur_acc - mean_acc) / init_iter_num;
        mean_gyr      += (cur_gyr - mean_gyr) / init_iter_num;
        //.cwiseProduct()对应系数相乘
        // 每次迭代之后均值都会发生变化，最后的方差公式中减的应该是最后的均值
        // https://blog.csdn.net/weixin_44479136/article/details/90510374 方差迭代计算公式
        // 按照博客推导出来的下面方差递推公式有两种
        // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        // cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
        // 上面连个协方差计算没意义
        init_iter_num ++;
    }
    // 设置重力
    state.gravity = - mean_acc / mean_acc.norm() * G_m_s2;
    state.bias_g = mean_gyr;
    last_imu = Measures.imu.back();
}
//判断点的时间是否先后颠倒
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
void FastLioProcess::UndistortPcl(){
    double last_lio_update_time = Measures.last_lio_update_time;
    auto v_imu = Measures.imu;
    v_imu.push_front(last_imu);
    last_imu = v_imu.back();
    // const double &imu_beg_time = v_imu.front()->header.stamp.toSec();  //拿到当前帧头部的imu的时间（也就是上一帧尾部的imu时间戳）
    // const double &imu_end_time = v_imu.back()->header.stamp.toSec();  //拿到当前帧尾部的imu的时间
    double pcl_beg_time = Measures.lidar_beg_time;  // 雷达点云开始的时间戳
    double pcl_end_time = Measures.lidar_end_time;  
    *pcl_undistort = *(Measures.lidar);  // 这是赋值操作
    sort(pcl_undistort->points.begin(), pcl_undistort->points.end(), time_list);
    IMUpose.clear();
    //##############
    double &&offs_t0 = last_lio_update_time - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t0, state.acc_g, state.gyro_imu, state.vel, state.pos, state.rot));   
    // cout << "offs_t0: " << offs_t0 << endl;
    // cout << "state.acc_g: " << state.acc_g << endl;
    // cout << "state.gyro_imu: " << state.gyro_imu << endl;
    // cout << "state.vel: " << state.vel << endl;
    // cout << "state.pos: " << state.pos << endl;
    // cout << "state.rot: " << state.rot << endl;
    //##############
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu; // angvel_avr为平均角速度，acc_avr为平均加速度，acc_imu为imu加速度，vel_imu为imu速度，pos_imu为imu位置
    M3D R_imu;   // imu旋转矩阵
    V3D acc1,acc2 ,angvel1,angvel2;
    double dt=0;
    double offs_t;
    for(int i=0;i < v_imu.size() - 1; i++){
        // cout << "###########################################i: " <<  i << endl;
        auto head = v_imu[i];
        auto tail = v_imu[i + 1];
        if (tail->header.stamp.toSec() < last_lio_update_time) continue;
        // 中值积分
        angvel_avr <<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                    0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                    0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                    0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                    0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm();
        acc1 = {head->linear_acceleration.x,head->linear_acceleration.y,head->linear_acceleration.z};
        acc2 = {tail->linear_acceleration.x,tail->linear_acceleration.y,tail->linear_acceleration.z};
        acc1 = acc1 * G_m_s2 / mean_acc.norm();
        acc2 = acc2 * G_m_s2 / mean_acc.norm();
        angvel1 = {head->angular_velocity.x,head->angular_velocity.y,head->angular_velocity.z};
        angvel2 = {tail->angular_velocity.x,tail->angular_velocity.y,tail->angular_velocity.z};
        // cout << "angvel_avr: " << angvel_avr << endl;
        // cout << "acc_avr: " << acc_avr << endl;
        if(head->header.stamp.toSec() < last_lio_update_time){ 
            // 开始前几帧IMU会有这情况
            dt = tail->header.stamp.toSec() - last_lio_update_time;
            // offs_t = tail->header.stamp.toSec() - last_lio_update_time; 
            
            offs_t = tail->header.stamp.toSec() - pcl_beg_time; 
        // }else{
        //     // 中间帧的情况
        //     dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        //     // offs_t = tail->header.stamp.toSec() - last_lio_update_time;   
        //     offs_t = tail->header.stamp.toSec() - pcl_beg_time;  
        // } 
        }else if(i != v_imu.size() - 2){
            // 中间帧的情况
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
            // offs_t = tail->header.stamp.toSec() - last_lio_update_time;   
            offs_t = tail->header.stamp.toSec() - pcl_beg_time;   
        }else{
            // 最后一帧的情况
            dt = pcl_end_time - head->header.stamp.toSec();
            // offs_t = pcl_end_time - last_lio_update_time;   
            offs_t = pcl_end_time - pcl_beg_time;   
        }
        // 根据dt angvel_avr acc_avr 计算姿态前向传播  
        state.forward_state(acc1, acc2, angvel1,angvel2, dt, Q);
        // state.forward_state(acc1, acc2, angvel_avr, dt, Q);
        // state.forward_state(acc_avr, angvel_avr, dt, Q);
        // state.acc_g = state.rot*acc_avr + state.gravity;  // 这是什么原因导致建图变好 凑巧而已
        IMUpose.push_back(set_pose6d(offs_t, state.acc_g, state.gyro_imu, state.vel, state.pos, state.rot));
        // cout << "bg: " << state.bias_g<< endl;
        // cout << "ba"  << state.bias_a<< endl;
        // cout << "offs_t: " << offs_t << endl;
        // cout << "state.acc_g: " << state.acc_g << endl;
        // cout << "state.gyro_imu: " << state.gyro_imu << endl;
        // cout << "state.vel: " << state.vel << endl;
        // cout << "state.pos: " << state.pos << endl;
        // cout << "state.rot: " << state.rot << endl;
    }

    // cout<< "前向传播位置： " << state.pos <<endl;
    // cout<< "前向传播x旋转： " << state.rot <<endl;
    if (pcl_undistort->points.size() < 1) return;

    // ############################################################
    // 这里添加 对点云offset_time进行处理 对齐last_lio_update_time  也可以在后面的循环中计算
    // for (int _i=0;_i<pcl_undistort->size();_i++){
    //     double point_time = pcl_beg_time + pcl_undistort->points[_i].curvature  / double(1000);
    //     pcl_undistort->points[_i].curvature = (point_time - last_lio_update_time)*double(1000);
    // }
    // ############################################################
    auto it_pcl = pcl_undistort->points.end() - 1; 
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--){
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu<<MAT_FROM_ARRAY(head->rot); //拿到前一帧的IMU旋转矩阵
        vel_imu<<VEC_FROM_ARRAY(head->vel); //拿到前一帧的IMU速度
        pos_imu<<VEC_FROM_ARRAY(head->pos);  //拿到前一帧的IMU位置
        acc_imu<<VEC_FROM_ARRAY(tail->acc);  //拿到后一帧的IMU加速度   与前一帧之间的平均加速度  为了计算两帧之间的点云位姿
        angvel_avr<<VEC_FROM_ARRAY(tail->gyr);  //拿到后一帧的IMU角速度  与前一帧之间的平均角速度
        for(; it_pcl->curvature / double(1000) >= head->offset_time; it_pcl --)  //时间除以1000单位化为s  遍历点云
        {   
            dt = it_pcl->curvature / double(1000) - head->offset_time;
            M3D R_i(R_imu * state.Exp(angvel_avr, dt)); // 该所在时刻imu坐标系的 旋转位姿
            V3D T_i(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt); // 该所在时刻坐imu标系的 位置位姿
            V3D P_l(it_pcl->x, it_pcl->y, it_pcl->z);  //点所在时刻的位置(雷达坐标系下)

            V3D P_w = R_i*(state.offset_R_L_I * P_l + state.offset_T_L_I) + T_i; // 将雷达点 从雷达坐标系转到世界坐标系中
            V3D P_compensate =state.offset_R_L_I.transpose()*(state.rot.transpose()*(P_w - state.pos)-state.offset_T_L_I) ; // 将雷达点 从世界坐标系转到 这帧点云结束时刻的雷达坐标系

            // 将计算结果替换原本的点云
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);
            if (it_pcl == pcl_undistort->points.begin()) break;
        }  
    }
}

void FastLioProcess::lasermap_fov_segment()  
{   //  cube_len 和 MOV_THRESHOLD * DET_RANGE 参数好像设错了， cube_len 应该要是MOV_THRESHOLD * DET_RANGE两倍以上的大小 
    // 这里 cube_len设为 200  ， MOV_THRESHOLD 450  DET_RANGE 1.5  ，好像不合理
    cub_needrm.clear();  // 清空需要移除的区域  两个点会表示一个区域
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    // // X轴分界点转换到w系下，好像没有用到
    // pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    //初始化局部地图包围盒角点，以为w系下lidar位置为中心,得到长宽高200*200*200的局部地图
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true; 
        return;
    }
    // 各个方向上Lidar与局部地图边界的距离，或者说是lidar与立方体盒子六个面的距离
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        // 与某个方向上的边界距离（例如1.5*300m）太小，标记需要移除need_move，参考论文Fig3 MOV_THRESHOLD=1.5写死了  DET_RANGE是配置里的可调参数
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    // 否则需要计算移动的距离
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    // 新的局部地图盒子边界点
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){ //与包围盒最小值边界点距离
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);  // 使用Boxs删除指定盒内的点
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}
//得到被剔除的点
void FastLioProcess::points_cache_collect()
{
    PointVector points_history;  
    ikdtree.acquire_removed_points(points_history);   //返回被剔除的点
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}
void FastLioProcess::state_estimation(){
    StatesGroup state_temp(state); //copy 一份状态
    // 定义变量
    MD(DIM_STATE,DIM_STATE) P_temp;
    VD(DIM_STATE) dx ,dx_new;
    int converged_num = 0;  // 匹配成功的次数
    M3D res_temp_SO3;
    flg_EKF_converged = true;  //  表示是否收敛
    for(int i=0;i<max_iterations;i++){
        // 将协方差copy 一份 ，然后将
        P_temp = state.cov;
        match_points();  // 匹配点云
        // ############################################  计算 H 和 z
        Eigen::MatrixXd Hsub(effct_feat_num, DIM_OBSERVE);
        Eigen::MatrixXd z(effct_feat_num, 1);
        // 定义 迭代时的矩阵
        // 这是H 矩阵前6列，代表 R T  维度为 （n,6）   因为H矩阵只有前6行有值，其他为0   如果添加其他信息则这个维度需要改变
        // H 矩阵乘以各自的协方差， 相当于H^T*R^-1 这里只有6 行 维度为 （6，n） 在这里好像用不上
        for (int num_i = 0; num_i < effct_feat_num; num_i++){
            const PointType &point_l  = pcl_match_body->points[num_i];
            V3D p_l(point_l.x, point_l.y, point_l.z);
            V3D p_i = state.offset_R_L_I * p_l + state.offset_T_L_I;  // 雷达转imu坐标系
            M3D p_i_crossmat;
            p_i_crossmat << SKEW_SYM_MATRX(p_i);
            const PointType &norm_p = corr_normvect->points[num_i]; 
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
            V3D A(p_i_crossmat*state.rot.transpose() *norm_vec);  // 残差对转台误差的旋转部分（李代数）的偏导
            // 这里使用了公式 -p1T*p^ =(p^p1)T=-(p1^p)T  因此和论文中不一致 
            // 设 V3D a = (norm_vec.transpose()*s.rot)T        V3D p = p_i
            // -aT*p^ = ((-aT*p^)T)T = (-(p^)T*a)T = (p^*a)T  = VEC_FROM_ARRAY(p^*a)
            // VD A(1,3)=-norm_vec.transpose()*s.rot*p_i_crossmat;  // 这是原公式  后面可以验证对比
            Hsub.row(num_i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
            z(num_i) = -norm_p.intensity;  //  每个点误差的负数，为 负的点到平面的距离
        }
        // ############################################
        if(!match_valid) continue; // 如果匹配不成功，则不优化
        
        // 计算 dx
        dx = state_temp - state; 
        dx_new = dx; // dx_new用于计算  J^-1*dx
        // ########################################### 这部分可以不要
        // 计算 P_temp  
        // 计算 p = (J)-1*P*(J)-T
        // 目前只有 一个旋转状态，
        /*   A 为右雅可比矩阵的转置  那么 AT 为右雅可比矩阵  这是论文的定义
        雅可比矩阵为J    	A^-T	    0
                            0	   I
        雅可比矩阵的逆J-1   AT		0
                            0	   I
        可以轻松推导
        */
        for(auto it=state.so3_indexes.begin(); it!=state.so3_indexes.end();++it){
            int index =  *it;
            V3D vec = dx.block<3,1>(index,0);
            res_temp_SO3 = A_matrix(vec);  //计算李代数的右雅可比矩阵
            dx_new.block<3,1>(index,0) =  res_temp_SO3 * dx_new.block<3, 1>(index, 0);  // J^-1*dx
            for(int col;col<DIM_STATE;++col){
                P_temp.block<3, 1>(index, col) = res_temp_SO3 * P_temp.block<3, 1>(index, col);  // 这里计算J^-1*P
            }
            for(int row;row<DIM_STATE;++row){
                P_temp.block<1, 3>(row, index) = P_temp.block<1, 3>(row, index) *res_temp_SO3.transpose();  // 这里计算P*J^-T
            }
        }
        // ###########################################
        // 计算 K 矩阵 K = (P^-1 + H^T * R^-1 * H)^-1 * H^T * R^-1
        // 当 R 为对角矩阵 且每个值都一样时  R = I*r
        // 则 K = ( (P^-1)/r + H^T *  H )^-1 * H^T
        // MD(DIM_STATE,effct_feat_num) K;  //K矩阵  维度是 (v,n)  (18, n)
        MD(DIM_STATE, DIM_STATE) HTH;
        HTH.setZero();
        HTH.block<DIM_OBSERVE, DIM_OBSERVE>(0, 0) = Hsub.transpose()*Hsub;
        // K_1 = (P^-1)/r + H^T *  H )^-1   是 K 的一部分
        MD(DIM_STATE,DIM_STATE) K_1 = ((P_temp/R).inverse()+HTH).inverse();  // (18,18)

        VD(DIM_STATE) HTz;  // (18,1)  只有前6个有值，其他都为0
        HTz.setZero();
        HTz.block<DIM_OBSERVE, 1>(0, 0)= Hsub.transpose()*z;  // H^T * z 
        VD(DIM_STATE) Kz;
        Kz.setZero();
        Kz = K_1.block<DIM_STATE, DIM_OBSERVE>(0, 0) * HTz.block<DIM_OBSERVE, 1>(0, 0);
        // Kz = K_1 * HTz;

        MD(DIM_STATE, DIM_STATE) KH;
        KH.setZero();
        KH.block<DIM_STATE, DIM_OBSERVE>(0, 0) = K_1.block<DIM_STATE, DIM_OBSERVE>(0, 0) * HTH.block<DIM_OBSERVE, DIM_OBSERVE>(0, 0);  // K * H

        dx = Kz + (KH- MD(DIM_STATE, DIM_STATE)::Identity())*dx_new;

        state += dx;
        
        flg_EKF_converged = true;  //  表示是否收敛
        // #####################################################
        // fast_lio2 的判断收敛逻辑
        for(int i = 0; i < DIM_STATE ; i++)
        {
            if(std::fabs(dx[i]) > limit[i])  // limit 里有23个值，全是 0.001
            {
                flg_EKF_converged = false;  // 表示收敛
                break;
            }
        }

        if(flg_EKF_converged) converged_num++; // 表示收敛的次数
        if(!converged_num && i == max_iterations - 2)  // 如果前面迭代中收敛的次数为0，且在最后第二次迭代了，就判这次收敛
        {
            flg_EKF_converged = true;
        }
        // #####################################################

        auto rot_add = dx.block<3, 1>(0, 0);
        auto t_add = dx.block<3, 1>(3, 0);
        // if ((rot_add.norm() * 57.3 > 0.01) || (t_add.norm() * 100 > 0.015)) { flg_EKF_converged = false; }  // 判断是否收敛，
        // if (flg_EKF_converged || ((converged_num == 0) && (i == (max_iterations - 2)))) {
        //     converged_num++; 
        //     flg_EKF_converged =true;
        // } // 收敛或者刚开始匹配或在最后第2次迭代中， rematch_num ++  ？？？？
        // #####################################################

        // if(!converged_num && i == max_iterations - 2)  // 如果前面迭代中未收敛的次数为0，且在最后第二次迭代了， 是为了重新匹配点云
        // {
        //     flg_EKF_converged = true;
        // }

        if((converged_num>1) || i == (max_iterations - 1)){
            P_temp = (MD(DIM_STATE, DIM_STATE)::Identity()-KH)*P_temp;
            // ########################################### 这部分可以不要
            // 计算 p = (J)-1*P*(J)-T
            for(auto it=state.so3_indexes.begin(); it!=state.so3_indexes.end();++it){
                int index =  *it;
                res_temp_SO3 = A_matrix(dx.block<3,1>(index,0));  //计算李代数的右雅可比矩阵
                for(int col;col<DIM_STATE;++col){
                    P_temp.block<3, 1>(index, col) = res_temp_SO3 * P_temp.block<3, 1>(index, col);  // 这里计算J^-1*P
                }
                for(int row;row<DIM_STATE;++row){
                    P_temp.block<1, 3>(row, index) = P_temp.block<1, 3>(row, index) *res_temp_SO3.transpose();  // 这里计算P*J^-T
                }
            }
            // ###########################################
            state.cov = P_temp;
            break;
        }

    }
}

void FastLioProcess::match_points(){

    effct_feat_num = 0;
    double total_residual = 0.0;  // 总误差 
    double res_mean_last = 0.0;
    #ifdef MP_EN  // 对后续的循环使用多线程
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for(int i = 0; i < feats_down_size; i++){
        PointType &point_world = feats_down_world->points[i]; 
        PointType &point_body  = feats_down_body->points[i]; 
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(state.rot * (state.offset_R_L_I*p_body + state.offset_T_L_I) + state.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);  // 用来存储 i点 匹配到的五个点的距离
        auto &points_near = Nearest_Points[i]; // 用来存储匹配到的五个点
        if (flg_EKF_converged)  // 如果迭代是发散的，就再重新匹配五个点
        {   
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);  // 搜索五个点
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }
        if (!point_selected_surf[i]) continue;
        VF(4) pabcd; // 为平面方程的参数，前三个代表法向量
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))  // 判断是否为平面
        {   
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()); // fabs(pd2) / sqrt(p_body.norm()) 点到平面距离与点到雷达距离的比
            if (s > 0.9) // 说明点到面的距离 小于 点到雷达距离的 1/9  就说明点在平面上
            // if (true)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;  // 这是点与平面的距离平方
                res_last[i] = abs(pd2); 
            }
        }
    }
    // 这部分可以写在上面的循环中
    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {   
            pcl_match_body->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }
    // cout << "匹配点数：" << effct_feat_num<<endl;
    if (effct_feat_num < 1)  // 匹配到的点与相应点是同一平面 的统计数
    {
        match_valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }else{
        match_valid = true;
    }
    
    res_mean_last = total_residual / effct_feat_num;  // 可以说时误差      
}

void FastLioProcess::pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot * (state.offset_R_L_I*p_body + state.offset_T_L_I) + state.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
    // po->curvature = pi->curvature;
}


template<typename T>
void FastLioProcess::set_posestamp(T & out)
{   
    out.pose.position.x =  state.pos(0);
    out.pose.position.y =  state.pos(1);
    out.pose.position.z =  state.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

//发布里程计  发布位姿
void FastLioProcess::publish_odometry()
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped); // 在topic上发布消息， 其他节点可以依据topic来接受消息
    
    auto P = state.cov;
    for (int i = 0; i < 6; i ++)
    {
        // int k = i < 3 ? i + 3 : i - 3;
        int k = i;
        // odomAftMapped.pose.covariance 格式是 rot pos 顺序的协方差
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 2);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 5);
    }
    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);

    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) ); //发布tf变换
    // 将位姿发布到 tf 系统中，其他节点能够调用，比如rviz建图调用
}

//每隔10个发布一下位姿  发布路线
void FastLioProcess::publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}
// 原始保存
void FastLioProcess::publish_frame_world()
{   
    // int size = feats_down_world->size();
    // PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1)); 
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI()); 
    if(dense_pub_en){
        laserCloudWorld = pcl_undistort_world;
    }else{
        laserCloudWorld = feats_down_world;
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg); //转换到ROS消息格式,并直接发布信息
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
}

void FastLioProcess::save(){

    PointCloudXYZI::Ptr pcl_save(new PointCloudXYZI());
    *pcl_save = *pcl_undistort_world;
    PointType _path;
    _path.x = state.pos[0];
    _path.y = state.pos[1];
    _path.z = state.pos[2];
    _path.intensity = 200;
    pcl_save->push_back(_path);
    if (fout_cloud.is_open()) {
        for (auto p : pcl_save->points) {
            fout_cloud << std::fixed << std::setprecision(2) << p.x << " " << p.y
                    << " " << p.z << " " << p.intensity << endl;
        }
    }

    // 保存单帧点云 为pcd
    string root_(string(string(ROOT_DIR) + "PCD/split"));
    createDirectoryIfNotExists(root_);
    string all_points_dir(string(string(ROOT_DIR) + "PCD/split/scans_") + to_string(frame_index) + string(".pcd"));
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pcl_undistort_world);
    cout << "单帧保存路径：" << all_points_dir << endl;
}


void FastLioProcess::map_incremental()  
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size); // 点与相应的近邻点不在一个方体内 ，则该点存入这里
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        // pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i])); // 在外面转换过了，这里是多余的
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && (!flg_first_scan))
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;  // 默认该点添加到地图中
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;  // 网格中心点，
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);  // 该点与方体中心点的距离
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); // 不在一个方体内 第0个近邻点在所有近邻点中是最远的点
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false; // 如果该点存在有近邻点(地图上)与方体中心点的距离比 该点与方体中心点的距离小 则该点不需要。 说明该方体内已经密集了。
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]); 
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);  // 在地图中添加这些点（进行降采样） 并检验树是否稳定进而重建地图
    ikdtree.Add_Points(PointNoNeedDownsample, false);  // 在地图中加入这些点， 不重建地图
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

// 用于创建文件夹
bool createDirectoryIfNotExists(const std::string& path) {
    if (fs::exists(path)) {
        if (fs::is_directory(path)) {
            std::cout << "文件夹已存在: " << path << std::endl;
            return true;
        } else {
            std::cerr << "错误: " << path << " 存在但不是文件夹" << std::endl;
            return false;
        }
    } else {
        try {
            if (fs::create_directories(path)) {
                std::cout << "成功创建文件夹: " << path << std::endl;
                return true;
            } else {
                std::cerr << "错误: 创建文件夹失败: " << path << std::endl;
                return false;
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "文件系统错误: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << std::endl;
            return false;
        }
    }
    return false;
}
// 右雅可比矩阵计算 
M3D A_matrix(const V3D & v){
    M3D res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);
	if(norm < tolerance<double>()){
		res = M3D::Identity();
	}
	else{
		res = M3D::Identity() + (1 - std::cos(norm)) / squaredNorm * hat(v) + (1 - std::sin(norm) / norm) / squaredNorm * hat(v) * hat(v);
	}
    return res;
}


M3D  hat(const V3D& v) {
    M3D res;
	res << 0, -v[2], v[1],
		v[2], 0, -v[0],
		-v[1], v[0], 0;
	return res;
}
// 计算点的距离
float calc_dist(PointType p1, PointType p2){
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}