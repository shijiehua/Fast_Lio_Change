#include <state.h>
#include <math.h>


StatesGroup::StatesGroup(){
    rot = M3D::Identity();
    pos= V3D::Zero();
    vel= V3D::Zero();
    bias_g = V3D::Zero();
    bias_a = V3D::Zero();
    gravity = V3D::Zero();
    //#
    // cov = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() * INIT_COV;
    // cov(6, 6) = 0.00001;
    // cov.block<9, 9>(9, 9) = MD(9, 9)::Identity() * 0.00001;
    // fast_lio2 初始化 协方差
    cov.setIdentity();
    cov(9,9) = cov(10,10) = cov(11,11) = 0.0001;  //欧拉角坐标系与imu坐标系之间的旋转差 协方差
    cov(12,12) = cov(13,13) = cov(14,14) = 0.001; 
    cov(15,15) = cov(16,16) = cov(17,17) = 0.00001; 

    offset_R_L_I = M3D::Identity();
    offset_T_L_I = V3D::Zero();
}
StatesGroup::StatesGroup(const StatesGroup &b) 
{
this->rot = b.rot;
this->pos = b.pos;
this->vel = b.vel;
this->bias_g = b.bias_g;
this->bias_a = b.bias_a;
this->gravity = b.gravity;
this->cov = b.cov;

// 其他参数
this->offset_R_L_I=b.offset_R_L_I;
this->offset_T_L_I=b.offset_T_L_I;
this->so3_indexes=b.so3_indexes;

};

StatesGroup::~StatesGroup(){}; // 析构函数未使用
void StatesGroup::forward_state(V3D acc1,V3D acc2,V3D gyro1,V3D gyro2,double dt,Matrix<double, DIM_Q, DIM_Q>  Q){
    // 直接硬性计算
    acc1 -=bias_a;  // 减去偏置项
    acc2 -=bias_a; 
    gyro1-=bias_g;
    gyro2-=bias_g;
    // V3D gyro = (gyro1+gyro2)/2;
    V3D gyro = (gyro1 + Exp(gyro1, dt/2)*Exp(gyro2, dt/2)* gyro2)/2;
    M3D Exp_f = Exp(gyro, dt);  // 旋转增量
    M3D acc_avr_skew;
    // acc_avr_skew<< SKEW_SYM_MATRX(acc1); // 转为反对称矩阵
    
    // 计算协方差  如果中间加了其他变量，则会导致协防擦维度发生变化，以及协方差矩阵格式会变化
    F_x.setIdentity(); // 真实状态误差（i+1）对 真实状态误差（i）的偏导
    cov_w.setZero();  // 这是 F_w*Q*f_W  直接省略了F_w 矩阵
    V3D acc = (acc1 + Exp_f*acc2)/2;
    acc_avr_skew<< SKEW_SYM_MATRX(acc); // 转为反对称矩阵
    
    F_w.setZero();  
    F_x.block<3, 3>(0, 0) = Exp(gyro, -dt);  // 真实旋转误差（i+1）对 真实旋转误差（i）的偏导
    F_x.block<3, 3>(0, 9) = -Eye3d * dt;    // 真实旋转误差（i+1）对 真实速度误差（i）的偏导
    F_x.block<3, 3>(3, 6) = Eye3d * dt;     // 真实位置误差（i+1）对 真实速度误差（i）的偏导
    F_x.block<3, 3>(6, 0) = -rot * acc_avr_skew * dt;  // 真实速度误差（i+1）对 真实旋转误差（i）的偏导
    F_x.block<3, 3>(6, 12) = -rot * dt;  // 真实速度误差（i+1）对 真实加速度偏置误差（i）的偏导
    F_x.block<3, 3>(6, 15) = Eye3d * dt; // 真实速度误差（i+1）对 真实重力加速度误差（i）的偏导

    //####################################
    // F_w.block<3, 3>(0, 0) = - Eye3d * dt;   // 真实旋转误差（i+1）对 真实角速度测量误差（i）的偏导
    // F_w.block<3, 3>(6, 3) = - rot * dt;   // 真实速度误差（i+1）对 真实加速度测量误差（i）的偏导
    // F_w.block<3, 3>(9, 6) = Eye3d * dt;   // 真实角速度偏置误差（i+1）对 真实角速度偏置测量误差（i）的偏导
    // F_w.block<3, 3>(12, 9) = Eye3d * dt;   // 真实加速度偏置误差（i+1）对 真实加速度偏置测量误差（i）的偏导
    
    // cov = F_x * cov * F_x.transpose() + F_w * Q * F_w.transpose();
    //####################################
    // 直接求解 F_w*Q*f_W
    V3D cov_gyr(Q(0,0),Q(1,1),Q(2,2)); 
    V3D cov_acc(Q(3,3),Q(4,4),Q(5,5)); 
    V3D cov_bias_gyr(Q(6,6),Q(7,7),Q(8,8)); 
    V3D cov_bias_acc(Q(9,9),Q(10,10),Q(11,11)); 
    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;  
    cov_w.block<3, 3>(6, 6) = rot * cov_acc.asDiagonal() * rot.transpose() * dt * dt; 
    cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt; 
    cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt; 

    cov = F_x * cov * F_x.transpose() + cov_w;
    //####################################

    // #######################################
    // 前向传播  这个计算循序有讲究
    V3D acc_g1 = rot * acc1 + gravity;  // 全局坐标系下的加速度  发现提前计算效果更好
    rot = rot * Exp_f; // 旋转前向传播
    V3D acc_g2 = rot * acc2 + gravity;  // 全局坐标系下的加速度
    acc_g = (acc_g1 + acc_g2) / 2;
    pos = pos + vel * dt + 0.5 * acc_g * dt * dt;  // 位置
    // pos = pos + vel * dt;  // 位置
    vel = vel + acc_g * dt;  // 速度值跟新
    gyro_imu = gyro;
    // acc_g = rot * acc + gravity;
    // #######################################

    return;
};
void StatesGroup::forward_state(V3D acc1,V3D acc2,V3D gyro,double dt,Matrix<double, DIM_Q, DIM_Q>  Q){
    // 直接硬性计算
    acc1 -=bias_a;  // 减去偏置项
    acc2 -=bias_a; 
    gyro-=bias_g;
    M3D Exp_f = Exp(gyro, dt);  // 旋转增量
    
    
    
    // 计算协方差  如果中间加了其他变量，则会导致协防擦维度发生变化，以及协方差矩阵格式会变化
    F_x.setIdentity(); // 真实状态误差（i+1）对 真实状态误差（i）的偏导
    cov_w.setZero();  // 这是 F_w*Q*f_W  直接省略了F_w 矩阵
    M3D acc_avr_skew;
    V3D acc = (acc1 + Exp_f*acc2)/2;
    acc_avr_skew<< SKEW_SYM_MATRX(acc); // 转为反对称矩阵
  
    F_w.setZero();  
    F_x.block<3, 3>(0, 0) = Exp(gyro, -dt);  // 真实旋转误差（i+1）对 真实旋转误差（i）的偏导
    F_x.block<3, 3>(0, 9) = -Eye3d * dt;    // 真实旋转误差（i+1）对 真实速度误差（i）的偏导
    F_x.block<3, 3>(3, 6) = Eye3d * dt;     // 真实位置误差（i+1）对 真实速度误差（i）的偏导
    F_x.block<3, 3>(6, 0) = -rot * acc_avr_skew * dt;  // 真实速度误差（i+1）对 真实旋转误差（i）的偏导
    F_x.block<3, 3>(6, 12) = -rot * dt;  // 真实速度误差（i+1）对 真实加速度偏置误差（i）的偏导
    F_x.block<3, 3>(6, 15) = Eye3d * dt; // 真实速度误差（i+1）对 真实重力加速度误差（i）的偏导

    //####################################
    // F_w.block<3, 3>(0, 0) = - Eye3d * dt;   // 真实旋转误差（i+1）对 真实角速度测量误差（i）的偏导
    // F_w.block<3, 3>(6, 3) = - rot * dt;   // 真实速度误差（i+1）对 真实加速度测量误差（i）的偏导
    // F_w.block<3, 3>(9, 6) = Eye3d * dt;   // 真实角速度偏置误差（i+1）对 真实角速度偏置测量误差（i）的偏导
    // F_w.block<3, 3>(12, 9) = Eye3d * dt;   // 真实加速度偏置误差（i+1）对 真实加速度偏置测量误差（i）的偏导
    
    // cov = F_x * cov * F_x.transpose() + F_w * Q * F_w.transpose();
    //####################################
    // 直接求解 F_w*Q*f_W
    V3D cov_gyr(Q(0,0),Q(1,1),Q(2,2)); 
    V3D cov_acc(Q(3,3),Q(4,4),Q(5,5)); 
    V3D cov_bias_gyr(Q(6,6),Q(7,7),Q(8,8)); 
    V3D cov_bias_acc(Q(9,9),Q(10,10),Q(11,11)); 
    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;  
    cov_w.block<3, 3>(6, 6) = rot * cov_acc.asDiagonal() * rot.transpose() * dt * dt; 
    cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt; 
    cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt; 

    cov = F_x * cov * F_x.transpose() + cov_w;
    //####################################
    // ####################################### 这个放在上面计算效果更好 ？？？？？
    // 前向传播  这个计算循序有讲究
    V3D acc_g1 = rot * acc1 + gravity;  // 全局坐标系下的加速度  发现提前计算效果更好
    rot = rot * Exp_f; // 旋转前向传播
    V3D acc_g2 = rot * acc2 + gravity;  // 全局坐标系下的加速度
    acc_g = (acc_g1 + acc_g2) / 2;
    pos = pos + vel * dt + 0.5 * acc_g * dt * dt;  // 位置
    // pos = pos + vel * dt;  // 位置
    vel = vel + acc_g * dt;  // 速度值跟新
    gyro_imu = gyro;
    // acc_g = rot * acc + gravity;
    // #######################################
    

    return;
};
void StatesGroup::forward_state(V3D acc,V3D gyro,double dt,Matrix<double, DIM_Q, DIM_Q>  Q){
    // 直接硬性计算
    acc -=bias_a;  // 减去偏置项
    gyro-=bias_g;
    M3D Exp_f = Exp(gyro, dt);  // 旋转增量
    M3D acc_avr_skew;
    acc_avr_skew<< SKEW_SYM_MATRX(acc); // 转为反对称矩阵
    
    
    // 计算协方差  如果中间加了其他变量，则会导致协防擦维度发生变化，以及协方差矩阵格式会变化
    F_x.setIdentity(); // 真实状态误差（i+1）对 真实状态误差（i）的偏导
    cov_w.setZero();  // 这是 F_w*Q*f_W  直接省略了F_w 矩阵

    
    F_w.setZero();  
    F_x.block<3, 3>(0, 0) = Exp(gyro, -dt);  // 真实旋转误差（i+1）对 真实旋转误差（i）的偏导
    F_x.block<3, 3>(0, 9) = -Eye3d * dt;    // 真实旋转误差（i+1）对 真实速度误差（i）的偏导
    F_x.block<3, 3>(3, 6) = Eye3d * dt;     // 真实位置误差（i+1）对 真实速度误差（i）的偏导
    F_x.block<3, 3>(6, 0) = -rot * acc_avr_skew * dt;  // 真实速度误差（i+1）对 真实旋转误差（i）的偏导
    F_x.block<3, 3>(6, 12) = -rot * dt;  // 真实速度误差（i+1）对 真实加速度偏置误差（i）的偏导
    F_x.block<3, 3>(6, 15) = Eye3d * dt; // 真实速度误差（i+1）对 真实重力加速度误差（i）的偏导

    //####################################
    // F_w.block<3, 3>(0, 0) = - Eye3d * dt;   // 真实旋转误差（i+1）对 真实角速度测量误差（i）的偏导
    // F_w.block<3, 3>(6, 3) = - rot * dt;   // 真实速度误差（i+1）对 真实加速度测量误差（i）的偏导
    // F_w.block<3, 3>(9, 6) = Eye3d * dt;   // 真实角速度偏置误差（i+1）对 真实角速度偏置测量误差（i）的偏导
    // F_w.block<3, 3>(12, 9) = Eye3d * dt;   // 真实加速度偏置误差（i+1）对 真实加速度偏置测量误差（i）的偏导
    
    // cov = F_x * cov * F_x.transpose() + F_w * Q * F_w.transpose();
    //####################################
    // 直接求解 F_w*Q*f_W
    V3D cov_gyr(Q(0,0),Q(1,1),Q(2,2)); 
    V3D cov_acc(Q(3,3),Q(4,4),Q(5,5)); 
    V3D cov_bias_gyr(Q(6,6),Q(7,7),Q(8,8)); 
    V3D cov_bias_acc(Q(9,9),Q(10,10),Q(11,11)); 
    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;  
    cov_w.block<3, 3>(6, 6) = rot * cov_acc.asDiagonal() * rot.transpose() * dt * dt; 
    cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt; 
    cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt; 

    cov = F_x * cov * F_x.transpose() + cov_w;
    //####################################
    // ####################################### 这个放在上面计算效果更好 ？？？？？
    // 前向传播  这个计算循序有讲究
    acc_g = rot * acc + gravity;  // 全局坐标系下的加速度  发现提前计算效果更好
    rot = rot * Exp_f; // 旋转前向传播
    // acc_g = rot * acc + gravity;  // 全局坐标系下的加速度
    pos = pos + vel * dt + 0.5 * acc_g * dt * dt;  // 位置
    // pos = pos + vel * dt;  // 位置
    vel = vel + acc_g * dt;  // 速度值跟新
    gyro_imu = gyro;
    // acc_g = rot * acc + gravity;
    // #######################################
    

    return;
};

Eigen::Matrix<double, 3, 3> StatesGroup::Exp(const double &v1, const double &v2, const double &v3)
{
  double &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<double, 3, 3> Eye3 = Eigen::Matrix<double, 3, 3>::Identity();
  if (norm > 0.00001)
  {
    double r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<double, 3, 3> K;
    K << SKEW_SYM_MATRX(r_ang);

    /// Roderigous Tranformation
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  }
  else { return Eye3; }
}
Eigen::Matrix<double, 3, 3> StatesGroup::Exp(const Eigen::Matrix<double, 3, 1> &ang_vel, const double &dt)
{
  double ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<double, 3, 3> Eye3 = Eigen::Matrix<double, 3, 3>::Identity();

  if (ang_vel_norm > 0.0000001)
  {
    Eigen::Matrix<double, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<double, 3, 3> K;

    K << SKEW_SYM_MATRX(r_axis);

    double r_ang = ang_vel_norm * dt;

    /// Roderigous Tranformation
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }
  else { return Eye3; }
}
Eigen::Matrix<double, 3, 3> StatesGroup::Exp(const Eigen::Matrix<double, 3, 1> &&ang)
{
  double ang_norm = ang.norm();
  Eigen::Matrix<double, 3, 3> Eye3 = Eigen::Matrix<double, 3, 3>::Identity();
  if (ang_norm > 0.0000001)
  {
    Eigen::Matrix<double, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<double, 3, 3> K;
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  else { return Eye3; }
}

StatesGroup& StatesGroup::operator=(const StatesGroup &b)
{
this->rot = b.rot;
this->pos = b.pos;
this->vel = b.vel;
this->bias_g = b.bias_g;
this->bias_a = b.bias_a;
this->gravity = b.gravity;
this->cov = b.cov;

// 其他参数
this->offset_R_L_I=b.offset_R_L_I;
this->offset_T_L_I=b.offset_T_L_I;
this->so3_indexes=b.so3_indexes;

return *this;
};


StatesGroup StatesGroup::operator+(const Matrix<double, DIM_STATE, 1> &state_add)
{
StatesGroup a;  // 加法为啥要另创一个对象
a.rot = this->rot * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
a.pos = this->pos+ state_add.block<3, 1>(3, 0);
a.vel = this->vel + state_add.block<3, 1>(6, 0);
a.bias_g = this->bias_g + state_add.block<3, 1>(9, 0);
a.bias_a = this->bias_a + state_add.block<3, 1>(12, 0);
a.gravity = this->gravity + state_add.block<3, 1>(15, 0);
a.cov = this->cov;

// 其他参数
a.offset_R_L_I = this->offset_R_L_I;
a.offset_T_L_I = this->offset_T_L_I;
a.so3_indexes = this->so3_indexes;

return a;
};


StatesGroup& StatesGroup::operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
{
this->rot= this->rot * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
this->pos += state_add.block<3, 1>(3, 0);
this->vel += state_add.block<3, 1>(6, 0);
this->bias_g += state_add.block<3, 1>(9, 0);
this->bias_a += state_add.block<3, 1>(12, 0);
this->gravity += state_add.block<3, 1>(15, 0);
return *this;
};

Matrix<double, DIM_STATE, 1> StatesGroup::operator-(const StatesGroup &b)
{
Matrix<double, DIM_STATE, 1> a;
M3D rotd(b.rot.transpose() * this->rot);
a.block<3, 1>(0, 0) = Log(rotd);
a.block<3, 1>(3, 0) = this->pos - b.pos;
a.block<3, 1>(6, 0) = this->vel - b.vel;
a.block<3, 1>(9, 0) = this->bias_g - b.bias_g;
a.block<3, 1>(12, 0) = this->bias_a - b.bias_a;
a.block<3, 1>(15, 0) = this->gravity - b.gravity;
return a;
};


