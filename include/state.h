#ifndef STATE_H
#define STATE_H
#include <common_lib.h>
#define DIM_STATE (18)   // 状态维度
#define DIM_Q (12)   // 测量误差维度
#define DIM_OBSERVE (6)   // 测量误差维度
#define INIT_COV (0.01)  // 初始协方差


struct imu_input
{   
    V3D acc;
    V3D gyro;
};


class StatesGroup{
public:
    StatesGroup();
    StatesGroup(const StatesGroup &b);
    ~StatesGroup();
    M3D rot;                             
    V3D pos;                             
    V3D vel;                             
    V3D bias_g;                              
    V3D bias_a;                             
    V3D gravity;

    // 协方差
    MD(DIM_STATE,DIM_STATE) cov;

    Eigen::Matrix<double, DIM_STATE, 1>get_f(V3D acc,V3D gyro);
    void forward_state(V3D acc,V3D gyro,double dt,Matrix<double, DIM_Q, DIM_Q>  Q);
    void forward_state(V3D acc1,V3D acc2,V3D gyro,double dt,Matrix<double, DIM_Q, DIM_Q>  Q);
    void forward_state(V3D acc1,V3D acc2,V3D gyro1,V3D gyro2,double dt,Matrix<double, DIM_Q, DIM_Q>  Q);
    
    // 状态计算工具
    Eigen::Matrix<double, 3, 3> Exp(const double &v1, const double &v2, const double &v3);
    Eigen::Matrix<double, 3, 3> Exp(const Eigen::Matrix<double, 3, 1> &ang_vel, const double &dt);
    Eigen::Matrix<double, 3, 3> Exp(const Eigen::Matrix<double, 3, 1> &&ang);

    // 状态加减计算 
    StatesGroup &operator=(const StatesGroup &b);
    StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add);
    StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add);
    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b);

    

    V3D acc_g, gyro_imu;
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> F_x, cov_w;
    Eigen::Matrix<double, DIM_STATE, DIM_Q> F_w;
    M3D Eye3d = M3D::Identity();

    // 外参
    M3D offset_R_L_I;
    V3D offset_T_L_I;

    // 待优化的李代数起始位置 
    vector<int> so3_indexes={0};
};

//######################################################工具

// 右雅可比矩阵计算 计算公式可以推导出来（没有约等于）
template<class scalar> inline scalar tolerance();

template<> inline float tolerance<float >() { return 1e-5f; }
template<> inline double tolerance<double>() { return 1e-11; }

M3D A_matrix(const V3D & v);
M3D  hat(const V3D& v);  // 计算反对称
// template<typename Base>
// Eigen::Matrix<typename Base::scalar, 3, 3> A_matrix(const Base & v){
//     Eigen::Matrix<typename Base::scalar, 3, 3> res;
//     double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
// 	double norm = std::sqrt(squaredNorm);
// 	if(norm < tolerance<typename Base::scalar>()){
// 		res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity();
// 	}
// 	else{
// 		res = Eigen::Matrix<typename Base::scalar, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * hat(v) + (1 - std::sin(norm) / norm) / squaredNorm * hat(v) * hat(v);
// 	}
//     return res;
// }


template <typename T> Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

#endif