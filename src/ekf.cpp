#include <ekf.h>
#include <cmath>
#include <iostream>

using namespace std;

ekf::ekf():viewer_thread_(&viewer::run, &Viewer_){
    enu_initialized_ = false;
    X_.setZero();
    P_.setZero();
    P_.diagonal() << 0.01, 0.01, 1e6;
    Q_.setZero();
    Q_.diagonal() << 1., 1., 2.;
    R_.setZero();
    R_.diagonal() << 0.8, 0.8, 1.5;
    I_ = Matrix3d::Identity();
    H_ = Matrix3d::Identity();
    F_.setZero();
    F_.diagonal() << 1., 1., 1.;
    
  
}
ekf::~ekf(){
    Viewer_.running_ = false;
    if (viewer_thread_.joinable()) {
        viewer_thread_.join(); // 等待线程结束
    }
}

void ekf::propagate(double v, double w, double t){
    //是否进行初始化
    if (!initialized_)
    {
        time_stamp_ = t;
        initialized_ = true;
        return;
    }
    // 时间戳对齐
    double delta_t = t - time_stamp_;
    if (delta_t < 1e-3) return;
    //更新状态时间戳
    time_stamp_ = t;
    // 计算雅可比矩阵F_
    F_(0, 2) = -v * delta_t * sin(X_[2]);
    F_(1, 2) =  v * delta_t * cos(X_[2]);

    // 计算先验状态
    Vector3d delta_X;
    delta_X[0] = v * delta_t * cos(X_[2]);
    delta_X[1] = v * delta_t * sin(X_[2]);
    delta_X[2] = w * delta_t;
    X_ += delta_X;
    Eigen::Vector3d p(X_[0], X_[1], 0.);
    Viewer_.AddGNSSOdom(p);

    // 将偏航角限制在[-pi,pi]
    while (abs(delta_X[2]) > M_PI)
    {
        if (delta_X[2] > 0) delta_X[2] -= 2* M_PI;
        
        else delta_X[2] += 2* M_PI;
    }
    // 更新预测协方差矩阵
    P_ = F_.eval() * P_ * F_.transpose() + Q_;
    P_ = P_.eval().selfadjointView<Eigen::Upper>();
    return;
}

void ekf::update(double x, double y, double theta, double t){
    // 时间戳对齐
    double delta_t = t - time_stamp_;
    if (delta_t < 1e-3) return;

    // 更新状态时间戳
    time_stamp_ = t;

    Vector3d Z(x, y, theta);
    
    // 计算卡尔曼增益
    Matrix3d A = P_ * H_.transpose();
    Matrix3d B = H_ * P_ * H_.transpose() + R_;
    Matrix3d K = A.eval() * B.eval().inverse();
    // 计算后验状态
    Vector3d delta_X = K * (Z - H_ * X_);
    X_ += delta_X;
    // 计算后验协方差
    P_ = (I_ - K * H_) * P_;
    cout<<"p"<<endl<<P_<<endl;
    // Eigen::Vector3d p(X_[0], X_[1], 0.);
    // Viewer_.AddGNSSOdom(p);
    return;
}
