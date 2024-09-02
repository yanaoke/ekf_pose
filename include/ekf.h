#pragma once

#include <Eigen/Dense>
#include <viewer.h>
#include <thread>
using namespace Eigen;
typedef Matrix<double, 5, 1> Vector5d;

class ekf
{
private:
    /* data */
    Vector3d X_;
    double time_stamp_;
    bool initialized_;
    Matrix3d P_, Q_, R_, F_, H_, I_;

public:

    bool enu_initialized_;

    viewer Viewer_;
    std::thread viewer_thread_;
    
    ekf();
    ~ekf();
    void propagate(double v, double w, double t);
    void update(double x, double y, double theta, double t);
};

