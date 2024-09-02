#pragma once
#include <mutex>
#include <vector>
using namespace std;

class viewer
{
private:

    mutex data_mutex_;
    vector<Eigen::Vector3d> wheel_points_;
    vector<Eigen::Vector3d> gnss_points_;

    void DrawGrid();
    void DrawWheelOdom();
    void DrawGNSSOdom();

public:
    bool running_ = true;
    double width_ = 1280.;
    double height_ = 960.;
    int fx_ = 800;
    int fy_ = 800;
    int cx_ = width_ / 2;
    int cy_ = height_ / 2;
    double znear_ = 0.1;
    double zfar_ = 1000;

    bool draw_grid_flag_ = true;
    int grid_scale_ = 20;
    // int scale_ = 10;

    bool draw_wheel_flag_ = true;
    int wheel_point_size_ = 3;

    bool draw_gnss_flag_ = true;
    int gnss_point_size_ = 3;

    viewer();
    void run();
    
    void AddWheelOdom(const Eigen::Vector3d &G_p_O);
    void AddGNSSOdom(const Eigen::Vector3d &G_p_gnss);
    // ~viewer();
};
