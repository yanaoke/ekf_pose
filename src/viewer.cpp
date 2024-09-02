#include <pangolin/pangolin.h>
#include <viewer.h>


viewer::viewer(){
    
}

void viewer::DrawGrid(){
    glColor3f(0.3f, 0.3f, 0.3f);
    pangolin::glDraw_z0(grid_scale_, 1000);    
}

void viewer::AddWheelOdom(const Eigen::Vector3d &G_p_O){
    lock_guard<mutex> lg(data_mutex_);
    wheel_points_.push_back(G_p_O);
}
void viewer::DrawWheelOdom(){
    glPointSize(wheel_point_size_);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_POINTS);
    std::lock_guard<std::mutex> lg(data_mutex_);
    for (const Eigen::Vector3d &pt : wheel_points_) {
        glVertex3f(pt[0], pt[1], pt[2]);
    }
    glEnd();
}

void viewer::AddGNSSOdom(const Eigen::Vector3d &G_p_gnss){
    lock_guard<mutex> lg(data_mutex_);
    gnss_points_.push_back(G_p_gnss);
}

void viewer::DrawGNSSOdom(){
    glPointSize(gnss_point_size_);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_POINTS);
    std::lock_guard<std::mutex> lg(data_mutex_);
    for (const Eigen::Vector3d &pt : gnss_points_) {
        glVertex3f(pt[0], pt[1], pt[2]);
    }
    glEnd();
}

void viewer::run(){
    // 创建窗口
    pangolin::CreateWindowAndBind("Viewer", width_, height_);
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    // 创建投影和模型视图矩阵
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(width_, height_, fx_, fy_, cx_, cy_, znear_, zfar_),
        pangolin::ModelViewLookAt(0, 0, 20, 0, 0, 0, pangolin::AxisX)
    );
    // 创建交互视图
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -width_ / height_)
        .SetHandler(&handler);
    while (running_){
        // 清除颜色和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 设置投影和模型视图矩阵
        d_cam.Activate(s_cam);

        // 绘制网格线
        if(draw_grid_flag_) DrawGrid();
        // 绘制轮速计轨迹
        if(draw_wheel_flag_) DrawWheelOdom();
        // 绘制GNSS轨迹
        if(draw_gnss_flag_) DrawGNSSOdom();

        // 交换前后缓冲区并处理窗口事件
        pangolin::FinishFrame();
    }
}
