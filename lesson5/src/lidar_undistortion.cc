/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <lesson5/lidar_undistortion.h>

LidarUndistortion::LidarUndistortion()
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> lidar undistortion node started.\033[0m");

    imu_subscriber_ = node_handle_.subscribe(
        "imu", 2000, &LidarUndistortion::ImuCallback, this, ros::TransportHints().tcpNoDelay());
    odom_subscriber_ = node_handle_.subscribe(
        "odom", 2000, &LidarUndistortion::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 5, &LidarUndistortion::ScanCallback, this, ros::TransportHints().tcpNoDelay());

    corrected_pointcloud_publisher_ = node_handle_.advertise<PointCloudT>(
        "corrected_pointcloud", 1, this);

    first_scan_ = true;
    corrected_pointcloud_.reset(new PointCloudT());

    ResetParameters();
}

LidarUndistortion::~LidarUndistortion()
{
}

void LidarUndistortion::ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    std::lock_guard<std::mutex> lock(imu_lock_);
    imu_queue_.push_back(*imuMsg);
}

void LidarUndistortion::OdomCallback(const nav_msgs::Odometry::ConstPtr &odometryMsg)
{
    std::lock_guard<std::mutex> lock(odom_lock_);
    odom_queue_.push_back(*odometryMsg);
}

void LidarUndistortion::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg)
{

    if (!CacheLaserScan(laserScanMsg))
        return;

    if (!PruneImuDeque() && !PruneOdomDeque())
        return;

    CorrectLaserScan();

    PublishCorrectedPointCloud();

    ResetParameters();
}

// 缓存两帧雷达数据，以防止imu或者odom的数据不能包含雷达数据
bool LidarUndistortion::CacheLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg)
{
    if (first_scan_)
    {
        first_scan_ = false;
        CreateAngleCache(laserScanMsg);
        scan_count_ = current_laserscan_.ranges.size();
        corrected_pointcloud_->points.resize(laserScanMsg->ranges.size());
    }

    laser_queue_.push_back(*laserScanMsg);

    if (laser_queue_.size() < 2)
        return false;

    current_laserscan_ = laser_queue_.front();
    laser_queue_.pop_front();

    current_laserscan_header_ = current_laserscan_.header;
    current_scan_time_start_ = current_laserscan_header_.stamp.toSec(); // 认为ros中header的时间为这一帧雷达数据的起始时间
    current_scan_time_increment_ = current_laserscan_.time_increment;
    current_scan_time_end_ = current_scan_time_start_ + current_scan_time_increment_ * (scan_count_ - 1);

    return true;
}

// 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
void LidarUndistortion::CreateAngleCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }
}

bool LidarUndistortion::PruneImuDeque()
{
    std::lock_guard<std::mutex> lock(imu_lock_);

    // imu数据队列的头尾的时间戳要在雷达数据的时间段外
    if (imu_queue_.empty() ||
        imu_queue_.front().header.stamp.toSec() > current_scan_time_start_ ||
        imu_queue_.back().header.stamp.toSec() < current_scan_time_end_)
    {
        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }

    // 修剪imu的数据队列，直到imu的时间接近这帧点云的时间
    while (!imu_queue_.empty())
    {
        if (imu_queue_.front().header.stamp.toSec() < current_scan_time_start_ - 0.1)
            imu_queue_.pop_front();
        else
            break;
    }

    if (imu_queue_.empty())
        return false;

    current_imu_index_ = 0;

    sensor_msgs::Imu tmp_imu_msg;
    double current_imu_time, time_diff;

    for (int i = 0; i < (int)imu_queue_.size(); i++)
    {
        tmp_imu_msg = imu_queue_[i];
        current_imu_time = tmp_imu_msg.header.stamp.toSec();

        if (current_imu_time < current_scan_time_start_)
        {
            // 初始角度为0
            if (current_imu_index_ == 0)
            {
                imu_rot_x_[0] = 0;
                imu_rot_y_[0] = 0;
                imu_rot_z_[0] = 0;
                imu_time_[0] = current_imu_time;
                ++current_imu_index_;
            }
            continue;
        }

        if (current_imu_time > current_scan_time_end_)
            break;

        // get angular velocity
        double angular_x, angular_y, angular_z;
        ImuAngular2RosAngular(&tmp_imu_msg, &angular_x, &angular_y, &angular_z);

        // 对imu的角速度进行积分，当前帧转过的角度 = 上一阵的角度 + 当前帧角速度 * (当前帧imu的时间 - 上一帧imu的时间)
        double time_diff = current_imu_time - imu_time_[current_imu_index_ - 1];
        imu_rot_x_[current_imu_index_] = imu_rot_x_[current_imu_index_ - 1] + angular_x * time_diff;
        imu_rot_y_[current_imu_index_] = imu_rot_y_[current_imu_index_ - 1] + angular_y * time_diff;
        imu_rot_z_[current_imu_index_] = imu_rot_z_[current_imu_index_ - 1] + angular_z * time_diff;
        imu_time_[current_imu_index_] = current_imu_time;
        ++current_imu_index_;
    }

    // 对current_imu_index_进行-1操作后，current_imu_index_指向当前雷达时间内的最后一个imu数据
    --current_imu_index_;
}

bool LidarUndistortion::PruneOdomDeque()
{
    std::lock_guard<std::mutex> lock(odom_lock_);

    // imu数据队列的头尾的时间戳要在雷达数据的时间段外
    if (odom_queue_.empty() ||
        odom_queue_.front().header.stamp.toSec() > current_scan_time_start_ ||
        odom_queue_.back().header.stamp.toSec() < current_scan_time_end_)
    {
        ROS_DEBUG("Waiting for Odometry data ...");
        return false;
    }

    // 修剪odom的数据队列，直到odom的时间接近这帧点云的时间
    while (!odom_queue_.empty())
    {
        if (odom_queue_.front().header.stamp.toSec() < current_scan_time_start_ - 0.1)
            odom_queue_.pop_front();
        else
            break;
    }

    if (odom_queue_.empty())
        return false;

    // get start odometry at the beinning of the scan
    nav_msgs::Odometry start_odom_msg, end_odom_msg;
    double current_odom_time;

    for (int i = 0; i < (int)odom_queue_.size(); i++)
    {
        current_odom_time = odom_queue_[i].header.stamp.toSec();

        if (current_odom_time < current_scan_time_start_)
        {
            start_odom_msg = odom_queue_[i];
            continue;
        }

        if (current_odom_time <= current_scan_time_end_)
        {
            end_odom_msg = odom_queue_[i];
        }
        else
            break;
    }

    if (int(round(start_odom_msg.pose.covariance[0])) != int(round(end_odom_msg.pose.covariance[0])))
        return false;

    tf::Quaternion orientation;
    double roll, pitch, yaw;

    tf::quaternionMsgToTF(start_odom_msg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    Eigen::Affine3f transBegin = pcl::getTransformation(
        start_odom_msg.pose.pose.position.x,
        start_odom_msg.pose.pose.position.y,
        start_odom_msg.pose.pose.position.z,
        roll, pitch, yaw);

    tf::quaternionMsgToTF(end_odom_msg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    Eigen::Affine3f transEnd = pcl::getTransformation(
        end_odom_msg.pose.pose.position.x,
        end_odom_msg.pose.pose.position.y,
        end_odom_msg.pose.pose.position.z,
        roll, pitch, yaw);

    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    // 通过　transBt　获取　odomIncreX等，每一帧点云数据的odom变化量
    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt,
                                      odom_incre_x_, odom_incre_y_, odom_incre_z_,
                                      rollIncre, pitchIncre, yawIncre);
    return true;
}

void LidarUndistortion::CorrectLaserScan()
{
    bool first_point_flag = true;
    double current_point_time = 0;
    double current_point_x = 0, current_point_y = 0, current_point_z = 1.0;

    Eigen::Affine3f transStartInverse, transFinal, transBt;

    for (int i = 0; i < scan_count_; i++)
    {
        if (!std::isfinite(current_laserscan_.ranges[i]) ||
            current_laserscan_.ranges[i] < current_laserscan_.range_min ||
            current_laserscan_.ranges[i] > current_laserscan_.range_max)
            continue;

        PointT &point_tmp = corrected_pointcloud_->points[i];

        current_point_time = current_scan_time_start_ + i * current_scan_time_increment_;
        current_point_x = current_laserscan_.ranges[i] * a_cos_[i];
        current_point_y = current_laserscan_.ranges[i] * a_sin_[i];

        float rotXCur, rotYCur, rotZCur;
        ComputeRotation(current_point_time, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        ComputePosition(current_point_time, &posXCur, &posYCur, &posZCur);

        // 点云中的第一个点 求 transStartInverse，之后在这帧数据畸变过程中不再改变
        if (first_point_flag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                        rotXCur, rotYCur, rotZCur))
                                    .inverse();
            first_point_flag = false;
        }

        // transform points to start
        transFinal = pcl::getTransformation(posXCur, posYCur, posZCur,
                                            rotXCur, rotYCur, rotZCur);

        // 该点相对第一个点的变换矩阵　=　第一个点在lidar世界坐标系下的变换矩阵的逆 × 当前点时lidar世界坐标系下变换矩阵
        transBt = transStartInverse * transFinal;

        // 根据lidar位姿变换，修正点云位置
        point_tmp.x = transBt(0, 0) * current_point_x + transBt(0, 1) * current_point_y + transBt(0, 2) * current_point_z + transBt(0, 3);
        point_tmp.y = transBt(1, 0) * current_point_x + transBt(1, 1) * current_point_y + transBt(1, 2) * current_point_z + transBt(1, 3);
        point_tmp.z = transBt(2, 0) * current_point_x + transBt(2, 1) * current_point_y + transBt(2, 2) * current_point_z + transBt(2, 3);
    }
}

// 根据点云中某点的时间戳赋予其对应插值得到的旋转量
void LidarUndistortion::ComputeRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

    // 找到在　pointTime　之后的imu数据
    int imuPointerFront = 0;
    while (imuPointerFront < current_scan_time_start_)
    {
        if (pointTime < imu_time_[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    // 如果上边的循环没进去或者到了最大执行次数，则只能近似的将当前的旋转　赋值过去
    if (pointTime > imu_time_[imuPointerFront] || imuPointerFront == 0)
    {

        *rotXCur = imu_rot_x_[imuPointerFront];
        *rotYCur = imu_rot_y_[imuPointerFront];
        *rotZCur = imu_rot_z_[imuPointerFront];
    }
    else
    {
        int imuPointerBack = imuPointerFront - 1;

        // 算一下该点时间戳在本组imu中的位置，线性插值
        double ratioFront = (pointTime - imu_time_[imuPointerBack]) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);
        double ratioBack = (imu_time_[imuPointerFront] - pointTime) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);

        // 按前后百分比赋予旋转量
        *rotXCur = imu_rot_x_[imuPointerFront] * ratioFront + imu_rot_x_[imuPointerBack] * ratioBack;
        *rotYCur = imu_rot_y_[imuPointerFront] * ratioFront + imu_rot_y_[imuPointerBack] * ratioBack;
        *rotZCur = imu_rot_z_[imuPointerFront] * ratioFront + imu_rot_z_[imuPointerBack] * ratioBack;
    }
}

void LidarUndistortion::ComputePosition(double pointTime, float *posXCur, float *posYCur, float *posZCur)
{
    // *posXCur = 0; *posYCur = 0; *posZCur = 0;

    // If the sensor moves relatively slow, like walking speed,
    // positional deskew seems to have little benefits. Thus code below is commented.
    float ratio = pointTime / (current_scan_time_end_ - current_scan_time_start_);
    *posXCur = ratio * odom_incre_x_;
    *posYCur = ratio * odom_incre_y_;
    *posZCur = ratio * odom_incre_z_;
}

void LidarUndistortion::PublishCorrectedPointCloud()
{
    corrected_pointcloud_->width = scan_count_;
    corrected_pointcloud_->height = 1;
    corrected_pointcloud_->is_dense = false; // contains nans

    // 将scan_msg的消息头 赋值到 PointCloudT的消息头
    pcl_conversions::toPCL(current_laserscan_header_, corrected_pointcloud_->header);

    // 由于ros中自动做了 pcl::PointCloud<PointT> 到 sensor_msgs/PointCloud2 的数据类型的转换
    // 所以这里直接发布 pcl::PointCloud<PointT> 即可
    corrected_pointcloud_publisher_.publish(corrected_pointcloud_);
}

void LidarUndistortion::ResetParameters()
{
    current_imu_index_ = 0;

    imu_time_.clear();
    imu_rot_x_.clear();
    imu_rot_y_.clear();
    imu_rot_z_.clear();

    imu_time_ = std::vector<double>(2000, 0);
    imu_rot_x_ = std::vector<double>(2000, 0);
    imu_rot_y_ = std::vector<double>(2000, 0);
    imu_rot_z_ = std::vector<double>(2000, 0);
}

template <typename T>
inline void LidarUndistortion::ImuAngular2RosAngular(sensor_msgs::Imu *tmp_imu_msg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = tmp_imu_msg->angular_velocity.x;
    *angular_y = tmp_imu_msg->angular_velocity.y;
    *angular_z = tmp_imu_msg->angular_velocity.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson5_lidar_undistortion");

    LidarUndistortion lidar_undistortion;

    // 开启3个线程同时工作
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::waitForShutdown();
    return (0);
}