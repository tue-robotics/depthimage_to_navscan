#include "depthimage_to_navscan/depthimage_to_navscan.h"

#include <geolib/ros/tf_conversions.h>
#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

// Decomposes 'pose' into a (X, Y, YAW) and (Z, ROLL, PITCH) component
void decomposePose(const geo::Pose3D& pose, geo::Pose3D& pose_xya, geo::Pose3D& pose_zrp)
{
    tf::Matrix3x3 m;
    geo::convert(pose.R, m);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_xya.R.setRPY(0, 0, yaw);
    pose_xya.t = geo::Vec3(pose.t.x, pose.t.y, 0);

    pose_zrp = pose_xya.inverse() * pose;
}

// ----------------------------------------------------------------------------------------------------

DepthSensorIntegrator::DepthSensorIntegrator()
{
}

// ----------------------------------------------------------------------------------------------------

DepthSensorIntegrator::~DepthSensorIntegrator()
{
}

// ----------------------------------------------------------------------------------------------------

void DepthSensorIntegrator::initialize(double slope_threshold, double floor_slope_height, double min_distance, double max_distance, double max_height, int num_samples, int slope_window_size)
{
    slope_threshold_ = slope_threshold;
    floor_slope_height_ = floor_slope_height;
    min_distance_ = min_distance;
    max_distance_ = max_distance;
    max_height_ = max_height;
    num_samples_ = num_samples;
    slope_window_size_ = slope_window_size;
    params_initialised_ = true;
}

bool DepthSensorIntegrator::imageToNavscan(std::vector<geo::Vector3> &measurements, const cv::Mat &depth, geo::Pose3D sensor_pose)
{
    if (!isInitialized())
        return false;

    bool visualize = false;

    // process input
    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
    decomposePose(sensor_pose, sensor_pose_xya, sensor_pose_zrp);

    int width = depth.cols;
    int height = depth.rows;

    cv::Mat obstacle_map, bla;
    if (visualize)
    {
        obstacle_map = cv::Mat(500, 500, CV_32FC1, 0.0);
        bla = depth.clone();
    }

    cv::Mat buffer(height, 1, CV_64FC4, 0.0);
    std::vector<geo::Vector3> p_floors(height);

    double x_step = 1;
    if (num_samples_ > 0 && num_samples_ < depth.cols)
        x_step = static_cast<double>(depth.cols) / num_samples_;

    // Iterate over the columns
    for (double x = 0; x < width; x += x_step)
    {
        geo::Vector3 p_floor_closest(1e6, 1e6, 0);

        buffer.at<cv::Vec4d>(0, 0) = cv::Vec4d(0, 0, 0, 0);

        // Iterate over the rows to find the closest obstacle point and apply the slope
        for (int y = 1; y < height; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d) // Ignore datapoints which are zero or NaN
            {
                buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0);
                continue;
            }

            geo::Vector3 p_sensor = rasterizer_.project2Dto3D(x, y) * d; // Point in sensor frame
            p_floors[y] = sensor_pose_zrp * p_sensor;
            const geo::Vector3& p_floor = p_floors[y]; // Point in floor frame

            // Find closest point above threshold height
            if (p_floor.z >= floor_slope_height_)
            {
                if (p_floor.y < p_floor_closest.y)
                    p_floor_closest = p_floor;

                if (visualize)
                {
                    cv::Point2i p(p_floor.x * 100 + obstacle_map.rows / 2,
                                  obstacle_map.cols - p_floor.y * 100);
                    if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                        obstacle_map.at<float>(p) = 0.5;
                }
            }

            // Apply slope
            buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0)
                                         + cv::Vec4d(p_floor.y, p_floor.z, p_floor.y * p_floor.z, p_floor.y * p_floor.y);
        }

        // Iterate over the rows, applying the slope_window, to find the closest point, when exceeding slope threshold
        for (int y = slope_window_size_; y < height - slope_window_size_; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d) // Ignore datapoints which are zero or NaN
                continue;

            const geo::Vector3& p_floor = p_floors[y];

            // Find closest point below threshold height, but which exceeds the slope threshold
            if (p_floor.z < floor_slope_height_)
            {
                cv::Vec4d b2 = buffer.at<cv::Vec4d>(y + slope_window_size_ / 2, 0)
                               - buffer.at<cv::Vec4d>(y - slope_window_size_ / 2, 0);
                double s2 = (slope_window_size_ * b2[2] - b2[0] * b2[1]) / (slope_window_size_ * b2[3] - b2[0] * b2[0]);

                if (std::abs(s2) > slope_threshold_)
                {
                    if (p_floor.y < p_floor_closest.y)
                        p_floor_closest = p_floor;

                    if (visualize)
                    {
                        bla.at<float>(y, x) = 10;
                        cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                      obstacle_map.cols / 2 - p_floor.y * 50);
                        if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                            obstacle_map.at<float>(p) = 1;
                    }
                }
            }
        }

        if (p_floor_closest.y < max_distance_ && p_floor_closest.y > min_distance_)
        {
            geo::Vec3 p_map = sensor_pose_xya * p_floor_closest;
            /*
             * Ignore resulting point above a threshold.
             * This may result in other points further away, but below the height threshold being ignored.
             */
            if (p_map.z > max_height_)
                continue;

            measurements.push_back(p_map);
        }
    }

    //TODO find better place for visualisation
    if (visualize)
    {
        cv::imshow("bla", bla / 10);
        cv::imshow("obstacle map", obstacle_map);
        cv::waitKey(3);
    }
    return true;
}
