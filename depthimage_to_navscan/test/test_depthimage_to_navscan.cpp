//
// Created by Peter van Dooren on 6-10-22.
//
#include <gtest/gtest.h>

#include <depthimage_to_navscan.h>

TEST(NavscanTest, CheckNotInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

DepthSensorIntegrator depthSensorIntegrator;

ASSERT_FALSE(depthSensorIntegrator.isInitialized()) << "depthSensorIntegrator believes itself to be initialized upon construction";
ASSERT_FALSE(depthSensorIntegrator.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when not initialised";
}

TEST(NavscanTest, CheckCameraNotInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

DepthSensorIntegrator depthSensorIntegrator;

uint num_samples = 20;
double slope_threshold = 1.0;
double min_distance = 0.4;
double max_distance = 2.0;
int slope_window_size = 30;
depthSensorIntegrator.initialize(slope_threshold, min_distance, max_distance, num_samples, slope_window_size);

ASSERT_FALSE(depthSensorIntegrator.isInitialized()) << "depthSensorIntegrator believes itself to be initialized even when camera model not set";
ASSERT_FALSE(depthSensorIntegrator.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when camera model not set";
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}