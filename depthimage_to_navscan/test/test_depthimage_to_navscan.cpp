//
// Created by Peter van Dooren on 6-10-22.
//
#include <gtest/gtest.h>

#include <depthimage_to_navscan.h>

class DepthSensorIntegratorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        //#TODO configure camera model

        dsi_cam_init.setCameraModel(cam_model);

        dsi_params_init.initialize(slope_threshold, min_distance, max_distance, num_samples, slope_window_size);

        dsi_full_init.setCameraModel(cam_model);
        dsi_full_init.initialize(slope_threshold, min_distance, max_distance, num_samples, slope_window_size);
    }

    DepthSensorIntegrator dsi_no_init;
    DepthSensorIntegrator dsi_params_init;
    DepthSensorIntegrator dsi_cam_init;
    DepthSensorIntegrator dsi_full_init;

    // sample cameramodel
    image_geometry::PinholeCameraModel cam_model;
    // sample config
    uint num_samples = 20;
    double slope_threshold = 1.0;
    double min_distance = 0.4;
    double max_distance = 2.0;
    int slope_window_size = 30;
};

TEST_F(DepthSensorIntegratorTest, CheckNotInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

ASSERT_FALSE(dsi_no_init.isInitialized()) << "depthSensorIntegrator believes itself to be initialized upon construction";
ASSERT_FALSE(dsi_no_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when not initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckCameraNotInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

ASSERT_FALSE(dsi_params_init.isInitialized()) << "depthSensorIntegrator believes itself to be initialized upon construction";
ASSERT_FALSE(dsi_params_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when not initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckParamsNotInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

ASSERT_FALSE(dsi_cam_init.isInitialized()) << "depthSensorIntegrator believes itself to be initialized upon construction";
ASSERT_FALSE(dsi_cam_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when not initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

ASSERT_TRUE(dsi_full_init.isInitialized()) << "depthSensorIntegrator believes itself to be initialized upon construction";
ASSERT_TRUE(dsi_full_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when not initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckOutputSize)
{
    //TODO proper configuration of depth, sensor pose and cam model
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

dsi_full_init.imageToNavscan(measurements, depth, sensor_pose);
ASSERT_EQ(measurements.size(), num_samples) << "Output navscan has incorrect number of points";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}