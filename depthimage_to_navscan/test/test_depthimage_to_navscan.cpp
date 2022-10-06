//
// Created by Peter van Dooren on 6-10-22.
//
#include <gtest/gtest.h>

#include <depthimage_to_navscan.h>

class DepthSensorIntegratorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // configure camera model
        cam_model.fx = 0.05;
        cam_model.fy = 0.05;
        cam_model.cx = 0.0;
        cam_model.cy = 0.0;
        cam_model.Tx = 0.0;
        cam_model.Ty = 0.0;

        dsi_cam_init.setCameraModel(cam_model);

        dsi_params_init.initialize(slope_threshold, min_distance, max_distance, num_samples, slope_window_size);

        dsi_full_init.setCameraModel(cam_model);
        dsi_full_init.initialize(slope_threshold, min_distance, max_distance, num_samples, slope_window_size);
    }

    DepthSensorIntegrator dsi_0;
    DepthSensorIntegrator dsi_params_init;
    DepthSensorIntegrator dsi_cam_init;
    DepthSensorIntegrator dsi_full_init;

    // sample cameramodel
    image_geometry::PinholeCameraModel cam_model
    // sample config
    uint num_samples = 20;
    double slope_threshold = 1.0;
    double min_distance = 0.4;
    double max_distance = 2.0;
    int slope_window_size = 30;
}

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