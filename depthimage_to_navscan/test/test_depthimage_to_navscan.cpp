//
// Created by Peter van Dooren on 6-10-22.
//
#include <gtest/gtest.h>

#include <depthimage_to_navscan.h>
#include <geolib/datatypes.h>
#include <math.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>


#define CAM_WIDTH 640
#define CAM_HEIGHT 480

sensor_msgs::CameraInfo getDefaultCamInfo()
{
    sensor_msgs::CameraInfo cam_info;
    cam_info.K = sensor_msgs::CameraInfo::_K_type({554.2559327880068, 0.0, 320.5,
                                                   0.0, 554.2559327880068, 240.5,
                                                   0.0, 0.0, 1.0});
    cam_info.P = sensor_msgs::CameraInfo::_P_type({554.2559327880068, 0.0, 320.5, 0.0,
                                                   0.0, 554.2559327880068, 240.5, 0.0,
                                                   0.0, 0.0, 1.0, 0.0});
    cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    cam_info.width = CAM_WIDTH;
    cam_info.height = CAM_HEIGHT;
    return cam_info;
}


class DepthSensorIntegratorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        sensor_msgs::CameraInfo cam_info_message = getDefaultCamInfo();
        cam_model.fromCameraInfo(cam_info_message);

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

ASSERT_FALSE(dsi_params_init.isInitialized()) << "depthSensorIntegrator believes itself to be initialized while camera was not initialised";
ASSERT_FALSE(dsi_params_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when camera not initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckParamsNotInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

ASSERT_FALSE(dsi_cam_init.isInitialized()) << "depthSensorIntegrator believes itself to be initialized even though params was not intialised";
ASSERT_FALSE(dsi_cam_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when parameters not initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckInitialised)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth;
geo::Pose3D sensor_pose;

ASSERT_TRUE(dsi_full_init.isInitialized()) << "depthSensorIntegrator believes itself to not be initialized while it was";
ASSERT_TRUE(dsi_full_init.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned false when initialised";
}

TEST_F(DepthSensorIntegratorTest, CheckOutputSize)
{
std::vector<geo::Vector3> measurements;

//generate input data
cv::Mat depth(CAM_WIDTH, CAM_HEIGHT, CV_32F, 1.0);
geo::Pose3D sensor_pose(0, 0, 1.0, M_PI_2, -M_PI_2, 0.0);

dsi_full_init.imageToNavscan(measurements, depth, sensor_pose);
ASSERT_EQ(measurements.size(), num_samples) << "Output navscan has incorrect number of points";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
