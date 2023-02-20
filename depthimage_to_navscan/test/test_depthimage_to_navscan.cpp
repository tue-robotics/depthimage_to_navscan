//
// Created by Peter van Dooren on 6-10-22.
//
#include <gtest/gtest.h>

#include <depthimage_to_navscan/depthimage_to_navscan.h>
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


class IntegratorNoInit : public ::testing::Test {
protected:
    DepthSensorIntegrator dsi;

    // sample config
    double slope_threshold = 1.0;
    double floor_slope_height = 0.2;
    double min_distance = 0.4;
    double max_distance = 2.0;
    double max_height = 1.7;
    uint num_samples = 20;
    int slope_window_size = 30;
};

class IntegratorParamInit : public IntegratorNoInit
{
protected:
    void SetUp() override
    {
        dsi.initialize(slope_threshold, floor_slope_height, min_distance, max_distance, max_height, num_samples, slope_window_size);
    }
};

class IntegratorCamInit : public IntegratorNoInit
{
protected:
    void SetUp() override
    {
        sensor_msgs::CameraInfo cam_info_message = getDefaultCamInfo();
        cam_model.fromCameraInfo(cam_info_message);
        dsi.setCameraModel(cam_model);
    }

    // sample cameramodel
    image_geometry::PinholeCameraModel cam_model;
};

class IntegratorFullInit : public IntegratorCamInit
{
protected:
    void SetUp() override
    {
        IntegratorCamInit::SetUp();
        dsi.initialize(slope_threshold, floor_slope_height, min_distance, max_distance, max_height, num_samples, slope_window_size);
    }
};

TEST_F(IntegratorNoInit, CheckNotInitialised)
{
    // Generate input data
    std::vector<geo::Vector3> measurements;
    cv::Mat depth;
    geo::Pose3D sensor_pose;

    ASSERT_FALSE(dsi.isInitialized()) << "DepthSensorIntegrator believes itself to be initialized upon construction";
    ASSERT_FALSE(dsi.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when not initialised";
}

TEST_F(IntegratorParamInit, CheckCameraNotInitialised)
{
    // Generate input data
    std::vector<geo::Vector3> measurements;
    cv::Mat depth;
    geo::Pose3D sensor_pose;

    ASSERT_FALSE(dsi.isInitialized()) << "DepthSensorIntegrator believes itself to be initialized, while the camera was not initialised";
    ASSERT_FALSE(dsi.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when the camera is not initialised";
}

TEST_F(IntegratorCamInit, CheckParamsNotInitialised)
{
    // Generate input data
    std::vector<geo::Vector3> measurements;
    cv::Mat depth;
    geo::Pose3D sensor_pose;

    ASSERT_FALSE(dsi.isInitialized()) << "DepthSensorIntegrator believes itself to be initialized even though, the parameters were not intialised";
    ASSERT_FALSE(dsi.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned true even when the parameters were not initialised";
}

TEST_F(IntegratorFullInit, CheckInitialised)
{
    // Generate input data
    std::vector<geo::Vector3> measurements;
    cv::Mat depth;
    geo::Pose3D sensor_pose;

    ASSERT_TRUE(dsi.isInitialized()) << "DepthSensorIntegrator believes itself to not be initialized, while it was";
    ASSERT_TRUE(dsi.imageToNavscan(measurements, depth, sensor_pose)) << "imageToNavscan returned false when initialised";
}

TEST_F(IntegratorFullInit, CheckOutput)
{
    // Generate input data
    std::vector<geo::Vector3> measurements;
    double distance = 1.0;
    cv::Mat depth(CAM_WIDTH, CAM_HEIGHT, CV_32F, distance);
    geo::Pose3D sensor_pose(0, 0, 1.0, M_PI_2, -M_PI_2, 0.0);

    dsi.imageToNavscan(measurements, depth, sensor_pose);
    ASSERT_EQ(measurements.size(), num_samples) << "Output navscan has incorrect number of points";
    for (uint i=0; i<measurements.size(); ++i)
    {
        ASSERT_EQ(measurements[i].y, distance) << "Measurement has incorrect distance";
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
