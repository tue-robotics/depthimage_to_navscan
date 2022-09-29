#ifndef _DEPTH_SENSOR_INTEGRATOR_H_
#define _DEPTH_SENSOR_INTEGRATOR_H_

#include <ed/kinect/image_buffer.h>
#include <tue/config/configuration.h>

//TODO check whether these are the rigth imports
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geolib/datatypes.h>
#include <geolib/sensors/DepthCamera.h>

#include <ros/publisher.h>

/**
 * @brief A depth image is converted to a pointcloud based on the normal. This is used for detection of obstacles
 */
class DepthSensorIntegrator
{

public:
    /**
     * @brief constructor
     */
    DepthSensorIntegrator();

    /**
     * @brief destructor
     */
    ~DepthSensorIntegrator();

    /**
     * @brief initialize
     * @param config tue::Configuration
     */
    void initialize(tue::Configuration config);

    /**
     * @brief New depth image is converted pointcloud
     * @return If update is executed correctly
     */
    bool update();

    /**
     * @brief isInitialized
     * @return If the instance is initialized
     */
    bool isInitialized() const { return !map_frame_.empty(); }

    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model) { rasterizer.initFromCamModel(cam_model); }

    bool imageToNavscan(std::vector<geo::Vector3> &measurements, const cv::Mat &depth, const geo::Pose3D sensor_pose);

private:
    geo::DepthCamera rasterizer;
    ImageBuffer image_buffer_;

    // Params

    std::string map_frame_;

    double slope_threshold_;
    double min_distance_;
    double max_distance_;

    int num_samples_;

    int slope_window_size_;

    ros::Publisher pointcloud2_publisher_;

};

#endif