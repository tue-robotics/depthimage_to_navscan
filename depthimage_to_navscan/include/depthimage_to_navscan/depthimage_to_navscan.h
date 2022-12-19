#ifndef DEPTHIMAGE_TO_NAVSCAN_DEPTHIMAGE_TO_NAVSCAN_H_
#define DEPTHIMAGE_TO_NAVSCAN_DEPTHIMAGE_TO_NAVSCAN_H_

#include <opencv2/core/mat.hpp>

#include <geolib/datatypes.h>
#include <geolib/sensors/DepthCamera.h>
#include <image_geometry/pinhole_camera_model.h>

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
     * @param
     */
    void initialize(double slope_threshold, double min_distance, double max_distance, int num_samples, int slope_window_size);

    double getSlopeThreshold() const {return slope_threshold_;}
    void setSlopeThreshold(double a) {slope_threshold_ = a;}

    double getMinDistance() const {return min_distance_;}
    void setMinDistance(double a) {min_distance_ = a;}

    double getMaxDistance() const {return max_distance_;}
    void setMaxDistance(double a) {max_distance_ = a;}

    int getNumSamples() const {return num_samples_;}
    void setNumSamples(int n) {num_samples_ = n;}

    int getSlopeWindowSize() const {return slope_window_size_;}
    void setSlopeWindowSize(int n) {slope_window_size_ = n;}

    /**
     * @brief isInitialized
     * @return If the instance is initialized
     */
    bool isInitialized() const { return params_initialised && depthcam_initialised; }

    void setCameraModel(const image_geometry::PinholeCameraModel& cam_model) { rasterizer.initFromCamModel(cam_model); depthcam_initialised = true;}

    bool imageToNavscan(std::vector<geo::Vector3> &measurements, const cv::Mat &depth, const geo::Pose3D sensor_pose);

private:
    bool params_initialised = false;
    bool depthcam_initialised = false;

    // depth camera model
    geo::DepthCamera rasterizer;

    // Params
    double slope_threshold_;
    double min_distance_;
    double max_distance_;

    int num_samples_;

    int slope_window_size_;
};

#endif
