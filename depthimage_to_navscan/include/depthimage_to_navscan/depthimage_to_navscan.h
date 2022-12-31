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
     * @param slope_threshold
     * @param min_distance
     * @param max_distance
     * @param num_samples
     * @param slope_window_size
     */
    void initialize(double slope_threshold, double min_distance, double max_distance, int num_samples, int slope_window_size);

    inline double getSlopeThreshold() const { return slope_threshold_; }
    inline void setSlopeThreshold(double d) { slope_threshold_ = d; }

    inline double getMinDistance() const { return min_distance_; }
    inline void setMinDistance(double d) { min_distance_ = d; }

    inline double getMaxDistance() const { return max_distance_; }
    inline void setMaxDistance(double d) { max_distance_ = d; }

    inline int getNumSamples() const { return num_samples_; }
    inline void setNumSamples(int n) { num_samples_ = n; }

    inline int getSlopeWindowSize() const { return slope_window_size_; }
    inline void setSlopeWindowSize(int n) { slope_window_size_ = n; }

    /**
     * @brief isInitialized
     * @return If the instance is initialized
     */
    inline bool isInitialized() const { return params_initialised_ && rasterizer_.initialized(); }

    inline void setCameraModel(const image_geometry::PinholeCameraModel& cam_model) { rasterizer_.initFromCamModel(cam_model); }

    bool imageToNavscan(std::vector<geo::Vector3> &measurements, const cv::Mat &depth, const geo::Pose3D sensor_pose);

private:
    bool params_initialised_ = false;

    geo::DepthCamera rasterizer_;

    // Params
    double slope_threshold_;
    double min_distance_;
    double max_distance_;

    int num_samples_;

    int slope_window_size_;
};

#endif
