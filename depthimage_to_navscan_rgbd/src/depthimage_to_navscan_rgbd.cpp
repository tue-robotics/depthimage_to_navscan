#include <ros/ros.h>
#include <depthimage_to_navscan/depthimage_to_navscan.h>

#include <rgbd/image.h>
#include <rgbd/image_buffer/image_buffer.h>
#include <sensor_msgs/PointCloud2.h>

// TODO docstring and usage
int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage_to_navscan_rgbd");
    ros::NodeHandle nh("~");

    std::string rgbd_topic;
    if (!nh.getParam("rgbd_topic", rgbd_topic))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read rgbd_topic from parameter server");
        return 1;
    }

    std::string frame_id;
    if (!nh.getParam("frame_id", frame_id))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read frame_id from parameter server");
        return 1;
    }
    
    // Optional
    double rate = 100;
    if (!nh.getParam("rate", rate))
    {
        ROS_DEBUG_STREAM("[Depthimage to Navscan RGBD] could not read rate from parameter server, defaulting to " << rate);
    }

    DepthSensorIntegrator depthSensorIntegrator;
    // get depth sensor integrator parameters
    double slope_threshold, floor_slope_height, min_distance, max_distance, max_height;
    int slope_window_size, num_samples;

    if (!nh.getParam("slope_threshold", slope_threshold))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read slope_threshold from parameter server");
        return 1;
    }
    if (!nh.getParam("slope_threshold", floor_slope_height))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read floor_slope_height from parameter server");
        return 1;
    }
    if (!nh.getParam("min_distance", min_distance))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read min_distance from parameter server");
        return 1;
    }
    if (!nh.getParam("max_distance", max_distance))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read max_distance from parameter server");
        return 1;
    }
    if (!nh.getParam("max_height", max_height))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read max_height from parameter server");
        return 1;
    }
    if (!nh.getParam("num_samples", num_samples))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read num_samples from parameter server");
        return 1;
    }
    if (!nh.getParam("slope_window_size", slope_window_size))
    {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read slope_window_size from parameter server");
        return 1;
    }

    depthSensorIntegrator.initialize(slope_threshold, floor_slope_height, min_distance, max_distance, max_height, num_samples, slope_window_size);
    ROS_INFO("[Depthimage to Navscan RGBD] initialised depth sensor integrator with:\n"
              " slope_threshold: %f, floor_slope_height: %f, min_distance: %f, max_distance: %f, max_height: %f, num_samples: %i, slope_window_size: %i",
              slope_threshold, floor_slope_height, min_distance, max_distance, max_height, num_samples, slope_window_size);

    rgbd::ImageBuffer image_buffer;
    image_buffer.initialize(rgbd_topic, frame_id);

    ros::Publisher pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2>("navscan", 20);

    ros::Rate r(rate);
    while (ros::ok())
    {
        rgbd::ImageConstPtr image;
        geo::Pose3D sensor_pose;
        if (!image_buffer.waitForRecentImage(image, sensor_pose, r.expectedCycleTime().toSec()))
        {
            r.sleep(); // So we do sleep after getting an image again after failing to get an image
            continue;
        }

        if (!depthSensorIntegrator.isInitialized())
        {
            ROS_INFO("[Depthimage to Navscan RGBD] configuring camera model");
            depthSensorIntegrator.setCameraModel(image->getCameraModel());
            if (!depthSensorIntegrator.isInitialized())
            {
                ROS_FATAL("[Depthimage to Navscan RGBD] depthSensorIntegrator could not be initialised correctly!");
                return 1;
            }
            ROS_INFO("[Depthimage to Navscan RGBD] configured");
        }

        std::vector <geo::Vector3> measurements;

        const cv::Mat& depth = image->getDepthImage();

        // perform magic
        depthSensorIntegrator.imageToNavscan(measurements, depth, sensor_pose);

        // fill output message
        sensor_msgs::PointCloud2 pointcloud2_msg;
        pointcloud2_msg.header.frame_id = frame_id;
        pointcloud2_msg.header.stamp.fromSec(image->getTimestamp());

        pointcloud2_msg.height = 1;

        pointcloud2_msg.fields.resize(3);

        pointcloud2_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud2_msg.fields[0].name = "x";
        pointcloud2_msg.fields[0].offset = 0;
        pointcloud2_msg.fields[0].count = 1;

        pointcloud2_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud2_msg.fields[1].name = "y";
        pointcloud2_msg.fields[1].offset = 4;
        pointcloud2_msg.fields[1].count = 1;

        pointcloud2_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        pointcloud2_msg.fields[2].name = "z";
        pointcloud2_msg.fields[2].offset = 8;
        pointcloud2_msg.fields[2].count = 1;

        pointcloud2_msg.point_step = 12;

        pointcloud2_msg.data.resize(pointcloud2_msg.point_step * measurements.size());
        pointcloud2_msg.row_step = pointcloud2_msg.point_step * measurements.size();
        pointcloud2_msg.width = measurements.size();

        unsigned int i = 0;
        for (std::vector<geo::Vector3>::const_iterator it = measurements.begin(); it != measurements.end(); ++it) {
            float x = it->x;
            float y = it->y;
            float z = 0.1;

            // Copy x y z values to byte array
            memcpy(pointcloud2_msg.data.data() + i * pointcloud2_msg.point_step, reinterpret_cast<uint8_t *>(&x), 4);
            memcpy(pointcloud2_msg.data.data() + i * pointcloud2_msg.point_step + 4, reinterpret_cast<uint8_t *>(&y),
                   4);
            memcpy(pointcloud2_msg.data.data() + i * pointcloud2_msg.point_step + 8, reinterpret_cast<uint8_t *>(&z),
                   4);
            ++i;
        }

        pointcloud2_publisher.publish(pointcloud2_msg);
        r.sleep();
    }
}
