#include <ros/ros.h>
#include <depthimage_to_navscan.h>

#include <rgbd/image.h>
#include <ed/kinect/image_buffer.h>
#include <sensor_msgs/PointCloud2.h>

// TODO docstring and usage
int main(int argc, char** argv)
{
    ros::init(argc, argv, "depthimage_to_navscan_rgbd");
    ros::NodeHandle nh;

    std::string rgbd_topic;
    if (!nh.getParam("rgbd_topic", rgbd_topic)) {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read rgbd_topic from parameter server");
        return 1;
    }

    std::string map_frame;
    if (!nh.getParam("map_frame", map_frame)) {
        ROS_FATAL("[Depthimage to Navscan RGBD] could not read map_frame from parameter server");
        return 1;
    }

    DepthSensorIntegrator depthSensorIntegrator;
    //TODO configure DepthSensorIntegrator

    ImageBuffer image_buffer;
    image_buffer.initialize(rgbd_topic, map_frame);

    ros::Publisher pointcloud2_publisher = nh.advertise<sensor_msgs::PointCloud2>("navigation/cloud", 20);

    while (ros::ok())
    {
        if (!depthSensorIntegrator.isInitialized())
            continue;

        rgbd::ImageConstPtr image;
        geo::Pose3D sensor_pose;
        if (!image_buffer.nextImage(image, sensor_pose))
            continue;

        std::vector <geo::Vector3> measurements;

        const cv::Mat& depth = image->getDepthImage();

        // perform magic
        depthSensorIntegrator.imageToNavscan(measurements, depth, sensor_pose);

        // fill output message
        sensor_msgs::PointCloud2 pointcloud2_msg;
        pointcloud2_msg.header.frame_id = map_frame;
        pointcloud2_msg.header.stamp = ros::Time::now();

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
    }
}
