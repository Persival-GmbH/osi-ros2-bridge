//
// Copyright 2025 Persival GmbH.
//
#ifndef _ROSOUTPUT_H_
#define _ROSOUTPUT_H_

#include <chrono>

#include <sensor_msgs/msg/detail/image__struct.hpp>

#include "osi_sensordata.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"  // needs to be .h instead of .hpp for compatibility reasons
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std;

class ROSOutput
{
public:

    ROSOutput();

    void SetSensorID(int sensor_id_in);

    void Init(const std::string& pcl_topic, const int detection_interval_ms, const std::string& frame_id);

    void PublishGtObjects(osi3::SensorData &sensor_data);
    void PublishDetections(const osi3::SensorData& sensor_data, const std::string& frame_id);
    void PublishImage(const osi3::SensorData& sensor_data, const std::string& frame_id);
    void PublishDetectedObjects(osi3::SensorData &sensor_data);
    void BroadcastTFs(osi3::SensorData& sensor_data);

private:
    rclcpp::Node::SharedPtr node_;
    shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray, allocator<void>>> gtobjects_publisher_;
    shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2, allocator<void>>> detections_publisher_;
    shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image, allocator<void>>> camera_publisher_;
    shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray, allocator<void>>> detectedobjects_publisher_;
    shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    string sensor_id;
    const string base_frame = "world";
    std::chrono::steady_clock::time_point last_publish_time;
    int detection_interval_ms_;
};

#endif // _ROSOUTPUT_H_