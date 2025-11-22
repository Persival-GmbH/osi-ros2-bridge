//
// Copyright 2023 Persival GmbH.
//

#include "ROSOutput.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/color_rgba.h"
#include "visualization_msgs/msg/marker.hpp"
#include "point_types.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

using namespace std;

ROSOutput::ROSOutput() : node_(nullptr), gtobjects_publisher_(nullptr), detections_publisher_(nullptr), transform_broadcaster_(nullptr)
{
    sensor_id = std::to_string(0);
    last_publish_time = std::chrono::steady_clock::now() - 100ms;
}

void ROSOutput::SetSensorID(int sensor_id_in)
{
    sensor_id = std::to_string(sensor_id_in);
}

void ROSOutput::Init(const std::string& pcl_topic, const int detection_interval_ms, const std::string& frame_id)
{
    if (!rclcpp::ok())
    {
        cout << "ROS: Creating ROS node 'osi_publisher'" << endl;
        int argc = 1;
        const char* argv = "";
        rclcpp::init(argc, &argv);
    }

    detection_interval_ms_ = detection_interval_ms;
    const string detections_topic = pcl_topic.empty() ? "/detections_" + sensor_id : pcl_topic;

    node_ = make_shared<rclcpp::Node>("osi_publisher_" + sensor_id);
    if (frame_id.empty())
    {
        gtobjects_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/gt_marker_" + sensor_id, rclcpp::QoS(1000));
        detectedobjects_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/detectedobjects_marker_" + sensor_id, rclcpp::QoS(1000));
        transform_broadcaster_ = make_shared<tf2_ros::TransformBroadcaster>(node_);
    }
    detections_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(detections_topic, rclcpp::QoS(1000));

}

void ROSOutput::PublishGtObjects(osi3::SensorData& sensor_data)
{
    osi3::Identifier ego_id = sensor_data.sensor_view(0).global_ground_truth().host_vehicle_id();
    /// Moving objects
    visualization_msgs::msg::MarkerArray marker_array;
    for (const osi3::MovingObject& gt_moving_object : sensor_data.sensor_view(0).global_ground_truth().moving_object())
    {
        // Creating the marker and initialising its fields
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = base_frame;
        marker.header.stamp = rclcpp::Time(0);
        std_msgs::msg::ColorRGBA color_gt_moving;
        if (gt_moving_object.id().value() == ego_id.value())
        {
            color_gt_moving.r = 0.8;
            color_gt_moving.g = 0.0;
            color_gt_moving.b = 0.0;
            color_gt_moving.a = 0.5;
        }
        else
        {
            color_gt_moving.r = 0.8;
            color_gt_moving.g = 0.8;
            color_gt_moving.b = 0.8;
            color_gt_moving.a = 0.5;
        }

        geometry_msgs::msg::Pose pose_gt_moving;
        pose_gt_moving.position.x = gt_moving_object.base().position().x();
        pose_gt_moving.position.y = gt_moving_object.base().position().y();
        pose_gt_moving.position.z = gt_moving_object.base().position().z();

        tf2::Quaternion orientation_q;
        orientation_q.setRPY(gt_moving_object.base().orientation().roll(),
                             gt_moving_object.base().orientation().pitch(),
                             gt_moving_object.base().orientation().yaw());

        pose_gt_moving.orientation.x = orientation_q.x();
        pose_gt_moving.orientation.y = orientation_q.y();
        pose_gt_moving.orientation.z = orientation_q.z();
        pose_gt_moving.orientation.w = orientation_q.w();

        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.pose = pose_gt_moving;
        marker.ns = "OSI_GT";
        marker.id = (int)gt_moving_object.id().value();

        marker.color = color_gt_moving;
        marker.lifetime = rclcpp::Duration(0, 0);
        marker.frame_locked = false;

        string model_reference;
        if (gt_moving_object.has_model_reference())
        {
            size_t extension_idx = gt_moving_object.model_reference().find_last_of('.');
            if (std::filesystem::exists(gt_moving_object.model_reference().substr(0, extension_idx) + ".dae"))
            {
                model_reference = gt_moving_object.model_reference().substr(0, extension_idx) + ".dae";
            }
        }

        if (!model_reference.empty())
        {
            marker.type = 10;
            marker.mesh_resource = "file://" + model_reference;
            marker.mesh_use_embedded_materials = true;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
        }
        else
        {
            if (gt_moving_object.base().dimension().length() > 0 &&
                gt_moving_object.base().dimension().width() > 0 &&
                gt_moving_object.base().dimension().height() > 0)
            {
                marker.scale.x = gt_moving_object.base().dimension().length();
                marker.scale.y = gt_moving_object.base().dimension().width();
                marker.scale.z = gt_moving_object.base().dimension().height();
            }
            else
            {
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
            }
        }

        marker_array.markers.push_back(marker);
    }

    gtobjects_publisher_->publish(marker_array);
}

void ROSOutput::PublishDetections(const osi3::SensorData& sensor_data, const std::string& frame_id)
{
    if (!sensor_data.has_feature_data())
    {
        return;
    }
    if (sensor_data.feature_data().lidar_sensor().empty())
    {
        return;
    }

    const auto& lidar_sensor = sensor_data.feature_data().lidar_sensor(0);

    pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
    pcl::PointCloud<persival_pcl::PointXYZIO> cloud;

    pcl_conversions::toPCL(rclcpp::Time(0), cloud_tmp.header.stamp);
    if (frame_id.empty())
    {
        cloud_tmp.header.frame_id = "detections_" + sensor_id;
    }
    else
    {
        cloud_tmp.header.frame_id = frame_id;
    }

    /// Run through all detections
    for (const auto& detection : lidar_sensor.detection())
    {
        auto x = static_cast<float>(detection.position().distance() * cos(detection.position().azimuth()) * cos(detection.position().elevation()));
        auto y = static_cast<float>(detection.position().distance() * sin(detection.position().azimuth()) * cos(detection.position().elevation()));
        auto z = static_cast<float>(detection.position().distance() * sin(detection.position().elevation()));
        cloud_tmp.points.emplace_back(x, y, z);
    }
    pcl::copyPointCloud(cloud_tmp, cloud);
    /// Add further information to all detections
    for (int detection_idx = 0; detection_idx < lidar_sensor.detection_size(); detection_idx++)
    {
        cloud.points[detection_idx].intensity = static_cast<float>(lidar_sensor.detection(detection_idx).intensity());
        cloud.points[detection_idx].reflectivity = static_cast<float>(lidar_sensor.detection(detection_idx).reflectivity());
        cloud.points[detection_idx].object_id = static_cast<int>(lidar_sensor.detection(detection_idx).object_id().value());
        cloud.points[detection_idx].rel_velocity = static_cast<float>(lidar_sensor.detection(detection_idx).radial_velocity());
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);

    // Wait a total of 100 ms between publishes
    auto MIN_INTERVAL = static_cast<std::chrono::milliseconds>(detection_interval_ms_);
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_publish_time;
    if (elapsed < MIN_INTERVAL) {
        auto wait_time = MIN_INTERVAL - elapsed;
        std::this_thread::sleep_for(wait_time);
    }
    last_publish_time = std::chrono::steady_clock::now();

    detections_publisher_->publish(output);
}

visualization_msgs::msg::Marker set_marker(const osi3::Vector3d& position,
                                           const osi3::Orientation3d& orientation,
                                           const osi3::Dimension3d& dimension,
                                           const uint64_t id,
                                           std_msgs::msg::ColorRGBA color,
                                           std::string frame,
                                           const rclcpp::Time& time)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = std::move(frame);
    marker.header.stamp = time;
    marker.id = static_cast<int>(id);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    tf2::Quaternion orientation_q;
    orientation_q.setRPY(orientation.roll(), orientation.pitch(), orientation.yaw());
    marker.pose.orientation.x = orientation_q.x();
    marker.pose.orientation.y = orientation_q.y();
    marker.pose.orientation.z = orientation_q.z();
    marker.pose.orientation.w = orientation_q.w();
    marker.scale.x = dimension.length();
    marker.scale.y = dimension.width();
    marker.scale.z = dimension.height();
    marker.color = color;
    marker.lifetime = rclcpp::Duration(1, 0);
    marker.frame_locked = false;
    return marker;
}

void ROSOutput::PublishDetectedObjects(osi3::SensorData& sensor_data)
{
    visualization_msgs::msg::MarkerArray marker_array_tracker;
    std_msgs::msg::ColorRGBA marker_color;
    if (sensor_data.moving_object_size() > 0)
    {
        for (const auto& moving_object : sensor_data.moving_object())
        {
            marker_color.set__r(0.0);
            marker_color.set__g(0.0);
            marker_color.set__b(1.0);
            marker_color.set__a(float(0.6 * moving_object.header().existence_probability()));
            visualization_msgs::msg::Marker marker = set_marker(moving_object.base().position(),
                                                                moving_object.base().orientation(),
                                                                moving_object.base().dimension(),
                                                                moving_object.header().tracking_id().value(),
                                                                marker_color,
                                                                "base_link",
                                                                rclcpp::Time(0));
            marker_array_tracker.markers.push_back(marker);
        }
    }

    detectedobjects_publisher_->publish(marker_array_tracker);
}

void ROSOutput::BroadcastTFs(osi3::SensorData& sensor_data)
{
    auto ros_time = rclcpp::Time(0);
    const osi3::MovingObject *ego_vehicle;

    /// Transform Bounding Box Center to world
    for (const osi3::MovingObject& gt_moving_object : sensor_data.sensor_view(0).global_ground_truth().moving_object())
    {
        if (gt_moving_object.id().value() == sensor_data.sensor_view(0).global_ground_truth().host_vehicle_id().value())
        {
            ego_vehicle = &gt_moving_object;
        }
    }

    if (ego_vehicle->has_base())
    {
        geometry_msgs::msg::TransformStamped bb2world_transform;
        bb2world_transform.header.stamp = ros_time;
        bb2world_transform.header.frame_id = base_frame;
        bb2world_transform.child_frame_id = "bb_center";
        bb2world_transform.transform.translation.x = ego_vehicle->base().position().x();
        bb2world_transform.transform.translation.y = ego_vehicle->base().position().y();
        bb2world_transform.transform.translation.z = ego_vehicle->base().position().z();
        tf2::Quaternion bb2world_q;
        bb2world_q.setRPY(ego_vehicle->base().orientation().roll(), ego_vehicle->base().orientation().pitch(), ego_vehicle->base().orientation().yaw());
        bb2world_transform.transform.rotation.x = bb2world_q.x();
        bb2world_transform.transform.rotation.y = bb2world_q.y();
        bb2world_transform.transform.rotation.z = bb2world_q.z();
        bb2world_transform.transform.rotation.w = bb2world_q.w();
        transform_broadcaster_->sendTransform(bb2world_transform);
    }
    else
    {
        std::cout << "OSI: Ego vehicle has no base!" << std::endl;
    }

    /// Transform Bounding Box Center to center of rear axle
    geometry_msgs::msg::TransformStamped bb2rear_transform;
    bb2rear_transform.header.stamp = ros_time;
    bb2rear_transform.header.frame_id = "bb_center";
    bb2rear_transform.child_frame_id = "base_link";
    if (ego_vehicle->has_vehicle_attributes())
    {
        bb2rear_transform.transform.translation.x = ego_vehicle->vehicle_attributes().bbcenter_to_rear().x();
        bb2rear_transform.transform.translation.y = ego_vehicle->vehicle_attributes().bbcenter_to_rear().y();
        bb2rear_transform.transform.translation.z = ego_vehicle->vehicle_attributes().bbcenter_to_rear().z();
    }
    else
    {
        bb2rear_transform.transform.translation.x = 0;
        bb2rear_transform.transform.translation.y = 0;
        bb2rear_transform.transform.translation.z = 0;
    }
    tf2::Quaternion bb2rear_q;
    bb2rear_q.setRPY(0, 0, 0);
    bb2rear_transform.transform.rotation.x = bb2rear_q.x();
    bb2rear_transform.transform.rotation.y = bb2rear_q.y();
    bb2rear_transform.transform.rotation.z = bb2rear_q.z();
    bb2rear_transform.transform.rotation.w = bb2rear_q.w();
    transform_broadcaster_->sendTransform(bb2rear_transform);

    /// Transform center of rear axle to sensor('s) origin
    if (sensor_data.feature_data().radar_sensor_size() > 0)
    {
        if (sensor_data.feature_data().radar_sensor(0).has_header())
        {
            if (sensor_data.feature_data().radar_sensor(0).header().has_mounting_position())
            {
                geometry_msgs::msg::TransformStamped rear2sensor_transform;
                rear2sensor_transform.header.stamp = ros_time;
                rear2sensor_transform.header.frame_id = "base_link";
                rear2sensor_transform.child_frame_id = "detections_" + sensor_id;
                rear2sensor_transform.transform.translation.x = sensor_data.feature_data().radar_sensor(0).header().mounting_position().position().x();
                rear2sensor_transform.transform.translation.y = sensor_data.feature_data().radar_sensor(0).header().mounting_position().position().y();
                rear2sensor_transform.transform.translation.z = sensor_data.feature_data().radar_sensor(0).header().mounting_position().position().z();
                tf2::Quaternion rear2sensor_q;
                rear2sensor_q.setRPY(sensor_data.feature_data().radar_sensor(0).header().mounting_position().orientation().roll(),
                                     sensor_data.feature_data().radar_sensor(0).header().mounting_position().orientation().pitch(),
                                     sensor_data.feature_data().radar_sensor(0).header().mounting_position().orientation().yaw());
                rear2sensor_transform.transform.rotation.x = rear2sensor_q.x();
                rear2sensor_transform.transform.rotation.y = rear2sensor_q.y();
                rear2sensor_transform.transform.rotation.z = rear2sensor_q.z();
                rear2sensor_transform.transform.rotation.w = rear2sensor_q.w();
                transform_broadcaster_->sendTransform(rear2sensor_transform);
            }
        }
        else
        {
            cerr << "Feature data radar_sensor header does not contain mounting position." << std::endl;
        }
    }
    else if (sensor_data.feature_data().lidar_sensor_size() > 0)
    {
        if (sensor_data.feature_data().lidar_sensor(0).has_header())
        {
            if (sensor_data.feature_data().lidar_sensor(0).header().has_mounting_position())
            {
                geometry_msgs::msg::TransformStamped rear2sensor_transform;
                rear2sensor_transform.header.stamp = ros_time;
                rear2sensor_transform.header.frame_id = "base_link";
                rear2sensor_transform.child_frame_id = "detections_" + sensor_id;
                rear2sensor_transform.transform.translation.x = sensor_data.feature_data().lidar_sensor(0).header().mounting_position().position().x();
                rear2sensor_transform.transform.translation.y = sensor_data.feature_data().lidar_sensor(0).header().mounting_position().position().y();
                rear2sensor_transform.transform.translation.z = sensor_data.feature_data().lidar_sensor(0).header().mounting_position().position().z();
                tf2::Quaternion rear2sensor_q;
                rear2sensor_q.setRPY(sensor_data.feature_data().lidar_sensor(0).header().mounting_position().orientation().roll(),
                                     sensor_data.feature_data().lidar_sensor(0).header().mounting_position().orientation().pitch(),
                                     sensor_data.feature_data().lidar_sensor(0).header().mounting_position().orientation().yaw());
                rear2sensor_transform.transform.rotation.x = rear2sensor_q.x();
                rear2sensor_transform.transform.rotation.y = rear2sensor_q.y();
                rear2sensor_transform.transform.rotation.z = rear2sensor_q.z();
                rear2sensor_transform.transform.rotation.w = rear2sensor_q.w();
                transform_broadcaster_->sendTransform(rear2sensor_transform);
            }
        }
        else
        {
            cerr << "Feature data lidar_sensor header does not contain mounting position." << std::endl;
        }
    }
}
