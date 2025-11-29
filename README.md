# ASAM OSI ROS2 Bridge

This [FMU](https://fmi-standard.org/) receives [ASAM OSI](https://github.com/OpenSimulationInterface/open-simulation-interface) SensorData and outputs various information as ROS2 messages.
It enables ROS2 nodes to receive data from ASAM OSI FMUs, e.g. synthetic perception sensor data from a lidar or radar sensor model FMU.
Furthermore, ROS2 visualization tools, such as [rviz2](https://github.com/ros2/rviz) or [Lichtblick](https://github.com/lichtblick-suite/lichtblick) can be used to visualize ASAM OSI data.
If the ASAM OSI model_reference contains a path to a 3D model file and in the same path a .dae file exists, it is set as the marker mesh in the ROS2 marker array.

Here is a list of all ASAM OSI fields that are sent as ROS2 messages.

| OSI Field                                         | ROS Topic                          | Message Type |
|---------------------------------------------------|------------------------------------|--------------|
| sensor_view(0).global_ground_truth.moving_object  | gt_marker_<sensor_id>              | MarkerArray  |
| feature_data.lidar/radar_sensor(0).detection      | detection_<sensor_id>              | PointCloud2  |
| sensor_view(0).camera_sensor_view(0).image_data() | camera_<sensor_id>                 | Image        |
| moving_object                                     | detectedobjects_marker_<sensor_id> | MarkerArray  |

The <sensor_id> is taken from the sensor_data.sensor_id field.

Next to the ROS2 topics named above, a tf is published.
It contains the following frames, that are coherent with the corresponding [ASAM OSI coordinate systems](https://opensimulationinterface.github.io/osi-antora-generator/asamosi/latest/interface/architecture/reference_points_coordinate_systems.html):

| Frame                  | Description                                                                                             |
|------------------------|---------------------------------------------------------------------------------------------------------|
| world                  | Fixed global coordinate system                                                                          |
| bb_center              | Center of the bounding box of the host vehicle                                                          |
| base_link              | Host vehicle coordinate system (center of rear axle) as defined by the osi3::bbcenter_to_rear parameter |
| detections_<sensor_id> | Sensor coordinate system of sensor with given ID                                                        |

## Parameters

| Parameter Name | Description                                                                | Default Value          |
|----------------|----------------------------------------------------------------------------|------------------------|
| pcl_topic      | Name of the point cloud topic.                                             | detection_<sensor_id>  |
| frame_id       | Frame_id of the TF. If set, no TFs and no Ground Truth data are published. | detections_<sensor_id> |

## Usage

Install dependencies and use built and packaged FMU in a co-simulation.

### Dependencies

Install [ROS 2](https://docs.ros.org/en/humble/) according to the official install instructions.
Also install the following package:

- `ros-<distro>-pcl-ros`
- `ros-<distro>-tf2-geometry-msgs`

### Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

You may set the cmake variable `INSTALL_DIR` to specify the installation directory for the FMU.
