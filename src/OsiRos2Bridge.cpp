//
// Copyright 2023 Persival GmbH
//

#include "OsiRos2Bridge.h"
#include "osi_sensordata.pb.h"

void OsiRos2Bridge::Init(const int sensor_id, const std::string& pcl_topic, const int detection_interval_ms, const std::string& frame_id)
{
    myros_.SetSensorID(sensor_id);
    myros_.Init(pcl_topic, detection_interval_ms, frame_id);
}

osi3::SensorData OsiRos2Bridge::Step(osi3::SensorData sensor_data, const std::string& frame_id)
{
    if (frame_id.empty())
    {
        myros_.BroadcastTFs(sensor_data);
        myros_.PublishGtObjects(sensor_data);
        myros_.PublishDetectedObjects(sensor_data);
    }

    myros_.PublishDetections(sensor_data, frame_id);
    myros_.PublishImage(sensor_data, frame_id);

    return sensor_data;
}

void OsiRos2Bridge::Term()
{

}

