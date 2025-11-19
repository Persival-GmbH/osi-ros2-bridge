// 
// Copyright 2022 Persival GmbH.
//

#ifndef POINT_TYPES_H
#define POINT_TYPES_H

namespace persival_pcl
{
struct PointXYZIO
{
  PCL_ADD_POINT4D;      // quad-word XYZ
  float intensity;      // detection intensity
  float reflectivity;   // detection reflectivity
  int object_id;        //id of hit object
  float rel_velocity;   //relative velocity
};
}  // namespace persival_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(persival_pcl::PointXYZIO,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, reflectivity, reflectivity)
                                  (int, object_id, object_id)
                                  (float, rel_velocity, rel_velocity))

#endif  // POINT_TYPES_H
