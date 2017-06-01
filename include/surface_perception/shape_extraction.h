#ifndef _SURFACE_PERCEPTION_SHAPE_EXTRACTION_H_
#define _SURFACE_PERCEPTION_SHAPE_EXTRACTION_H_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace surface_perception {
bool FitBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input,
            const pcl::PointIndicesPtr& indices,
            const pcl::ModelCoefficients::Ptr& model, geometry_msgs::Pose* pose,
            geometry_msgs::Vector3* dimensions);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SHAPE_EXTRACTION_
