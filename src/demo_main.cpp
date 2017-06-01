#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"

using surface_perception::SurfaceObjects;

void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  PointCloudC::Ptr pcl_cloud(new PointCloudC);
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  surface_perception::Segmentation seg;
  seg.set_input_cloud(pcl_cloud);

  std::vector<SurfaceObjects> surface_objects;
  bool success = seg.Segment(&surface_objects);

  if (!success || surface_objects.size() == 0) {
    ROS_ERROR("Failed to segment scene!");
  } else {
    ROS_INFO("Found %ld objects", surface_objects[0].objects.size());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_perception_demo");
  ros::NodeHandle nh;
  ros::Subscriber pc_sub =
      nh.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, Callback);
  ros::spin();
}
