#include <vector>

#include "Eigen/Eigen"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"
#include "surface_perception/visualization.h"

using surface_perception::SurfaceObjects;
using surface_perception::SurfaceViz;

class Demo {
 public:
  Demo(const SurfaceViz& viz);
  void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud);

 private:
  SurfaceViz viz_;
};

Demo::Demo(const SurfaceViz& viz) : viz_(viz) {}

void Demo::Callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  PointCloudC::Ptr pcl_cloud(new PointCloudC);
  pcl::fromROSMsg(*cloud, *pcl_cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);

  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(pcl_cloud);
  Eigen::Vector4f min;
  min << 0, -1, 0.3, 1;
  crop.setMin(min);
  Eigen::Vector4f max;
  max << 1, 1, 2, 1;
  crop.setMax(max);
  crop.filter(point_indices->indices);

  double horizontal_tolerance_degrees;
  ros::param::param("horizontal_tolerance_degrees",
                    horizontal_tolerance_degrees, 10.0);
  double margin_above_surface;
  ros::param::param("margin_above_surface", margin_above_surface, 0.005);
  double cluster_distance;
  ros::param::param("cluster_distance", cluster_distance, 0.01);
  int min_cluster_size;
  ros::param::param("min_cluster_size", min_cluster_size, 10);
  int max_cluster_size;
  ros::param::param("max_cluster_size", max_cluster_size, 10000);

  surface_perception::Segmentation seg;
  seg.set_input_cloud(pcl_cloud);
  seg.set_indices(point_indices);
  seg.set_horizontal_tolerance_degrees(horizontal_tolerance_degrees);
  seg.set_margin_above_surface(margin_above_surface);
  seg.set_cluster_distance(cluster_distance);
  seg.set_min_cluster_size(min_cluster_size);
  seg.set_max_cluster_size(max_cluster_size);

  std::vector<SurfaceObjects> surface_objects;
  bool success = seg.Segment(&surface_objects);

  if (!success || surface_objects.size() == 0) {
    ROS_ERROR("Failed to segment scene!");
  } else {
    ROS_INFO("Found %ld objects", surface_objects[0].objects.size());
  }

  viz_.Hide();
  viz_.set_surface_objects(surface_objects);
  viz_.Show();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_perception_demo");
  ros::NodeHandle nh;
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("surface_objects", 20);
  SurfaceViz viz(marker_pub);
  Demo demo(viz);
  ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "cloud_in", 1, &Demo::Callback, &demo);
  ros::spin();
}
