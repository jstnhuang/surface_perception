#ifndef _SURFACE_PERCEPTION_VISUALIZATION_H_
#define _SURFACE_PERCEPTION_VISUALIZATION_H_

#include <vector>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "surface_perception/object.h"
#include "surface_perception/surface.h"
#include "surface_perception/surface_objects.h"

namespace surface_perception {
class SurfaceViz {
 public:
  explicit SurfaceViz(const ros::Publisher& marker_pub);
  void set_surface_objects(const std::vector<SurfaceObjects>& surfaces);
  void Show();
  void Hide();

 private:
  ros::Publisher marker_pub_;
  std::vector<SurfaceObjects> surfaces_;
  std::vector<visualization_msgs::Marker> markers_;
};

// Generates a marker for the given surface. Does not set the namespace or ID.
void SurfaceMarker(const Surface& surface, visualization_msgs::Marker* marker);

// Generates markers for the given objects. Does not set the namespaces or IDs.
void ObjectMarkers(const std::vector<Object>& objects,
                   std::vector<visualization_msgs::Marker>* markers);

// Generates markers for the given surfaces. Sets namespaces and IDs like:
// First surface: ns=surface, id=0
// First object on first surface: ns=surface_0, id=0
void SurfaceMarkers(const std::vector<SurfaceObjects>& surfaces,
                    std::vector<visualization_msgs::Marker>* markers);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_VISUALIZATION_H_
