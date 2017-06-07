#ifndef _SURFACE_PERCEPTION_VISUALIZATION_H_
#define _SURFACE_PERCEPTION_VISUALIZATION_H_

#include <vector>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "surface_perception/object.h"
#include "surface_perception/surface.h"
#include "surface_perception/surface_objects.h"

namespace surface_perception {
/// \brief A visualization for a surface.
///
/// Publishes markers to a topic of type visualization_msgs/Marker. The markers
/// show the bounding boxes around the surface and the objects. The point clouds
/// are not visualized.
///
/// \b Example:
/// \code
///   SurfaceViz viz(marker_pub);
///   viz.Hide();
///   viz.set_surface_objects(surfaces);
///   viz.Show();
/// \endcode
class SurfaceViz {
 public:
  /// \brief Construct the visualization with the given publisher.
  explicit SurfaceViz(const ros::Publisher& marker_pub);

  /// \brief Sets the SurfaceObjects to use.
  void set_surface_objects(const std::vector<SurfaceObjects>& surfaces);

  /// \brief Publishes the markers to the marker topic.
  void Show();

  /// \brief Deletes the markers that were most recently published with Show().
  void Hide();

 private:
  ros::Publisher marker_pub_;
  std::vector<SurfaceObjects> surfaces_;
  std::vector<visualization_msgs::Marker> markers_;
};

/// Generates a marker for the given surface. Does not set the namespace or ID.
void SurfaceMarker(const Surface& surface, visualization_msgs::Marker* marker);

/// Generates markers for the given objects. Does not set the namespaces or IDs.
void ObjectMarkers(const std::vector<Object>& objects,
                   std::vector<visualization_msgs::Marker>* markers);

/// Generates markers for the given surfaces. Sets namespaces and IDs like:
/// First surface: ns=surface, id=0
/// First object on first surface: ns=surface_0, id=0
void SurfaceMarkers(const std::vector<SurfaceObjects>& surfaces,
                    std::vector<visualization_msgs::Marker>* markers);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_VISUALIZATION_H_
