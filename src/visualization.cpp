#include "surface_perception/visualization.h"

#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "surface_perception/surface_objects.h"
#include "hcr_common_markers/axes_marker.h"

using visualization_msgs::Marker;

namespace surface_perception {
SurfaceViz::SurfaceViz(const ros::Publisher& marker_pub)
    : marker_pub_(marker_pub), surfaces_(), markers_() {}

void SurfaceViz::set_surface_objects(
    const std::vector<SurfaceObjects>& surfaces) {
  surfaces_ = surfaces;
}

void SurfaceViz::Show() {
  Hide();
  SurfaceMarkers(surfaces_, &markers_);
  for (size_t i = 0; i < markers_.size(); ++i) {
    const Marker& marker = markers_[i];
    marker_pub_.publish(marker);
  }
}

void SurfaceViz::Hide() {
  for (size_t i = 0; i < markers_.size(); ++i) {
    Marker marker;
    marker.ns = markers_[i].ns;
    marker.id = markers_[i].id;
    marker.action = Marker::DELETE;
    marker_pub_.publish(marker);
  }
  markers_.clear();
}

void SurfaceMarker(const Surface& surface, visualization_msgs::Marker* marker) {
  marker->type = Marker::CUBE;
  marker->header = surface.pose_stamped.header;
  marker->pose = surface.pose_stamped.pose;
  marker->scale = surface.dimensions;
  marker->color.r = 1;
  marker->color.b = 1;
  marker->color.a = 0.5;
}

void ObjectMarkers(const std::vector<Object>& objects,
                   std::vector<visualization_msgs::Marker>* markers) {
  for (size_t i = 0; i < objects.size(); ++i) {
    const Object& object = objects[i];
    Marker marker;
    marker.type = Marker::CUBE;
    marker.header = object.pose_stamped.header;
    marker.pose = object.pose_stamped.pose;
    marker.scale = object.dimensions;
    marker.color.g = 1;
    marker.color.a = 0.5;
    markers->push_back(marker);

    Marker axesMarker;
    hcr_common_markers::GetAxesMarker(&axesMarker);
    axesMarker.header = object.pose_stamped.header;
    axesMarker.pose = object.pose_stamped.pose;
    markers->push_back(axesMarker);
  }
}

void SurfaceMarkers(const std::vector<SurfaceObjects>& surfaces,
                    std::vector<visualization_msgs::Marker>* markers) {
  for (size_t surface_i = 0; surface_i < surfaces.size(); ++surface_i) {
    const SurfaceObjects& surface_objects = surfaces[surface_i];
    Marker surface_marker;
    SurfaceMarker(surface_objects.surface, &surface_marker);
    surface_marker.ns = "surface";
    surface_marker.id = surface_i;
    markers->push_back(surface_marker);

    std::stringstream obj_ns;
    obj_ns << "surface_" << surface_i;
    std::vector<Marker> object_markers;
    ObjectMarkers(surface_objects.objects, &object_markers);
    for (size_t obj_i = 0; obj_i < object_markers.size(); ++obj_i) {
      object_markers[obj_i].ns = obj_ns.str();
      object_markers[obj_i].id = obj_i;
      markers->push_back(object_markers[obj_i]);
    }
  }
}
}  // namespace surface_perception
