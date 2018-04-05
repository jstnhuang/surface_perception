#ifndef _SURFACE_PERCEPTION_AXES_MARKER_H_
#define _SURFACE_PERCEPTION_AXES_MARKER_H_

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace surface_perception {
/// \brief This helper function generates a marker array where axes are
///  represented by three markers of colored cylinder bars.
///
/// \b Example usage:
/// \code
///   geometry_msgs::Pose pose;
///   pose.position.x = pose.position.y = pose.position.z = 1.0;
///   pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
///   pose.orientation.w = 1.0;
///   visualization_msgs::MarkerArray axes_markers =
///       surface_perception::GetAxesMarkerArray("markers", "base_link", pose,
///                       1.0);
/// \endcode
///
/// \param[in] name_space The name space for the three markers to be created.
/// \param[in] frame_id The id of the frame where the markers will be.
/// \param[in] pose The pose of the origin where the three axes intersect.
/// \param[in] scale The length of each colored cylinder bars. The cylinder
///  diameters are 10% of scale, or 1 cm if 10% scale is less than 1 cm.
///
/// \return A MarkerArray of three Marker objects is returned where x-axis is
///  a red cylinder, y-axis is a green cylinder and z-axis is a blue cylinder.
visualization_msgs::MarkerArray GetAxesMarkerArray(
    const std::string& name_space, const std::string& frame_id,
    geometry_msgs::Pose pose, double scale);
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_AXES_MARKER_H_
