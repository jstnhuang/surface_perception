#ifndef _SURFACE_PERCEPTION_SURFACE_OBJECTS_H_
#define _SURFACE_PERCEPTION_SURFACE_OBJECTS_H_

#include <vector>

#include "surface_perception/object.h"
#include "surface_perception/surface.h"

namespace surface_perception {
/// \brief Represents a surface with objects above it.
struct SurfaceObjects {
 public:
  Surface surface;
  std::vector<Object> objects;
};
}  // namespace surface_perception

#endif  // _SURFACE_PERCEPTION_SURFACE_OBJECTS_H_
