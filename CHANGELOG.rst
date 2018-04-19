^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package surface_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2018-04-18)
------------------
* Removed warning message for surfaces with no objects.
* Fixed objects intersecting with surface.
* Fixed bugs with shape extraction.
* Decouple the usage of margin_above_surface and max_point_distance
* Fixed bug with standardizing object orientation (`#9 <https://github.com/jstnhuang/surface_perception/issues/9>`_)
  * Pulled code to standardize object orientations into a separate function
  * Fixed a bug with this function
  * Added tests
  * Added a visualization of the object orientations
* Updated Doxygen mainpage.
* Contributors: Justin Huang, Yu-Tang Peng

1.0.1 (2018-03-06)
------------------
* Allowed surfaces with no object on them.
* Fixed occasional crashes due to convex hull computation.
* Added optional point cloud frame argument in demo to support target frame other than base_link.
* Contributors: Justin Huang, Yu-Tang Peng

1.0.0 (2018-02-13)
------------------
* Multi-shelf detection
* Contributors: Justin Huang, Yu-Tang Peng

0.2.1 (2017-08-28)
------------------
* Make new x-axis the one closer to the data's x-axis.
* Contributors: Justin Huang

0.2.0 (2017-08-28)
------------------
* Changed oriented bounding box fitting algorithm so that the x direction always points towards the shorter side.
* Contributors: Justin Huang

0.1.3 (2017-07-24)
------------------
* Added mainpage documentation.
* Contributors: Justin Huang

0.1.2 (2017-07-19)
------------------
* Fixed header files not being installed.
* Contributors: Justin Huang

0.1.1 (2017-06-07)
------------------
* Added missing dependency.
* Contributors: Justin Huang

0.1.0 (2017-06-07)
------------------
* Initial implementation of surface_perception.
* Contributors: Justin Huang
