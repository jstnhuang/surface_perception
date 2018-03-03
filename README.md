# surface_perception

Simple library for segmentation of tabletop and shelf surfaces.

## Try out the demo
To run the demo, run `rosrun surface_perception demo BASE_FRAME cloud_in:=your_cloud_topic` in your terminal.
`BASE_FRAME` is the name of a frame whose Z-axis points "up" away from gravity.
It not specified, `base_link` will be used as the `BASE_FRAME`.

Add a Marker topic to see the segmentation output.
