# 3D Wound Contour Extractor

A C++ program that uses an Ensenso 3D camera to obtain the colored point cloud of a wound in a body. It maps the XYZ coordinates of the points in the point cloud to their corresponding RGBA values to extract XYZ coordinates of the contour of the wound. These XYZ coordinates are then fed into a Robot Operating System (ROS) node to trace the path of an ink-jet nozzle that will move along the wound contour to fill the wound with healing fluids.
