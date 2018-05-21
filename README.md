# Canny Graph Scene Segmentation

This is a version of the Canny Graph Segmentation algorithm presented by Giorgio Toscana and Stefano Rosa (https://arxiv.org/abs/1605.03746) implemented to work as a ROS node with an associated service. The service takes a RGB image and depth image, and returns a segmented scene as an array of image masks -- one mask for each segment discovered. It has also been updated and modified to work with ROS kinetic and OpenCV3, though this required adaptation of some visualisation functionality.

As in the authors' original implementation, parameters are contained within the code (and there are quite a lot of them, and tweaking them for your use-case is essential). I would recommend using the original implementation to work out parameter tunings etc. as the provided visualisation components are excellent, which this version lacks.

In the tests directory is an example of loading some images, sending them to the service, and visualising the result. Please pay attention to this, as depth images must be provided in the right format or else things get weird.

If you use this in your systems, please cite the original paper:
