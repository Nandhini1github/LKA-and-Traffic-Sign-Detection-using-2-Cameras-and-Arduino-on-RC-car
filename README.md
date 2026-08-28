# LKA and Traffic Sign Detection Using 2 Cameras and Arduino on an RC Car

This MATLAB project uses one camera with Hough-transform lane detection and a Stanley lateral controller to command the RC car's steering and throttle through an Arduino, while a second camera uses an R-CNN detector with a color-based fallback to recognize stop and school-zone signs. The included scripts provide CIFAR-10 and R-CNN training, calibrated lane-camera data, the trained sign detector, and TCP communication between the sign-detection laptop and the vehicle.
