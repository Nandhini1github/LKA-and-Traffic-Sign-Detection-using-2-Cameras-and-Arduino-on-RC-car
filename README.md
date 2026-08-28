# Group 3 Autonomous Robo-Car Project

MATLAB code for a two-laptop autonomous robo-car prototype. One laptop detects lane markings and commands steering/throttle through an Arduino. The second laptop detects stop and school-zone signs and sends vehicle commands over TCP.

## Project layout

| File | Purpose |
| --- | --- |
| `LAPTOP1LANEDETECTION.m` | USB-camera lane detection, lane tracking, Stanley steering control, and Arduino servo/ESC output. |
| `LAPTOP2SIGNDETECTION.m` | USB-camera sign detection using the trained R-CNN detector plus a color-based fallback; sends commands to the vehicle over TCP. |
| `SIGNDETECTIONTRAININGPART1.m` | Trains and evaluates the CIFAR-10 classification network used as the R-CNN backbone. |
| `RCNNSIGNDETECTIONTRAININGPART2.m` | Trains the stop/school-zone R-CNN from Image Labeler ground truth and the Part 1 network. |
| `lane_camera_params.mat` | Camera-calibration data associated with the lane camera. The lane runtime also contains the calibrated intrinsic values used at 640 x 480. |
| `roboCarSignDetector.mat` | Trained sign detector loaded by the Laptop 2 runtime. |

## System flow

```text
Lane camera -> image preprocessing -> Hough lane segments -> lane tracking
            -> cross-track error -> Stanley controller -> Arduino steering/ESC

Sign camera -> trained R-CNN + color fallback -> stop/school-zone decision
            -> TCP command -> vehicle-side command receiver
```

The two runtime scripts are independent MATLAB programs intended to run on their respective laptops. This repository does not include the vehicle-side TCP receiver used by `LAPTOP2SIGNDETECTION.m`.

## Why these methods were chosen

- **Stanley steering control** combines heading error and cross-track error, making it suitable for following a visible lane centerline without requiring a full vehicle dynamics model. The script adds filtering, stale-lane handling, and steering limits to reduce abrupt corrections on the indoor track.
- **Hough-transform lane extraction** is computationally light and works well for high-contrast painted lane boundaries. It is practical for real-time processing at the script's 320 x 240 processing resolution.
- **R-CNN sign detection** provides both a class and a bounding box, which is more useful than image-level classification when signs occupy only part of the camera frame. The CIFAR-10 network supplies the learned feature backbone.
- **Color-based sign fallback** provides a secondary cue for strongly red stop signs and yellow school-zone signs when the learned detector has low confidence. Region, size, saturation, and aspect-ratio filters limit false detections.
- **TCP coordination** keeps sign perception on a second laptop while allowing it to send simple decisions to the vehicle controller over the lab network.

## Requirements

- MATLAB (the source project was prepared for MATLAB R2026a)
- Computer Vision Toolbox
- Deep Learning Toolbox
- MATLAB Support Package for USB Webcams
- MATLAB Support Package for Arduino Hardware
- Arduino Uno, steering servo, ESC, and a compatible USB camera for the lane runtime
- A USB camera and access to the vehicle command receiver for the sign runtime
- CIFAR-10 MATLAB batch files for training Part 1
- Labeled sign images exported from MATLAB Image Labeler as `gTruth` for training Part 2

Use `ver`, `webcamlist`, and `arduinosetup` in MATLAB to confirm the installed products and hardware before running the vehicle.

## Configuration before running

Review these project-specific values instead of using them unchanged on another computer:

| Script | Setting | Current value |
| --- | --- | --- |
| `LAPTOP1LANEDETECTION.m` | `CAM_INDEX` | `2` |
| `LAPTOP1LANEDETECTION.m` | `ARDUINO_PORT` | `COM3` |
| `LAPTOP1LANEDETECTION.m` | `STEER_PIN`, `ESC_PIN` | `D9`, `D13` |
| `LAPTOP2SIGNDETECTION.m` | `VEHICLE_IP` | `172.27.37.191` |
| `LAPTOP2SIGNDETECTION.m` | `VEHICLE_PORT` | `5005` |
| `LAPTOP2SIGNDETECTION.m` | camera name | `USB Camera` |
| `SIGNDETECTIONTRAININGPART1.m` | `cifarFolder` | update to the local CIFAR-10 folder |
| `RCNNSIGNDETECTIONTRAININGPART2.m` | `testImagePath` | update to a local test image |

The runtime scripts can move physical hardware. Raise the wheels, keep an emergency stop available, verify neutral PWM and steering center, and test at low power before placing the vehicle on the track.

## Run the included detector

1. Open MATLAB and make this repository the Current Folder.
2. Confirm `roboCarSignDetector.mat` is present.
3. Update the vehicle IP, port, camera name, and camera orientation in `LAPTOP2SIGNDETECTION.m`.
4. Start the vehicle-side TCP receiver.
5. Run:

```matlab
run('LAPTOP2SIGNDETECTION.m')
```

The script saves `diag_frame_v10.png` as a camera-orientation diagnostic. If TCP connection fails, it continues in display-only mode.

## Run lane following

1. Raise the drive wheels and verify the Arduino, camera, steering center, and ESC neutral values.
2. Confirm the selected camera with `webcamlist` and the Arduino COM port in Windows Device Manager.
3. Run:

```matlab
run('LAPTOP1LANEDETECTION.m')
```

Keep the MATLAB window and physical emergency stop accessible throughout the test.

## Re-train the sign detector

### Part 1: classification backbone

1. Download the CIFAR-10 MATLAB version and extract its batch files.
2. Set `cifarFolder` in `SIGNDETECTIONTRAININGPART1.m` to that directory.
3. Run:

```matlab
run('SIGNDETECTIONTRAININGPART1.m')
```

The script trains on CPU by default and writes `cifar10Net.mat` to the Current Folder.

### Part 2: R-CNN detector

1. Label stop and school-zone signs in MATLAB Image Labeler.
2. Export the labels to the workspace with the variable name `gTruth`.
3. Keep `cifar10Net.mat` in the Current Folder.
4. Update `testImagePath` if a post-training test is wanted.
5. Run:

```matlab
run('RCNNSIGNDETECTIONTRAININGPART2.m')
```

The script writes `roboCarSignDetector.mat`. Review detector performance on images that were not used for training before using the model to influence vehicle motion.

## Validation scope

Static MATLAB code analysis can verify syntax and flag code-quality issues without connecting hardware. End-to-end validation still requires the configured cameras, Arduino/ESC/servo assembly, vehicle-side TCP receiver, and the intended indoor track. A successful script start by itself does not establish safe steering, braking, or sign-recognition performance.
