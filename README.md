# Camera Publisher 
A image from camera or video file and a camera info publisher node for ROS2.

## Features
- Works with USB cameras that are recognized by V4L2 and also IP cameras.
- It is also possible to publish frames of a video file.
- Undistortion of frames according to calibration data.
- Importing necessary configuration file generated by [camera_calibrator](https://git.fh-aachen.de/kurhan/camera_calibrator) node
- Additional parameters are stored in [Launch File](https://git.fh-aachen.de/kurhan/camera_rgb#launch-file)

## ROS Topics
### Publishers
|         Topic Name        |   Message Type  |
|:-------------------------:|:---------------:|
| `camera/image_color`        | `CompressedImage` |
| `camera/image_undist_color` | `CompressedImage` |
| `camera/camera_info`        | `CameraInfo`      |

## Getting Started
### Dependencies
* OpenCV  
To install:
  ```sh
  $ python3 -m pip install opencv-python
  ```
* PyYAML  
To install:
  ```sh
  $ python3 -m pip install pyyaml
  ```
  When you run the node, if you still get an error about PyYAML, check its version
  ```sh
  $ python3
  > import yaml
  > yaml.__version__
  ```
  If it is smaller than `5.1`, you should update it  
  ```sh
  $ python3 -m pip install -U pyyaml
  ```
* CvBridge   
To install:
  ```sh
  $ sudo apt install ros-eloquent-cv-bridge
  ```
  
### Installation
1. Clone repository to `/your_workspace/src`
    ```sh
    $ git clone https://git.fh-aachen.de/kurhan/camera_rgb.git
    ```
2. Build `/your_workspace` with `colcon`
   ```sh
   $ colcon build
   ```

## Usage
### Launch File 
It has only two parameters.
1. `read_video` is a boolean variable which set image acquisiton source either a camera or video file
     
   * If `True` then the node tries to read video file which is given in parameter `video_path` 
   
   * If `False` then the node tries to open designated camera in configuration file.

2. `video_path` keeps the absolute path of desired video file, e.g. `/home/<user>/Desktop/sample.mp4`

### How to Run Node
After setting all parameters in Launch File and configuration file properly then,
```sh
$ ros2 launch camera_rgb camera_rgb.launch.py
```

## Imported Configuration file
The node imports a YAML file generated by [camera_calibrator](https://git.fh-aachen.de/kurhan/camera_calibrator) as a configuration file. This file contains matrices for calibration and some more additional parameters which are required.
* The name of the file must be `camera.yaml` and it must be in `camera_calibration/config` directory
* `image_width` and `image_height` set resolution of camera
* `camera_source` holds index number of camera. It is also possible to enter address of an IP Camera, e.g. `http://192.168.0.123:5000/video`
* `camera_fps` sets camera frame rate.
* `camera_name` is useful when there are multiple cameras on the system.
* `camera_matrix` is a unique result derived from calculations done on checkerboard and used for undistortion of captured frames. $`f_x`$ and $`f_y`$ are focal lengths and $`c_x`$ and $`c_y`$ are optical centers
```math
camera~matrix = \begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
```
* `distortion_model` is set to `plumb bob` by default. This model requires `1x5` distrotion matrix.
* `distortion_coefficients` contains 5 coefficients and used for undistortion of captured frames.
```math
distortion~coefficients = \begin{bmatrix}
k_1 & k_2 & p_1 & p_2 & k_3
\end{bmatrix}
```
  For more info you can check OpenCV [tutorial](https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html) and [documentation](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)
