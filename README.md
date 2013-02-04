# Dense Visual Odometry (dvo)

These packages provide an implementation of the rigid body motion estimation of an RGB-D camera from consecutive images.

 *  **dvo_core**
    
    Core implementation of the motion estimation algorithm. 
    
 *  **dvo_ros**
    
    Integration of *dvo_core* with ROS.
    
 *  **dvo_benchmark**
    
    Integration of *dvo_core* with TUM RGB-D benchmark, see http://vision.in.tum.de/data/datasets/rgbd-dataset.
    
 *  **sophus**
    
    ROS package wrapper for Hauke Strasdat's Sophus library, see https://github.com/strasdat/Sophus.
    

## Installation

Checkout the branch for your ROS version into a folder in your `ROS_PACKAGE_PATH` and build the packages with `rosmake`.

 *  ROS Fuerte:
    
    ```bash
    git clone -b fuerte git://github.com/tum-vision/dvo.git
    rosmake dvo_core dvo_ros dvo_benchmark
    ```

 *  ROS Electric:
    
    You need to install `perception_pcl_unstable` with PCL version 1.5+.
    
    ```bash
    git clone -b electric git://github.com/tum-vision/dvo.git
    rosmake dvo_core dvo_ros dvo_benchmark
    ```

## Usage

Estimating the camera trajectory from an RGB-D image stream:

 *  Start the OpenNI camera driver: `roslaunch openni_launch openni.launch`
 *  Start the dvo *camera_tracker* node: `rosrun dvo_ros camera_tracker`
 *  Start dynamic_reconfigure GUI
    *   In `/camera/driver` enable *depth_registration* on
    *   In `/camera_tracker` enable *reconstruction*, *use_weighting*, *run_dense_tracking*, and *use_dense_tracking_estimate*

If everything works, the stdout of the *camera_tracker* node should show `[ WARN] [1355131430.132265592]: RGB image size has changed, resetting tracker!` and the camera pose is published on the `/rgbd/pose` topic. You can restart the camera motion estimation by disabling and enabling the *run_dense_tracking* option.

For visualization:

 *  Start RVIZ
 *  Set the *Target Frame* to `/world`
 *  Add an *Interactive Marker* display and set its *Update Topic* to `/dvo_vis/update`
 *  Add a *PointCloud2* display and set its *Topic* to `/dvo_vis/cloud`

The red camera shows the current camera position. The blue camera displays the initial camera position.

## Publications

The following publications describe the approach:

 *   **Robust Odometry Estimation for RGB-D Cameras** (C. Kerl, J. Sturm, D. Cremers), In Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA), 2013
 *   **Real-Time Visual Odometry from Dense RGB-D Images** (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.

## License

The packages *dvo_core*, *dvo_ros*, and *dvo_benchmark* are licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

The package *sophus* is licensed under the MIT License, see http://opensource.org/licenses/MIT.
