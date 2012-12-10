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

## Publications

The following publications describe the approach:

 *   **Real-Time Visual Odometry from Dense RGB-D Images** (F. Steinbruecker, J. Sturm, D. Cremers), In Workshop on Live Dense Reconstruction with Moving Cameras at the Intl. Conf. on Computer Vision (ICCV), 2011.

## License

The packages *dvo_core*, *dvo_ros*, and *dvo_benchmark* are licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

The package *sophus* is licensed under the MIT License, see http://opensource.org/licenses/MIT.
