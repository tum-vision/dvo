/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dvo_ros/visualization/ros_camera_trajectory_visualizer.h>

#include <dvo/visualization/async_point_cloud_builder.h>
#include <pcl_ros/point_cloud.h>

#include <interactive_markers/interactive_marker_server.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <eigen_conversions/eigen_msg.h>

namespace dvo_ros
{
namespace visualization
{

namespace internal
{

using namespace dvo::visualization;

class RosCameraVisualizer : public CameraVisualizer
{
public:
  RosCameraVisualizer(ros::NodeHandle& nh, std::string name, interactive_markers::InteractiveMarkerServer& marker_server) :
    marker_server_(marker_server),
    name_(name),
    visibility_(ShowCameraAndCloud)
  {
    std::string topic = std::string("dvo_vis/") + name + std::string("_cloud");
    point_cloud_topic_ = nh.advertise<AsyncPointCloudBuilder::PointCloud>(topic, 1, true);

    AsyncPointCloudBuilder::DoneCallback done = boost::bind(&RosCameraVisualizer::onPointCloudBuilt, this, _1);
    point_cloud_builder_.done(done);

    createInteractiveCameraMarker(marker_);
    marker_callback_ = boost::bind(&RosCameraVisualizer::onMarkerFeedback, this, _1);
  }

  virtual ~RosCameraVisualizer() {};

  virtual void show(Option option = ShowCameraAndCloud)
  {
    visibility_ = option;

    updateVisualization();
  }

  virtual void hide()
  {
    show(ShowNothing);
  }

  virtual CameraVisualizer& update(const dvo::core::RgbdImage& img, const dvo::core::IntrinsicMatrix& intrinsics, const Eigen::Affine3d& pose)
  {
    // update marker
    tf::poseEigenToMsg(pose, marker_.pose);

    visualization_msgs::InteractiveMarker tmp;
    if(!marker_server_.get(name_, tmp) || hasColorChanged(tmp))
    {
      updateMarkerColor(marker_);
      marker_server_.insert(marker_, marker_callback_);
    }
    else
    {
      marker_server_.setPose(name_, marker_.pose);
    }

    // delete old cloud
    cloud_.reset();

    // build new cloud
    point_cloud_builder_.build(img, intrinsics, pose);

    return *this;
  }
private:
  interactive_markers::InteractiveMarkerServer& marker_server_;
  visualization_msgs::InteractiveMarker marker_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback marker_callback_;
  std::string name_;
  ros::Publisher point_cloud_topic_;

  AsyncPointCloudBuilder point_cloud_builder_;
  AsyncPointCloudBuilder::PointCloud::Ptr cloud_;

  Option visibility_;

  void onPointCloudBuilt(const AsyncPointCloudBuilder::PointCloud::Ptr& cloud)
  {
    cloud_ = cloud;
    cloud_->is_dense = true;

    updateVisualization();
  }

  void onMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback)
  {
    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
      switch(visibility_)
      {
        case ShowCameraAndCloud:
          show(ShowCamera);
          break;
        case ShowCamera:
          show(ShowCameraAndCloud);
          break;
        default:
          hide();
          break;
      }
    }
  }

  void updateVisualization()
  {
    // empty cloud somehow needs one point for rviz to update
    AsyncPointCloudBuilder::PointCloud::Ptr empty_cloud(new AsyncPointCloudBuilder::PointCloud);
    empty_cloud->points.push_back(pcl::PointXYZRGB());

    AsyncPointCloudBuilder::PointCloud::Ptr cloud;

    switch(visibility_)
    {
      case ShowCameraAndCloud:
        cloud = cloud_;
        break;
      case ShowCamera:
        cloud = empty_cloud;
        break;
      default:
        marker_server_.erase(name_);
        cloud = empty_cloud;
        break;
    }

    marker_server_.applyChanges();

    if(cloud)
    {
      cloud->header.frame_id = "world";
      point_cloud_topic_.publish(cloud);
    }
  }

  bool hasColorChanged(const visualization_msgs::InteractiveMarker& m)
  {
    return
    std::abs(m.controls[0].markers[0].color.r - color().r) > 1e-3 ||
    std::abs(m.controls[0].markers[0].color.g - color().g) > 1e-3 ||
    std::abs(m.controls[0].markers[0].color.b - color().b) > 1e-3;
  }

  void createInteractiveCameraMarker(visualization_msgs::InteractiveMarker& marker)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    createCameraMarker(control);

    marker.name = name_;
    marker.header.frame_id = "world";
    marker.controls.push_back(control);
  }

  void createCameraMarker(visualization_msgs::InteractiveMarkerControl& parent)
  {
    visualization_msgs::Marker m, mbox;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.scale.x = 0.005;
    m.color.a = 1.0f;

    mbox.type = visualization_msgs::Marker::CUBE;
    mbox.color.a = 1.0f;

    double small, big, length;
    small = 0.02;
    big = small * 2.0;
    length = 0.05;

    mbox.scale.x = big;
    mbox.scale.y = big;
    mbox.scale.z = small;
    mbox.pose.position.z = -0.5 * small;

    geometry_msgs::Point p;

    // small rectangle
    p.z = 0.0;

    p.x = small;
    p.y = small;

    m.points.push_back(p);

    p.x = -small;
    p.y = small;

    m.points.push_back(p);
    m.points.push_back(p);

    p.x = -small;
    p.y = -small;

    m.points.push_back(p);
    m.points.push_back(p);

    p.x = small;
    p.y = -small;

    m.points.push_back(p);
    m.points.push_back(p);

    p.x = small;
    p.y = small;

    m.points.push_back(p);

    // big rectangle
    p.z = length;

    p.x = big;
    p.y = big;

    m.points.push_back(p);

    p.x = -big;
    p.y = big;

    m.points.push_back(p);
    m.points.push_back(p);

    p.x = -big;
    p.y = -big;

    m.points.push_back(p);
    m.points.push_back(p);

    p.x = big;
    p.y = -big;

    m.points.push_back(p);
    m.points.push_back(p);

    p.x = big;
    p.y = big;

    m.points.push_back(p);

    // connecting lines
    p.x = small;
    p.y = small;
    p.z = 0.0;

    m.points.push_back(p);

    p.x = big;
    p.y = big;
    p.z = length;

    m.points.push_back(p);
    p.x = -small;
    p.y = small;
    p.z = 0.0;

    m.points.push_back(p);

    p.x = -big;
    p.y = big;
    p.z = length;

    m.points.push_back(p);

    p.x = -small;
    p.y = -small;
    p.z = 0.0;

    m.points.push_back(p);

    p.x = -big;
    p.y = -big;
    p.z = length;

    m.points.push_back(p);

    p.x = small;
    p.y = -small;
    p.z = 0.0;

    m.points.push_back(p);

    p.x = big;
    p.y = -big;
    p.z = length;

    m.points.push_back(p);

    parent.markers.push_back(m);
    parent.markers.push_back(mbox);
  }

  void updateMarkerColor(visualization_msgs::InteractiveMarker& marker)
  {
    marker.controls[0].markers[0].color.r = color().r;
    marker.controls[0].markers[0].color.g = color().g;
    marker.controls[0].markers[0].color.b = color().b;
    marker.controls[0].markers[1].color.r = color().r;
    marker.controls[0].markers[1].color.g = color().g;
    marker.controls[0].markers[1].color.b = color().b;
  }
};

class RosTrajectoryVisualizer : public TrajectoryVisualizer
{
public:
  RosTrajectoryVisualizer(std::string& name, interactive_markers::InteractiveMarkerServer& marker_server) :
    marker_server_(marker_server)
  {
    createTrajectoryMarker(name, marker_);
  }

  virtual ~RosTrajectoryVisualizer()
  {
    marker_server_.erase(marker_.name);
  };

  virtual TrajectoryVisualizer& add(const Eigen::Affine3d& pose)
  {
    updateMarkerColor();

    geometry_msgs::Point p;
    p.x = pose.translation()(0);
    p.y = pose.translation()(1);
    p.z = pose.translation()(2);

    marker_.controls[0].markers[0].points.push_back(p);

    marker_server_.insert(marker_);
    marker_server_.applyChanges();

    return *this;
  }
private:
  interactive_markers::InteractiveMarkerServer& marker_server_;
  visualization_msgs::InteractiveMarker marker_;

  void createTrajectoryMarker(std::string& name, visualization_msgs::InteractiveMarker& marker)
  {
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.color.a = 1.0f;
    m.scale.x = 0.005;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(m);

    marker.header.frame_id = "world";
    marker.name = name + std::string("_trajectory");
    marker.controls.push_back(control);
  }

  void updateMarkerColor()
  {
    marker_.controls[0].markers[0].color.r = float(color().r);
    marker_.controls[0].markers[0].color.g = float(color().g);
    marker_.controls[0].markers[0].color.b = float(color().b);
  }
};

struct RosCameraTrajectoryVisualizerImpl
{
  typedef std::map<std::string, CameraVisualizer::Ptr> CameraVisualizerMap;
  typedef std::map<std::string, TrajectoryVisualizer::Ptr> TrajectoryVisualizerMap;

  RosCameraTrajectoryVisualizerImpl(ros::NodeHandle& nh) :
    nh_(nh),
    marker_server_("dvo_vis")
  {
  }

  ~RosCameraTrajectoryVisualizerImpl()
  {
  }

  CameraVisualizer::Ptr camera(std::string name)
  {
    CameraVisualizerMap::iterator camera = camera_visualizers_.find(name);

    if(camera_visualizers_.end() == camera)
    {
      camera = camera_visualizers_.insert(
          std::make_pair(name, CameraVisualizer::Ptr(new RosCameraVisualizer(nh_, name, marker_server_)))
      ).first;
    }

    return camera->second;
  }

  TrajectoryVisualizer::Ptr trajectory(std::string name)
  {
    TrajectoryVisualizerMap::iterator trajectory = trajectory_visualizers_.find(name);

    if(trajectory_visualizers_.end() == trajectory)
    {
      trajectory = trajectory_visualizers_.insert(
          std::make_pair(name, TrajectoryVisualizer::Ptr(new RosTrajectoryVisualizer(name, marker_server_)))
      ).first;
    }

    return trajectory->second;
  }

  void reset()
  {
    camera_visualizers_.clear();
    trajectory_visualizers_.clear();
  }
private:
  ros::NodeHandle& nh_;
  interactive_markers::InteractiveMarkerServer marker_server_;
  CameraVisualizerMap camera_visualizers_;
  TrajectoryVisualizerMap trajectory_visualizers_;
};

} /* namespace internal */

RosCameraTrajectoryVisualizer::RosCameraTrajectoryVisualizer(ros::NodeHandle& nh) :
    impl_(new internal::RosCameraTrajectoryVisualizerImpl(nh))
{
}

RosCameraTrajectoryVisualizer::~RosCameraTrajectoryVisualizer()
{
  delete impl_;
}

dvo::visualization::CameraVisualizer::Ptr RosCameraTrajectoryVisualizer::camera(std::string name)
{
  return impl_->camera(name);
}

dvo::visualization::TrajectoryVisualizer::Ptr RosCameraTrajectoryVisualizer::trajectory(std::string name)
{
  return impl_->trajectory(name);
}

void RosCameraTrajectoryVisualizer::reset()
{
  impl_->reset();
}

} /* namespace visualization */
} /* namespace dvo_ros */
