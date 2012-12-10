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

#ifndef RGBDIMAGE_H_
#define RGBDIMAGE_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <boost/smart_ptr.hpp>

//#include <tf/transform_datatypes.h>

#include <dvo/core/datatypes.h>
#include <dvo/core/intrinsic_matrix.h>

namespace dvo
{
namespace core
{

struct RgbdImage
{
public:

  typedef Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor> PointCloud;

  cv::Mat intensity;
  cv::Mat intensity_dx;
  cv::Mat intensity_dy;

  cv::Mat depth;
  cv::Mat depth_dx;
  cv::Mat depth_dy;

  cv::Mat rgb;

  PointCloud pointcloud;

  size_t width, height;

  bool hasIntensity() const;
  bool hasDepth() const;
  bool hasRgb() const;

  void initialize();

  void calculateDerivatives();
  bool calculateIntensityDerivatives();
  void calculateDepthDerivatives();

  void buildPointCloud(const IntrinsicMatrix& intrinsics);

  // inverse warping
  // transformation is the transformation from reference to this image
  void warpIntensity(const AffineTransform& transformation, const PointCloud& reference_pointcloud, const IntrinsicMatrix& intrinsics, RgbdImage& result, PointCloud& transformed_pointcloud);

  // SSE version
  void warpIntensitySse(const AffineTransform& transformation, const PointCloud& reference_pointcloud, const IntrinsicMatrix& intrinsics, RgbdImage& result, PointCloud& transformed_pointcloud);
  // SSE version without warped pointcloud
  void warpIntensitySse(const AffineTransform& transformation, const PointCloud& reference_pointcloud, const IntrinsicMatrix& intrinsics, RgbdImage& result);

  // forward warping
  // transformation is the transformation from this image to the reference image
  void warpIntensityForward(const AffineTransform& transformation, const IntrinsicMatrix& intrinsics, RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud);
  void warpDepthForward(const AffineTransform& transformation, const IntrinsicMatrix& intrinsics, RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud);

  void warpDepthForwardAdvanced(const AffineTransform& transformation, const IntrinsicMatrix& intrinsics, RgbdImage& result);

  bool inImage(const float& x, const float& y) const;
private:
  bool intensity_requires_calculation_, depth_requires_calculation_, pointcloud_requires_build_;

  template<typename T>
  void calculateDerivativeX(const cv::Mat& img, cv::Mat& result);

  //template<typename T>
  //void calculateDerivativeXSse(const cv::Mat& img, cv::Mat& result);

  template<typename T>
  void calculateDerivativeY(const cv::Mat& img, cv::Mat& result);

  void calculateDerivativeYSseFloat(const cv::Mat& img, cv::Mat& result);

  enum WarpIntensityOptions
  {
    WithPointCloud,
    WithoutPointCloud,
  };

  template<int PointCloudOption>
  void warpIntensitySseImpl(const AffineTransform& transformation, const PointCloud& reference_pointcloud, const IntrinsicMatrix& intrinsics, RgbdImage& result, PointCloud& transformed_pointcloud);
};

struct RgbdImagePyramid
{
public:
  typedef boost::shared_ptr<dvo::core::RgbdImagePyramid> Ptr;

  RgbdImagePyramid(cv::Mat intensity, cv::Mat depth);

  void compute(const size_t num_levels);

  RgbdImage& level(size_t idx);

private:
  std::vector<RgbdImage> levels;
};

} /* namespace core */
} /* namespace dvo */
#endif /* RGBDIMAGE_H_ */
