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

#ifndef DENSE_TRACKER_H_
#define DENSE_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>

#include <dvo/core/datatypes.h>
#include <dvo/core/intrinsic_matrix.h>
#include <dvo/core/rgbd_image.h>
#include <dvo/core/least_squares.h>
//#include "core/blur_detection.h"
#include <dvo/core/weight_calculation.h>

namespace dvo
{

/**
 * Implementation of:
 *
 *   "Robust Odometry Estimation for RGB-D Cameras"
 *   "Real-Time Visual Odometry from Dense RGB-D Images"
 *
 * similar to:
 *
 *   "Direct Iterative Closest Point for Real-time Visual Odometry".
 */
class DenseTracker
{
public:
  struct Config
  {
    int FirstLevel, LastLevel;
    int MaxIterationsPerLevel;
    double Precision;
    double Lambda; // weighting factor for temporal smoothing
    double Mu;     // weighting factor for driving solution close to imu estimate

    bool UseInitialEstimate;
    bool UseWeighting;

    dvo::core::InfluenceFunctions::enum_t InfluenceFuntionType;
    float InfluenceFunctionParam;

    dvo::core::ScaleEstimators::enum_t ScaleEstimatorType;
    float ScaleEstimatorParam;

    Config();
    size_t getNumLevels() const;

    bool UseTemporalSmoothing() const;

    bool UseEstimateSmoothing() const;

    bool IsSane() const;
  };

  struct IterationContext
  {
    const Config& cfg;

    int Level;
    int Iteration;

    size_t NumConstraints;

    double Error, LastError;

    IterationContext(const Config& cfg);

    // returns true if this is the first iteration
    bool IsFirstIteration() const;

    // returns true if this is the first iteration on the current level
    bool IsFirstIterationOnLevel() const;

    // returns true if this is the first level
    bool IsFirstLevel() const;

    // returns true if this is the last level
    bool IsLastLevel() const;

    bool IterationsExceeded() const;

    // returns LastError - Error
    double ErrorDiff() const;
  };

  static const Config& getDefaultConfig();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // TODO: make cfg pass by value
  DenseTracker(dvo::core::IntrinsicMatrix& intrinsics, const Config& cfg = getDefaultConfig());

  // TODO: need some method to get current configuration
  // TODO: pass new config in here
  void configure();

  const dvo::core::IntrinsicMatrix& intrinsics(size_t level);

  bool match(dvo::core::RgbdImagePyramid& reference, dvo::core::RgbdImagePyramid& current, Eigen::Affine3d& transformation);

  // TODO: remove
  void updateLastTransform(Eigen::Affine3d& last_transformation);

  // TODO: remove
  void getCovarianceEstimate(Eigen::Matrix<double, 6, 6>& covariance) const;

  static inline void computeJacobianOfProjectionAndTransformation(const dvo::core::Vector4& p, dvo::core::Matrix2x6& jacobian);

  IterationContext itctx_;
private:
  const Config& cfg;
  std::vector<dvo::core::IntrinsicMatrix> intrinsics_;
  dvo::core::WeightCalculation weight_calculation_;

  Sophus::SE3 last_xi_;

  dvo::core::Matrix6x6 last_a_;

  void computeLeastSquaresEquationsForwardAdditive(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls);
  void computeLeastSquaresEquationsForwardCompositional(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls);
  void computeLeastSquaresEquationsInverseCompositional(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls);
  void computeLeastSquaresEquationsForwardCompositionalESM(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls);
  inline void computeLeastSquaresEquationsGeneric(const cv::Mat& residuals, const cv::Mat& Jix, const cv::Mat& Jiy, const dvo::core::RgbdImage::PointCloud& points, dvo::core::LeastSquaresInterface& ls);

  inline void computeWeights(const cv::Mat& residuals, cv::Mat& weights);

  void compute3rdRowOfJacobianOfTransformation(dvo::core::Vector4& p, dvo::core::Vector6& j);
};

} /* namespace dvo */

std::ostream& operator<< (std::ostream &out, dvo::DenseTracker::Config &config);
std::ostream& operator<< (std::ostream &out, dvo::DenseTracker::IterationContext &ctx);

#endif /* DENSE_TRACKER_H_ */
