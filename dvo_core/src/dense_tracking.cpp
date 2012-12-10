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

#include <dvo/dense_tracking.h>

#include <assert.h>
#include <sophus/se3.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <dvo/core/datatypes.h>
#include <dvo/util/revertable.h>
#include <dvo/util/stopwatch.h>
#include <dvo/visualization/visualizer.h>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>

namespace dvo
{

using namespace dvo::core;
using namespace dvo::util;

static inline bool isfinite(const float& v)
{
  return std::isfinite(v);
}

const DenseTracker::Config& DenseTracker::getDefaultConfig()
{
  static Config defaultConfig;

  return defaultConfig;
}

static const Eigen::IOFormat YamlArrayFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");

DenseTracker::DenseTracker(IntrinsicMatrix& intrinsics, const Config& cfg) :
    cfg(cfg),
    weight_calculation_(),
    itctx_(cfg)
{
  intrinsics_.push_back(intrinsics);

  configure();
}

void DenseTracker::configure()
{
  assert(cfg.IsSane());

  IntrinsicMatrix current = intrinsics_.back();

  for(size_t idx = intrinsics_.size(); idx <= cfg.FirstLevel; ++idx)
  {
    current.scale(0.5f);

    intrinsics_.push_back(current);
  }

  if(cfg.UseWeighting)
  {
    weight_calculation_
      .scaleEstimator(ScaleEstimators::get(cfg.ScaleEstimatorType))
      .scaleEstimator()->configure(cfg.ScaleEstimatorParam);

    weight_calculation_
      .influenceFunction(InfluenceFunctions::get(cfg.InfluenceFuntionType))
      .influenceFunction()->configure(cfg.InfluenceFunctionParam);
  }
  else
  {
    weight_calculation_
      .scaleEstimator(ScaleEstimators::get(ScaleEstimators::Unit))
      .influenceFunction(InfluenceFunctions::get(InfluenceFunctions::Unit));
  }
}

const IntrinsicMatrix& DenseTracker::intrinsics(size_t level)
{
  return intrinsics_.at(level);
}

void DenseTracker::updateLastTransform(Eigen::Affine3d& last_transformation)
{
  last_xi_ = Sophus::SE3(last_transformation.rotation(), last_transformation.translation());
}

void DenseTracker::getCovarianceEstimate(Eigen::Matrix<double, 6, 6>& covariance) const
{
  covariance = last_a_.cast<double>().lu().inverse();
}

bool DenseTracker::match(RgbdImagePyramid& reference, RgbdImagePyramid& current, Eigen::Affine3d& transformation)
{
  reference.compute(cfg.getNumLevels());
  current.compute(cfg.getNumLevels());

  bool success = true;

  if(!cfg.UseInitialEstimate)
  {
    transformation.setIdentity();
  }

  // our first increment is the given guess
  Sophus::SE3 inc(transformation.rotation(), transformation.translation());

  Revertable<Sophus::SE3> old(last_xi_);
  Revertable<Sophus::SE3> initial(inc);
  Revertable<AffineTransform> estimate(AffineTransform::Identity());

  bool accept = true;

  static stopwatch_collection sw_level(5, "l", 100);
  static stopwatch_collection sw_it(5, "it@l", 500);

  for(itctx_.Level = cfg.FirstLevel; itctx_.Level >= cfg.LastLevel; --itctx_.Level)
  {
    // reset error after every pyramid level? yes because errors from different levels are not comparable
    itctx_.Iteration = 0;
    itctx_.Error = 1;

    RgbdImage& ref = reference.level(itctx_.Level);
    RgbdImage& cur = current.level(itctx_.Level);
    const IntrinsicMatrix& intrinsics = intrinsics_[itctx_.Level];

    NormalEquationsLeastSquares ls;
    Vector6 x;

    Vector6 last_x;
    sw_level[itctx_.Level].start();
    do
    {
      sw_it[itctx_.Level].start();
      estimate.update() = inc.matrix().cast<NumType>() * estimate().matrix();

      if(cfg.UseTemporalSmoothing())
      {
        old.update() = inc.inverse() * old();
      }

      if(cfg.UseEstimateSmoothing())
      {
        initial.update() = inc.inverse() * initial();
      }

      computeLeastSquaresEquationsInverseCompositional(ref, cur, intrinsics, estimate(), ls);

      itctx_.LastError = itctx_.Error;
      itctx_.Error = ls.error + 0.5 * cfg.Lambda * old().log().squaredNorm() + 0.5 * cfg.Mu * initial().log().squaredNorm();

      // accept the last increment?
      accept = itctx_.ErrorDiff() > 0.0;

      //ROS_DEBUG_STREAM_COND(!accept, itctx_);

      // if we get a worse result, we revert the increment and try our luck on the next pyramid level
      if(!accept)
      {
        old.revert();
        initial.revert();
        estimate.revert();
        inc = Sophus::SE3::exp(Vector6::Zero().cast<double>());

        break;
      }

      // calculate new increment
      Matrix6x6 A_diagonal = Matrix6x6::Identity();

      if(cfg.UseTemporalSmoothing())
      {
        ls.A += cfg.Lambda * A_diagonal;
        ls.b += cfg.Lambda * old().log().cast<NumType>();
      }

      if(cfg.UseEstimateSmoothing())
      {
        ls.A += cfg.Mu * A_diagonal;
        ls.b += cfg.Mu * initial().log().cast<NumType>();
      }

      // first estimate rotation on lowest level
      //if(itctx_.IsFirstLevel())
      //{
      //  Eigen::Vector3f rot = ls.A.bottomRightCorner(3, 3).ldlt().solve(ls.b.tail(3));
      //  x.setZero();
      //  x.tail<3>() = rot;
      //}
      //else
      //{
      //  ls.solve(x);
      //}

      ls.solve(x);
      if(itctx_.IsLastLevel())
      {
        // TODO: should only be used if we also accept the solution
        last_a_ = ls.A;
      }

      //ROS_DEBUG_STREAM_COND(accept, itctx_ << ", Increment: " << x.format(YamlArrayFmt));

      inc = Sophus::SE3::exp(x.cast<double>());

      itctx_.Iteration++;
      itctx_.NumConstraints = ls.num_constraints;

      sw_it[itctx_.Level].stopAndPrint();
    }
    while(accept && itctx_.ErrorDiff() > cfg.Precision && !itctx_.IterationsExceeded());

    sw_level[itctx_.Level].stopAndPrint();
  }

  // if last increment wasn't incorporated because of iterations exceeded, incorporate it
  if(itctx_.IterationsExceeded())
  {
    estimate.update() = inc.matrix().cast<NumType>() * estimate().matrix();
  }

  // log reason for termination on last level
  //ROS_DEBUG_STREAM_COND_NAMED(!itctx_.IterationsExceeded() && itctx_.ErrorDiff() > 0.0 && itctx_.ErrorDiff() <= cfg.Precision, "abort", "error_precision");
  //ROS_DEBUG_STREAM_COND_NAMED(!itctx_.IterationsExceeded() && itctx_.ErrorDiff() < 0.0, "abort", "error_increase");
  //ROS_DEBUG_STREAM_COND_NAMED(itctx_.IterationsExceeded(), "abort", "iteration_exceeded");

  if(success)
  {
    last_xi_ = Sophus::SE3(estimate().rotation().cast<double>(), estimate().translation().cast<double>());
  }

  transformation = estimate().inverse().cast<double>();

  return success;
}

void DenseTracker::computeLeastSquaresEquationsForwardAdditive(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls)
{
  RgbdImage cur_warped;
  RgbdImage::PointCloud ref_transformed;
  cv::Mat residuals, cur_dx, cur_dy;

  ref.buildPointCloud(intrinsics);
  cur.warpIntensitySse(transformation, ref.pointcloud, intrinsics, cur_warped, ref_transformed);

  // compute I_{2,x} and I_{2,y}
  cur_warped.calculateIntensityDerivatives();
  cur_dx = cur_warped.intensity_dx * intrinsics.fx();
  cur_dy = cur_warped.intensity_dy * intrinsics.fy();

  // compute residuals
  residuals = cur_warped.intensity - ref.intensity;

  computeLeastSquaresEquationsGeneric(residuals, cur_dx, cur_dy, ref_transformed, ls);
}

void DenseTracker::computeLeastSquaresEquationsForwardCompositional(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls)
{
  RgbdImage cur_warped;
  cv::Mat residuals, cur_dx, cur_dy;

  ref.buildPointCloud(intrinsics);
  cur.warpIntensitySse(transformation, ref.pointcloud, intrinsics, cur_warped);

  // compute I_{2,x} and I_{2,y}
  cur_warped.calculateIntensityDerivatives();
  cur_dx = cur_warped.intensity_dx * intrinsics.fx();
  cur_dy = cur_warped.intensity_dy * intrinsics.fy();

  // compute residuals
  residuals = cur_warped.intensity - ref.intensity;

  computeLeastSquaresEquationsGeneric(residuals, cur_dx, cur_dy, ref.pointcloud, ls);
}

void DenseTracker::computeLeastSquaresEquationsForwardCompositionalESM(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls)
{
  //static stopwatch_collection sw_warp(5, "warp@l"), sw_eqs(5, "eqs@l"), sw_total(5, "total@l");
  //sw_total[itctx_.Level].start();
  RgbdImage cur_warped;
  cv::Mat residuals, dx, dy;

  ref.buildPointCloud(intrinsics);
  //sw_warp[itctx_.Level].start();
  cur.warpIntensitySse(transformation, ref.pointcloud, intrinsics, cur_warped);
  //sw_warp[itctx_.Level].stopAndPrint();

  // compute I_{1,x} and I_{1,y}
  ref.calculateIntensityDerivatives();
  // compute I_{2,x} and I_{2,y}
  cur_warped.calculateIntensityDerivatives();

  dx = (ref.intensity_dx + cur_warped.intensity_dx) * 0.5f * intrinsics.fx();
  dy = (ref.intensity_dy + cur_warped.intensity_dy) * 0.5f * intrinsics.fy();

  // compute residuals
  residuals = cur_warped.intensity - ref.intensity;

  //sw_eqs[itctx_.Level].start();
  computeLeastSquaresEquationsGeneric(residuals, dx, dy, ref.pointcloud, ls);
  //sw_eqs[itctx_.Level].stopAndPrint();
  //sw_total[itctx_.Level].stopAndPrint();
}

void DenseTracker::computeLeastSquaresEquationsInverseCompositional(dvo::core::RgbdImage& ref, dvo::core::RgbdImage& cur, const dvo::core::IntrinsicMatrix& intrinsics, const dvo::core::AffineTransform& transformation, dvo::core::LeastSquaresInterface& ls)
{
  RgbdImage cur_warped;
  cv::Mat residuals, ref_dx, ref_dy;

  ref.buildPointCloud(intrinsics);
  cur.warpIntensitySse(transformation, ref.pointcloud, intrinsics, cur_warped);

  // compute I_{1,x} and I_{1,y}
  ref.calculateIntensityDerivatives();
  ref_dx = ref.intensity_dx * intrinsics.fx();
  ref_dy = ref.intensity_dy * intrinsics.fy();

  // compute residuals
  residuals = cur_warped.intensity - ref.intensity;

  computeLeastSquaresEquationsGeneric(residuals, ref_dx, ref_dy, ref.pointcloud, ls);
}

struct LeastSquaresEquationsReduction
{
  dvo::core::NormalEquationsLeastSquares* ls;
  const cv::Mat& residuals/*, weights*/, Jix, Jiy;
  const dvo::core::RgbdImage::PointCloud& points;
  const dvo::core::WeightCalculation& weights;
  LeastSquaresEquationsReduction(const cv::Mat& residuals,const dvo::core::WeightCalculation& weights, const cv::Mat& Jix, const cv::Mat& Jiy, const dvo::core::RgbdImage::PointCloud& points) :
    ls(new dvo::core::NormalEquationsLeastSquares()),
    residuals(residuals),
    weights(weights),
    Jix(Jix),
    Jiy(Jiy),
    points(points)
  {
    ls->initialize(1);
  }

  LeastSquaresEquationsReduction(LeastSquaresEquationsReduction& other, tbb::split) :
    ls(new dvo::core::NormalEquationsLeastSquares()),
    residuals(other.residuals),
    weights(other.weights),
    Jix(other.Jix),
    Jiy(other.Jiy),
    points(other.points)
  {
    ls->initialize(1);
  }

  ~LeastSquaresEquationsReduction()
  {
    delete ls;
  }

  void operator()(const tbb::blocked_range<size_t>& r)
  {
    const float *jix_ptr = Jix.ptr<float>() + r.begin(), *jiy_ptr = Jiy.ptr<float>() + r.begin(), *residual_ptr = residuals.ptr<float>() + r.begin();//, *weights_ptr = weights.ptr<float>() + r.begin();
    Matrix1x2 Ji;
    Matrix2x6 Jw;

    dvo::core::NormalEquationsLeastSquares* tmp = ls;//new dvo::core::NormalEquationsLeastSquares();

    for (size_t idx = r.begin(); idx != r.end(); ++idx, ++jix_ptr, ++jiy_ptr, ++residual_ptr)//, ++weights_ptr)
    {
      if(!isfinite(*jix_ptr) || !isfinite(*jiy_ptr) || !isfinite(*residual_ptr)) continue;

      Ji << *jix_ptr, *jiy_ptr;

      const Vector4& p3d = points.col(idx).cast<NumType>();
      DenseTracker::computeJacobianOfProjectionAndTransformation(p3d, Jw);

      tmp->update(Ji * Jw, *residual_ptr, weights.calculateWeight(*residual_ptr));
    }

    ls = tmp;
  }

  void join(LeastSquaresEquationsReduction& other)
  {
    ls->combine(*other.ls);
  }
};

inline void DenseTracker::computeLeastSquaresEquationsGeneric(const cv::Mat& residuals, const cv::Mat& Jix, const cv::Mat& Jiy, const dvo::core::RgbdImage::PointCloud& points, dvo::core::LeastSquaresInterface& ls)
{
  //static stopwatch_collection sw_weights(5, "weights@l"), sw_reduce(5, "reduce@l");

  cv::Mat weights;

  // compute weights
  //sw_weights[itctx_.Level].start();
  computeWeights(residuals, weights);
  //sw_weights[itctx_.Level].stopAndPrint();
  LeastSquaresEquationsReduction body(residuals, weight_calculation_, Jix, Jiy, points);

  //sw_reduce[itctx_.Level].start();
  tbb::parallel_reduce(tbb::blocked_range<size_t>(0, residuals.size().area()), body);
  //sw_reduce[itctx_.Level].stopAndPrint();

  NormalEquationsLeastSquares& xls = (NormalEquationsLeastSquares&)ls;
  xls = *body.ls;
  xls.finish();
}

inline void DenseTracker::computeWeights(const cv::Mat& residuals, cv::Mat& weights)
{
  if(itctx_.IsFirstLevel() || itctx_.IsFirstIterationOnLevel())
  {
    weight_calculation_.calculateScale(residuals);
  }
}

// jacobian computation
inline void DenseTracker::computeJacobianOfProjectionAndTransformation(const Vector4& p, Matrix2x6& j)
{
  NumType z = 1.0f / p(2);
  NumType z_sqr = 1.0f / (p(2) * p(2));

  j(0, 0) =  z;
  j(0, 1) =  0.0f;
  j(0, 2) = -p(0) * z_sqr;
  j(0, 3) = j(0, 2) * p(1);//j(0, 3) = -p(0) * p(1) * z_sqr;
  j(0, 4) = 1.0f - j(0, 2) * p(0);//j(0, 4) =  (1.0 + p(0) * p(0) * z_sqr);
  j(0, 5) = -p(1) * z;

  j(1, 0) =  0.0f;
  j(1, 1) =  z;
  j(1, 2) = -p(1) * z_sqr;
  j(1, 3) = -1.0f + j(1, 2) * p(1); //j(1, 3) = -(1.0 + p(1) * p(1) * z_sqr);
  j(1, 4) = -j(0, 3); //j(1, 4) =  p(0) * p(1) * z_sqr;
  j(1, 5) =  p(0) * z;
}

void DenseTracker::compute3rdRowOfJacobianOfTransformation(Vector4& p, Vector6& j)
{
  j(0) = 0.0;
  j(1) = 0.0;
  j(2) = 1.0;
  j(3) = p(1);
  j(4) = -p(0);
  j(5) = 0.0;
}

} /* namespace dvo */
