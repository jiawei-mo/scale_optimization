// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_SCALE_ALIGN_H_
#define SVO_SCALE_ALIGN_H_

#include <nlls_solver_1d.h>
#include <vikit/performance_monitor.h>
#include <svo/global.h>

namespace vk {
class PinholeCamera;
}

namespace svo {

class Feature;

/// Optimize the scale of the points by minimizing the photometric error of feature patches.
class ScaleOptimizer : public vk::NLLSSolver1D
{
  static const int patch_halfsize_ = 2;
  static const int patch_size_ = 2*patch_halfsize_;
  static const int patch_area_ = patch_size_*patch_size_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cv::Mat refimg_;
  cv::Mat resimg_;

  ScaleOptimizer(
      int n_levels,
      int min_level,
      int n_iter,
      Method method,
      bool display,
      bool verbose);

  void rescale(const Vector3d& fixed_pos, FramePtr frame, double vo_scale);

  size_t run(
      FramePtr ref_frame, FramePtr stereo_frame, double& vo_scale);

  /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
  /// at the converged state.
  double getFisherInformation();

protected:
  FramePtr ref_frame_;            //!< reference frame, has depth for gradient pixels.
  FramePtr stereo_frame_;         //!< the stereo image
  int level_;                     //!< current pyramid level on which the optimization runs.
  bool display_;                  //!< display residual image.
  int max_level_;                 //!< coarsest pyramid level for the alignment.
  int min_level_;                 //!< finest pyramid level for the alignment.

  // cache:
  bool have_ref_patch_cache_;
  cv::Mat ref_patch_cache_;
  std::vector<bool> visible_fts_;
  std::vector<double> fxr1tz_fxr3tx;           // nominator for frame jac 0
  std::vector<double> fyr2tz_fyr3ty;           // nominator for frame jac 1
  std::vector<double> r3;                      // param for frame jac

  void precomputeReferencePatches();
  virtual double computeResiduals(const double& model, bool linearize_system, bool compute_weight_scale = false);
  virtual int solve();
  virtual void update (const double& old_model, double& new_model);
  virtual void startIteration();
  virtual void finishIteration();
};

} // namespace svo

#endif // SVO_SCALE_ALIGN_H_
