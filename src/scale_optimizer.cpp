#include <algorithm>
#include <scale_optimizer.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/config.h>
#include <svo/point.h>
#include <vikit/pinhole_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>

namespace svo {

ScaleOptimizer::ScaleOptimizer(
    int max_level, int min_level, int n_iter,
    Method method, bool display, bool verbose) :
        display_(display),
        max_level_(max_level),
        min_level_(min_level)
{
  n_iter_ = n_iter;
  n_iter_init_ = n_iter_;
  method_ = method;
  verbose_ = verbose;
  eps_ = 0.000001;
}

void ScaleOptimizer::rescale(const Vector3d& fixed_pos, FramePtr frame, double vo_scale)
{
  // rescale relative translation with new scale
  frame->T_f_w_.translation() =
      -frame->T_f_w_.rotation_matrix()*(fixed_pos + vo_scale*(frame->pos() - fixed_pos));

  // rescale point with new scale
  for(auto it=frame->fts_.begin(), ite=frame->fts_.end();
      it!=ite; ++it)
  {
    if((*it)->point == NULL) continue;

    (*it)->point->pos_ = fixed_pos + ((*it)->point->pos_ - fixed_pos)*vo_scale;
  }
}

size_t ScaleOptimizer::run(FramePtr ref_frame, FramePtr stereo_frame, double& vo_scale)
{
  reset();

  if(ref_frame->fts_.empty())
  {
    SVO_WARN_STREAM("ScaleOptimizer: no features to track!");
    return 0;
  }

  ref_frame_ = ref_frame;
  stereo_frame_ = stereo_frame;
  ref_patch_cache_ = cv::Mat(ref_frame_->fts_.size(), patch_area_, CV_32F);
  visible_fts_.resize(ref_patch_cache_.rows, false); // TODO: should it be reset at each level?
  fxr1tz_fxr3tx.resize(ref_patch_cache_.rows, 0.0);
  fyr2tz_fyr3ty.resize(ref_patch_cache_.rows, 0.0);
  r3.resize(ref_patch_cache_.rows, 0.0);

  for(level_=max_level_; level_>=min_level_; --level_)
  {
    mu_ = 0.1;
    have_ref_patch_cache_ = false;
    if(verbose_)
      printf("\nPYRAMID LEVEL %i\n---------------\n", level_);
    optimize(vo_scale);
  }

  return n_meas_/patch_area_;
}

double ScaleOptimizer::getFisherInformation()
{
  double sigma_i_sq = 5e-4*255*255; // image noise
  double I = H_/sigma_i_sq;
  return I;
}

void ScaleOptimizer::precomputeReferencePatches()
{
  const int border = patch_halfsize_+1;
  const cv::Mat& ref_img = ref_frame_->img_pyr_.at(level_);
  const int stride = ref_img.cols;
  const float level_scale = 1.0f/(1<<level_);
  const Vector3d ref_pos = ref_frame_->pos();
  const double focal_length = ref_frame_->cam_->errorMultiplier2();
  size_t feature_counter = 0;
  std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
  std::vector<double>::iterator fxr1tz_fxr3tx_it = fxr1tz_fxr3tx.begin();
  std::vector<double>::iterator fyr2tz_fyr3ty_it = fyr2tz_fyr3ty.begin();
  std::vector<double>::iterator r3_it = r3.begin();
  for(auto it=ref_frame_->fts_.begin(), ite=ref_frame_->fts_.end();
      it!=ite; ++it, ++feature_counter, ++visiblity_it, ++fxr1tz_fxr3tx_it, ++fyr2tz_fyr3ty_it, ++r3_it)
  {
    // check if reference with patch size is within image
    const float u_ref = (*it)->px[0]*level_scale;
    const float v_ref = (*it)->px[1]*level_scale;
    const int u_ref_i = floorf(u_ref);
    const int v_ref_i = floorf(v_ref);
    if((*it)->point == NULL || u_ref_i-border < 0 || v_ref_i-border < 0 || u_ref_i+border >= ref_img.cols || v_ref_i+border >= ref_img.rows)
    {
        *visiblity_it = false;
        continue;
    }
    *visiblity_it = true;

    // cache data for evaluating projection jacobian
    vk::PinholeCamera* stereo_cam = dynamic_cast<vk::PinholeCamera*>(stereo_frame_->cam_);
    double fx = stereo_cam->fx() / (1<<level_);
    double fy = stereo_cam->fy() / (1<<level_);
    double cx = stereo_cam->cx() / (1<<level_);
    double cy = stereo_cam->cy() / (1<<level_);
    double tx = stereo_frame_->T_f_w_.translation()(0);
    double ty = stereo_frame_->T_f_w_.translation()(1);
    double tz = stereo_frame_->T_f_w_.translation()(2);
    const double depth(((*it)->point->pos_ - ref_pos).norm());
    const Vector3d xyz_ref((*it)->f*depth);
    Vector3d rotP = stereo_frame_->T_f_w_.so3() * xyz_ref;
    double r1 = rotP(0);
    double r2 = rotP(1);
    double r3_ = rotP(2);
    *fxr1tz_fxr3tx_it = fx*r1*tz - fx*r3_*tx;
    *fyr2tz_fyr3ty_it = fy*r2*tz - fy*r3_*ty;
    *r3_it = r3_;

    // compute bilateral interpolation weights for reference image
    const float subpix_u_ref = u_ref-u_ref_i;
    const float subpix_v_ref = v_ref-v_ref_i;
    const float w_ref_tl = (1.0-subpix_u_ref) * (1.0-subpix_v_ref);
    const float w_ref_tr = subpix_u_ref * (1.0-subpix_v_ref);
    const float w_ref_bl = (1.0-subpix_u_ref) * subpix_v_ref;
    const float w_ref_br = subpix_u_ref * subpix_v_ref;
    size_t pixel_counter = 0;
    float* cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* ref_img_ptr = (uint8_t*) ref_img.data + (v_ref_i+y-patch_halfsize_)*stride + (u_ref_i-patch_halfsize_);
      for(int x=0; x<patch_size_; ++x, ++ref_img_ptr, ++cache_ptr, ++pixel_counter)
      {
        // precompute interpolated reference patch color
        *cache_ptr = w_ref_tl*ref_img_ptr[0] + w_ref_tr*ref_img_ptr[1] + w_ref_bl*ref_img_ptr[stride] + w_ref_br*ref_img_ptr[stride+1];
      }
    }
    if(display_)
      cv::drawMarker(refimg_, cv::Point2d(u_ref*(1<<level_), v_ref*(1<<level_)), cv::Scalar(0,0,255), cv::MARKER_CROSS, 8);
  }
  have_ref_patch_cache_ = true;
}

double ScaleOptimizer::computeResiduals(
    const double& vo_scale,
    bool linearize_system,
    bool compute_weight_scale)
{
  // Warp the stereo image such that it aligns with the (ref)erence image
  const cv::Mat& stereo_img = stereo_frame_->img_pyr_.at(level_);

  if(linearize_system && display_)
  {
    resimg_ = stereo_frame_->img_pyr_.at(0).clone();
    cv::cvtColor(resimg_, resimg_, cv::COLOR_GRAY2BGR);
  }

  if(have_ref_patch_cache_ == false)
  {
    refimg_ = ref_frame_->img_pyr_.at(0).clone();
    cv::cvtColor(refimg_, refimg_, cv::COLOR_GRAY2BGR);
    precomputeReferencePatches();
  }
  // compute the weights on the first iteration
  std::vector<float> errors;
  if(compute_weight_scale)
    errors.reserve(visible_fts_.size());
  const int stride = stereo_img.cols;
  const int border = patch_halfsize_+1;
  const float level_scale = 1.0f/(1<<level_);
  const Vector3d ref_pos(ref_frame_->pos());
  float chi2 = 0.0;
  size_t feature_counter = 0; // is used to compute the index of the cached jacobian
  std::vector<bool>::iterator visiblity_it = visible_fts_.begin();
  std::vector<double>::iterator fxr1tz_fxr3tx_it = fxr1tz_fxr3tx.begin();
  std::vector<double>::iterator fyr2tz_fyr3ty_it = fyr2tz_fyr3ty.begin();
  std::vector<double>::iterator r3_it = r3.begin();
  for(auto it=ref_frame_->fts_.begin(); it!=ref_frame_->fts_.end();
      ++it, ++feature_counter, ++visiblity_it, ++fxr1tz_fxr3tx_it, ++fyr2tz_fyr3ty_it, ++r3_it)
  {
    // check if feature is within image
    if(!*visiblity_it)
      continue;

    // evaluate projection jacobian
    Vector2d frame_jac;
    double tz = stereo_frame_->T_f_w_.translation()(2);
    double deno = (vo_scale*(*r3_it)+tz)*(vo_scale*(*r3_it)+tz);
    frame_jac(0) = *fxr1tz_fxr3tx_it / deno;
    frame_jac(1) = *fyr2tz_fyr3ty_it / deno;

    // compute pixel location in stereo img
    const double depth = ((*it)->point->pos_ - ref_pos).norm()*vo_scale;
    // std::cout<<"depth "<<depth<<std::endl;
    Vector3d xyz_ref((*it)->f*depth);
    Vector3d xyz_stereo(stereo_frame_->T_f_w_ * xyz_ref);
    const Vector2f uv_stereo_pyr(stereo_frame_->cam_->world2cam(xyz_stereo).cast<float>() * level_scale);
    // std::cout<<"xyzuv "<<xyz_stereo(0)<<" "<<xyz_stereo(1)<<" "<<xyz_stereo(2)<<" "<<uv_stereo_pyr(0)<<" "<<uv_stereo_pyr(1)<<std::endl;
    const float u_stereo = uv_stereo_pyr[0];
    const float v_stereo = uv_stereo_pyr[1];
    const int u_stereo_i = floorf(u_stereo);
    const int v_stereo_i = floorf(v_stereo);

    // check if projection is within the image
    if(u_stereo_i < 0 || v_stereo_i < 0 || u_stereo_i-border < 0 || v_stereo_i-border < 0 || u_stereo_i+border >= stereo_img.cols || v_stereo_i+border >= stereo_img.rows)
      continue;

    // compute bilateral interpolation weights for the stereorent image
    const float subpix_u_stereo = u_stereo-u_stereo_i;
    const float subpix_v_stereo = v_stereo-v_stereo_i;
    const float w_stereo_tl = (1.0-subpix_u_stereo) * (1.0-subpix_v_stereo);
    const float w_stereo_tr = subpix_u_stereo * (1.0-subpix_v_stereo);
    const float w_stereo_bl = (1.0-subpix_u_stereo) * subpix_v_stereo;
    const float w_stereo_br = subpix_u_stereo * subpix_v_stereo;
    float* ref_patch_cache_ptr = reinterpret_cast<float*>(ref_patch_cache_.data) + patch_area_*feature_counter;
    size_t pixel_counter = 0; // is used to compute the index of the cached jacobian
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* stereo_img_ptr = (uint8_t*) stereo_img.data + (v_stereo_i+y-patch_halfsize_)*stride + (u_stereo_i-patch_halfsize_);

      for(int x=0; x<patch_size_; ++x, ++pixel_counter, ++stereo_img_ptr, ++ref_patch_cache_ptr)
      {
        // compute residual
        const float intensity_stereo = w_stereo_tl*stereo_img_ptr[0] + w_stereo_tr*stereo_img_ptr[1] + w_stereo_bl*stereo_img_ptr[stride] + w_stereo_br*stereo_img_ptr[stride+1];
        const float res = intensity_stereo - (*ref_patch_cache_ptr);

        // used to compute scale for robust cost
        if(compute_weight_scale)
          errors.push_back(fabsf(res));

        // robustification
        float weight = 1.0;
        if(use_weights_) {
          weight = weight_function_->value(res/scale_);
        }

        chi2 += res*res*weight;
        n_meas_++;
        if(linearize_system)
        {
          // get gradient of warped image (~gradient at warped position)
          float dx = 0.5f * ((w_stereo_tl*stereo_img_ptr[1] + w_stereo_tr*stereo_img_ptr[2] + w_stereo_bl*stereo_img_ptr[stride+1] + w_stereo_br*stereo_img_ptr[stride+2])
                            -(w_stereo_tl*stereo_img_ptr[-1] + w_stereo_tr*stereo_img_ptr[0] + w_stereo_bl*stereo_img_ptr[stride-1] + w_stereo_br*stereo_img_ptr[stride]));
          float dy = 0.5f * ((w_stereo_tl*stereo_img_ptr[stride] + w_stereo_tr*stereo_img_ptr[1+stride] + w_stereo_bl*stereo_img_ptr[stride*2] + w_stereo_br*stereo_img_ptr[stride*2+1])
                            -(w_stereo_tl*stereo_img_ptr[-stride] + w_stereo_tr*stereo_img_ptr[1-stride] + w_stereo_bl*stereo_img_ptr[0] + w_stereo_br*stereo_img_ptr[1]));

          // compute Jacobian, weighted Hessian and weighted "steepest descend images" (times error)
          const double J = dx*frame_jac(0) + dy*frame_jac(1);
          // std::cout<<"J "<<J<<" res "<<res<<std::endl;
          H_ += J*J*weight;
          Jres_ += J*res*weight;
        }
      }
    }
    if(display_)
      cv::drawMarker(resimg_, cv::Point2d(u_stereo*(1<<level_), v_stereo*(1<<level_)), cv::Scalar(0,0,255), cv::MARKER_CROSS, 8);
  }

  if(display_)
  {
    cv::Mat residuals_img;
    cv::vconcat(refimg_, resimg_, residuals_img);
    cv::namedWindow("residuals", CV_WINDOW_NORMAL);
    cv::imshow("residuals", residuals_img);
    cv::waitKey(0);
  }

  // compute the weights on the first iteration
  if(compute_weight_scale && iter_ == 0)
    scale_ = scale_estimator_->compute(errors);

  return chi2/n_meas_;
}

int ScaleOptimizer::solve()
{
  // x_ = H_.ldlt().solve(Jres_);
  // std::cout<<"Jres_ "<<Jres_<<" H_ "<<H_<<std::endl;
  x_ = -Jres_ / H_;
  if((bool) std::isnan(x_))
    return 0;
  return 1;
}

void ScaleOptimizer::update(
    const double& vo_scale_old,
    double& vo_scale_new)
{
  vo_scale_new =  vo_scale_old + x_;
}

void ScaleOptimizer::startIteration()
{}

void ScaleOptimizer::finishIteration()
{
  // if(display_)
  // {
  //   cv::namedWindow("residuals", CV_WINDOW_AUTOSIZE);
  //   cv::imshow("residuals", resimg_*10);
  //   cv::waitKey(0);
  // }
}

} // namespace svo
