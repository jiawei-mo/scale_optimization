/*
 * Abstract Nonlinear Least-Squares Solver Class
 *
 * nlls_solver.h
 *
 *  Created on: Nov 5, 2012
 *      Author: cforster
 */

#include <stdexcept>
#include <nlls_solver_1d.h>

void vk::NLLSSolver1D::optimize(double& model)
{
  if(method_ == GaussNewton)
    optimizeGaussNewton(model);
  else if(method_ == LevenbergMarquardt)
    optimizeLevenbergMarquardt(model);
}

void vk::NLLSSolver1D::optimizeGaussNewton(double& model)
{
  // Compute weight scale
  if(use_weights_)
    computeResiduals(model, false, true);

  // Save the old model to rollback in case of unsuccessful update
  double old_model(model);

  // perform iterative estimation
  for (iter_ = 0; iter_<n_iter_; ++iter_)
  {
    rho_ = 0;
    startIteration();

    H_=0;
    Jres_=0;

    // compute initial error
    n_meas_ = 0;
    double new_chi2 = computeResiduals(model, true, false);

    // add prior
    if(have_prior_)
      applyPrior(model);

    // solve the linear system
    if(!solve())
    {
      // matrix was singular and could not be computed
      std::cout << "Matrix is close to singular! Stop Optimizing." << std::endl;
      std::cout << "H = " << H_ << std::endl;
      std::cout << "Jres = " << Jres_ << std::endl;
      stop_ = true;
    }

    // check if error increased since last optimization
    if((iter_ > 0 && new_chi2 > chi2_) || stop_)
    {
      if(verbose_)
      {
        std::cout << "It. " << iter_
                  << "\t Failure"
                  << "\t new_chi2 = " << new_chi2
                  << "\t Error increased. Stop optimizing."
                  << std::endl;
      }
      model = old_model; // rollback
      break;
    }

    // update the model
    double new_model;
    update(model, new_model);
    old_model = model;
    model = new_model;

    chi2_ = new_chi2;

    if(verbose_)
    {
      std::cout << "It. " << iter_
                << "\t Success"
                << "\t new_chi2 = " << new_chi2
                << "\t n_meas = " << n_meas_
                << "\t x_ = " << x_
                << std::endl;
    }

    finishIteration();

    // stop when converged, i.e. update step too small
    if(x_<=eps_)
      break;
  }
}

void vk::NLLSSolver1D::optimizeLevenbergMarquardt(double& model)
{
  // Compute weight scale
  if(use_weights_)
    computeResiduals(model, false, true);

  // compute the initial error
  chi2_ = computeResiduals(model, true, false);

  if(verbose_)
    cout << "init chi2 = " << chi2_
         << "\t n_meas = " << n_meas_
         << endl;

  // TODO: compute initial lambda
  // Hartley and Zisserman: "A typical init value of lambda is 10^-3 times the
  // average of the diagonal elements of J'J"

  // Compute Initial Lambda
  if(mu_ < 0)
  {
    double H_max_diag = 0;
    double tau = 1e-4;
    H_max_diag = max(H_max_diag, H_);
    mu_ = tau*H_max_diag;
  }

  // perform iterative estimation
  for (iter_ = 0; iter_<n_iter_; ++iter_)
  {
    rho_ = 0;
    startIteration();

    // try to compute and update, if it fails, try with increased mu
    n_trials_ = 0;
    do
    {
      // init variables
      double new_model;
      double new_chi2 = -1;
      H_=0;
      Jres_=0;

      // compute initial error
      n_meas_ = 0;
      computeResiduals(model, true, false);

      // add damping term:
      H_ += H_*mu_;

      // add prior
      if(have_prior_)
        applyPrior(model);

      // solve the linear system
      if(solve())
      {
        // update the model
        update(model, new_model);

        // compute error with new model and compare to old error
        n_meas_ = 0;
        new_chi2 = computeResiduals(new_model, false, false);
        rho_ = chi2_-new_chi2;
      }
      else
      {
        // matrix was singular and could not be computed
        cout << "Matrix is close to singular!" << endl;
        cout << "H = " << H_ << endl;
        cout << "Jres = " << Jres_ << endl;
        rho_ = -1;
      }

      if(rho_>0)
      {
        // update decrased the error -> success
        model = new_model;
        chi2_ = new_chi2;
        stop_ = x_<=eps_;
        mu_ *= max(1./3., min(1.-pow(2*rho_-1,3), 2./3.));
        nu_ = 2.;
        if(verbose_)
        {
          cout << "It. " << iter_
               << "\t Trial " << n_trials_
               << "\t Success"
               << "\t n_meas = " << n_meas_
               << "\t new_chi2 = " << new_chi2
               << "\t mu = " << mu_
               << "\t nu = " << nu_
               << endl;
        }
      }
      else
      {
        // update increased the error -> fail
        mu_ *= nu_;
        nu_ *= 2.;
        ++n_trials_;
        if (n_trials_ >= n_trials_max_)
          stop_ = true;

        if(verbose_)
        {
          cout << "It. " << iter_
               << "\t Trial " << n_trials_
               << "\t Failure"
               << "\t n_meas = " << n_meas_
               << "\t new_chi2 = " << new_chi2
               << "\t mu = " << mu_
               << "\t nu = " << nu_
               << endl;
        }
      }

      finishTrial();

    } while(!(rho_>0 || stop_));
    if (stop_)
      break;

    finishIteration();
  }
}


void vk::NLLSSolver1D::setRobustCostFunction(
    ScaleEstimatorType scale_estimator,
    WeightFunctionType weight_function)
{
  switch(scale_estimator)
  {
    case TDistScale:
      if(verbose_)
        printf("Using TDistribution Scale Estimator\n");
      scale_estimator_.reset(new robust_cost::TDistributionScaleEstimator());
      use_weights_=true;
      break;
    case MADScale:
      if(verbose_)
        printf("Using MAD Scale Estimator\n");
      scale_estimator_.reset(new robust_cost::MADScaleEstimator());
      use_weights_=true;
    break;
    case NormalScale:
      if(verbose_)
        printf("Using Normal Scale Estimator\n");
      scale_estimator_.reset(new robust_cost::NormalDistributionScaleEstimator());
      use_weights_=true;
      break;
    default:
      if(verbose_)
        printf("Using Unit Scale Estimator\n");
      scale_estimator_.reset(new robust_cost::UnitScaleEstimator());
      use_weights_=false;
  }

  switch(weight_function)
  {
    case TDistWeight:
      if(verbose_)
        printf("Using TDistribution Weight Function\n");
      weight_function_.reset(new robust_cost::TDistributionWeightFunction());
      break;
    case TukeyWeight:
      if(verbose_)
        printf("Using Tukey Weight Function\n");
      weight_function_.reset(new robust_cost::TukeyWeightFunction());
      break;
    case HuberWeight:
      if(verbose_)
        printf("Using Huber Weight Function\n");
      weight_function_.reset(new robust_cost::HuberWeightFunction());
      break;
    default:
      if(verbose_)
        printf("Using Unit Weight Function\n");
      weight_function_.reset(new robust_cost::UnitWeightFunction());
  }
}

void vk::NLLSSolver1D::setPrior(
    const double&  prior,
    const double&  Information)
{
  have_prior_ = true;
  prior_ = prior;
  I_prior_ = Information;
}

void vk::NLLSSolver1D::reset()
{
  have_prior_ = false;
  chi2_ = 1e10;
  mu_ = mu_init_;
  nu_ = nu_init_;
  n_meas_ = 0;
  n_iter_ = n_iter_init_;
  iter_ = 0;
  stop_ = false;
}

inline const double& vk::NLLSSolver1D::getChi2() const
{
  return chi2_;
}

inline const double& vk::NLLSSolver1D::getInformationMatrix() const
{
  return H_;
}
