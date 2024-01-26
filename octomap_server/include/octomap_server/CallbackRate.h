#ifndef OCTOMAP_SERVER_CALLBACK_RATE_H
#define OCTOMAP_SERVER_CALLBACK_RATE_H

#include <cmath>

#include <octomap/octomap_utils.h>
#include <ros/time.h>

namespace octomap_server {

class CallbackRate
{
public:
  CallbackRate()
  {
    reset();
  }

  void reset()
  {
    n_ = 0;
  }

  void update()
  {
    ros::Time now = ros::Time::now();
    if (n_ == 0)
    {
      last_time_ = now;
    }
    else
    {
      double delta_t = (now - last_time_).toSec();
      if (n_ == 1)
      {
        delta_t_ema_ = delta_t;
      }
      else
      {
        if (delta_t > delta_t_invalid_factor_ * delta_t_ema_)
        {
          reset();
          last_time_ = now;
        }
        else
        {
          delta_t_ema_ = delta_t * ema_alpha_ + delta_t_ema_ * (1.0 - ema_alpha_);
        }
      }
    }
    ++n_;
  }

  bool valid()
  {
    return n_ >= min_n_ && delta_t_ema_ > 0.0;
  }

  double getCurrentRate()
  {
    return 1.0 / delta_t_ema_;
  }

  bool rateIsApproximately(double expected_rate)
  {
    return valid() && std::abs(getCurrentRate() - expected_rate) < expected_rate * approximate_epsilon_;
  }

protected:
  ros::Time last_time_;
  size_t n_;
  // Current delta_t exponential moving average
  double delta_t_ema_;
  // Alpha to use when calculating the exponential moving average
  double ema_alpha_ = 0.05;
  // Minimum number of samples before considering moving average valid.
  double min_n_ = 20;
  // If a delta_t of more than the factor times the current delta_t is encountered the EMA is reset.
  double delta_t_invalid_factor_ = 5.0;
  // What factor of the expected rate will be considered approximately equal to the tracked rate.
  double approximate_epsilon_ = .2;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SERVER_CALLBACK_RATE_H
