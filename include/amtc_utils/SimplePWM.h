/**
 * @file SimplePWM.h
 *
 * @author Isao Parra
 *
 * @date Dec 13, 2016 (creation)
 */

#ifndef AMTC_UTILS_SIMPLEPWM_H
#define AMTC_UTILS_SIMPLEPWM_H

#include <ros/ros.h>
#include <amtc_common/Debug.h>

#include <string>

namespace amtc
{

class SimplePWM
{
public:
  SimplePWM(float period = 1.0, float duty_cycle = 0.5, unsigned int max_cycles = 0)
  {
    set_period(period);
    set_duty_cycle(duty_cycle);
    _is_running = false;
    set_max_cycles(max_cycles);
  }

  ~SimplePWM(){}

  bool set_period(double period)
  {
    _period = period;
    _init_time = ros::Time::now();

    if( _period < 0.01)
    {
      _period = 0.01;

      ROS_ERROR("SimplePWM: period < 0.01, set as 0.01");
      set_duty_cycle(_duty_cycle);

      return false;
    }

    set_duty_cycle(_duty_cycle);

    return true;
  }

  bool set_duty_cycle(double duty_cycle)
  {
    _duty_cycle = duty_cycle;
    _calculed_duty_cycle = duty_cycle * _period;

    if(_duty_cycle < 0.0)
    {
      _duty_cycle = 0.0;
      _calculed_duty_cycle = 0.0;
      ROS_ERROR("SimplePWM: duty_cycle < 0.0, set as 0.0");

      return false;
    }

    if(_duty_cycle > 1.0)
    {
      _duty_cycle = 1.0;
      _calculed_duty_cycle = _period;
      ROS_ERROR("SimplePWM: duty_cycle > 1.0, set as 1.0");

      return false;
    }

    return true;
  }

  void set_max_cycles(unsigned int max_cycles)
  {
    _max_cycles = max_cycles;
  }

  void start()
  {
    _init_time = ros::Time::now();
    _is_running = true;
  }

  void stop()
  {
    _is_running = false;
  }

  bool is_running()
  {
    return _is_running;
  }

  unsigned int count()
  {
    if(_is_running == true)
    {
      return (unsigned int)std::floor(((ros::Time::now() - _init_time).toSec())/_period);
    }

    return  0;
  }

  bool value()
  {
    if( _is_running == true)
    {
      if(_max_cycles != 0 && _max_cycles <= count())
      {
        stop();
        return false;
      }

      if(_duty_cycle == 0.0)
      {
        return false;
      }

      if(_duty_cycle == 1.0)
      {
        return true;
      }

      return (std::fmod(((ros::Time::now() - _init_time).toSec()), _period)> _calculed_duty_cycle) ? false : true;
    }

    return false;
  }

private:

  ros::Time _init_time;
  double _duty_cycle;
  double _calculed_duty_cycle;
  double _period;
  bool _is_running;
  unsigned int _max_cycles;
};

}

#endif /* AMTC_UTILS_SIMPLEPWM_H_ */