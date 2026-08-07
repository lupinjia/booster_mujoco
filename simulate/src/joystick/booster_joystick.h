#pragma once

// A self-contained joystick abstraction for booster_mujoco.
// The booster_robotics_sdk does not provide a joystick class (unlike
// unitree_sdk2's unitree::common::UnitreeJoystick), so this project keeps its
// own implementation. The API intentionally mirrors the UnitreeJoystick
// interface so that the gamepad mapping code in physics_joystick.h stays
// unchanged.

#include <math.h>
#include <stdint.h>
#include <iostream>
#include <chrono>

namespace booster
{
namespace common
{

class KeyBase
{
public:
  KeyBase() = default;

  bool pressed = false;
  bool on_pressed = false;
  bool on_released = false;
  int click_cnt = 0;          // used at `on_pressed`
  float pressed_time = 0.0;

protected:
  void update(bool is_pressed)
  {
    pressed = is_pressed;
    on_pressed = (pressed && !last_pressed_);
    on_released = (!pressed && last_pressed_);
    last_pressed_ = pressed;

    auto now = std::chrono::steady_clock::now();
    if (on_pressed)
    {
      if (std::chrono::duration<float>(now - last_click_time_).count() < double_click_threshold_)
      {
        click_cnt++;
      }
      else
      {
        click_cnt = 1;
      }
      last_click_time_ = now;
    }

    if (pressed)
    {
      pressed_time = std::chrono::duration<float>(now - last_click_time_).count();
    }
    if (!(pressed || on_released))
    { // save once time on released
      pressed_time = 0.0;
    }
  }

private:
  bool last_pressed_{false};
  std::chrono::steady_clock::time_point last_click_time_{};
  const float double_click_threshold_{0.5}; // Unit: second
};

template <typename T> // int, bool, string
class Button : public KeyBase
{
public:
  void operator()(const T& data)
  {
    bool is_pressed = (data != dataNull_);
    update(is_pressed);
    data_ = data;
  }
  const T& operator()() { return data_; }

private:
  T data_{}, dataNull_{};
};

class Axis : public KeyBase
{
public:
  void operator()(const float& data)
  { // update
    auto data_deadzone = std::fabs(data) < deadzone ? 0.0 : data;
    float new_data = data_ * (1.0 - smooth) + data_deadzone * smooth;
    bool is_pressed = (new_data > threshold);
    update(is_pressed);
    data_ = new_data;
  }

  const float& operator()() { return data_; }

  float smooth = 0.03;
  float deadzone = 0.01;

  float threshold{0.5}; // Change an axis value to a button
private:
  float data_{};
};

class BoosterJoystick
{
public:
  BoosterJoystick() = default;

  // Adopts standard joystick key names
  Button<int> back;
  Button<int> start;
  Button<int> LS;
  Button<int> RS;
  Button<int> LB;
  Button<int> RB;
  Button<int> A;
  Button<int> B;
  Button<int> X;
  Button<int> Y;
  Button<int> up;
  Button<int> down;
  Button<int> left;
  Button<int> right;
  Button<int> F1;
  Button<int> F2;
  Axis lx;
  Axis ly;
  Axis rx;
  Axis ry;
  Axis LT;
  Axis RT;

  virtual void update() {};
};

} // namespace common
} // namespace booster
