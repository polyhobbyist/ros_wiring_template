/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"

#include "Arduino.h"
#include "Wire.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class I2CPublisher : public rclcpp::Node
{
  public:
    I2CPublisher()
    : Node("i2cpublisher")
    {
    }

    void initialize()
    {
      get_parameter_or<uint8_t>("id", _id, 0x52); 
      get_parameter_or<double>("poll", _poll, 500.0);

      Wire.begin();

      _timer = this->create_wall_timer(std::chrono::duration<double, std::milli>(_poll), std::bind(&I2CPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      //Poll sensor for new data 
    }
    rclcpp::TimerBase::SharedPtr _timer;

    uint8_t _id;
    double _poll;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<I2CPublisher>();
    node->declare_parameter("id");
    node->declare_parameter("poll");

    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}