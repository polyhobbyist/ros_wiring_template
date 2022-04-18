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
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

#ifdef _WIN32
#pragma optimize( "", off )
#else
#include "i2c/i2c.h"

#pragma GCC optimize ("O0")
#endif

using namespace std::chrono_literals;
using std::placeholders::_1;

const uint32_t kFrequencyOccilator = 25000000;
const float kPwmFrequency = 1000.0f;

const uint16_t kMaxThrottle = 500; // +/- microseconds 
const uint16_t kPulseNeutral = 1500; // microseconds
const uint16_t kPulseMin = kPulseNeutral - kMaxThrottle; // microseconds
const uint16_t kPulseMax = kPulseNeutral + kMaxThrottle; // microseconds

const uint8_t kChannels = 16;

typedef enum
{
   ServoCommand_Mode1 = 0x00,
   ServoCommand_ChannelBegin_On_Low = 0x06,
   ServoCommand_ChannelBegin_On_High = 0x07,
   ServoCommand_ChannelBegin_Off_Low = 0x08,
   ServoCommand_ChannelBegin_Off_High = 0x09,

   // ... 15
   

   ServoCommand_Prescale = 0xFE
} QwiicServoCommand;

typedef enum
{
    Mode1_CalibrateAll = 0x01,
    Mode1_Sub3 = 0x02,
    Mode1_Sub2 = 0x04,
    Mode1_Sub1 = 0x08,
    Mode1_Sleep = 0x10,
    Mode1_AI = 0x20,
    Mode1_ExternalClock = 0x40,
    Mode1_Restart = 0x80
} QwiicServoMode1Value;


class ServoSubscriber : public rclcpp::Node
{

  public:
    ServoSubscriber()
    : Node("ros_qwiic_servo")
    {
    }

    void start()
    {
        #ifndef _WIN32
        if ((_i2cFileDescriptor = i2c_open("/dev/i2c-1")) == -1) 
        {
            return;
        }

        _i2cDevice.bus = _i2cFileDescriptor;
        _i2cDevice.addr = 0x40;
        _i2cDevice.tenbit = 0;
        _i2cDevice.delay = 10;
        _i2cDevice.flags = 0;
        _i2cDevice.page_bytes = 8;
        _i2cDevice.iaddr_bytes = 8;
        #endif

        reset();
        setPWMFrequency(kPwmFrequency);

        for (int channel = 0; channel < kChannels; channel++)
        {
            std::ostringstream channelTopic;
            channelTopic << "/servo/channel_" << channel;

            _subscription[channel] = create_subscription<std_msgs::msg::Float32>(
                channelTopic.str(), 1,
                [=] (const std_msgs::msg::Float32::SharedPtr msg)
                {
                    // -1 , 0, 1
                    
                    float currentThrottle = std::abs(msg->data) * (float)kMaxThrottle;
                    if (msg->data > 0)
                    {
                        writeMicroseconds(channel, kPulseNeutral + currentThrottle);
                    }
                    else
                    {
                        writeMicroseconds(channel, kPulseNeutral - currentThrottle);
                    }

                });      
        }
    }

  private:
    void reset()
    {
        command(ServoCommand_Mode1, Mode1_Restart);
    }

    void setPWMFrequency(float frequency)
    {
        if (frequency < 1.0f)
        {
            frequency = 1;
        }
        
        if (frequency > 3500.0f)
        {
            frequency = 3500;
        }

        float prescaleFloat = ((kFrequencyOccilator / (frequency * 4096.0f)) + 0.5f) - 1.0f;
        if (prescaleFloat < 3.0f)
            prescaleFloat = 3.0f;
        if (prescaleFloat > 255.0f)
            prescaleFloat = 255.09f;

        uint8_t prescale = (uint8_t)prescaleFloat;

        uint8_t oldMode = readByte(ServoCommand_Mode1);
        uint8_t newMode = (oldMode & ~Mode1_Restart) | Mode1_Sleep;

        command(ServoCommand_Mode1, newMode);
        command(ServoCommand_Prescale, prescale);
        command(ServoCommand_Mode1, oldMode);
        rclcpp::sleep_for(5ms);
        command(ServoCommand_Mode1, oldMode | Mode1_Restart | Mode1_AI);
    }

    void command(QwiicServoCommand command, uint8_t value)
    {
        #ifndef _WIN32
        int ret = i2c_ioctl_write(&_i2cDevice, command, &value, 1);
        if (ret == -1 || (size_t)ret != 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("servo"), "failed to write to servo hat: [%d]", ret);
        }
        #endif
    }

    uint8_t readByte(QwiicServoCommand command)
    {
        uint8_t ret = 0;
        #ifndef _WIN32
        i2c_ioctl_read(&_i2cDevice, command, &ret, 1);
        #endif

        return ret;
    }

    uint16_t readShort(QwiicServoCommand command)
    {
        uint16_t ret = 0;
        #ifndef _WIN32
        i2c_ioctl_read(&_i2cDevice, command, &ret, sizeof(ret));
        #endif

        return ret;
    }

    void setPWM(uint8_t channel,  uint16_t on, uint16_t off)
    {
        #ifndef _WIN32
        uint8_t commandBuffer[] = 
        { 
            (uint8_t)on, (uint8_t)(on >> 8),
            (uint8_t)off, (uint8_t)(off >> 8)
        };

        i2c_ioctl_write(&_i2cDevice, ServoCommand_ChannelBegin_On_Low + 4 * channel, commandBuffer, sizeof(commandBuffer));
        #endif
    }

    void writeMicroseconds(uint8_t channel, uint16_t microseconds)
    {
        double pulse = microseconds;

        double pulseLength = 1000000.0; // 1,000,000 us/second
        uint16_t prescale = readShort(ServoCommand_Prescale);
        prescale += 1;  //?
        pulseLength *= prescale;
        pulseLength /= kFrequencyOccilator;

        pulse /= pulseLength;

        setPWM(channel, 0, pulse);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscription[kChannels];

    #ifndef _WIN32
    int _i2cFileDescriptor;
    I2CDevice _i2cDevice;  
    #endif  
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServoSubscriber>();

    node->start();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}