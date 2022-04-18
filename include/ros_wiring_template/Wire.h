#ifndef ARDUINO_WIRE_H
#define ARDUINO_WIRE_H
#include "Arduino.h"

class WireClass
{
    public:
    static void begin()
    {

    }

    static void beginTransmission(uint16_t id)
    {

    }

    static void endTransmission()
    {

    }

    static bool available()
    {
        return false;
    }

    static uint8_t read()
    {
        return 0;

    }

    static void write(uint8_t value)
    {
        
    }

    static void requestFrom(uint16_t, size_t)
    {

    }

};

extern WireClass Wire;

#endif