#ifndef ARDUINO_SPI_H
#define ARDUINO_SPI_H

#define SPI_CLOCK_DIV32 0

#define MSBFIRST 0

#define SPI_MODE0 0

class SPIClass
{
    public:
    static void begin()
    {

    }

    static void setClockDivider(uint16_t)
    {

    }

    static void setBitOrder(uint8_t)
    {

    }

    static void setDataMode(uint8_t)
    {
        
    
    }
    static uint8_t transfer(uint8_t)
    {
        return 0;

    }

};

extern SPIClass SPI;

#endif