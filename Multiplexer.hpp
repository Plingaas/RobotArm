#ifndef MULTIPLEXER_HPP
#define MULTIPLEXER_HPP
#include <Arduino.h>
#include <Wire.h>


class MultiPlexer {

public:
    MultiPlexer(uint8_t _address = 0x70);
    
    void addChannel(uint8_t channel);
    void removeChannel(uint8_t channel);
    uint8_t getActiveChannelCount();

    void enableChannel(uint8_t channel);
    void disableChannel(uint8_t channel);

private:
    uint8_t address;
    bool channelStates[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t activeChannels = 0;
};
    
#endif