#include "Multiplexer.hpp"

MultiPlexer::MultiPlexer(uint8_t _address) {
    address = _address;
}

void MultiPlexer::addChannel(uint8_t channel) {
    channelStates[channel] = 1;
    activeChannels++;
}

void MultiPlexer::removeChannel(uint8_t channel) {
    channelStates[channel] = 0;
    activeChannels--;
}

uint8_t MultiPlexer::getActiveChannelCount() {
    return activeChannels;
}

void MultiPlexer::enableChannel(uint8_t channel) {
    Wire.beginTransmission(address);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void MultiPlexer::disableChannel(uint8_t channel) {
    Wire.beginTransmission(address);
    Wire.write(0 << channel);
    Wire.endTransmission();
}