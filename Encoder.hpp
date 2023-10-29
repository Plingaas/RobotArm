#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "Multiplexer.hpp"
#include <AS5600.h>

class Encoder {

public:
    Encoder();
    Encoder(MultiPlexer& mp, uint8_t channel);
    void setMPChannel(uint8_t channel);

    float getPosition();
    void resetPosition();
    void setDPR(float dpr);
    void setMP(MultiPlexer& mp);
private:
    MultiPlexer* MP;
    uint8_t MPChannel;
    AS5600 encoder;
    int encoderSteps = 0;
    float revolutions = 0;
    float position = 0;
    float DPR = 360;
};

#endif