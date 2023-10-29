#include "Encoder.hpp"

Encoder::Encoder() {

}

Encoder::Encoder(MultiPlexer& mp, uint8_t channel) {
    MP = &mp;
    MPChannel = channel;
    MP->addChannel(MPChannel);
}

void Encoder::setMPChannel(uint8_t channel) {
    MPChannel = channel;
    MP->addChannel(MPChannel);
}

float Encoder::getPosition() {
    MP->enableChannel(MPChannel);
    encoderSteps = encoder.getCumulativePosition();
    revolutions = encoderSteps * 0.000244140625f; // (1/4096) Multiplication is faster than division
    position = revolutions * DPR;
    return position;
}

void Encoder::resetPosition() {
    MP->enableChannel(MPChannel);
    encoder.resetCumulativePosition(0);
    position = 0;
    revolutions = 0;
}

void Encoder::setDPR(float dpr) {
    DPR = dpr;
}

void Encoder::setMP(MultiPlexer& mp) {
    MP = &mp;
}