#include "Phasor.hpp"

Phasor::Phasor(float sample_rate) :
    fs_(sample_rate),
    increment_(0),
    wraparound_(1),
    phase_(0)
{
}

float Phasor::Process()
{
    phase_ += increment_;
    while (phase_ >= wraparound_) {
        phase_ -= wraparound_;
    }
    return phase_;
}

void Phasor::SetF0(float f0)
{
    increment_ = f0 / fs_;
    wraparound_ = 1;
}

bool Phasor::ToPulse(float ph, float width)
{
    if (ph <= width) {
        return true;
    } else {
        return false;
    }
}
