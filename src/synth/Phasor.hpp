#ifndef __PHASOR_HPP__
#define __PHASOR_HPP__


class Phasor {
 public:

    Phasor(float sample_rate);
    float Process();
    void SetF0(float f0);
    static bool ToPulse(float ph, float width = 0.4);

 protected:
    const float fs_;
    float increment_;
    float wraparound_;
    float phase_;
};


#endif  // __PHASOR_HPP__
