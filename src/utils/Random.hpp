#ifndef __RANDOM_HPP__
#define __RANDOM_HPP__

#include <random>


namespace Random {

inline float FloatUniform(float lower_range = 0, float upper_range=1.f)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(lower_range, upper_range);

    return dis(gen);
}

}


#endif  // __RANDOM_HPP__
