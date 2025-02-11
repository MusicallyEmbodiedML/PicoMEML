#ifndef __SCOPE_HPP__
#define __SCOPE_HPP__

#include "../PicoDefs.hpp"
#include <Arduino.h>


class Scope {
 
 public:

    static inline void Setup() {
#if SCOPE_ON
        for (unsigned int n = 0; n < kNProbes; n++) {
            pinMode(gpio_idx[n], OUTPUT);
        }
#endif
    }

    static inline void Set(const size_t probe_idx, const bool val) {
#if SCOPE_ON
        if (probe_idx < kNProbes) {
            digitalWrite(gpio_idx[probe_idx], val);
        }
#endif
    }

 protected:

#if SCOPE_ON
    static const size_t kNProbes = 2;
    static constexpr size_t gpio_idx[kNProbes] {
        led_Scope0,
        led_Scope1
    };
#endif
};


#endif  // __SCOPE_HPP__
