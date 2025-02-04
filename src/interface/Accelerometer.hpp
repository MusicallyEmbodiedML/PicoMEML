#ifndef __ACCELEROMETER_HPP__
#define __ACCELEROMETER_HPP__

#include <array>
#include <vector>

template<size_t N>
class accumulator() {
public:
    accumulator() {
        buf.fill(0);
    }
    
    float Process(float v) {
        sum -= buf.at(index);
        buf.at(index)=v;
        sum += v;
        index++;
        if (index == N) {
            index=0;
        }
    }
private:
    std::array<float, N> buf;
    size_t index=0;
    float sum=0;
}

template<size_t N_DIM>
class Accelerometer {

 public:
    Accelerometer();
    void Process(const std::vector<float> &x) {
        //assuming three params
        
        for(size_t i=0; i < 3; i++) {
            x[i] = accumulators_stage1[i].Process(x[i]);
            x[i] = accumulators_stage2[i].Process(x[i]);
        }

    }

 private:
    std::vector<accumulator<N_DIM>, 3> accumulators_stage1;
    std::vector<accumulator<N_DIM>, 3> accumulators_stage2;
};




#endif  // __ACCELEROMETER_HPP__