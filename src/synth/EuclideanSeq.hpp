#ifndef __EUCLIDEAN_SEQ_HPP__
#define __EUCLIDEAN_SEQ_HPP__

#include "Arduino.h"

#include <vector>
#include "Phasor.hpp"
#include "../PicoDefs.hpp"

class EuclideanSeq {

 public:

    struct params {
        int n;
        int k;
        int offset_n;
        int offset_d;
        int ph_mult;
    };
    static constexpr size_t n_params = sizeof(params) / sizeof(int);
    
    EuclideanSeq();
    EuclideanSeq(const std::vector<int> &n_choices);
    bool Process(float phasor);
    void SetParams(params p);
    void MapNNParams(std::vector<float> nn_params);
    inline float Probe(size_t probe_n) {
        return probes_[probe_n];
    }

 protected:

    const std::vector<int> n_choices_;

    static constexpr float pulse_width_ = 0.5;
    bool is_init_;
    params params_;
    float offset_;
    static constexpr size_t n_probes_ = 2;
    float probes_[n_probes_];

    int MakeN_(float param);
    int MakeMul_(float param);
};

const size_t kNGenerators = 4;
constexpr size_t kN_synthparams = kNGenerators * EuclideanSeq::n_params;


class EuclideanSeqApp {
public:
    void init(uint sample_rate) {
        ph = new Phasor(sample_rate);
        ph->SetF0(0.5);
        for (unsigned int n = 0; n < kNGenerators; n++) {
            seq[n] = new EuclideanSeq();
            // Do not set params until NN sends them
            //seq[n]->SetParams(seq_params[n]);
        }
        for(auto &v: pulsePins) {
            Serial.printf("Pulse out: %d\n", v);
            pinMode(v, OUTPUT);
        }
    }

    static void GenParams(std::vector<float> &param_vector)
    {
        float rand_scale = 1.f / static_cast<float>(RAND_MAX);

        for(size_t i=0; i < kN_synthparams; i++) {
            param_vector[i] = std::rand() * rand_scale;
        }
    }

    void loop() {
        float phasor = ph->Process();
        bool pulse = ph->ToPulse(phasor);
        static size_t ct=0;

        // Generate     
        for(size_t i_generator=0; i_generator < kNGenerators; i_generator++) {
            bool euclideanSig = seq[i_generator]->Process(phasor);
            // Port outs
            // // - Pulse
            // port2Out = pulse;
            // - Sequencers
            digitalWrite(pulsePins[i_generator], euclideanSig);
            // if (ct % 1000==0) {
            //     Serial.printf("%d %d\t",i_generator, euclideanSig);
            // }

        }
        // if (ct % 1000==0) {
        //     Serial.println();
        // }
        // ct++;
    }

    void mapParameters(std::vector<float> &params) {
        Serial.printf("MP: ");
        for (unsigned int n_gen = 0; n_gen < kNGenerators; n_gen++) {
            // Chop up the parameter vector into smaller vectors of n_params parameters
            std::vector<num_t>::iterator start = params.begin() + n_gen*EuclideanSeq::n_params;
            std::vector<num_t>::iterator end = start + EuclideanSeq::n_params;
            std::vector<num_t> single_gen_params(start, end);
            assert(single_gen_params.size() == EuclideanSeq::n_params);

            seq[n_gen]->MapNNParams(single_gen_params);
            Serial.printf("%f\t", single_gen_params[0]);
        }
        Serial.println();
    }
private:
    Phasor *ph;
    EuclideanSeq *seq[kNGenerators] = { nullptr };

    const size_t pulsePins[kNGenerators] = {PinConfig::pulse0,PinConfig::pulse1,PinConfig::pulse2,PinConfig::pulse3};

};

#endif  // __EUCLIDEAN_SEQ_HPP__
