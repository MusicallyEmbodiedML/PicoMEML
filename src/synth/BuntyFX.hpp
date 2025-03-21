#pragma once

//#define NDEBUG
#include <cassert>
#include <vector>
#include <array>
#include "maximilian.h"
#include "OnePoleSmoother.hpp"

const size_t kN_synthparams = 30;

inline float vox_fasttanh_ultra( const float x )
{
	const float ax = fabs( x );
	const float x2 = x * x;
	const float z = x * ( 0.773062670268356 + ax +
		( 0.757118539838817 + 0.0139332362248817 * x2 * x2 ) *
		x2 * ax );

	return( z / ( 0.795956503022967 + fabs( z )));
}


class BuntyFXApp {
public:

    BuntyFXApp(size_t sample_rate): smoother_(100.f, sample_rate) {
        maxiSettings::setup(sample_rate, 1, 16);
        unsmoothParams.resize(kN_synthparams);
        params.resize(kN_synthparams);
                
    }

    static void GenParams(std::vector<float> &param_vector)
    {
        float rand_scale = 1.f / static_cast<float>(RAND_MAX);

        for(size_t i=0; i < kN_synthparams; i++) {
            param_vector[i] = std::rand() * rand_scale;
        }
        //printf("\n");
    }

    float play(float x) {
        // Smooth parameters first
        smoother_.Process(unsmoothParams.data(), params.data());

        const size_t ofs = 0; //offset from mixer params

        // float flange = flanger.flange(x, 800, params[ofs+5] * 0.99, params[ofs+6] * 10.f, 0.5 + (params[ofs+7] * 0.5));

        // float dist = distortion.softclip(flange *  params[ofs+4] * 2) * 0.3;

        float flanged = flanger.flange(x, 800, params[ofs+5] * 0.99, params[ofs+6] * 10.f, 0.5 + (params[ofs+7] * 0.5));

        float delayed = dl.play(flanged, 5998, params[ofs+10] * 0.99) 
                        + dl2.play(flanged, 1528, params[ofs+9] * 0.99);

        // float rmMod = sinosc.sinebuf(1.f + (params[ofs+11] * params[ofs+11] * 800));
        // float rmSig = delayed * rmMod;

        // return rmSig;
        return x + delayed;
    }

    void mapParameters(std::vector<float> &newparams) {
        for(size_t i=0; i < params.size(); i++) {
            unsmoothParams[i] = newparams[i];
        }
    }

private:
    maxiNonlinearity distortion;
    maxiDelayline<6000> dl;
    maxiDelayline<1600> dl2;
    maxiFlanger<2000> flanger;
    maxiOsc sinosc;
    maxiBiquad filt;

    std::vector<float> unsmoothParams, params;
    OnePoleSmoother<kN_synthparams> smoother_;

};

