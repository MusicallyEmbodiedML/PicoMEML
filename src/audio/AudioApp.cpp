#include <Arduino.h>
#include "AudioApp.hpp"
#include "../PicoDefs.hpp"
#if FM_SYNTH
#include "../synth/FMSynth.hpp"
#elif FX_PROCESSOR
#include "../synth/matrixMix.hpp"
#elif EUCLIDEAN
#include "../synth/EuclideanSeq.hpp"
#endif
#include <vector>
#include <cmath>
#include "AnalysisParams.hpp"


#if FM_SYNTH
static AUDIO_MEM FMSynth fm_synth_(kSampleRate);
#elif FX_PROCESSOR
static AUDIO_MEM MaxtrixMixApp multi_fx_app_(kSampleRate);
#elif EUCLIDEAN
static AUDIO_MEM EuclideanSeqApp euclideanApp;
#endif  // FM_SYNTH


void AudioAppSetup(void)
{
#if FM_SYNTH
maxiSettings::setup(kSampleRate, 1, 64);
fm_synth_.EnableMIDI(false);
#elif EUCLIDEAN
euclideanApp.init(kSampleRate);
#endif
}

size_t count=0;

stereosample_t AUDIO_FUNC(AudioAppProcess)(stereosample_t y)
{
#if FM_SYNTH
    y.L = fm_synth_.process();
    y.R = y.L;
#elif FX_PROCESSOR
    y.L = multi_fx_app_.play(y.L);
    y.R = y.L;
#elif EUCLIDEAN
    euclideanApp.loop();
    //no sound, just CV
    y.L=y.R=0;
#endif // FM_SYNTH

    std::vector<float> params { std::abs(y.L) };
    AnalysisParamsWrite(params);

    return y;
}

void AudioAppSetParams(std::vector<float> &params)
{
#if FM_SYNTH
    fm_synth_.mapParameters(params);
#elif FX_PROCESSOR
    multi_fx_app_.mapParameters(params);
#elif EUCLIDEAN
    euclideanApp.mapParameters(params);    
#endif
}
