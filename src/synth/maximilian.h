/*
*  platform independent synthesis library using portaudio or rtaudio
 *  maximilian.h
 *
 *  Created by Mick Grierson on 29/12/2009.
 *  Copyright 2009 Mick Grierson & Strangeloop Limited. All rights reserved.
 *	Thanks to the Goldsmiths Creative Computing Team.
 *	Special thanks to Arturo Castro for the PortAudio implementation.
 *
 *	Permission is hereby granted, free of charge, to any person
 *	obtaining a copy of this software and associated documentation
 *	files (the "Software"), to deal in the Software without
 *	restriction, including without limitation the rights to use,
 *	copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the
 *	Software is furnished to do so, subject to the following
 *	conditions:
 *
 *	The above copyright notice and this permission notice shall be
 *	included in all copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *	OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef MAXIMILIAN_H
#define MAXIMILIAN_H

#define MAXIMILIAN_RT_AUDIO

#include  <cstdlib>
#include <cmath>
#include <functional>

using namespace std;

#ifndef PI
#define PI 3.1415926
#endif
#define TWOPI 6.2831853

#define MAXITYPE float

#define LOG(x) 

#define USE_STRINGS

#ifdef USE_STRINGS
#include <string>
#define STRINGCLASS std::string
#endif


#define DOUBLEARRAY_REF vector<MAXITYPE> &
#define DOUBLEARRAY vector<MAXITYPE> 
#define NORMALISE_ARRAY_TYPE(invar, outvar) vector<MAXITYPE> outvar = vector<MAXITYPE>(invar.begin(), invar.end()); //emplace into new variable
#define DECLARE_F64_ARRAY(x) std::vector<MAXITYPE> x;
#define F64_ARRAY_SIZE(x) x.size()
#define F64_ARRAY_SETFROM(to,from) to = from;
#define F64_ARRAY_CLEAR(x) x.clear();
#define F64_ARRAY_AT(x,i) x[i]
// #define LOG(x) cout << x;
#define F64_ARRAY_CREATE(len) std::vector<MAXITYPE>(len);
#define F64_ARRAY_FILL(x, val) std::fill(x.begin(), x.end(), val);



const float pitchRatios[256] = {0.0006517771980725, 0.0006905338959768, 0.0007315951515920, 0.0007750981021672, 0.0008211878011934, 0.0008700182079338, 0.0009217521874234, 0.0009765623835847, 0.0010346318595111, 0.0010961542138830, 0.0011613349197432, 0.0012303915573284, 0.0013035543961450, 0.0013810677919537, 0.0014631903031841, 0.0015501962043345, 0.0016423756023869, 0.0017400364158675, 0.0018435043748468, 0.0019531247671694, 0.0020692637190223, 0.0021923084277660, 0.0023226698394865, 0.0024607831146568, 0.0026071087922901, 0.0027621355839074, 0.0029263808391988, 0.0031003924086690, 0.0032847514376044, 0.0034800728317350, 0.0036870087496936, 0.0039062497671694, 0.0041385274380445, 0.0043846168555319, 0.0046453396789730, 0.0049215662293136, 0.0052142175845802, 0.0055242711678147, 0.0058527616783977, 0.0062007848173380, 0.0065695028752089, 0.0069601456634700, 0.0073740174993873, 0.0078124995343387, 0.0082770548760891, 0.0087692337110639, 0.0092906802892685, 0.0098431324586272, 0.0104284351691604, 0.0110485423356295, 0.0117055233567953, 0.0124015696346760, 0.0131390057504177, 0.0139202913269401, 0.0147480349987745, 0.0156249990686774, 0.0165541097521782, 0.0175384692847729, 0.0185813605785370, 0.0196862649172544, 0.0208568722009659, 0.0220970865339041, 0.0234110467135906, 0.0248031392693520, 0.0262780115008354, 0.0278405826538801, 0.0294960699975491, 0.0312499981373549, 0.0331082195043564, 0.0350769385695457, 0.0371627211570740, 0.0393725298345089, 0.0417137444019318, 0.0441941730678082, 0.0468220934271812, 0.0496062822639942, 0.0525560230016708, 0.0556811690330505, 0.0589921437203884, 0.0624999962747097, 0.0662164390087128, 0.0701538771390915, 0.0743254423141479, 0.0787450596690178, 0.0834274888038635, 0.0883883461356163, 0.0936441868543625, 0.0992125645279884, 0.1051120460033417, 0.1113623380661011, 0.1179842874407768, 0.1249999925494194, 0.1324328780174255, 0.1403077542781830, 0.1486508846282959, 0.1574901193380356, 0.1668549776077271, 0.1767766922712326, 0.1872883737087250, 0.1984251290559769, 0.2102240920066833, 0.2227246761322021, 0.2359685748815536, 0.2500000000000000, 0.2648657560348511, 0.2806155085563660, 0.2973017692565918, 0.3149802684783936, 0.3337099552154541, 0.3535533845424652, 0.3745767772197723, 0.3968502581119537, 0.4204482138156891, 0.4454493522644043, 0.4719371497631073, 0.5000000000000000, 0.5297315716743469, 0.5612310171127319, 0.5946035385131836, 0.6299605369567871, 0.6674199104309082, 0.7071067690849304, 0.7491535544395447, 0.7937005162239075, 0.8408964276313782, 0.8908987045288086, 0.9438742995262146, 1.0000000000000000, 1.0594631433486938, 1.1224620342254639, 1.1892070770263672, 1.2599210739135742, 1.3348398208618164, 1.4142135381698608, 1.4983071088790894, 1.5874010324478149, 1.6817928552627563, 1.7817974090576172, 1.8877485990524292, 2.0000000000000000, 2.1189262866973877, 2.2449240684509277, 2.3784141540527344, 2.5198421478271484, 2.6696796417236328, 2.8284270763397217, 2.9966142177581787, 3.1748020648956299, 3.3635857105255127, 3.5635950565338135, 3.7754974365234375, 4.0000000000000000, 4.2378525733947754, 4.4898481369018555, 4.7568287849426270, 5.0396842956542969, 5.3393597602844238, 5.6568546295166016, 5.9932284355163574, 6.3496046066284180, 6.7271714210510254, 7.1271901130676270, 7.5509948730468750, 8.0000000000000000, 8.4757051467895508, 8.9796962738037109, 9.5136575698852539, 10.0793685913085938, 10.6787195205688477, 11.3137092590332031, 11.9864568710327148, 12.6992092132568359, 13.4543428421020508, 14.2543802261352539, 15.1019897460937500, 16.0000000000000000, 16.9514102935791016, 17.9593944549560547, 19.0273151397705078, 20.1587371826171875, 21.3574390411376953, 22.6274185180664062, 23.9729137420654297, 25.3984184265136719, 26.9086875915527344, 28.5087604522705078, 30.2039794921875000, 32.0000000000000000, 33.9028205871582031, 35.9187889099121094, 38.0546302795410156, 40.3174743652343750, 42.7148780822753906, 45.2548370361328125, 47.9458274841308594, 50.7968368530273438, 53.8173751831054688, 57.0175209045410156, 60.4079589843750000, 64.0000076293945312, 67.8056411743164062, 71.8375778198242188, 76.1092605590820312, 80.6349563598632812, 85.4297561645507812, 90.5096740722656250, 95.8916625976562500, 101.5936737060546875, 107.6347503662109375, 114.0350418090820312, 120.8159179687500000, 128.0000152587890625, 135.6112823486328125, 143.6751556396484375, 152.2185211181640625, 161.2699127197265625, 170.8595123291015625, 181.0193481445312500, 191.7833251953125000, 203.1873474121093750, 215.2695007324218750, 228.0700836181640625, 241.6318511962890625, 256.0000305175781250, 271.2225646972656250, 287.3503112792968750, 304.4370422363281250, 322.5398254394531250, 341.7190246582031250, 362.0386962890625000, 383.5666503906250000, 406.3746948242187500, 430.5390014648437500, 456.1401977539062500, 483.2637023925781250, 512.0000610351562500, 542.4451293945312500, 574.7006225585937500, 608.8740844726562500, 645.0796508789062500, 683.4380493164062500, 724.0773925781250000, 767.1333007812500000, 812.7494506835937500, 861.0780029296875000, 912.2803955078125000, 966.5274047851562500, 1024.0001220703125000, 1084.8903808593750000, 1149.4012451171875000, 1217.7481689453125000, 1290.1593017578125000, 1366.8762207031250000, 1448.1549072265625000, 1534.2666015625000000, 1625.4989013671875000};

/**
 * This gives Maximilian essential information it needs
 */
class maxiSettings
{
public:
    maxiSettings();

    /*! The sample rate */
    static size_t sampleRate;
    static float one_over_sampleRate;
    static size_t channels;
    static size_t bufferSize;
    /**
     * Configure Maximilian
     * \param initSampleRate the sample rate
     * \param initChannels the number of audio channels
     * \param initBufferSize the buffer size of your audio system
     */
    static void setup(size_t initSampleRate, size_t initChannels, size_t initBufferSize)
    {
        maxiSettings::sampleRate = initSampleRate;
        maxiSettings::one_over_sampleRate = 1.f / static_cast<float>(maxiSettings::sampleRate);
        maxiSettings::channels = initChannels;
        maxiSettings::bufferSize = initBufferSize;
    }
    static size_t getSampleRate()
    {
        return maxiSettings::sampleRate;
    }
};

/**
 * \class A variety of oscillators
 */
 
class maxiOsc
{
public:

    //float frequency;
    float phase;
    //float startphase;
    //float endphase;
    float output;
    //float tri;
    float constant_by_one_over_sr_;

    maxiOsc();
    void UpdateParams(void);
    /*!Square wave oscillator
    \param frequency in Hz */
    float square(float frequency);
    /*!Sine wave oscillator
    \param frequency in Hz */
    float sinewave(float frequency);
    /*!Cosine wave oscillator
    \param frequency in Hz */
    float coswave(float frequency);
    /*!Saw wave oscillator \param frequency in Hz */
    float saw(float frequency);
    /*!A ramp rising from 0 to 1 \param frequency in Hz */
    float phasor(float frequency);
    /*!A ramp rising from 0 to 1 \param frequency in Hz 
    \param startPhase the start point of the phasor (0-1)
    \param endPhase the end point of the phasor (0-1)
    */
    float phasorBetween(float frequency, float startphase, float endphase); //renamed to avoid overrides
    /*!Triangle oscillator \param frequency in Hz */
    float triangle(float frequency);
    /*!Pulse wave oscillator \param frequency in Hz \param duty Pulse width (0-1)*/
    float pulse(float frequency, float duty);
    /*!Impulse generator \param frequency in Hz */
    float impulse(float frequency);
    /*!Fast sine wave oscillator, generated from a wavetable with linear interpolation \param frequency in Hz */
    float sinebuf(float frequency);
    /*!White noise generator*/
    float noise();
    /*!Fast sine wave oscillator, generated from a wavetable with quadratic interpolation \param frequency in Hz */
    float sinebuf4(float frequency);
    /*!Anti-aliases saw wave oscillator \param frequency in Hz */
    float sawn(float frequency);
    /*!Set the phase of the oscillator \param phaseIn The phase, from 0 to 1*/
    void phaseReset(float phaseIn);
};


/**
 * \class A delay line
 */
template<size_t DELAYTIME>
class maxiDelayline
{
public:
    maxiDelayline();

    /*! Apply a delay to a signal \param input a signal, \param size the size of the delay in samples \param feedback the amount of feedback*/
    float play(float input, size_t size, float feedback);

 protected:
    int phase;
    float memory[DELAYTIME];
};

template<size_t DELAYTIME>
maxiDelayline<DELAYTIME>::maxiDelayline() {
	memset( memory, 0, DELAYTIME * sizeof(float) );        
};

template<size_t DELAYTIME>
inline float maxiDelayline<DELAYTIME>::play(const float input, const size_t size, const float feedback)  {
	if (size >= DELAYTIME) {
		return 0;
	}
	if ( phase >= size ) {
		phase = 0;
	}
	const float output=memory[phase];
	memory[phase]=(output * feedback) + input;
	phase+=1;
	return(output);

}

/**
* A selection of filters
*/
class maxiFilter
{
private:
    float gain;
    float input;
    float output;
    float inputs[10];
    float outputs[10];
    float cutoff1;
    float x; //speed
    float y; //pos
    float z; //pole
    float c; //filter coefficient

public:
    maxiFilter();
    float cutoff;
    float resonance;
    /** A resonant low pass filter
    * \param input A signal
    * \param cutoff1 The cutoff frequency (in Hz)
    * \param resonance The amount of resonance
    */
    float lores(float input, float cutoff1, float resonance);
    /** A resonant high pass filter
    * \param input A signal
    * \param cutoff1 The cutoff frequency (in Hz)
    * \param resonance The amount of resonance
    */
    float hires(float input, float cutoff1, float resonance);

    /** A resonant band pass filter
    * \param input A signal
    * \param cutoff1 The cutoff frequency (in Hz)
    * \param resonance The amount of resonance
    */
    float bandpass(float input, float cutoff1, float resonance);

    /** A simple low pass filter
    * \param input A signal
    * \param cutoff1 The cutoff frequency (between 0 and 1)
    */
    float lopass(float input, float cutoff);

    /** A simple low pass filter
    * \param input A signal
    * \param cutoff1 The cutoff frequency (between 0 and 1)
    */
    float hipass(float input, float cutoff);

    // ------------------------------------------------
    // getters/setters

    /*!\Sets the cutoff frequency \param cut The cutoff frequency*/
    void setCutoff(float cut)
    {
        cutoff = cut;
    }

    /*!\Sets the resonance \param res The resonance*/
    void setResonance(float res)
    {
        resonance = res;
    }

    /*!\returns the cutoff value*/
    float getCutoff()
    {
        return cutoff;
    }

    /*!\returns the resonance value*/
    float getResonance()
    {
        return resonance;
    }
    // ------------------------------------------------
};


/**
 * Functions for multichannel panning 
 */
class maxiMix
{
    float input;
    float two[2];
    float four[4];
    float eight[8];

public:
    //	float x;
    //	float y;
    //	float z;

    // ------------------------------------------------
    // getters/setters

    // ------------------------------------------------

    //	float *stereo(float input,float two[2],float x);
    //	float *quad(float input,float four[4], float x,float y);
    //	float *ambisonic(float input,float eight[8],float x,float y, float z);

    // should return or just be void function
    /**
     * Stereo panning
     * \param input A mono signal
     * \param two A vector (size 2) into which the signal is panned in-place
     * \param x The panning level (0 to 1)
     */
    void stereo(float input, std::vector<float> &two, float x);

    /**
     * Quadraphonic panning
     * \param input A mono signal
     * \param four A vector (size 4) into which the signal is panned
     * \param x Left-right panning level (0 to 1)
     * \param y Forward-backward panning level (0 to 1)
     */
    void quad(float input, std::vector<float> &four, float x, float y);
    /**
     * Ambisonic panning
     * \param input A mono signal
     * \param eight A vector (size 8) into which the signal is panned
     * \param x Left-right panning level (0 to 1)
     * \param y Forward-backward panning level (0 to 1)
     * \param z Up-down panning level (0 to 1)
     */
    void ambisonic(float input, std::vector<float> &eight, float x, float y, float z);
};

// /**
//  * A ring buffer. This enables you to look at the last N values in a time series.
//  */
class maxiRingBuf {
public:
    maxiRingBuf();

    /*!Allocate memory for the buffer \param N The maximum length of the buffer*/
    void setup(size_t N) {
        buf = F64_ARRAY_CREATE(N);
        F64_ARRAY_FILL(buf,0);
    }
    /*!Add the latest value to the buffer \param x A value*/
    void push(float x) {
        buf[idx] = x;
        idx++;
        if (idx==F64_ARRAY_SIZE(buf)) {
            idx=0;
        }
    }
    
    /*! \returns The size of the buffer*/
    size_t size() {return F64_ARRAY_SIZE(buf);}

    /*! \returns the value at the front of the buffer*/
    float head() {return idx == 0 ? buf[F64_ARRAY_SIZE(buf)-1] : buf[idx-1];}

    /*! \returns the oldest value in the buffer, for a particular window size \param N The size of the window, N < the size of the buffer*/
    float tail(size_t N) {
        float val=0;
        if (idx >= N) {
            val = buf[idx-N];
        }else{
            size_t tailIdx = F64_ARRAY_SIZE(buf) - (N-idx);
            val = buf[tailIdx];
        }
        return val;
    }

    using reduceFunction = std::function<float(float, float)>;
    /**
     * Apply a function of the previous N values in the buffer
     * \param N The number of values in the window
     * \param func A function in the form float func(float previousResult, float nextValue)
     * \param initval The initial value to pass into the function (usually 0)
     * \returns The last result of the function, after passing in all values from the window
     * Example: this function will sum the values in the window: 
     *     auto sumfunc = [](float val, float n) {return val + n;};
     */
    float reduce(size_t N, reduceFunction func, float initval) {
        float val=0;
        if (idx >= N) {
            for(size_t i=idx-N; i < idx; i++) {
                val = func(val, buf[i]);
            }
        }else{
            //first chunk
            for(size_t i=F64_ARRAY_SIZE(buf)-(N-idx); i < buf.size(); i++) {
                val = func(val, buf[i]);
            }
            //second chunk
            for(int i=0; i < idx; i++) {
                val = func(val, buf[i]);
            }
        }
        return val;
    }

    
    
private:
    DOUBLEARRAY buf;
    size_t idx=0;
};


//lagging with an exponential moving average
//a lower alpha value gives a slower lag
template <class T>
class maxiLagExp
{
public:
    T alpha, alphaReciprocal;
    T val;

    maxiLagExp()
    {
        init(0.5, 0.0);
    };

    maxiLagExp(T initAlpha, T initVal)
    {
        init(initAlpha, initVal);
    }

    void init(T initAlpha, T initVal)
    {
        alpha = initAlpha;
        alphaReciprocal = 1.0 - alpha;
        val = initVal;
    }

    inline void addSample(T newVal)
    {
        val = (alpha * newVal) + (alphaReciprocal * val);
    }

    // getters/setters
    void setAlpha(T alpha_)
    {
        alpha = alpha_;
    }

    void setAlphaReciprocal(T alphaReciprocal_)
    {
        alphaReciprocal = alphaReciprocal_;
    }

    void setVal(T val_)
    {
        val = val_;
    }

    T getAlpha() const
    {
        return alpha;
    }

    T getAlphaReciprocal() const
    {
        return alphaReciprocal;
    }

    inline T value() const
    {
        return val;
    }
};

/**
 * Generator triggers (a single 1 in a stream of 0s) according to certain conditions
 */

class maxiTrigger
{
public:
    maxiTrigger();
    /*! Generate a trigger when a signal transitions from <=0 to above 0 \param input A signal*/
    float onZX(float input)
    {
        float isZX = 0.0;
        if ((previousValue <= 0.0 || firstTrigger) && input > 0)
        {
            isZX = 1.0;
        }
        previousValue = input;
        firstTrigger = 0;
        return isZX;
    }

    /*! Generate a trigger when a signal changes beyond a certain amount \param input A signal \param tolerance The amount of chance allowed before a trigger is generated*/
    float onChanged(float input, float tolerance)
    {
        float changed = 0;
        if (abs(input - previousValue) > tolerance)
        {
            changed = 1;
        }
        previousValue = input;
        return changed;
    }

private:
    float previousValue = 1;
    bool firstTrigger = 1;
};


/**
 * A sampler
 */
class maxiSample
{

private:
    float position, recordPosition;
    float speed;
    float output;
    maxiLagExp<float> loopRecordLag;
    // DualModeF64Array test;

public:
    short myChannels;
    int mySampleRate;
    /*!\returns The length of the sample, in samples*/
    inline size_t getLength() { return F64_ARRAY_SIZE(amplitudes); };
    short myBitsPerSample;
    maxiTrigger zxTrig;


    DECLARE_F64_ARRAY(amplitudes);

    maxiSample();

    /*! Use the = operator to copy from one sample to another \param source another maxiSample instance*/
    maxiSample &operator=(const maxiSample &source)
    {
        if (this == &source)
            return *this;
        position = 0;
        recordPosition = 0;
        myChannels = source.myChannels;
        mySampleRate = maxiSettings::sampleRate;
        F64_ARRAY_SETFROM(amplitudes,source.amplitudes);
        return *this;
    }

    string myPath;
    int myChunkSize;
    int mySubChunk1Size;
    int readChannel;
    short myFormat;
    int myByteRate;
    short myBlockAlign;


//     /*! \returns a printable summary of the wav file */
    // string getSummary();

    // -------------------------

    /*! Check if the sample is loaded \returns true if the sample is ready to play*/
    bool isReady() {return F64_ARRAY_SIZE(amplitudes) > 1;}

    /*! Set the sample from an external array \param _sampleData An float array (JS) or vector (C++) of data*/
    void setSample(DOUBLEARRAY_REF _sampleData)
    {
        // NORMALISE_ARRAY_TYPE(_sampleData, sampleData)
        // amplitudes = sampleData;
        F64_ARRAY_SETFROM(amplitudes, _sampleData);
        // amplitudes.setFrom(_sampleData);
        mySampleRate = 44100;
        position = F64_ARRAY_SIZE(amplitudes) - 1;
    }

    /*! Set the sample from an external array, and set the sample rate 
     * \param _sampleData An float array (JS) or vector (C++) of data
     * \param sampleRate the sample rate
     */
    void setSampleAndRate(DOUBLEARRAY_REF _sampleData, int sampleRate)
    {
        setSample(_sampleData);
        mySampleRate = sampleRate;
    }

    /*! Clear the sample data*/
    void clear() { F64_ARRAY_CLEAR(amplitudes) }
    // // -------------------------


    /*! Trigger the sample from the start*/
    void trigger();

    /**
     * Record into the sample buffer
     * \param newSample a signal
     * \param recordEnabled set to true to record into the loop
     * \param recordMix the balance between existing sample and the new signal (1 = overdub, 0=no recording, 0.5=equal mix)
     * \param start the loop start point (0-1)
     * \param end the loop end point (0-1)
     */
    void loopRecord(float newSample, const bool recordEnabled, const float recordMix, float start, float end)
    {
        loopRecordLag.addSample(recordEnabled);
        if (recordPosition < start * F64_ARRAY_SIZE(amplitudes))
            recordPosition = start * F64_ARRAY_SIZE(amplitudes);
        if (recordEnabled)
        {
            float currentSample = F64_ARRAY_AT(amplitudes,(int)recordPosition) / 32767.0;
            newSample = (recordMix * currentSample) + ((1.0 - recordMix) * newSample);
            newSample *= loopRecordLag.value();
            amplitudes[(unsigned long)recordPosition] = newSample * 32767;
        }
        ++recordPosition;
        if (recordPosition >= end * F64_ARRAY_SIZE(amplitudes))
            recordPosition = start * F64_ARRAY_SIZE(amplitudes);
    }

    /*! Reset the sample to play from the start*/
    void reset() {position=0;}

    /*! Play the sample, with no modification */
    float play();
    float play(float speed);


    /*! Play the sample, providing a phasor to control the position \param pha a phasor signal (from 0 to 1)*/
    float playWithPhasor(float pha);
    bool phasorFirst=1;
    float phasorPrev=0;

    /*! Play the sample in a loop \param start start position (0-1) \param end end position (0-1)*/
    float playLoop(float start, float end); // start and end are between 0.0 and 1.0

    /*! Play the sample once and stop*/
    float playOnce();

    /*! Play the sample when a trigger is received \param trigger a signal*/
    float playOnZX(float trigger);

    /*! Play the sample when a trigger is received, at a modified speed \param trigger a signal \param speed the speed multiplier (1=no change, 2=float etc)*/
    float playOnZXAtSpeed(float trig, float speed); //API CHANGE
    /*! Play the sample when a trigger is received, at a modified speed from a specific position \param trigger a signal \param speed the speed multiplier (1=no change, 2=float etc) \param offset the start position (0-1)*/
    float playOnZXAtSpeedFromOffset(float trig, float speed, float offset); //API CHANGE
    /*! Play the sample when a trigger is received, at a modified speed beween two positions \param trigger a signal \param speed the speed multiplier (1=no change, 2=float etc) \param offset the start position (0-1) \param length the length of the segment (0-1)*/
    float playOnZXAtSpeedBetweenPoints(float trig, float speed, float offset, float length); //API CHANGE

    /*! Loop the sample, and set the playback position to a specific point when a trigger is received \param trigger a signal \param position the position to move to when a trigger is recevied*/
    float loopSetPosOnZX(float trigger, float position); // position between 0 and 1.0

    /*! Play the samples once, at a modified speed \param speed a speed multiplier*/
    float playOnceAtSpeed(float speed); //API CHANGE

    /*! Set the playback position \param newPos (0-1)*/
    void setPosition(float newPos); // between 0.0 and 1.0

    /*! Play from the start to a specific position \param end the end point (0-1)*/
    float playUntil(float end);
    /*! Play from the start to a specific position, at a modified speed \param end the end point (0-1) \param speed a speed multiplier*/    
    float playUntilAtSpeed(float end, float speed);

    /*! Play at a modified speed \param speed a speed multiplier*/
    float playAtSpeed(float speed); //API CHANGE


    /*! Play at a modified speed between two points, from a position \param frequency the playback pitch (in Hz) \param start the start point (in samples) \param end the end point (in samples) \param pos the starting position (in samples)*/
    float playAtSpeedBetweenPointsFromPos(float frequency, float start, float end, float pos); //API CHANGE

    /*! Play at a modified speed between two points \param frequency the playback pitch (in Hz) \param start the start point (in samples) \param end the end point (in samples)*/
    float playAtSpeedBetweenPoints(float frequency, float start, float end); //API CHANGE

    /*! Play at a modified speed between two points, using quadratic interpolation \param frequency the playback pitch (in Hz) \param start the start point (in samples) \param end the end point (in samples)*/
    float play4(float frequency, float start, float end);


    /*! Normalise the sample buffer \param maxLevel the maximum absolute level*/
    void normalise(float maxLevel);                                            

    /*! Trim the sample buffer to remove silence from the ends \param alpha the sensitivity \param threshold the value above which to start trimming \param trimStart true if the start should be trimmed \param trimEnd true if the end should be trimmed */
    void autoTrim(float alpha, float threshold, bool trimStart, bool trimEnd); //alpha of lag filter (lower == slower reaction), threshold to mark start and end, < 32767
};

/**
 * Various mapping functions
 */
class maxiMap
{
public:
    maxiMap();

    /** Map from one range to another, linearly
     * \param val a signal
     * \param inMin the lowest expected value of the signal
     * \param inMax the highest expected value of the signal
     * \param outMin the lowest value in the new range of the signal
     * \param outMax the highest value in the new range of the signal
     * \returns a signal
     */
    static float inline linlin(float val, float inMin, float inMax, float outMin, float outMax)
    {
        val = max(min(val, inMax), inMin);
        return ((val - inMin) / (inMax - inMin) * (outMax - outMin)) + outMin;
    }

    /** Map from one range to another, converting from linear to expontial - useful from mapping control values to frequencies etc
     * \param val a signal
     * \param inMin the lowest expected value of the signal
     * \param inMax the highest expected value of the signal
     * \param outMin the lowest value in the new range of the signal
     * \param outMax the highest value in the new range of the signal
     * \returns a signal
     */
    static float inline linexp(float val, float inMin, float inMax, float outMin, float outMax)
    {
        //clipping
        val = max(min(val, inMax), inMin);
        return pow((outMax / outMin), (val - inMin) / (inMax - inMin)) * outMin;
    }

    /** Map from one range to another, converting from an exponential value to a linear one
     * \param val a signal
     * \param inMin the lowest expected value of the signal
     * \param inMax the highest expected value of the signal
     * \param outMin the lowest value in the new range of the signal
     * \param outMax the highest value in the new range of the signal
     * \returns a signal
     */
    static float inline explin(float val, float inMin, float inMax, float outMin, float outMax)
    {
        //clipping
        val = max(min(val, inMax), inMin);
        return (log(val / inMin) / log(inMax / inMin) * (outMax - outMin)) + outMin;
    }

    /** Restrict a signal to upper and lower bounds 
     * \param v a signal
     * \param low the lowest value
     * \param high the highest value
     * \returns a signal
     */
    static float inline clamp(float v, const float low, const float high)
    {
        if (v > high)
        {
            v = high;
        }
        else if (v < low)
        {
            v = low;
        }
        return v;
    }
};

// /* The class is deprecated, use maxiDynamics instead */
// class maxiDyn
// {

// public:
//     //	float gate(float input, float threshold=0.9, long holdtime=1, float attack=1, float release=0.9995);
//     //	float compressor(float input, float ratio, float threshold=0.9, float attack=1, float release=0.9995);
//     float gate(float input, float threshold = 0.9, long holdtime = 1, float attack = 1, float release = 0.9995);
//     float compressor(float input, float ratio, float threshold = 0.9, float attack = 1, float release = 0.9995);
//     float compress(float input);

//     float input;
//     float ratio;
//     float currentRatio;
//     float threshold;
//     float output;
//     float attack;
//     float release;
//     float amplitude;

//     void setAttack(float attackMS);
//     void setRelease(float releaseMS);
//     void setThreshold(float thresholdI);
//     void setRatio(float ratioF);
//     long holdtime;
//     long holdcount;
//     int attackphase, holdphase, releasephase;
// };

// /* The class is deprecated, use maxiEnvGen instead */

// class maxiEnv
// {

// public:
//     float ar(float input, float attack = 1, float release = 0.9, long holdtime = 1, int trigger = 0);
//     float adsr(float input, float attack = 1, float decay = 0.99, float sustain = 0.125, float release = 0.9, long holdtime = 1, int trigger = 0);
//     float adsr(float input, int trigger);
//     float input;
//     float output;
//     float attack;
//     float decay;
//     float sustain;
//     float release;
//     float amplitude;

//     void setRelease(float releaseMS);
//     void setDecay(float decayMS);

//     //old method - not actually in MS
//     void setAttack(float attackMS);
//     //new methods: these are in MS
//     void setAttackMS(float attackMS);

//     void setSustain(float sustainL);

//     int trigger;

//     long holdtime = 1;
//     long holdcount;
//     int attackphase, decayphase, sustainphase, holdphase, releasephase;

//     // ------------------------------------------------
//     // getters/setters
//     int getTrigger() const
//     {
//         return trigger;
//     }

//     void setTrigger(int trigger)
//     {
//         this->trigger = trigger;
//     }

//     // ------------------------------------------------
// };

/**
Conversion functions
*/
class maxiConvert
{
public:
    /*!Convert from MIDI note number to frequency (Hz) \param midinote A MIDI note number*/
    static float mtof(int midinote);

    /*!Convert from milliseconds to samples \param timeMs The number of milliseconds*/
    static size_t msToSamps(float timeMs)
    {
        return static_cast<size_t>(timeMs / 1000.0 * maxiSettings::sampleRate);
    }
    /*!Convert from samples to milliseconds \param samples The number of samples*/
    static float sampsToMs(size_t samples)
    {
        return samples / maxiSettings::sampleRate * 1000.0;
    }

    /*!Convert from amplitude to decibels \param amp Amplitude*/
    static float ampToDbs(float amp) {
        return std::log10(amp) * 20.0;
    }
    /*!Convert from decibels to amplitude \param dbs Decibels*/
    static float dbsToAmp(float dbs) {
        return std::pow(10.0, dbs * 0.05);
    }
};

using convert = maxiConvert;

/**
 * Sample and hold effect:  samples the input signal periodically and outputs that value
 */
class maxiSampleAndHold
{
public:
    /*!Process a signal \param sigIn a signal \param holdTimeMs The length of the sampling period \returns the sampled signal*/
    inline float sah(float sigIn, float holdTimeMs)
    {
        float holdTimeSamples = convert::msToSamps(holdTimeMs);

        if (phase >= holdTimeSamples)
        {
            phase -= holdTimeSamples;
        }
        if (phase < 1.0)
            holdValue = sigIn;
        phase++;
        return holdValue;
    }

private:
    float phase = 0;
    float holdValue = 0;
    bool firstRun = 1;
};

/**
 * Detect positive zero-crossings: the point in time when a signal transitions from 0 or less, to above zero
 */
class maxiZeroCrossingDetector
{
public:
    maxiZeroCrossingDetector();

    /*! Returns true when a positive zero-crossing occurs, othersize false \param x A signal */
    inline bool zx(float x)
    {
        bool res = 0;
        if (previous_x <= 0 && x > 0)
        {
            res = 1;
        }
        previous_x = x;
        return res;
    }

private:
    float previous_x = 0;
};

/**
 * Calculate the zero crossing rate of a signal
 * This is a fairly crude measure of frequency, which is confounded by complex waveforms and noise
 */
class maxiZeroCrossingRate {
    public:
        maxiZeroCrossingRate();
        /*!Calculate the zero cross rate \param signal a signal \returns the zero crossing rate in Hz*/
        float play(float signal) {
            if (zxd.zx(signal)) {
                buf.push(1);
                runningCount++;
            }else{
                buf.push(0);
            }
            runningCount -= buf.tail(maxiSettings::sampleRate);
            return runningCount;
        }

    private:
        maxiRingBuf buf;
        size_t runningCount=0;
        maxiZeroCrossingDetector zxd;
};

// //needs oversampling

class maxiNonlinearity
{
public:
    maxiNonlinearity(){};
    /** atan distortion, see http://www.musicdsp.org/showArchiveComment.php?ArchiveID=104
    * \param in A signal
    * \param shape from 1 (soft clipping) to infinity (hard clipping)
    */ 
    float atanDist(const float in, const float shape);
    /** Faster but 'lower quality' version of atan distortion
     * \param in A signal
     * \param shape from 1 (soft clipping) to infinity (hard clipping)
    */
    float fastAtanDist(const float in, const float shape);
    /** Cliping with nicer harmonics \param x A signal*/
    float softclip(float x);
    /** Cliping with nastier harmonics \param x A signal*/
    float hardclip(float x);
    /**
     * asymmetric clipping: chose the shape of curves for both positive and negative values of x
     * try it here https://www.desmos.com/calculator/to6eixatsa
     * \param x A signal
     * \param a Exponent for the positive curve
     * \param b Exponent for the negative curve
     */
    float asymclip(float x, float a, float b);
    /*! Fast atan distortion \param x A signal */
    float fastatan(float x);
};

inline float maxiNonlinearity::asymclip(float x, float a, float b)
{

    if (x >= 1)
    {
        x = 1;
    }
    else if (x <= -1)
    {
        x = -1;
    }
    else if (x < 0)
    {
        x = -(std::powf(-x, a));
    }
    else
    {
        x = std::powf(x, b);
    }
    return x;
}

inline float maxiNonlinearity::hardclip(float x)
{
    x = x >= 1 ? 1 : (x <= -1 ? -1 : x);
    return x;
}

inline float maxiNonlinearity::softclip(float x)
{
    if (x >= 1.f)
    {
        x = 1.f;
    }
    else if (x <= -1.f)
    {
        x = -1.f;
    }
    else
    {
        x = (2.f / 3.0f) * (x - std::powf(x, 3.f) / 3.0f);
    }
    return x;
}

inline float maxiNonlinearity::fastatan(float x)
{
    return (x / (1.0f + 0.28f * (x * x)));
}

inline float maxiNonlinearity::atanDist(const float in, const float shape)
{
    float out;
    out = (1.0f / atan(shape)) * atan(in * shape);
    return out;
}

inline float maxiNonlinearity::fastAtanDist(const float in, const float shape)
{
    float out;
    out = (1.0f / fastatan(shape)) * fastatan(in * shape);
    return out;
}

/**
 * A flanger effect
 */
template<size_t DELAYTIME>
class maxiFlanger
{
public:

    maxiFlanger();
    //delay = delay time - ~800 sounds good
    //feedback = 0 - 1
    //speed = lfo speed in Hz, 0.0001 - 10 sounds good
    //depth = 0 - 1
    /**
     * Apply a flanger effect to a signal
     * \param input a signal
     * \param delay the amount of delay (in samples)
     * \param feedback the amount of feedback, 0-1
     * \param speed the speed of the flanger LFO, in Hz
     * \param depth the depth of the LFO, 0-1
     * \returns the input signal, flanged
     */

    float flange(const float input, const unsigned int delay, const float feedback, const float speed, const float depth);
    maxiDelayline<DELAYTIME> dl;
    maxiOsc lfo;
};


template<size_t DELAYTIME>
maxiFlanger<DELAYTIME>::maxiFlanger() {
}

template<size_t DELAYTIME>
inline float maxiFlanger<DELAYTIME>::flange(const float input, const unsigned int delay, const float feedback, const float speed, const float depth)
{
    float output;
    float lfoVal = lfo.triangle(speed);
    output = dl.play(input, delay + (lfoVal * depth * delay) + 1.f, feedback);
    return (output + input) / 2.0f;
}

/**
 * A chorus effect
 */ 

template<size_t DELAYTIME>
class maxiChorus
{
public:
    //delay = delay time - ~800 sounds good
    //feedback = 0 - 1
    //speed = lfo speed in Hz, 0.0001 - 10 sounds good
    //depth = 0 - 1
    /**
     * Apply a chorus effect to a signal
     * \param input a signal
     * \param delay the amount of delay (in milliseconds, recommended 1-1000)
     * \param feedback the amount of feedback, 0-1
     * \param speed the speed of the chorus, in Hz
     * \param depth the depth of the chorus effect, 0-1
     * \returns the input signal with chorus applied
     */
    float chorus(const float input, const unsigned int delay, const float feedback, const float speed, const float depth);
private:
    maxiDelayline<DELAYTIME> dl, dl2;
    maxiOsc lfo;
    maxiFilter lopass;
};


template<size_t DELAYTIME>
inline float maxiChorus<DELAYTIME>::chorus(const float input, const unsigned int delay, const float feedback, const float speed, const float depth)
{
    float output1, output2;
    float lfoVal = lfo.noise();
    lfoVal = lopass.lores(lfoVal, speed, 1.0) * 2.0;
    output1 = dl.dl(input, delay + (lfoVal * depth * delay) + 1, feedback);
    output2 = dl2.dl(input, (delay + (lfoVal * depth * delay * 1.02) + 1) * 0.98, feedback * 0.99);
    output1 *= (1.0 - fabs(output1));
    output2 *= (1.0 - fabs(output2));
    return (output1 + output2 + input) / 3.0;
}

template <typename T>
class maxiEnvelopeFollowerType
{
public:
    maxiEnvelopeFollowerType()
    {
        setAttack(100);
        setRelease(100);
        env = 0;
    }
    void setAttack(T attackMS)
    {
        attack = pow(0.01, 1.0 / (attackMS * maxiSettings::sampleRate * 0.001));
    }
    void setRelease(T releaseMS)
    {
        release = pow(0.01, 1.0 / (releaseMS * maxiSettings::sampleRate * 0.001));
    }
    inline T play(T input)
    {
        input = fabs(input);
        if (input > env)
            env = attack * (env - input) + input;
        else
            env = release * (env - input) + input;
        return env;
    }
    void reset() { env = 0; }
    inline T getEnv() { return env; }
    inline void setEnv(T val) { env = val; }

private:
    T attack, release, env;
};

typedef maxiEnvelopeFollowerType<double> maxiEnvelopeFollower;
typedef maxiEnvelopeFollowerType<float> maxiEnvelopeFollowerF;

/**
 * A simple high pass filter to block DC
 */
class maxiDCBlocker
{
public:
    float xm1, ym1;
    maxiDCBlocker() : xm1(0), ym1(0) {}
    /*! Remove DC from a signal \param input a signal \param R the sensitivity (0-1) */
    inline float play(float input, float R)
    {
        ym1 = input - xm1 + R * ym1;
        xm1 = input;
        return ym1;
    }   
};

/**
 State Variable Filter

 algorithm from  http://www.cytomic.com/files/dsp/SvfLinearTrapOptimised.pdf
 usage:

 filter.setCutoff(param1);
 filter.setResonance(param2);

 w = filter.play(w, 0.0, 1.0, 0.0, 0.0);

 */
class maxiSVF
{
public:
    maxiSVF() : v0z(0), v1(0), v2(0) { setParams(1000, 1); }

    /*!Set the cutoff frequency \param cutoff Cuttoff frequency (20 < cutoff < 20000)*/
    inline void setCutoff(float cutoff)
    {
        setParams(cutoff, res);
    }

    /*! Set the resonance of the filter \param q From 0 upwards, starts to ring from 2-3ish, cracks a bit around 10*/
    inline void setResonance(float q)
    {
        setParams(freq, q);
    }

    /**run the filter, and get a mixture of lowpass, bandpass, highpass and notch outputs
     *\param w The signal to be filtered
     \param lpmix the amount of low pass filtering (0-1) 
     \param bpmix the amount of bandpass pass filtering (0-1) 
     \param hpmix the amount of high pass filtering (0-1) 
     \param notchmix the amount of notch filtering (0-1) 
    */
    inline float play(float w, float lpmix, float bpmix, float hpmix, float notchmix)
    {
        float low, band, high, notch;
        float v1z = v1;
        float v2z = v2;
        float v3 = w + v0z - 2.0 * v2z;
        v1 += g1 * v3 - g2 * v1z;
        v2 += g3 * v3 + g4 * v1z;
        v0z = w;
        low = v2;
        band = v1;
        high = w - k * v1 - v2;
        notch = w - k * v1;
        return (low * lpmix) + (band * bpmix) + (high * hpmix) + (notch * notchmix);
    }

private:
    inline void setParams(float _freq, float _res)
    {
        freq = _freq;
        res = _res;
        g = tanf(PI * freq * maxiSettings::one_over_sampleRate);
        damping = res == 0 ? 0 : 1.0 / res;
        k = damping;
        ginv = g / (1.0 + g * (g + k));
        g1 = ginv;
        g2 = 2.0 * (g + k) * ginv;
        g3 = g * ginv;
        g4 = 2.0 * ginv;
    }

    float v0z, v1, v2, g, damping, k, ginv, g1, g2, g3, g4;
    float freq, res;
};

// /** Biquad filters
//  * based on http://www.earlevel.com/main/2011/01/02/biquad-formulas/ and https://ccrma.stanford.edu/~jos/fp/Direct_Form_II.html
//  */
class maxiBiquad
{
public:
    maxiBiquad() {};
    /*! A variety of filter types*/
    enum filterTypes
    {
        LOWPASS,
        HIGHPASS,
        BANDPASS,
        NOTCH,
        PEAK,
        LOWSHELF,
        HIGHSHELF
    };

    /*! Process a signal through the filter \param input A signal*/
    inline float play(float input)
    {
        v[0] = input - (b1 * v[1]) - (b2 * v[2]);
        float y = (a0 * v[0]) + (a1 * v[1]) + (a2 * v[2]);
        v[2] = v[1];
        v[1] = v[0];
        return y;
    }

    /** Configure the filter
     * \param filtType  The type of filter, set from maxiBiquad::filterTypes
     * \param cutoff The filter cutoff frequency in Hz
     * \param Q The resonance of the filter
     * \param peakGain The gain of the filter (only used for PEAK, HIGHSHELF and LOWSHELF)
     */
    inline void set(filterTypes filtType, float cutoff, float Q, float peakGain)
    {
        float norm = 0;
        float V = powf(10.0, abs(peakGain) / 20.0);
        float K = tanf(PI * cutoff / maxiSettings::sampleRate);
        switch (filtType)
        {
        case LOWPASS:
            norm = 1.0 / (1.0 + K / Q + K * K);
            a0 = K * K * norm;
            a1 = 2.0 * a0;
            a2 = a0;
            b1 = 2.0 * (K * K - 1.0) * norm;
            b2 = (1.0 - K / Q + K * K) * norm;
            break;

        case HIGHPASS:
            norm = 1. / (1. + K / Q + K * K);
            a0 = 1 * norm;
            a1 = -2 * a0;
            a2 = a0;
            b1 = 2 * (K * K - 1) * norm;
            b2 = (1 - K / Q + K * K) * norm;
            break;

        case BANDPASS:
            norm = 1. / (1. + K / Q + K * K);
            a0 = K / Q * norm;
            a1 = 0.;
            a2 = -a0;
            b1 = 2. * (K * K - 1.) * norm;
            b2 = (1. - K / Q + K * K) * norm;
            break;

        case NOTCH:
            norm = 1. / (1. + K / Q + K * K);
            a0 = (1. + K * K) * norm;
            a1 = 2. * (K * K - 1.) * norm;
            a2 = a0;
            b1 = a1;
            b2 = (1. - K / Q + K * K) * norm;
            break;

        case PEAK:
            if (peakGain >= 0.0)
            { // boost
                norm = 1. / (1. + 1. / Q * K + K * K);
                a0 = (1. + V / Q * K + K * K) * norm;
                a1 = 2. * (K * K - 1.) * norm;
                a2 = (1. - V / Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1. - 1. / Q * K + K * K) * norm;
            }
            else
            { // cut
                norm = 1. / (1. + V / Q * K + K * K);
                a0 = (1. + 1 / Q * K + K * K) * norm;
                a1 = 2. * (K * K - 1) * norm;
                a2 = (1. - 1. / Q * K + K * K) * norm;
                b1 = a1;
                b2 = (1. - V / Q * K + K * K) * norm;
            }
            break;
        case LOWSHELF:
            if (peakGain >= 0.)
            { // boost
                norm = 1. / (1. + SQRT2 * K + K * K);
                a0 = (1. + sqrt(2. * V) * K + V * K * K) * norm;
                a1 = 2. * (V * K * K - 1.) * norm;
                a2 = (1. - sqrt(2. * V) * K + V * K * K) * norm;
                b1 = 2. * (K * K - 1.) * norm;
                b2 = (1. - SQRT2 * K + K * K) * norm;
            }
            else
            { // cut
                norm = 1. / (1. + sqrt(2. * V) * K + V * K * K);
                a0 = (1. + SQRT2 * K + K * K) * norm;
                a1 = 2. * (K * K - 1.) * norm;
                a2 = (1. - SQRT2 * K + K * K) * norm;
                b1 = 2. * (V * K * K - 1.) * norm;
                b2 = (1. - sqrt(2. * V) * K + V * K * K) * norm;
            }
            break;
        case HIGHSHELF:
            if (peakGain >= 0.)
            { // boost
                norm = 1. / (1. + SQRT2 * K + K * K);
                a0 = (V + sqrt(2. * V) * K + K * K) * norm;
                a1 = 2. * (K * K - V) * norm;
                a2 = (V - sqrt(2. * V) * K + K * K) * norm;
                b1 = 2. * (K * K - 1) * norm;
                b2 = (1. - SQRT2 * K + K * K) * norm;
            }
            else
            { // cut
                norm = 1. / (V + sqrt(2. * V) * K + K * K);
                a0 = (1. + SQRT2 * K + K * K) * norm;
                a1 = 2. * (K * K - 1.) * norm;
                a2 = (1. - SQRT2 * K + K * K) * norm;
                b1 = 2. * (K * K - V) * norm;
                b2 = (V - sqrt(2. * V) * K + K * K) * norm;
            }
            break;
        }
    }

private:
    float a0 = 0, a1 = 0, a2 = 0, b1 = 0, b2 = 0;
    // filterTypes filterType;
    const float SQRT2 = sqrt(2.0);
    float v[3] = {0, 0, 0};
};

/**
 * Cross-fade between two signals, using equal-power panning
 */
class maxiXFade
{
public:
    maxiXFade() {}

    /**
     * Cross-fade between stereo signals
     * \param ch1 a vector containing left and right components of channel 1 
     * \param ch2 a vector containing left and right components of channel 2 
     * \param xfader the cross-fader position, -1=100% ch1, 1=100% ch2, 0=an equal mix of both channels
     */
    static vector<float> xfade(vector<float> &ch1, vector<float> &ch2, float xfader)
    {
        xfader = maxiMap::clamp(xfader, -1, 1);
        float xfNorm = maxiMap::linlin(xfader, -1, 1, 0, 1);
        float gainCh1 = sqrt(1.0 - xfNorm);
        float gainCh2 = sqrt(xfNorm);
        vector<float> output(ch1.size(), 0.0);
        for (size_t i = 0; i < output.size(); i++)
        {
            output[i] = (ch1[i] * gainCh1) + (ch2[i] * gainCh2);
        }
        return output;
    }
    /**
     * Cross-fade between mono signals
     * \param ch1 the signal for channel 1
     * \param ch2 the signal for channel 2 
     * \param xfader the cross-fader position, -1=100% ch1, 1=100% ch2, 0=an equal mix of both channels
     */
    static float xfade(float ch1, float ch2, float xfader)
    {
        vector<float> vch1 = {ch1};
        vector<float> vch2 = {ch2};
        return maxiXFade::xfade(vch1, vch2, xfader)[0];
    }
};

/**
 * A line generator (you can also use maxiEnvGen)
 */
class maxiLine
{
public:
    maxiLine() {}
    /*! Generate a line, when a trigger is received \param trigger a signal*/
    inline float play(float trigger)
    {
        if (!lineComplete)
        {
            if (trigEnable && !triggered)
            {
                triggered = (trigger > 0.0 && lastTrigVal <= 0.0);
                lineValue = lineStart;
            }
            if (triggered)
            {
                lineValue += inc;
                if (inc <= 0)
                {
                    lineComplete = lineValue <= lineEnd;
                }
                else
                {
                    lineComplete = lineValue >= lineEnd;
                }
                if (lineComplete)
                {
                    if (!oneShot)
                    {
                        reset();
                    }
                }
            }
            lastTrigVal = trigger;
        }
        return lineValue;
    }

    /** Setup the line before it is triggered 
     * \param start the starting value of the line
     * \param end the ending value of the line
     * \param durationMs the duration of the line (in milliseconds)
     * \param isOneShot true is the line should not play once, or false if it should loop
     * \returns a signal
     */

    inline void prepare(float start, float end, float durationMs, bool isOneShot)
    {
        lineValue = lineStart;
        lineStart = start;
        lineEnd = end;
        float lineMag = end - start;
        float durInSamples = durationMs / 1000.0 * maxiSettings::sampleRate;
        inc = lineMag / durInSamples;
        oneShot = isOneShot;
        reset();
    }

    /*! \param 0 or less to protect the generator from triggering, more than 0 to enable it to be triggered*/
    inline void triggerEnable(float on)
    {
        trigEnable = on > 0.0;
    }
    /*! \returns true if the line generator has finished*/
    inline bool isLineComplete()
    {
        return lineComplete;
    }

private:
    float phase = 0;
    float lineValue = 0;
    float inc = 0;
    float lastTrigVal = -1;
    float trigEnable = false;
    float triggered = false;
    bool lineComplete = false;
    float lineStart = 0;
    float lineEnd = 0;
    bool oneShot = 1;
    void reset()
    {
        triggered = false;
        lineComplete = false;
    }
};

// /**
//  * A kuramoto oscillator
//  * 
//  * This is an adaptive oscillator that adjusts its own phase in relation to the phases of other kuramoto oscillators
//  * 
//  * For further info, see
//  * https://tutorials.siam.org/dsweb/cotutorial/index.php?s=3&p=0
//  * https://www.complexity-explorables.org/explorables/ride-my-kuramotocycle/
//  */
// class maxiKuramotoOscillator
// {
// public:
//     maxiKuramotoOscillator() {}

//     /**
//      * Run the oscillator
//      * \param freq the intended frequency of the oscillator
//      * \param K the strengh of coupling between oscillators
//      * \param phases a vector of the phases of other kuramoto oscillators
//      * \returns the current amplitude of the oscillator
//      */
//     inline float play(float freq, float K, std::vector<float> phases)
//     {

//         float phaseAdj = 0;
//         for (float v : phases)
//         {
//             phaseAdj += sin(v - phase);
//         }
//         phase += dt * (freq + ((K / phases.size()) * phaseAdj));
//         if (phase >= TWOPI)
//             phase -= TWOPI;
//         else if (phase < 0)
//             phase += TWOPI;
//         return phase;
//     }
//     /*! Set the phase of the oscillator \param newPhase the phase, 0 - 2PI*/
//     inline void setPhase(float newPhase) { phase = newPhase; }
//     /*! \returns the current phase of the oscillator*/
//     inline float getPhase() { return phase; }

// private:
//     float phase = 0.0;
//     float dt = TWOPI / maxiSettings::sampleRate;
// };

// /**
//  * This class managed a group of Kuramoto oscillators (see maxiKuramotoOscillator)
//  */
// class maxiKuramotoOscillatorSet
// {
// public:
//     /**
//      * \param N The number of oscillators in the group
//      */
//     maxiKuramotoOscillatorSet(const size_t N)
//     {
//         oscs.resize(N);
//         phases.resize(N);
//     };
//     /*! Set the phases of all of the oscillators \param phases a vector of phases (all 0 - 2PI)*/
//     void setPhases(const std::vector<float> &phases)
//     {
//         size_t iOsc = 0;
//         for (float v : phases)
//         {
//             oscs[iOsc].setPhase(v);
//             iOsc++;
//         }
//     }

//     /*! Set the phase of a single oscillator \param phase the phase, 0 - 2PI \param oscillatorIdx the index of the oscillator*/
//     void setPhase(const float phase, const size_t oscillatorIdx)
//     {
//         oscs[oscillatorIdx].setPhase(phase);
//     }

//     /*! Get the phase of a single oscillator \param i the index of the oscillator \returns the oscillator's phase*/
//     float getPhase(size_t i)
//     {
//         return oscs[i].getPhase();
//     }

//     /*! \returns the number of oscillators in the group*/
//     size_t size()
//     {
//         return oscs.size();
//     }

//     /**
//      * Run all of the oscillators
//      * \param freq the intended frequency
//      * \param K the coupling strength between oscillators
//      * \returns a mix of all of the oscillator signals in the group
//      */
//     float play(float freq, float K)
//     {
//         float mix = 0.0;
//         //gather phases
//         for (size_t i = 0; i < phases.size(); i++)
//         {
//             phases[i] = oscs[i].getPhase();
//         }
//         for (auto &v : oscs)
//         {
//             mix += v.play(freq, K, phases);
//         }
//         return mix / phases.size();
//     }

// protected:
//     std::vector<maxiKuramotoOscillator> oscs;
//     std::vector<float> phases;
// };


// /**
//  * Run a group of kuramoto oscillators with asynchronous updates.  Instead of setting all of the phasors at once, you can set the phases or arbitrary individuals at abritrary times. This class updates the local oscillator according to best guesses of the phase of remote oscillators.
//  * This is useful if the other oscillators are not running on your computer, but are linked on a network. For example you could use this class for a shared network clock that is robust to timing jitter.
//  * 
//  */
// class maxiAsyncKuramotoOscillator : public maxiKuramotoOscillatorSet
// {
// public:
//     /*! \param N the number of oscillators in the group (including this one)*/

//     maxiAsyncKuramotoOscillator(const size_t N) : maxiKuramotoOscillatorSet(N){};

//     /*! Set the phase of a single oscillator, probably in response to receiving this information over your network
//      * \param phase the phase, 0 - 2PI \param oscillatorIdx the index of the oscillator
//      */
//     void setPhase(const float phase, const size_t oscillatorIdx)
//     {
//         oscs[oscillatorIdx].setPhase(phase);
//         update = 1;
//     }

//     /*! Set the phases of all of the oscillators \param phases a vector of phases (all 0 - 2PI)*/   
//     void setPhases(const std::vector<float> &phases)
//     {
//         size_t iOsc = 0;
//         for (float v : phases)
//         {
//             oscs[iOsc].setPhase(v);
//             iOsc++;
//         }
//         update = 1;
//     }


//     /**
//      * Run all of the oscillators
//      * \param freq the intended frequency
//      * \param K the coupling strength between oscillators
//      * \returns a mix of all of the oscillator signals in the group
//      */
//     float play(float freq, float K)
//     {
//         float mix = 0.0;
//         //gather phases
//         if (update)
//         {
//             for (size_t i = 0; i < phases.size(); i++)
//             {
//                 phases[i] = oscs[i].getPhase();
//             }
//         }
//         for (auto &v : oscs)
//         {
//             mix += v.play(freq, update ? K : 0, phases);
//         }
//         update = 0;
//         return mix / phases.size();
//     }

//     /*! Get the phase of a single oscillator \param i the index of the oscillator \returns the oscillator's phase*/
//     float getPhase(size_t i)
//     {
//         return maxiKuramotoOscillatorSet::getPhase(i);
//     }

//     /*! \returns the number of oscillators in the group*/
//     size_t size()
//     {
//         return maxiKuramotoOscillatorSet::size();
//     }

// private:
//     bool update = 0;
// };

// class maxiBits
// {
// public:
//     typedef uint32_t bitsig;

//     // static bitsig sig(bitsig v) return v;
//     // maxiBits() {}
//     // maxiBits(const bitsig v) : t(v) {}

//     static bitsig sig(bitsig v) { return v; }

//     static bitsig at(const bitsig v, const bitsig idx)
//     {
//         return 1 & (v >> idx);
//     }
//     static bitsig shl(const bitsig v, const bitsig shift)
//     {
//         return v << shift;
//     }
//     static bitsig shr(const bitsig v, const bitsig shift)
//     {
//         return v >> shift;
//     }
//     static bitsig r(const bitsig v, const bitsig offset, const bitsig width)
//     {
//         bitsig mask = maxiBits::l(width);
//         bitsig shift = offset - width + 1;
//         bitsig x = 0;
//         x = v & shl(mask, shift);
//         x = x >> shift;
//         return x;
//     }
//     static bitsig land(const bitsig v, const bitsig x)
//     {
//         return v & x;
//     }
//     static bitsig lor(const bitsig v, const bitsig x)
//     {
//         return v | x;
//     }
//     static bitsig lxor(const bitsig v, const bitsig x)
//     {
//         return v ^ x;
//     }
//     static bitsig neg(const bitsig v)
//     {
//         return ~v;
//     }
//     static bitsig inc(const bitsig v)
//     {
//         return v + 1;
//     }
//     static bitsig dec(const bitsig v)
//     {
//         return v - 1;
//     }
//     static bitsig add(const bitsig v, const bitsig m)
//     {
//         return v + m;
//     }
//     static bitsig sub(const bitsig v, const bitsig m)
//     {
//         return v - m;
//     }
//     static bitsig mul(const bitsig v, const bitsig m)
//     {
//         return v * m;
//     }
//     static bitsig div(const bitsig v, const bitsig m)
//     {
//         return v / m;
//     }
//     static bitsig gt(const bitsig v, const bitsig m)
//     {
//         return v > m;
//     }
//     static bitsig lt(const bitsig v, const bitsig m)
//     {
//         return v < m;
//     }
//     static bitsig gte(const bitsig v, const bitsig m)
//     {
//         return v >= m;
//     }
//     static bitsig lte(const bitsig v, const bitsig m)
//     {
//         return v <= m;
//     }
//     static bitsig eq(const bitsig v, const bitsig m)
//     {
//         return v == m;
//     }
//     static bitsig ct(const bitsig v, const bitsig width)
//     {
//         bitsig x = 0;
//         for (size_t i = 0; i < width; i++)
//         {
//             x += (v & (1 << i)) > 0;
//         }
//         return x;
//     }
//     static bitsig l(const bitsig width)
//     {
//         bitsig v = 0;
//         for (size_t i = 0; i < width; i++)
//         {
//             v += (1 << i);
//         }
//         return v;
//     }

//     static bitsig noise()
//     {
//         bitsig v = static_cast<bitsig>(rand());
//         return v;
//     }

//     static float toSignal(const bitsig t)
//     {
//         return maxiMap::linlin(t, 0, (float)std::numeric_limits<uint32_t>::max(), -1, 1);
//     }

//     static float toTrigSignal(const bitsig t)
//     {
//         return t > 0 ? 1.0 : -1.0;
//     }

//     static bitsig fromSignal(const float t)
//     {
//         const bitsig halfRange = (std::numeric_limits<uint32_t>::max() / 2);
//         const bitsig val = halfRange + (t * (halfRange - 1));
//         return val;
//     }

//     // void sett(maxiBits::bitsig v){t=v;}
//     // maxiBits::bitsig gett() const {return t;};

//     // maxiBits::bitsig t=0;
// };

/**
 * Count triggers
 */
class maxiCounter
{
public:
    /** Increase each time a trigger is received
     * \param incTrigger a signal that triggers the counter to increment 
     * \param resetTrigger a signal that resets the counter to zero
     * \returns the number of triggers received since the beginning or the last reset
     */
    float count(float incTrigger, float resetTrigger)
    {
        if (inctrig.onZX(incTrigger))
        {
            value++;
        }
        if (rstrig.onZX(resetTrigger))
        {
            value = 0;
        }
        return value;
    }

private:
    float value = 0;
    maxiTrigger inctrig, rstrig;
};

/**
 * Pull values from an array when a trigger is received, according to a modulateable index
 */
class maxiIndex
{
public:
    maxiIndex();
    /**
     * \param trigSig a signal
     * \param indexSig a normalised index into the array (0-1)
     * \param _values an array of values or signals (modulateable)
     * \returns the value at [indexSig] in the array, when a trigger is received in [trigSig]
     */
    float pull(const float trigSig, float indexSig, DOUBLEARRAY_REF _values)
    {
        // float *__arrayStart = __builtin_cheerp_make_regular<float>(_values, 0);
        // size_t __arrayLength = _values->get_length();
        // vector<float> values = vector<float>(__arrayStart, __arrayStart + __arrayLength);
        NORMALISE_ARRAY_TYPE(_values, values)
        if (trig.onZX(trigSig))
        {
            if (indexSig < 0)
                indexSig = 0;
            if (indexSig > 1)
                indexSig = 1;
            size_t arrayIndex = static_cast<size_t>(floor(indexSig * 0.99999999 * values.size()));
            value = values[arrayIndex];
        }
        return value;
    }

private:
    maxiTrigger trig;
    float value = 0;
};

/**
 * Read from an array of signals - like supercollider Select.ar
 */
class maxiSelect {
public:
    maxiSelect();

    /**
     * \param index an index into the array
     * \param values a modulateable array of values
     * \param normalised if true, the index should be between 0 and 1, if false, then the index should be between 0 and length(values)-1
     * \returns an item from the array of values, according to the floor or the index value
     */
    float play(float index, DOUBLEARRAY_REF values, bool normalised) {
        auto arrayLen = F64_ARRAY_SIZE(values); 

        if (normalised) {
            index *= (arrayLen - 1e-9); 
        }else{
            //assume index is direct mapping to array element
        }
        if (index < 0) {
            index = 0;
        }else if (index >= arrayLen) {
            index = arrayLen - 1;
        }
        float value = F64_ARRAY_AT(values, static_cast<size_t>(index));
        return value;
    }
private:

};

/**
 * Read from an array of signals or values, with linear interpolation between neighbours - like supercollider SelectX.ar
 */
class maxiSelectX {
public:
    maxiSelectX();

    /**
     * Read from an array with linear interpolation.  This can be useful for cross-fading across sets of signals.
     * \param index an index into the array
     * \param values a modulateable array of values
     * \param normalised if true, the index should be between 0 and 1, if false, then the index should be between 0 and length(values)-1
     * \returns an item from the array of values, according to the index, and interpolating between neighbouring values.\n
     * e.g if values = {2,3} and normalised index = 0.5, then 2.5 will be returns
     */
    float play(float index, DOUBLEARRAY_REF values, bool normalised) {
        auto arrayLen = F64_ARRAY_SIZE(values); 

        if (normalised) {
            index *= (arrayLen - 1e-9); 
        }else{
            //assume index is direct mapping to array element
        }
        if (index < 0) {
            index = 0;
        }else if (index >= arrayLen) {
            index = arrayLen - 1;
        }
        //get indices and interpolation factor
        size_t a1 = floor(index);
        float mix = index - a1;
        size_t a2 = a1 + 1;
        if (a2  == arrayLen) a2=0;
        //interpolate
        float value = (F64_ARRAY_AT(values, a1) * (1.0 -mix)) + 
                        (F64_ARRAY_AT(values, a2) * mix);
        return value;
    }
private:

};

/**
 * Pull sequential values from an array
 */
class maxiStep
{
public:
    maxiStep();
    /**
     * Take values from the array when triggered
     * \param trigSig A signal to trigger a new value on a positive zero crossing
     * \param values An array of values
     * \param step The amount that the array index should increase after pulling a new value.  This wraps around to zero at the end of the array
     */
    float pull(const float trigSig, DOUBLEARRAY values, float step)
    {
        if (trig.onZX(trigSig))
        {
            if (first) {
                first=false;
                index = 0;
            }else{
                auto arrayLen = F64_ARRAY_SIZE(values); 
                //should this be step % arraylen?  how about -ve vals?
                if (step > arrayLen) {
                    step = arrayLen;
                }
                // LOG(index);
                // LOG(arrayLen);
                index = index + step;
                // LOG(index);
                if (index < 0) {
                    index = arrayLen + index;
                // LOG(index);
                }else if (index >= arrayLen) {
                    index = index - arrayLen;
                }
            }
        }
        float value = F64_ARRAY_AT(values, static_cast<size_t>(index));
        return value;
    }

    /*! Get the current array index*/
    float getIndex() {
        return index;
    }

private:
    maxiTrigger trig;
    bool first=true;
    float index=0;
};

/**
 * Sequence triggers and numbers, using a list of modulateable ratios to control timing.
 */
class maxiRatioSeq
{
public:
    maxiRatioSeq();
    /**
     * Divide a phasor into periods according to a set o ratios, and send a trigger at the start of each period.\n
     * Examples ratios: (assuming the phasor takes the length of one bar to cycle and 4/4 timing):\n
     * {1,1,1,1} four crotchets\n
     * {2,2,2,1,1} three crotchets then two quavers\n
     * {4,4,4,1,1,1,1} three crotchets then four semi-quavers\n
     * {3,3,2} two dotted crotchets then a crotchet\n
     * {1} a semibrieve\n
     * {3,3,3,1,1,1} three crotchets followed by a triplet\n
     * {33,991,13,153} hmmm - well it might sound interesting?\n
     * {maxiMap::linlin(osc.phasor(0.4),0,1,10,20),100} modulate the ratios\n
     * \param phase a phasor signal, rising from 0 to 1 (you could use maxiOsc::phasor)\n
     * \param times a list of time ratios.  The phasor will be divided up into these ratios, and a trigger will be returned at the start of each period\n
     * \returns a trigger at the start of each period\n
     */
    float playTrig(float phase, DOUBLEARRAY times)
    {
        if (first) {
            first=false;
            prevPhase = phase - 1.0 / maxiSettings::sampleRate;
        }
        float trig = 0;
        float sum=0;
        size_t seqlen = F64_ARRAY_SIZE(times);
        for(size_t i=0; i < seqlen; i++) sum += F64_ARRAY_AT(times,i);
        if (prevPhase > phase)
        {
            //wrapping point
            prevPhase = -1.0 / maxiSettings::sampleRate;
        }
        float accumulatedTime = 0;
        for (size_t i = 0; i < seqlen; i++)
        {
            accumulatedTime += F64_ARRAY_AT(times,i);
            float normalisedTime = accumulatedTime / sum;
            if (normalisedTime == 1.0)
                normalisedTime = 0.0;
            if ((prevPhase <= normalisedTime && phase > normalisedTime))
            {
                trig = 1;
                break;
            }
        }
        prevPhase = phase;
        return trig;
    }

    /**
     * Take values incrementally from a list, with timing controlled by ratios
     * \param phase see playTrig
     * \param times see playTrig
     * \param values an array of numbers.  Each time a period starts, a number is returned from this list.  Values are taken incrementally and with looping. The contents and length of the list are modulateable.  This function is useful for sequencing pitches or controller values.
     * \returns a value taken incrementally from the list of values, updated each time a new timing period begins (according to the list of ratios)
     */
    float playValues(float phase, DOUBLEARRAY_REF times, DOUBLEARRAY_REF values)
    {
        // NORMALISE_ARRAY_TYPE(_times, times)
        // NORMALISE_ARRAY_TYPE(_values, values)
        size_t vallen = F64_ARRAY_SIZE(values);
        if (lengthOfValues != vallen)
        {
            lengthOfValues = vallen;
            counter = lengthOfValues - 1;
        }
        if (playTrig(phase, times))
        {
            counter++;
            if (counter == vallen)
            {
                counter = 0;
            }
        }
        return F64_ARRAY_AT(values,counter);
    }

private:
    float prevPhase = 0;
    size_t counter = 0;
    size_t lengthOfValues = 0;
    bool first=true;
};

/**
 * Extend a trigger into a pulse. This is useful for making basic gates in sequences. Use maxiEnvGen for more advanced gate and envelope generation.
 */ 
class maxiZXToPulse
{
public:
    maxiZXToPulse();
    /**
     * Extend a trigger into a pulse.
     * \param input a signal
     * \param holdTimeInSamples the length of the pulse in samples (use maxiConvert to get this value from milliseconds)
     * \returns a pulse, triggered by a zero crossing in the input
     */
    float play(float input, float holdTimeInSamples) {
        float output =0;
        
        if (trig.onZX(input)) {
            holdCounter = holdTimeInSamples;
        }

        if (holdCounter > 0) {
            output = 1;
            holdCounter--;
        }

        return output;
    }
private:
    maxiTrigger trig;
    float holdCounter = 0;
};


/**
 * An envelope generator. 
 */
class maxiEnvGen {
    public:

        static constexpr float HOLD = -46692;

        maxiEnvGen();

        /*! Get the latest value of the envelope \param trigger A positive zero crossing (or constant 1) starts the envelope*/
        float play(float trigger) {
            switch(state) {
                case WAITING:
                {
                    if (trigDetector.onZX(trigger)) {
                        if (F64_ARRAY_SIZE(stages) > 0) {
                            state = TRIGGERED;
                            nxcHappened = false;
                        }else{
                            break;
                        }
                    }
                    else {
                        break;
                    }
                }
                case TRIGGERED: 
                {
                    //calculate the current value of the envelope
                    envStage *currStage = &stages[phase];
                    //check if the trigger went -ve yet? (used for hold function)
                    if (holdDetector.onZX(-trigger)) {
                        nxcHappened = true;
                    }
                    if (currStage->hold) {
                        state = playStates::HOLDING;
                    }else{
                        envval = maxiMap::linlin(pow(currStage->currentlevel,currStage->curve), 0, 1, 
                            currStage->startlevel, 
                            currStage->endlevel);
                        currStage->counter++;
                        // cout << currStage->counter << endl;
                        //move to the next phase?
                        if (currStage->counter == currStage->length) {
                            currStage->counter=0;
                            currStage->currentlevel=0;
                            phase++;
                        
                        }else{
                            //calc next bit of current phase
                            currStage->currentlevel += currStage->gradient;
                            // cout << currStage.currentlevel << endl;
                        }
                        if (retrigger) {
                            if (retriggerDetector.onZX(trigger)) {
                                nxcHappened = false;
                                reset();
                            }
                        }
                        break;
                    }
                }
                case HOLDING:
                {
                    //wait for negative zero crossing
                    //envval remains unchanged
                    if (holdDetector.onZX(-trigger)) {
                        nxcHappened = true;

                    }
                    if (nxcHappened) {
                        state = playStates::TRIGGERED;
                        phase++;
                    }
                    if (retrigger) {
                        if (retriggerDetector.onZX(trigger)) {
                            nxcHappened = false;
                            reset();
                        }
                    }
                    break;
                }
            }
            if (phase == F64_ARRAY_SIZE(stages)) {
                if (loop) {
                    reset();
                }else{
                    resetAndArm();
                }
            }

            return envval;
        }

        /**
         * Configure the envelope generator
         * \param levels An array of levels
         * \param times An array of times between the levels, in milliseconds.  To make the envelope hold until the first negative zero crossing, use maxiEnvGen::HOLD as the time. Only one hold segment is allowed.
         * \param curves An array of exponential curve values to shape each segment (1 = straight, 2= squared, 0.5 = square root etc)
         * \param looping True if the envelope should infinitely repeat
         * \param allowRetrigger Set to true if the envelope should allow retriggering before it is finished, or false if the envelope should always reach the end before being allowed to restart
         * Example C++ usage  env.setup({0,1,0},{100,400}, {0.5,1}, false).
         * levels should be one element longer than times and curves
         * \returns True if the configuration was successful
         */
        bool setup(DOUBLEARRAY levels, DOUBLEARRAY times, DOUBLEARRAY curves, bool looping, bool allowRetrigger = false) {
            if ((F64_ARRAY_SIZE(levels) == F64_ARRAY_SIZE(times)+1) && (F64_ARRAY_SIZE(levels) == F64_ARRAY_SIZE(curves) + 1)) {
                stages.clear();
                float accumulatedTime = 0;
                containsHold =0;
                for(size_t i=0; i < F64_ARRAY_SIZE(times); i++) {
                    envStage stage;
                    stage.startlevel = F64_ARRAY_AT(levels,i);
                    stage.endlevel = F64_ARRAY_AT(levels,i+1);
                    float stageTime = F64_ARRAY_AT(times,i);
                    if (stageTime == maxiEnvGen::HOLD) {
                        if (containsHold) {
                            return 0;
                        }
                    }
                    accumulatedTime = setupSegmentTime(stageTime, stage, accumulatedTime);
                    stage.curve = F64_ARRAY_AT(curves,i);
                    stage.counter=0;
                    stage.currentlevel = 0;
                    stages.push_back(stage);
                    // cout << "Stage " << stage.startlevel << "\t" << stage.endlevel << "\t" << stage.length << "\t" << stage.gradient << "\t" << stage.curve << endl;
                }
                loop = looping;
                retrigger = allowRetrigger;
                resetAndArm();
                return 1;
            }else{
                // cout << "maxiEnv::setup - levels array should be one longer than times and curves\n";
                return 0;
            }
        }

        /*!Restart the envelope immeadiately*/
        void reset() {
            if (phase < stages.size()){
                envStage *currStage = &stages[phase];
                currStage->counter = 0;
                currStage->currentlevel = 0;
            }
            phase=0;
            state = TRIGGERED;
        }

        /*!Stop the envelope and wait for a new trigger to start it*/
        void resetAndArm() {
            reset();
            state = WAITING;
        }

        /** Set the level of a segment of the envelope \param index The index of the level \param value The new level*/
        bool setLevel(size_t index, float value) {
            bool error = 0;
            if (index <= stages.size()) {
                if (index == stages.size()) {
                    stages[index-1].endlevel = value;
                }else{
                    stages[index].startlevel = value;
                    if (index > 0){
                        stages[index-1].endlevel = value;
                    }
                }
            }else{
                error = 1;
            }
            return error;
        }

        /** Set the curve of a segment of the envelope \param index The index of the segment \param value The new curve value*/
        bool setCurve(size_t index, float value) {
            bool error=0;
            if (index <= stages.size()) {
                stages[index].curve = value;
            }
            return error;
        }

        /** Set the length of a segment of the envelope \param index The index of the segment \param value The new length (in ms)*/
        bool setTime(size_t index, float value) {
            bool error=0;
            if (index <= stages.size()) {
                if (value == maxiEnvGen::HOLD && containsHold) {
                    error = 1;
                }
                if (!error) {
                    setupSegmentTime(value, stages[index], 0);
                }
            }else{
                error = 1;
            }
            return error;
        }

        /**
         * Helper function to create a triangular envelope
         * \param attack Rise time in ms
         * \param release Fall time in ms
         */
        void setupAR(const float attack, const float release) {
            setup({0,1,0}, {attack, release}, {1,1}, false, false);
        }

        /**
         * Helper function to create a triangular(ish) envelope with sustain section at peak
         * The sustain section will end when the trigger input drops below 0
         * \param attack Rise time in ms
         * \param release Fall time in ms
         */
        void setupASR(const float attack, const float release) {
            setup({0,1,1,0}, {attack, maxiEnvGen::HOLD, release}, {1,1,1}, false, false);
        }
        /**
         * Helper function to create an attack-decay-sustain-release envelope
         * The sustain section will end when the trigger input drops below 0
         * The envelope will rise from 0 to 1 in the attack segment, then drop to the sustain level before falling to 0.
         * \param attack Rise time in ms
         * \param decay Decay time in ms
         * \param sustain Sustain level 
         * \param release Fall time in ms
         */
        void setupADSR(const float attack, const float decay, const float sustain, const float release) {
            setup({0,1,sustain,sustain,0}, {attack, decay, maxiEnvGen::HOLD, release}, {1,1,1,1}, false, false);
        }

        /*!Set the envelope to retrigger or not \param val True is the envelope should retrigger, false if not*/
        void setRetrigger(const bool val) {
            retrigger = val;
        }
        /*!Find out if the envelope retriggers*/
        bool getRetrigger() {return retrigger;}
        /*!Set the envelope to loop or not \param val True is the envelope should loop, false if not*/
        void setLoop(const bool val) {
            loop = val;
        }
        /*!Find out if the envelope loops */
        bool getLoop() {return loop;}

    private:
        size_t phase=0;
        float envval = 0;        
        bool loop = false;
        bool retrigger = false;
        enum playStates {WAITING, TRIGGERED, HOLDING} state;
        bool nxcHappened;
        bool containsHold;
        struct envStage {
            float startlevel;
            float endlevel;
            float currentlevel;
            float gradient;
            float curve;
            size_t length;
            size_t counter;
            bool hold;
        };
        vector<envStage> stages;

        maxiTrigger trigDetector;
        maxiTrigger holdDetector;
        maxiTrigger retriggerDetector;

        float setupSegmentTime(const float stageTime, envStage &stage, float accumulatedTime) {
            if (stageTime == maxiEnvGen::HOLD) {
                stage.length = 0;
                stage.hold = 1;
                stage.gradient=0;
                containsHold = 1;
            }else{
                float len = ((stageTime / 1000.0) * maxiSettings::sampleRate) + accumulatedTime;
                stage.length = static_cast<size_t>(floor(len));
                accumulatedTime = len - stage.length;
                stage.gradient = 1.0 / stage.length;
                stage.hold=0;
            }   
            return accumulatedTime;         
        }
};


/**
 * Poll values to stdout at regular intervals
 */
#ifdef USE_STRINGS
class maxiPoll {
    public:
        maxiPoll();
        /**
         * \param val The value to poll
         * \param frequency How often to print this value (Hz)
         * \param txt Additional text to br printed before the value
         * \returns the value being polled, so this can used as a pass-through function e.g. filter.play(obj.poll(osc.saw(2)),0.5)
         */
        float poll(float val, float frequency=4, STRINGCLASS txt="", STRINGCLASS end="\n") {
            if (imp.impulse(frequency)) {
                LOG(txt);
                LOG(val);
                LOG(end);
            }
            return val;
        }
    private:
        maxiOsc imp;
};
#endif

/**
 * Calculate the Root Mean Square of a signal over a window of time
 * This is a good measurement of the amount of power in a signal
 */
class maxiRMS {
    public:
        maxiRMS();

        /*!Configure the analyser \param maxLength The maximum length of time to analyse (ms) \param windowSize The size of the window of time to analyse initially (ms, <= maxLength) */
        void setup(float maxLength, float windowSize) {
            buf.setup(maxiConvert::msToSamps(maxLength));
            setWindowSize(windowSize);
        }

        /*!Set the size of the analysis window \param newWindowSize the size of the analysis window (in ms). Large values will smooth out the measurement, and make it less responsive to transients*/
        void setWindowSize(float newWindowSize) {
            size_t windowSizeInSamples = maxiConvert::msToSamps(newWindowSize);
            if (windowSizeInSamples <= buf.size()) {
                windowSize = windowSizeInSamples;
            }
            runningRMS = 0;
        }

        /*!Find out the size of the analysis window (in ms)*/
        float getWindowSize() {
            return maxiConvert::sampsToMs(windowSize);
        }

        /*Analyse the signal \param signal a signal \returns RMS*/
        float play(float signal) {
            float sigPow2 = (signal * signal);
            buf.push(sigPow2);
            runningRMS += sigPow2;
            runningRMS -= buf.tail(windowSize);
            return sqrt(runningRMS/windowSize);
        }

    private:
        maxiRingBuf buf;
        size_t windowSize=0; // in samples
        float runningRMS=0;
};


/**
 * The class provides a range of dynamics processing: downward and upward compression, downward and upward expansion,
 * sidechaining, attack and release, lookahead and peak or RMS detection
 */


class maxiDynamics {

    public:

        enum ANALYSERS {PEAK, RMS};
        
        maxiDynamics() {
            //define detector functions
            inputPeak = [](float sig) {
                return abs(sig);
            };

            rms.setup(500,50);
            inputRMS = [&](float sig) {
                return rms.play(sig);
            };

            //default RMS
            inputAnalyser = inputRMS;

            //setup envelopes
            arEnvHigh.setupASR(10,10);
            arEnvHigh.setRetrigger(false);
            arEnvLow.setupASR(10,10);
            arEnvLow.setRetrigger(false);

            lookAheadDelay.setup(maxiSettings::sampleRate * 1); //max 1s
        }


        /**
         * This functions compands the signal, providing download compression or upward expansion above an upper thresold, and
         * upward compression or downward expansion below a lower threshold.
         * \param sig The input signal to be companded
         * \param control This signal is used to trigger the compander. Use it for sidechaining, or if no sidechain is needed, use the same signal for this and the input signal
         * \param thresholdHigh The high threshold, in Dbs
         * \param ratioHigh The ratio for companding above the high threshold
         * \param kneeHigh The size of the knee for companding above the high threshold (in Dbs)
         * \param thresholdLow The low threshold, in Dbs
         * \param ratioLow The ratio for companding below the low threshold
         * \param kneeLow The size of the knee for companding below the low threshold (in Dbs)
         * \returns a companded signal
         */
        float play(float sig, float control, 
            float thresholdHigh, float ratioHigh, float kneeHigh,
            float thresholdLow, float ratioLow, float kneeLow
        ) {
            float controlDB = maxiConvert::ampToDbs(inputAnalyser(control));
            float outDB = maxiConvert::ampToDbs(sig);
            //companding above the high threshold
            if (ratioHigh > 0) {
                if (kneeHigh > 0) {
                    float lowerKnee = thresholdHigh - (kneeHigh/2.0);
                    float higherKnee = thresholdHigh  +(kneeHigh/2.0);
                    //attack/release
                    float envRatio = 1;
                    if (controlDB >= lowerKnee) {
                        float envVal = arEnvHigh.play(1);
                        envRatio = envToRatio(envVal, ratioHigh);
                    }else {
                        float envVal = arEnvHigh.play(-1);
                    }
                    if ((controlDB >= lowerKnee) && (controlDB < higherKnee)) {
                        float kneeHighOut = ((higherKnee - thresholdHigh) / envRatio) + thresholdHigh;
                        float kneeRange = (kneeHighOut - lowerKnee);
                        float t = (controlDB - lowerKnee) / kneeHigh;
                        //bezier on x only
                        float curve =  ratioHigh > 1 ? 0.8 : 0.2;
                        float kneex = (2 * (1-t) * t * curve) + (t*t);
                        outDB = lowerKnee + (kneex * kneeRange);
                    }
                    else if (controlDB >= higherKnee) {
                        outDB = ((controlDB - thresholdHigh) / envRatio) + thresholdHigh;
                    }
                }
                else {
                    //no knee
                    if (controlDB > thresholdHigh) {
                        float envVal = arEnvHigh.play(1);
                        float envRatio = envToRatio(envVal, ratioHigh);
                        outDB = ((controlDB - thresholdHigh) / envRatio) + thresholdHigh;  
                    }else {
                        float envVal = arEnvHigh.play(-1);
                    }
                }
            }  
            //companding below the low threshold
            if (ratioLow > 0) {
                if (kneeLow > 0) {
                    float lowerKnee = thresholdLow - (kneeLow/2.0);
                    float higherKnee = thresholdLow  +(kneeLow/2.0);
                    //attack/release
                    float envRatio = 1;
                    if (controlDB < lowerKnee) {
                        float envVal = arEnvLow.play(1);
                        envRatio = envToRatio(envVal, ratioLow);
                    }else {
                        float envVal = arEnvLow.play(-1);
                    }
                    if ((controlDB >= lowerKnee) && (controlDB < higherKnee)) {
                        float kneeLowOut = thresholdLow - ((thresholdLow-lowerKnee) / ratioLow);
                        float kneeRange = (higherKnee - kneeLowOut);
                        float t = (controlDB - lowerKnee) / kneeLow;
                        //bezier on x only
                        float curve =  ratioLow > 1 ? 0.2 : 0.8;
                        float kneex = (2 * (1-t) * t * curve) + (t*t);
                        outDB = kneeLowOut + (kneex * kneeRange);
                    }
                    else if (controlDB < lowerKnee) {
                        outDB = thresholdLow - ((thresholdLow-controlDB) / ratioLow);
                    }
                }
                else {
                    //no knee
                    if (controlDB < thresholdLow) {
                        float envVal = arEnvLow.play(1);
                        float envRatio = envToRatio(envVal, ratioLow);
                        outDB = thresholdLow - ((thresholdLow-controlDB) / ratioLow);
                    }else {
                        float envVal = arEnvLow.play(-1);
                    }
                }
            }  
            //scale the signal according to the amount of compansion on the control signal
            float outAmp = maxiConvert::dbsToAmp(outDB);
            float sigOut = 0;
            if (outAmp > 0) {
                if (lookAheadSize > 0) {
                    lookAheadDelay.push(sig);
                    sigOut = lookAheadDelay.tail(lookAheadSize);
                }else{
                    sigOut = sig;
                }
                sigOut = sigOut * (control / outAmp);
            }
            return sigOut;
        }

        /**
         * Compress a signal (using downward compression)
         * \param sig The input signal to be compressed
         * \param threshold The threshold, in Dbs
         * \param ratio The compression ratio (>1 provides compression, <1 provides expansion)
         * \param knee The size of the knee (in Dbs)
         * \returns a compressed signal
         */
        float compress(float sig, float threshold, float ratio, float knee) {
            return play(sig, sig, threshold, ratio, knee, 0, 0, 0);
        }
        /**
         * Compress a signal with sidechaining (using downward compression)
         * \param sig The input signal to be compressed
         * \param control The sidechain signal
         * \param threshold The threshold, in Dbs
         * \param ratio The compression ratio (>1 provides compression, <1 provides expansion)
         * \param knee The size of the knee (in Dbs)
         * \returns a compressed signal
         */
        float sidechainCompress(float sig, float control, float threshold, float ratio, float knee) {
            return play(sig, control, threshold, ratio, knee, 0, 0, 0);
        }
        /**
         * Compand a signal, using detection above a threshold (provides downward compression or upward expansion)
         * \param sig The input signal to be compressed
         * \param control The sidechain signal
         * \param threshold The threshold, in Dbs
         * \param ratio The compression ratio (>1 provides compression, <1 provides expansion)
         * \param knee The size of the knee (in Dbs)
         * \returns a companded signal
         */
        float compandAbove(float sig, float control, float threshold, float ratio, float knee) {
            return play(sig, control, threshold, ratio, knee, 0, 0, 0);
        }
        /**
         * Compand a signal, using detection below a threshold (provides upward compression or downward expansion)
         * \param sig The input signal to be compressed
         * \param control The sidechain signal
         * \param threshold The threshold, in Dbs
         * \param ratio The compression ratio (>1 provides compression, <1 provides expansion)
         * \param knee The size of the knee (in Dbs)
         * \returns a companded signal
         */
        float compandBelow(float sig, float control, float threshold, float ratio, float knee) {
            return play(sig, control, 0, 0, 0, threshold, ratio, knee);
        }

        /**
         * Set the attack time for the high threshold. This is the amount of time over which the ratio moves from 1 to its full value, following the input analyser going over the threshold.
         * \param attack The attack time (in milliseconds)
         */
        void setAttackHigh(float attack) {
            arEnvHigh.setTime(0, attack);
        }
        /**
         * Set the release time for the high threshold. This is the amount of time over which the ratio moves from its full value to 1, following the input analyser going under the threshold.
         * \param release The release time (in milliseconds)
         */
        void setReleaseHigh(float release) {
            arEnvHigh.setTime(2, release);
        }
        /**
         * Set the attack time for the low threshold. This is the amount of time over which the ratio moves from 1 to its full value, following the input analyser going under the threshold.
         * \param attack The attack time (in milliseconds)
         */
        void setAttackLow(float attack) {
            arEnvLow.setTime(0, attack);
        }
        /**
         * Set the release time for the low threshold. This is the amount of time over which the ratio moves from its full value to 1, following the input analyser going over the threshold.
         * \param release The release time (in milliseconds)
         */
        void setReleaseLow(float release) {
            arEnvLow.setTime(2, release);
        }

        /**
         * The look ahead creates a delay on the input signal, meaning that that the signal is compressed according to event that have already happened in the control signal.  This can be useful for limiting and catching fast transients.
         * \param length The amount of time the compressor looks ahead (in milliseconds)
         */
        void setLookAhead(float length) {
            lookAheadSize = maxiConvert::msToSamps(length);
            lookAheadSize = min(lookAheadSize, lookAheadDelay.size());
        }
        /**
         * \returns the look ahead time (in milliseconds)
         */
        float getLookAhead() {
            return maxiConvert::sampsToMs(lookAheadSize);
        }

        /**
         * Set the size of the RMS window.  Longer times give a slower response
         * \param winSize The size of the window (in milliseconds)
         */
        void setRMSWindowSize(float winSize) {
            rms.setWindowSize(min(winSize, 500.f));
        }

        /**
         * Set the method by which the compressor analyses the control input
         * \mode maxiDynamics::PEAK for peak analysis, maxiDynamics::RMS for rms analysis
         */
        void setInputAnalyser(ANALYSERS mode) {
            if (mode == PEAK) {
                inputAnalyser = inputPeak;
            }else{
                inputAnalyser = inputRMS;
            }
        }


    private:
        maxiEnvGen arEnvHigh, arEnvLow;
        maxiRingBuf lookAheadDelay;
        size_t lookAheadSize = 0;
        maxiRMS rms;
        std::function<float(float)> inputPeak;        
        std::function<float(float)> inputRMS;        
        std::function<float(float)> inputAnalyser;
        maxiPoll poll;

        //mapping from attack/release envelope to ratio
        float envToRatio(float envVal, float ratio) {
            float envRatio = 1;
            if (ratio > 1) {
                envRatio = 1 + ((ratio-1) * envVal);
            }else {
                envRatio = 1 - ((1-ratio) * envVal);
            }
            return envRatio;
        }

};
#endif
