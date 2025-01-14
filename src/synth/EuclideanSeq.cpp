#include "EuclideanSeq.hpp"
#include <cstdio>
#include <cmath>


EuclideanSeq::EuclideanSeq() :
    n_choices_{ 2, 4, 8, 12, 16 },
    is_init_(0),
    params_{ 0 },
    probes_{ 0 }
{
}


EuclideanSeq::EuclideanSeq(const std::vector<int> &n_choices) :
    n_choices_(n_choices),
    is_init_(0),
    params_{ 0 },
    probes_{ 0 }
{
    // for (auto &c: n_choices) {
    //     xassert(c > 1);
    // }
}


bool EuclideanSeq::Process(float phasor)
{
    bool result = false;

    if (is_init_) {

        // Multiply and wrap around
        phasor *= params_.ph_mult;
        while (phasor >= 1){
            phasor -= 1;
        }
        // Apply offset
        phasor += offset_;
        while (phasor >= 1){
            phasor -= 1;
        }
        probes_[0] = phasor;

        // NOTE: Phasor is the last arg
        const float fi         = phasor * params_.n;
        int i                  = static_cast<int>(fi);
        const float rem        = fi - i;
        if (i == params_.n)
        {
            i--;
        }
        const int idx = ((i + params_.n) * params_.k) % params_.n;
        probes_[1] = idx;
        result = static_cast<bool>(idx < params_.k && rem < pulse_width_ ? 1 : 0);
    }

    return result;
}


void EuclideanSeq::SetParams(params p)
{
    // n
    if (p.n <= 0) {
        std::printf("Eucl- n is < 1, rounding to 1.\n");
        p.n = 1;
    }
    // k
    if (p.k > p.n) {
        std::printf("Eucl- k is > n, rounding to %d.\n", p.n);
        p.k = p.n;
    } else if (p.k <= 0) {
        std::printf("Eucl- k is < 1, rounding to 1.\n");
        p.k = 1;
    }
    // offset
    if (p.offset_d <= 0) {
        std::printf("Eucl- offset_d is < 1, rounding to 1.\n");
        p.offset_d = 1;
    }
    // k
    if (p.offset_n >= p.offset_d) {
        std::printf("Eucl- offset_n is > offset_d, wrapping around.\n");
        while (p.offset_n >= p.offset_d) {
            p.offset_n -= p.offset_d;
        }
    }
    if (p.offset_n < 0) {
        std::printf("Eucl- offset_n is < 0, rounding to 0.\n");
        p.offset_n = 0;
    }
    // ph_mult
    if (p.ph_mult <= 0) {
        std::printf("Eucl- ph_mult is < 1, rounding to 1.\n");
        p.ph_mult = 1;
    }

    params_ = p;
    offset_ = static_cast<float>(p.offset_n) /
              static_cast<float>(p.offset_d);

    is_init_ = true;
}


int EuclideanSeq::MakeN_(float param)
{
    float n = param * n_choices_.size();
    n = std::floor(n);
    while (n >= n_choices_.size()) {
        n -= 1.;
    }
    while (n < 0) {
        n += 1.;
    }

    return n_choices_[static_cast<size_t>(n)];
}


int EuclideanSeq::MakeMul_(float param) {
    static std::vector<int> mul_lookup{ 1, 2, 4 };

    // Law multiplier like x^3
    param = param * param * param;

    float n = param * mul_lookup.size();
    n = std::floor(n);
    while (n >= mul_lookup.size()) {
        n -= 1.;
    }
    while (n < 0) {
        n += 1.;
    }

    return mul_lookup[static_cast<size_t>(n)];
}


void EuclideanSeq::MapNNParams(std::vector<float> nn_params)
{
    assert(nn_params.size() == n_params);

    constexpr float k_n_min = 1;
    // constexpr float k_n_max = 32;
    // constexpr float k_n_range = k_n_max - k_n_min;
    constexpr float k_offset_range = 8;
    constexpr float k_k_scale = 1.;

    // n
    float n = MakeN_(nn_params[0]);

    // k
    float k_range = n - k_n_min;
    float k = nn_params[1] * (k_range * k_k_scale);
    k += k_n_min;
    k = std::round(k);

    // offset
    // float offset_d = nn_params[3] * k_offset_range;
    // offset_d += k_n_min;
    // offset_d = std::round(offset_d);
    float offset_d = k_offset_range;
    float offset_n_range = offset_d - k_n_min;
    float offset_n = nn_params[2] * offset_n_range;

    // ph_mult
    int ph_mult = MakeMul_(nn_params[4]);

    const params set_p {
        static_cast<int>(n),
        static_cast<int>(k),
        static_cast<int>(offset_n),
        static_cast<int>(offset_d),
        ph_mult,
    };

    SetParams(set_p);
}
