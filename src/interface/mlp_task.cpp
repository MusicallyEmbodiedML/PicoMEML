#include "mlp_task.hpp"
#include "../mlp/MLP.h"
#include "../mlp/Data.h"
#include "../mlp/Dataset.hpp"
#include "../utils/PrintVector.hpp"
#include "../interface/DaisyUARTSend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <string>
#include <deque>
#include <memory>

#include "../PicoDefs.hpp"

#include <Arduino.h>
//#include "RingBuf.h"


// Private "methods"
static void mlp_load_all_();
static void mlp_save_all_();
static void mlp_pretrain_centre_();

static std::unique_ptr<DaisyUARTSend> daisy_uart_;

// MLP config constants
static const unsigned int kBias = 1;
static const std::vector<ACTIVATION_FUNCTIONS> layers_activfuncs = {
    RELU, RELU, RELU, SIGMOID
};
static const bool use_constant_weight_init = false;
static const float constant_weight_init = 0;
static input_data_t kZoom_mode_reset;

// Dataset memory
static Dataset dataset_[kMaxDatasets];
static size_t ds_n_ = 0;

// MLP memory
static MLP<float> *mlp_[kMaxModels] = { nullptr };
static size_t n_output_params_ = 0;
static MLP<float>::mlp_weights mlp_stored_weights_;
static input_data_t zoom_mode_centre_;
static input_data_t mlp_stored_input;
std::vector<float> mlp_stored_output;
static bool randomised_state_ = false;
static bool redraw_weights_ = true;
static bool flag_zoom_in_ = false;
static float speed_ = 1.0f;
static size_t nn_n_ = 0;

static std::shared_ptr<MIDIDevice> midi_;


/******************************
 * MLP TASK
 ******************************/

queue_t *nn_paramupdate_ = nullptr;

std::deque<float> inputRB; //ring buffer for inputs. Maybe a more efficient way to do this?

void mlp_init(queue_t *nn_paramupdate, size_t n_inputs, size_t n_params, size_t n_inputbuffer, std::shared_ptr<MIDIDevice> _midiDev)
{
    Serial.printf("MLP- Initialise with: %d -> %d\n", n_inputs, n_params);
    daisy_uart_ = std::make_unique<DaisyUARTSend>(uart_DaisyPIOTx);

    n_inputs *= n_inputbuffer;
    // fill up a ring buffer

    if (n_inputbuffer > 1) {
        for(size_t i=0; i < n_inputs; i++) {
            inputRB.push_back(0);
        }
    }

    const std::vector<size_t> layers_nodes = {
        n_inputs + kBias,
        10, 10, 14,
        n_params
    };

    n_output_params_ = n_params;

    // Instantiate objects
    assert(kMaxDatasets == kMaxModels);
    for (size_t n = 0; n < kMaxModels; n++) {
        //Serial.println("Creating dataset...");
        //dataset_[n] = new(dataset_mem_[n]) Dataset();
        //Serial.println("--- We got here!");

        mlp_[n] = new (std::nothrow) MLP<float>(
            layers_nodes,
            layers_activfuncs,
            loss::LOSS_MSE,
            use_constant_weight_init,
            constant_weight_init
        );
        if (!mlp_[n]) {
            Serial.printf("Out of memory allocating MLP %ul\n", n);
        }else{
            Serial.printf("MLP %d allocated\n", n);
        }
    }

    // Instantiate channels
    nn_paramupdate_ = nn_paramupdate;

    // Init the caching of input data
    const float init_val = 0.5;
    kZoom_mode_reset.resize(n_inputs, init_val);
    zoom_mode_centre_.resize(n_inputs, init_val);
    mlp_stored_input.resize(n_inputs, init_val);

    // midi_ = _midiDev;

    Serial.println("MLP- Initialised.");

    mlp_load_all_();
}


static void mlp_trigger_redraw_()
{
    redraw_weights_ = true;
}

void mlp_train()
{
    // Restore weights first
    if (randomised_state_ && mlp_stored_weights_.size() > 0) {
        mlp_[nn_n_]->SetWeights(mlp_stored_weights_);
        randomised_state_ = false;
        Serial.println("MLP- Restored pre-random weights.");
    }

    // We're in inference, clear all zoom
    flag_zoom_in_ = false;
    Serial.print("MLP- zoom: ");
    Serial.println(flag_zoom_in_);

    MLP<float>::training_pair_t dataset(dataset_[ds_n_].GetFeatures(), dataset_[ds_n_].GetLabels());

    // Check and report on dataset size
    Serial.print("MLP- Feature size ");
    Serial.print(dataset.first.size());
    Serial.print(", label size ");
    Serial.println(dataset.second.size());
    if (!dataset.first.size() || !dataset.second.size()) {
        return;
    }
    Serial.print("MLP- Feature dim ");
    Serial.print(dataset.first[0].size());
    Serial.print(", label dim ");
    Serial.println(dataset.second[0].size());
    if (!dataset.first[0].size() || !dataset.second[0].size()) {
        return;
    }

    // Train!
    Serial.print("MLP- Training for max ");
    Serial.print(gAppState.n_iterations);
    Serial.println(" iterations...");
    num_t loss = mlp_[nn_n_]->Train(dataset,
              1.,
              gAppState.n_iterations,
              0.0001,
              false);
    Serial.print("MLP- Trained, loss = ");
    Serial.println(loss, 10);

    mlp_save_all_();

    // Report loss back to state and UI
    gAppState.last_error = loss;
    //uart_update_loss(loss);
}


void mlp_load_all_()
{
    Serial.println("MLP- mlp_load_all_() requires FLASH");
}


void mlp_save_all_()
{
    Serial.println("MLP- mlp_load_all_() requires FLASH");
}

void mlp_draw(float speed)
{
    speed_ = speed;

    if (!randomised_state_) {
        mlp_stored_weights_ = mlp_[nn_n_]->GetWeights();
        randomised_state_ = true;
        Serial.println("MLP- Stored pre-random weights.");
        mlp_trigger_redraw_();
        flag_zoom_in_ = false;
    }
    //if (redraw_weights_) {
        mlp_[nn_n_]->DrawWeights();
        Serial.println("MLP- Weights randomised.");
        //redraw_weights_ = false;
#if 0
    } else {
        if (gAppState.current_expl_mode == expl_mode_zoom) {

            flag_zoom_in_ = true;
            zoom_mode_centre_ = mlp_stored_input;

            // Debug
            char serial_data[256];
            std::sprintf(
                serial_data,
                "MLP- Moving with speed %f%% around (%.3f etc...)",
                speed*100.f, zoom_mode_centre_[0]);
            Serial.println(serial_data);

        } else if (gAppState.current_expl_mode == expl_mode_pretrain) {

            zoom_mode_centre_ = kZoom_mode_reset;
            flag_zoom_in_ = true;
            // Train network with only one data point at the centre
            mlp_pretrain_centre_();
            Serial.print("MLP- Pretrained on centre (speed ");
            Serial.print(speed*100.f);
            Serial.println("%).");

        } else if (gAppState.current_expl_mode == expl_mode_nnweights) {

            zoom_mode_centre_ = kZoom_mode_reset;
            // Randomise weights less ("move" by speed)
            mlp_[nn_n_]->MoveWeights(speed);
            Serial.print("MLP- Weights moved ");
            Serial.print(speed*100.f);
            Serial.println("%.");

        }
    }
#endif
}

void mlp_add_data_point(const std::vector<float> &in, const std::vector<float> &out)
{
    dataset_[ds_n_].Add(in, out);
    mlp_trigger_redraw_();
}

void mlp_buffer_datapoint_tdnn(const std::vector<float> &in) {
for(size_t i=0; i < in.size(); i++) {
    inputRB.pop_front();
    inputRB.push_back(in.at(i));
}
}

void mlp_add_data_point_tdnn(const std::vector<float> &in, const std::vector<float> &out)
{
    std::vector<float> inputVector(inputRB.begin(), inputRB.end());
    dataset_[ds_n_].Add(inputVector, out);
    mlp_trigger_redraw_();
}

void mlp_clear_data()
{
    dataset_[ds_n_].Clear();
    Serial.print("MLP- Dataset ");
    Serial.print(ds_n_);
    Serial.print(" cleared.");
}

void mlp_clear_model()
{
    mlp_[nn_n_]->DrawWeights();
    flag_zoom_in_ = false;
    Serial.print("MLP- Model ");
    Serial.print(nn_n_);
    Serial.print(" re-initialised.");
}

void mlp_pretrain_centre_()
{
    if (mlp_stored_output.size() == 0) {
        Serial.println("MLP- mlp_stored_output.size() == 0!");
        return;
    }
    std::vector<float> single_feature(kNInputParams, 0.5f);
    single_feature.push_back(1.f);  // with bias
    std::vector<std::vector<float>> features {
        single_feature,
    };
    std::vector<std::vector<float>> labels {
        mlp_stored_output
    };
    MLP<float>::training_pair_t dataset(features, labels);

    Serial.print("Feature input dim: ");
    Serial.print(features[0].size());
    Serial.print(", output dim: ");
    Serial.println(labels[0].size());

    // Re-init weights
    mlp_[nn_n_]->DrawWeights();
    // Train with one point at centre
    Serial.println("Training MLP on centre...");
    mlp_[nn_n_]->Train(dataset,
              1.,
              1000,
              0.00001,
              false);
}

void mlp_set_speed(float speed)
{
    Serial.print("MLP- Speed: ");
    Serial.println(speed);
    speed_ = speed;
}

void mlp_set_expl_mode(te_expl_mode mode)
{
    gAppState.current_expl_mode = mode;
    // redraw_weights_ = true;
    zoom_mode_centre_ = kZoom_mode_reset;
}

void mlp_set_model_idx(size_t idx)
{
    if (idx >= kMaxModels) {
        Serial.print("MLP- Model index ");
        Serial.print(idx);
        Serial.println(" not valid.");
    } else {
        Serial.print("MLP- Model index: ");
        Serial.println(idx);
        nn_n_ = idx;
        mlp_stored_output.clear();
        mlp_stored_weights_ = mlp_[nn_n_]->GetWeights();
        mlp_trigger_redraw_();
    }
}

void mlp_set_dataset_idx(size_t idx)
{
    if (idx >= kMaxDatasets) {
        Serial.print("MLP- Dataset index ");
        Serial.print(idx);
        Serial.println(" not valid.");
    } else {
        Serial.print("MLP- Dataset index: ");
        Serial.println(idx);
        ds_n_ = idx;
    }
}

void mlp_inference(input_data_t joystick_read) {

    // Function to zoom and offset by given range
    static const auto zoom_in_ = [](float x, float move_by) {
        float local_range = 0.5f*speed_;
        float move_by_mod = move_by;
        // Nudge if too big
        float max_radius = local_range + move_by - 1.0f;
        if (max_radius > 0) {
            move_by_mod -= max_radius;
        }
        // Nudge if too small
        float min_radius = move_by - local_range;
        if (min_radius < 0) {
            move_by_mod -= min_radius;
        }
        // Scale and move
        float y = (x - 0.5f) * speed_ + move_by_mod;
        y = (y < 0) ? 0 : y;
        y = (y > 1.f) ? 1.f : y;
        return y;
    };

    // If we're zooming in, we want speed to shrink our view
    if (flag_zoom_in_) {
        for (unsigned int n = 0; n < joystick_read.size(); n++) {
            joystick_read[n] = zoom_in_(joystick_read[n], zoom_mode_centre_[n]);
        }
    }

        // Store current joystick read
        mlp_stored_input = joystick_read;

    // Instantiate data in/out
    joystick_read.push_back(1.0f);
    std::vector<num_t> output(n_output_params_);
    // Run model
    mlp_[nn_n_]->GetOutput(joystick_read, &output);

    // TODO apply transform here if set

    mlp_stored_output = output;

    // Send result
    // queue_try_add(
    //     nn_paramupdate_,
    //     reinterpret_cast<void *>(mlp_stored_output.data())
    // );
    daisy_uart_->SendParams(mlp_stored_output);

    // Send MIDI
    // midi_->SendParamsAsCC(mlp_stored_output);
}
