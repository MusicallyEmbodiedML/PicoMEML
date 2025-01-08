#include "mlp_task.hpp"
#include "../mlp/MLP.h"
#include "../mlp/Data.h"
#include "../mlp/Dataset.hpp"
#include "../utils/PrintVector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <string>

#include <Arduino.h>


// Private "methods"
static void mlp_load_all_();
static void mlp_save_all_();
static void mlp_pretrain_centre_();


// MLP config constants
static const unsigned int kBias = 1;
static const std::vector<ACTIVATION_FUNCTIONS> layers_activfuncs = {
    RELU, RELU, RELU, SIGMOID
};
static const bool use_constant_weight_init = false;
static const float constant_weight_init = 0;
static constexpr ts_joystick_read kZoom_mode_reset { 0.5, 0.5, 0.5 };

// Dataset memory
static char dataset_mem_[kMaxDatasets][sizeof(Dataset)];
static Dataset *dataset_[kMaxDatasets] = { nullptr };
static size_t ds_n_ = 0;

// MLP memory
static MLP<float> *mlp_[kMaxModels] = { nullptr };
static char mlp_mem_[kMaxModels][sizeof(MLP<float>)];
static size_t n_output_params_ = 0;
static MLP<float>::mlp_weights mlp_stored_weights_;
static ts_joystick_read zoom_mode_centre_ = kZoom_mode_reset;
static ts_joystick_read mlp_stored_input = kZoom_mode_reset;
std::vector<float> mlp_stored_output;
static bool randomised_state_ = false;
static bool redraw_weights_ = true;
static bool flag_zoom_in_ = false;
static float speed_ = 1.0f;
static te_expl_mode expl_mode_internal_ = expl_mode_pretrain;
static size_t nn_n_ = 0;


/******************************
 * MLP TASK
 ******************************/

queue_t *nn_paramupdate_ = nullptr;

void mlp_init(queue_t *nn_paramupdate, size_t n_params)
{
    const std::vector<size_t> layers_nodes = {
        sizeof(ts_joystick_read)/sizeof(float) + kBias,
        10, 10, 14,
        n_params
    };

    n_output_params_ = n_params;

    // Instantiate objects
    assert(kMaxDatasets == kMaxModels);
    for (unsigned int n = 0; n < kMaxModels; n++) {
        dataset_[n] = new(dataset_mem_[n]) Dataset();
        mlp_[n] = new (mlp_mem_[n]) MLP<float>(
            layers_nodes,
            layers_activfuncs,
            loss::LOSS_MSE,
            use_constant_weight_init,
            constant_weight_init
        );
    }

    // Instantiate channels
    nn_paramupdate_ = nn_paramupdate;

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

    MLP<float>::training_pair_t dataset(dataset_[ds_n_]->GetFeatures(), dataset_[ds_n_]->GetLabels());

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
    if (redraw_weights_) {
        mlp_[nn_n_]->DrawWeights();
        Serial.println("MLP- Weights randomised.");
        redraw_weights_ = false;
    } else {
        if (expl_mode_internal_ == expl_mode_zoom) {

            flag_zoom_in_ = true;
            zoom_mode_centre_ = mlp_stored_input;

            // Debug
            char serial_data[256];
            std::sprintf(
                serial_data,
                "MLP- Moving with speed %f%% around (%.3f, %.3f, %.3f)",
                speed*100.f, zoom_mode_centre_.potX, zoom_mode_centre_.potY,
                zoom_mode_centre_.potRotate);
            Serial.println(serial_data);

        } else if (expl_mode_internal_ == expl_mode_pretrain) {

            zoom_mode_centre_ = kZoom_mode_reset;
            flag_zoom_in_ = true;
            // Train network with only one data point at the centre
            mlp_pretrain_centre_();
            Serial.print("MLP- Pretrained on centre (speed ");
            Serial.print(speed*100.f);
            Serial.println("%).");

        } else if (expl_mode_internal_ == expl_mode_nnweights) {

            zoom_mode_centre_ = kZoom_mode_reset;
            // Randomise weights less ("move" by speed)
            mlp_[nn_n_]->MoveWeights(speed);
            Serial.print("MLP- Weights moved ");
            Serial.print(speed*100.f);
            Serial.println("%.");

        }
    }
}

void mlp_add_data_point(const std::vector<float> &in, const std::vector<float> &out)
{
    dataset_[ds_n_]->Add(in, out);
    mlp_trigger_redraw_();
}

void mlp_clear_data()
{
    dataset_[ds_n_]->Clear();
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
    std::vector<std::vector<float>> features {
        {0.5f, 0.5f, 0.5f, 1.}  // with bias
    };
    std::vector<std::vector<float>> labels {
        mlp_stored_output
    };
    MLP<float>::training_pair_t dataset(features, labels);

    // Re-init weights
    mlp_[nn_n_]->DrawWeights();
    // Train with one point at centre
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
    expl_mode_internal_ = mode;
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

void mlp_inference_nochannel(ts_joystick_read joystick_read) {

    // Function to zoom and offset by given range
    static const auto zoom_in_ = [](float x, float move_by, char coord) {
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
        joystick_read.potX = zoom_in_(joystick_read.potX, zoom_mode_centre_.potX, 'x');
        joystick_read.potY = zoom_in_(joystick_read.potY, zoom_mode_centre_.potY, 'y');
        joystick_read.potRotate = zoom_in_(joystick_read.potRotate, zoom_mode_centre_.potRotate, 'z');
    }

        // Store current joystick read
        mlp_stored_input = joystick_read;

    // Instantiate data in/out
    std::vector<num_t> input{
        joystick_read.potX,
        joystick_read.potY,
        joystick_read.potRotate,
        1.f  // bias
    };
    std::vector<num_t> output(n_output_params_);
    // Run model
    mlp_[nn_n_]->GetOutput(input, &output);

    // TODO apply transform here if set

    mlp_stored_output = output;

    // Send result
    queue_try_add(
        nn_paramupdate_,
        reinterpret_cast<void *>(mlp_stored_output.data())
    );
}
