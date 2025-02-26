#ifndef __MLP_TASK_HPP__
#define __MLP_TASK_HPP__

#include "../common/common_defs.h"
#include "../PicoDefs.hpp"

#include <vector>
#include <memory>

#include "MIDI.hpp"

#include "pico/util/queue.h"


using input_data_t = std::vector<float>;

void mlp_init(queue_t *nn_paramupdate,
        size_t n_inputs,
        size_t n_params,
        size_t n_inputbuffer,
        std::shared_ptr<MIDIDevice> _midiDev = nullptr);
void mlp_inference(input_data_t joystick_read);
void mlp_train();
void mlp_draw(float speed = 0.01);
void mlp_add_data_point(const std::vector<float> &in, const std::vector<float> &out);
void mlp_add_data_point_tdnn(const std::vector<float> &in, const std::vector<float> &out);
void mlp_clear_data();
void mlp_clear_model();
void mlp_set_speed(float speed);
void mlp_set_expl_mode(te_expl_mode mode = gAppState.current_expl_mode);
void mlp_set_model_idx(size_t idx);
void mlp_set_dataset_idx(size_t idx);

extern std::vector<float> mlp_stored_output;

#endif  // __MLP_TASK_HPP__