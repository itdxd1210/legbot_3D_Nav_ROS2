//
// Created by biao on 24-10-6.
//

#include "ObservationBuffer.h"
#include <cstring>
#include <stdexcept>

ObservationBuffer::ObservationBuffer(int num_envs,
                                     const int num_obs,
                                     const int include_history_steps)
    : num_envs_(num_envs),
      num_obs_(num_obs),
      include_history_steps_(include_history_steps) {
    num_obs_total_ = num_obs_ * include_history_steps_;
    obs_buffer_ = torch::zeros({num_envs_, num_obs_total_}, dtype(torch::kFloat32));
    frame_major_buffer_ = torch::zeros_like(obs_buffer_);
    term_major_buffer_ = torch::zeros_like(obs_buffer_);
}

void ObservationBuffer::reset(const std::vector<int> &reset_index, const torch::Tensor &new_obs) {
    std::vector<torch::indexing::TensorIndex> indices;
    for (int index: reset_index) {
        indices.emplace_back(torch::indexing::Slice(index));
    }
    obs_buffer_.index_put_(indices, new_obs.repeat({1, include_history_steps_}));
}

void ObservationBuffer::clear()
{
    obs_buffer_.zero_();
    frame_major_buffer_.zero_();
    term_major_buffer_.zero_();
}

void ObservationBuffer::insert(const torch::Tensor &new_obs) {
    if (new_obs.device().is_cuda() || new_obs.scalar_type() != torch::kFloat32 ||
        !new_obs.is_contiguous() || new_obs.dim() != 2 ||
        new_obs.size(0) != num_envs_ || new_obs.size(1) != num_obs_)
        throw std::runtime_error("ObservationBuffer requires contiguous CPU float32 [env, obs] input");
    float* buffer = obs_buffer_.data_ptr<float>();
    const float* input = new_obs.data_ptr<float>();
    const size_t shifted_bytes = static_cast<size_t>(num_obs_total_ - num_obs_) * sizeof(float);
    const size_t frame_bytes = static_cast<size_t>(num_obs_) * sizeof(float);
    for (int env = 0; env < num_envs_; ++env) {
        float* row = buffer + env * num_obs_total_;
        std::memmove(row, row + num_obs_, shifted_bytes);
        std::memcpy(row + num_obs_total_ - num_obs_, input + env * num_obs_, frame_bytes);
    }
}

torch::Tensor ObservationBuffer::getObsVec(const std::vector<int> &obs_ids) const {
    if (obs_ids.empty() || static_cast<int>(obs_ids.size()) > include_history_steps_)
        throw std::runtime_error("Invalid frame-major history selection");
    const float* source = obs_buffer_.data_ptr<float>();
    float* output = frame_major_buffer_.data_ptr<float>();
    int output_frame = 0;
    for (int i = static_cast<int>(obs_ids.size()) - 1; i >= 0; --i, ++output_frame) {
        const int obs_id = obs_ids[static_cast<size_t>(i)];
        if (obs_id < 0 || obs_id >= include_history_steps_)
            throw std::runtime_error("Frame-major history index is out of range");
        const int slice_idx = include_history_steps_ - obs_id - 1;
        for (int env = 0; env < num_envs_; ++env) {
            std::memcpy(
                output + env * num_obs_total_ + output_frame * num_obs_,
                source + env * num_obs_total_ + slice_idx * num_obs_,
                static_cast<size_t>(num_obs_) * sizeof(float));
        }
    }
    return frame_major_buffer_.narrow(1, 0, static_cast<int64_t>(obs_ids.size()) * num_obs_);
}

torch::Tensor ObservationBuffer::getTermMajorObsVec(const std::vector<int>& term_dims) const
{
    int term_total = 0;
    for (const int dim : term_dims)
    {
        if (dim <= 0)
            throw std::runtime_error("Observation term dimensions must be positive");
        term_total += dim;
    }
    if (term_total != num_obs_)
        throw std::runtime_error("Observation term dimensions do not match one policy frame");

    const float* source = obs_buffer_.data_ptr<float>();
    float* output = term_major_buffer_.data_ptr<float>();
    int term_offset = 0;
    int output_offset = 0;
    for (const int dim : term_dims)
    {
        for (int frame = 0; frame < include_history_steps_; ++frame)
        {
            const int begin = frame * num_obs_ + term_offset;
            for (int env = 0; env < num_envs_; ++env) {
                std::memcpy(
                    output + env * num_obs_total_ + output_offset + frame * dim,
                    source + env * num_obs_total_ + begin,
                    static_cast<size_t>(dim) * sizeof(float));
            }
        }
        output_offset += include_history_steps_ * dim;
        term_offset += dim;
    }
    return term_major_buffer_;
}
