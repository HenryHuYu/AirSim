#pragma once

#include <vector>
#include <algorithm>
#include "Params.hpp"
#include "interfaces/CommonStructs.hpp"

namespace simple_flight
{

class Mixer
{
public:
    Mixer(const Params* params)
        : params_(params)
    {
    }

    void getMotorOutput(const Axis4r& controls, std::vector<float>& motor_outputs) const
    {
        const float throttle = std::max(params_->motor.min_motor_output,
                                        std::min(controls.throttle(), params_->motor.max_motor_output));

        if (throttle < params_->motor.min_angling_throttle) {
            motor_outputs.assign(params_->motor.motor_count, throttle);
            return;
        }

        float attitude_mix[kMotorCount];
        float attitude_scale = 1.0f;
        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index) {
            attitude_mix[motor_index] =
                controls.pitch() * mixerQuadX[motor_index].pitch + controls.roll() * mixerQuadX[motor_index].roll + controls.yaw() * mixerQuadX[motor_index].yaw;

            if (attitude_mix[motor_index] < 0) {
                attitude_scale = std::min(attitude_scale, (throttle - params_->motor.min_motor_output) / -attitude_mix[motor_index]);
            }
            else if (attitude_mix[motor_index] > 0) {
                attitude_scale = std::min(attitude_scale, (params_->motor.max_motor_output - throttle) / attitude_mix[motor_index]);
            }
        }
        attitude_scale = std::max(0.0f, std::min(attitude_scale, 1.0f));

        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index) {
            motor_outputs[motor_index] =
                throttle * mixerQuadX[motor_index].throttle + attitude_mix[motor_index] * attitude_scale;
        }

        for (int motor_index = 0; motor_index < kMotorCount; ++motor_index)
            motor_outputs[motor_index] = std::max(params_->motor.min_motor_output,
                                                  std::min(motor_outputs[motor_index], params_->motor.max_motor_output));
    }

private:
    static constexpr int kMotorCount = 4;

    const Params* params_;

    // Custom mixer data per motor
    typedef struct motorMixer_t
    {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } motorMixer_t;

    //only thing that this matrix does is change the sign
    const motorMixer_t mixerQuadX[4] = {
        //QuadX config
        { 1.0f, -1.0f, 1.0f, 1.0f }, // FRONT_R
        { 1.0f, 1.0f, -1.0f, 1.0f }, // REAR_L
        { 1.0f, 1.0f, 1.0f, -1.0f }, // FRONT_L
        { 1.0f, -1.0f, -1.0f, -1.0f }, // REAR_R
    };
};

} //namespace