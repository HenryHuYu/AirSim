// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_vehicles_SimpleFlightQuadX_hpp
#define msr_airlib_vehicles_SimpleFlightQuadX_hpp

#include "vehicles/multirotor/firmwares/simple_flight/SimpleFlightApi.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class SimpleFlightQuadXParams : public MultiRotorParams
    {
    public:
        SimpleFlightQuadXParams(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<const SensorFactory> sensor_factory)
            : vehicle_setting_(vehicle_setting), sensor_factory_(sensor_factory)
        {
        }

        virtual ~SimpleFlightQuadXParams() = default;

        virtual std::unique_ptr<MultirotorApiBase> createMultirotorApi() override
        {
            return std::unique_ptr<MultirotorApiBase>(new SimpleFlightApi(this, vehicle_setting_));
        }

    protected:
        virtual void setupParams() override
        {
            auto& params = getParams();

            // Use connection_info_.model for the model name, see Px4MultiRotorParams for example

            // Only Generic for now
            if (vehicle_setting_->mass < 0.1) {
                std::cout << "using roma quad init " << std::endl;
                setupFrameRomaQuad(params);
                return;
            }
            else {
                std::cout << "using settings init, max thrust is  " << vehicle_setting_->max_thrust << std::endl;
                params.arm_length = vehicle_setting_->arm_length;
                params.mass = vehicle_setting_->mass;
                params.linear_drag_coefficient = vehicle_setting_->linear_drag_coefficient;

                params.body_box.x() = vehicle_setting_->box_x;
                params.body_box.y() = vehicle_setting_->box_y;
                params.body_box.z() = vehicle_setting_->box_z;

                params.rotor_params.max_thrust = vehicle_setting_->max_thrust;
                params.rotor_params.max_torque = vehicle_setting_->max_torque;
                params.rotor_params.calculateMaxSpeed();

                setupFrameQuadFromSettings(params);
            }
        }

        virtual const SensorFactory* getSensorFactory() const override
        {
            return sensor_factory_.get();
        }

    private:
        const AirSimSettings::VehicleSetting* vehicle_setting_; //store as pointer because of derived classes
        std::shared_ptr<const SensorFactory> sensor_factory_;
    };
}
} //namespace
#endif
