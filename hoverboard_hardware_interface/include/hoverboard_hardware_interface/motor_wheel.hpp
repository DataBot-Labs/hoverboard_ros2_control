// Copyright 2023 Robert Gruberski (Viola Robotics Sp. z o.o. Poland)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>
#include <cmath>

#define ENCODER_MIN_VALUE 0
#define ENCODER_MAX_VALUE 9000
#define ENCODER_LOW_WRAP_FACTOR 0.3
#define ENCODER_HIGH_WRAP_FACTOR 0.7

namespace hoverboard_hardware_interface
{
    class MotorWheel {
    public:
        std::string name = "";

        int64_t encoderTicks = 0;
        int16_t encoderTicksPrevious = 0;
        int16_t encoderOverflowCount = 0;

        int16_t encoderLowWrap;
        int16_t encoderHighWrap;

        double command = 0.0;
        double position = 0.0;
        double velocity = 0.0;

        double radiansPerRevolution = 0.0;

        MotorWheel() = default;

        MotorWheel(const std::string &wheelJointName, int encoderTicksPerRevolution)
        {
            name = wheelJointName;
            radiansPerRevolution = ((2 * M_PI) / encoderTicksPerRevolution);

            encoderLowWrap = ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE;
            encoderHighWrap = ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX_VALUE - ENCODER_MIN_VALUE) + ENCODER_MIN_VALUE;
        }

        double calculateEncoderAngle()
        {
            return encoderTicks * radiansPerRevolution;
        }

        void updateEncoderTicks(int16_t newTicks)
        {
            if(newTicks < encoderLowWrap && encoderTicksPrevious > encoderHighWrap)
            {
                encoderOverflowCount++;
            }

            if(newTicks > encoderHighWrap && encoderTicksPrevious < encoderLowWrap)
            {
                encoderOverflowCount--;
            }

            encoderTicks = (encoderOverflowCount * ENCODER_MAX_VALUE) + newTicks;
            encoderTicksPrevious = newTicks;
        }

    private:

    };
}
