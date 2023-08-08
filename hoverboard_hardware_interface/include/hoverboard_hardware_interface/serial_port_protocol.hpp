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

#include <cstdint>

#define HEAD_FRAME 0xABCD

// enum class MOTOR_STATES {
//     UNOCUPPIED = 0b00,
//     RUN = 0b01,
//     BRAKE = 0b11,
//     LOCK_SHAFT = 0b10,
// };

typedef struct {
    uint16_t head;
    int16_t  command1;
    int16_t  command2;
    int16_t  rightMotorSpeed;
    int16_t  leftMotorSpeed;
    int16_t  rightMotorEncoderCumulativeCount;
    int16_t  leftMotorEncoderCumulativeCount;
    int16_t  batteryVoltage;
    int16_t  boardTemperature;
    uint16_t commandLed;
    uint16_t checksum;
} MotorWheelFeedback;

typedef struct {
    uint16_t head;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} MotorWheelDriveControl;
