#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <utility>
#include "Definitions.h"

class AP_TDoA_Backend;

class AP_TDoA{
public:
    friend class AP_TDoA_Backend;
    AP_TDoA();
    void init();

private:
    AP_TDoA_Backend *_driver;
};
