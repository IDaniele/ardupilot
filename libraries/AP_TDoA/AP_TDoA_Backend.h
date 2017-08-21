#pragma once

#include "AP_TDoA.h"

class AP_TDoA_Backend
{
public:
    AP_TDoA_Backend(AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev);
    bool conf_dwm();
private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_sem;
};
