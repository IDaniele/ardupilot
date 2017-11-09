#pragma once

#include "AP_Beacon_Backend.h"
#include <AP_TDoA/AP_TDoA.h>
#include <AP_HAL/AP_HAL.h>



extern const AP_HAL::HAL& hal;

class AP_Beacon_TDoA : public AP_Beacon_Backend
{
public:
    // constructor
    AP_Beacon_TDoA(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    bool healthy(){return true;}

    // update
    void update();

private:

    static AP_TDoA *_tdoa;
};
