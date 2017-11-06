#pragma once

#include "AP_Beacon_Backend.h"
#include "AP_TDoA/AP_TDoA.h"

class AP_Beacon_TDoA : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_TDoA(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // update
    void update();

private:

    static AP_TDoA* _tdoa = nullptr;
};
