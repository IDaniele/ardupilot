#include <AP_HAL/AP_HAL.h>
#include "AP_Beacon_TDoA.h"
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Beacon_TDoA::AP_Beacon_TDoA(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    if(_tdoa == nullptr){
        _tdoa = new AP_TDoA;
        _tdoa->init();
        _tdoa->conf_dwm();
    }
    _tdoa->setAnchorPosition(&_frontend.beacon1_pos, &_frontend.beacon2_pos, &_frontend.beacon3_pos,
                             &_frontend.beacon4_pos, &_frontend.beacon5_pos, &_frontend.beacon6_pos, &_frontend.beacon7_pos);
}

// update the state of the sensor
void AP_Beacon_TDoA::update(void)
{
    _tdoa->enqueue();
    float nonlosoancòra = 1.0f;
    set_vehicle_position(_tdoa->getEstimatedPosition(), nonlosoancòra);
}