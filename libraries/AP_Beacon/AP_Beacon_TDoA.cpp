#include "AP_Beacon_TDoA.h"
#include <ctype.h>
#include <stdio.h>

AP_TDoA *AP_Beacon_TDoA::_tdoa;

extern const AP_HAL::HAL& hal;

// constructor
AP_Beacon_TDoA::AP_Beacon_TDoA(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend)
{
    if(this->_tdoa == nullptr){
        this->_tdoa = new AP_TDoA;
        this->_tdoa->init();
        this->_tdoa->conf_dwm();
    }
    /*_tdoa->setAnchorPosition(&_frontend.beacon1_pos, &_frontend.beacon2_pos, &_frontend.beacon3_pos,
                             &_frontend.beacon4_pos, &_frontend.beacon5_pos, &_frontend.beacon6_pos, &_frontend.beacon7_pos);
    */
}

// update the state of the sensor
void AP_Beacon_TDoA::update(void)
{
    this->_tdoa->enqueue();
    float nonlosoancora = 1.0f;
    set_vehicle_position(this->AP_Beacon_TDoA::_tdoa->AP_TDoA::getEstimatedPosition(), nonlosoancora);
}
