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
        update_bcn_pos();
        this->_tdoa->init();
        this->_tdoa->conf_dwm();
    }
}

// update the state of the sensor
void AP_Beacon_TDoA::update(void)
{
    this->_tdoa->enqueue();
    float nonlosoancora = 1.0f;
    set_vehicle_position(this->_tdoa->AP_TDoA::getEstimatedPosition(), nonlosoancora);
}

void AP_Beacon_TDoA::update_bcn_pos(void)
{
    float bpos[7*3];
    bpos[0] = get_beacon1_posx();   bpos[1] = get_beacon1_posy();   bpos[2] = get_beacon1_posz();
    bpos[3] = get_beacon2_posx();   bpos[4] = get_beacon2_posy();   bpos[5] = get_beacon2_posz();
    bpos[6] = get_beacon3_posx();   bpos[7] = get_beacon3_posy();   bpos[8] = get_beacon3_posz();
    bpos[9] = get_beacon4_posx();   bpos[10] = get_beacon4_posy();   bpos[11] = get_beacon4_posz();
    bpos[12] = get_beacon5_posx();   bpos[13] = get_beacon5_posy();   bpos[14] = get_beacon5_posz();
    bpos[15] = get_beacon6_posx();   bpos[16] = get_beacon6_posy();   bpos[17] = get_beacon6_posz();
    bpos[18] = get_beacon7_posx();   bpos[19] = get_beacon7_posy();   bpos[20] = get_beacon7_posz();
    this->_tdoa->setAnchorPosition(bpos);
}
