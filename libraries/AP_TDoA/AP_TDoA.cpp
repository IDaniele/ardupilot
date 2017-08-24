#include "AP_TDoA.h"
#include "AP_TDoA_Backend.cpp"


//#include <AP_HAL/board/px4.h>


AP_TDoA::AP_TDoA() : _driver(nullptr){
    hal.console->printf("\n AP_TDoA.cpp constructor \n");
}

void AP_TDoA::init(){
    hal.console->printf("\n AP_TDoA.cpp  init \n");

    _dev = hal.spi->get_device("dwm1000");
    if (!_dev) {
        hal.console->printf("\n AP_TDoA.cpp init DEV NOT FOUND \n");
        return;
    }
    _dev->set_read_flag(0x00);
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    _sem = _dev->get_semaphore();
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA.cpp init SEMAPHORE ERROR \n");
        return;
    }

    uint32_t id = 0x00000000;
    if(!_dev->read_registers(DEV_ID, (uint8_t*)&id, LEN_DEV_ID)){
        hal.console->printf("\n AP_TDoA.cpp init READ ERROR \n");
        _sem->give();
        return;
    }
    if(id != DWM1000_ID){
        hal.console->printf("\n AP_TDoA.cpp init ID ERROR \n ID = %08x \n", id);
        _dev->get_semaphore()->give();
        return;
    }
    _sem->give();
    hal.console->printf("\n AP_TDoA.cpp init FINISHED \n");
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

void AP_TDoA::conf_dwm(){
    if (!_dev) {
        hal.console->printf("\n AP_TDoA.cpp conf_dwm DEV NOT FOUND \n");
        return;
    }
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA.cpp conf_dwm SEMAPHORE ERROR \n");
        return;
    }

    uint32_t id = 0x00000000;
    //value to set sys_cfg register
    uint8_t send1[5] = {0x84, 0x00, 0x12, 0x00, 0x20};
    //value to set sys_ctrl register.
    //Can't check to verify as it's automatically cleared to 0 by the DWM1000
    uint8_t send2[5] = {0x8D, 0x00, 0x01, 0x00, 0x00};
    if(!_dev->transfer(send1,5,nullptr,0)){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm WRITE ERROR \n");
            _sem->give();
            return;
    }
    if(!_dev->read_registers(SYS_CFG, (uint8_t*)&id, LEN_SYS_CFG)){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm READ ERROR \n");
        _sem->give();
        return;
    }
    if(id != SET_SYS_CFG){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm WRONG VALUE \n set_sys_cfg= %08x \n", id);
        _sem->give();
        return;
    }
    hal.console->printf("\n sys_cfg = %08x \n", id);

    if(!_dev->transfer(send2,5,nullptr,0)){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm WRITE ERROR \n");
            _sem->give();
            return;
    }
    if(!_dev->read_registers(SYS_CTRL, (uint8_t*)&id, LEN_SYS_CTRL)){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm READ ERROR \n");
        _sem->give();
        return;
    }
    _sem->give();
}
