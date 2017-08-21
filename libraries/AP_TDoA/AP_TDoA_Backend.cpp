#include "AP_TDoA_Backend.h"

AP_TDoA_Backend::AP_TDoA_Backend(AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev){
    hal.console->printf("\n AP_TDoA_Backend.cpp constructor \n");

    if (!dev) {
        hal.console->printf("\n AP_TDoA_Backend.cpp constructor DEV NOT FOUND \n");
        return;
    }
    _dev = dev;
    _dev->set_read_flag(0x00);
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    _sem = _dev->get_semaphore();
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA_Backend.cpp constructor SEMAPHORE ERROR \n");
        return;
    }

    uint32_t id = 0x00000000;
    if(!_dev->read_registers(DEV_ID, (uint8_t*)&id, LEN_DEV_ID)){
        hal.console->printf("\n AP_TDoA_Backend.cpp constructor READ ERROR \n");
        _dev->get_semaphore()->give();
        return;
    }
    if(id != DWM1000_ID){
        hal.console->printf("\n AP_TDoA_Backend.cpp constructor ID ERROR \n ID = %08x \n", id);
        _dev->get_semaphore()->give();
        return;
    }
    _sem->give();
    hal.console->printf("\n AP_TDoA_Backend.cpp constructor FINISHED");
}


bool AP_TDoA_Backend::conf_dwm(){
    hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm \n");
    hal.scheduler->delay(1000);

    if (!_dev) {
        hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm DEV NOT FOUND \n");
        return false;
    }
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm SEMAPHORE ERROR \n");
        return false;
    }

    uint32_t id = 0x00000000;
//*
    if(!_dev->read_registers(DEV_ID, (uint8_t*)&id, LEN_DEV_ID)){
        hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm READ ERROR \n");
        _sem->give();
        return false;
    }
//*/
    hal.console->printf("\n sys_cfg = %08x \n", id);
/*
    uint8_t send[5] = {0x84, 0x00, 0x12, 0x00, 0x20};
    if(!_dev->transfer(send,5,nullptr,0)){
        hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm WRITE ERROR \n");
            _sem->give();
            return false;
    }
    uint32_t id = 0x00000000;
    if(!_dev->read_registers(SYS_CFG, (uint8_t*)&id, LEN_SYS_CFG)){
        hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm READ ERROR \n");
        _sem->give();
        return false;
    }
    if(id != SET_SYS_CFG){
        hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm WRONG VALUE \n set_sys_cfg= %08x \n", id);
        _sem->give();
        return false;
    }
//*/
    _sem->give();
    hal.console->printf("\n AP_TDoA_Backend.cpp conf_dwm ALL SET \n");
    return true;
}
