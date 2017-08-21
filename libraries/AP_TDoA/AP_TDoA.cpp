#include "AP_TDoA.h"
#include "AP_TDoA_Backend.cpp"


//#include <AP_HAL/board/px4.h>


AP_TDoA::AP_TDoA() : _driver(nullptr){
    hal.console->printf("\n AP_TDoA.cpp constructor \n");
}

void AP_TDoA::init(){
    hal.console->printf("\n AP_TDoA.cpp  init \n");

    _driver = new AP_TDoA_Backend(hal.spi->get_device("dwm1000"));
    if(!_driver->conf_dwm()){
        hal.console->printf("\n AP_TDoA.cpp  CONF ERROR \n");
    }
    hal.scheduler->delay(1000);
}
