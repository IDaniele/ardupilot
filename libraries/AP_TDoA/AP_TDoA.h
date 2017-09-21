#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <utility>
#include <math.h>
#include "Definitions.h"

class AP_TDoA_Backend;

class AP_TDoA{
private:
    dwDevice_t* dwDev;
    uint64_t* prevPackTime;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_sem;
public:
    friend class AP_TDoA_Backend;
    AP_TDoA();
    void init();
    void initDwDev();
    void dwSoftReset();
    void conf_dwm();
    void set_sys_reg();
    void enableAllLeds();
    void newConfig();
    void committConf();
    void tune();
    void getTime(dwTime_t* time);
    void loop();
    void dwEnableClock(dwClock_t clock);
    void newReceive();
    void dw_read(uint8_t reg, uint8_t* val, uint8_t len);
    void dw_read(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len);
    void dw_write(uint8_t reg, uint8_t* val, uint8_t len);
    void dw_write(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len);
    void dummy(void);

    void enqueue(uint8_t anchor, packet_t newpack,uint64_t rxtime);
    void tdoa();
};
