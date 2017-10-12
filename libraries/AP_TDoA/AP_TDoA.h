#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <utility>

#include "Definitions.h"

using namespace std;

class AP_TDoA_Backend;

class AP_TDoA{
private:
    dwDevice_t* dwDev;
    uint64_t* prevPackTime;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_sem;

    recvData recvQueue[2][LOCODECK_NR_OF_ANCHORS];
    uint8_t currentQueue;

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
    void loopDouble();
    void getTime(dwTime_t* time);
    void loop();
    void loopblock();
    void dwEnableClock(dwClock_t clock);
    void newReceive();
    void printpos();
    void dw_read(uint8_t reg, uint8_t* val, uint8_t len);
    void dw_read(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len);
    void dw_write(uint8_t reg, uint8_t* val, uint8_t len);
    void dw_write(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len);
    void dummy(void);
    void prova();

    //void enqueue(uint8_t anchor, packet_t newpack,uint64_t rxtime);
    void enqueue();
    void tdoa();
    void minimize(double b[], double teta0[],int m, double r1);
};
