#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>
#include <utility>

#include "Definitions.h"

using namespace std;

static const int DIM_QUEUE = 10;

class AP_TDoA_Backend;

class AP_TDoA{
private:
    dwDevice_t* dwDev;
    uint64_t* prevPackTime;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::Semaphore *_sem;

    recvData recvQueue[DIM_QUEUE][LOCODECK_NR_OF_ANCHORS];
    uint8_t currentQueue;

    static AP_Vector3f estimatedPosition;

    static double _anchorPosition[LOCODECK_NR_OF_ANCHORS][3];

public:
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

    void setAnchorPosition(AP_Vector3f *b1, AP_Vector3f *b2, AP_Vector3f *b3, AP_Vector3f *b4, AP_Vector3f *b5, AP_Vector3f *b6, AP_Vector3f *b7){
        memset(_anchorPosition[0],  0, 3*sizeof(float));
        memcpy(_anchorPosition[1], b1, 3*sizeof(float));
        memcpy(_anchorPosition[2], b2, 3*sizeof(float));
        memcpy(_anchorPosition[3], b3, 3*sizeof(float));
        memcpy(_anchorPosition[4], b4, 3*sizeof(float));
        memcpy(_anchorPosition[5], b5, 3*sizeof(float));
        memcpy(_anchorPosition[6], b6, 3*sizeof(float));
        memcpy(_anchorPosition[7], b7, 3*sizeof(float));
    }
    AP_Vector3f getEstimatedPosition(){return this->estimatedPosition;}

    //void enqueue(uint8_t anchor, packet_t newpack,uint64_t rxtime);
    void enqueue();
    void tdoa(int8_t readyQueue);
    void minimize(double b[], double teta0[],int m, double r1);
};
