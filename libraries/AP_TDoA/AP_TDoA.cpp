#include "AP_TDoA.h"

using namespace std;

//*
# include "qr_solve.h"
# include "r8lib.h"
//*/

static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];
static double frameTime_in_cl_A[LOCODECK_NR_OF_ANCHORS];
static double clockCorrection_T_To_A[LOCODECK_NR_OF_ANCHORS];
static uint8_t previousAnchor;

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val);
static bool getBit(uint8_t data[], unsigned int n, unsigned int bit);
static void writeValueToBytes(uint8_t data[], long val, unsigned int n);

static bool ongoing;
static bool received[LOCODECK_NR_OF_ANCHORS];
static float distdiff[LOCODECK_NR_OF_ANCHORS];

static uint64_t timestampToUint64(const uint8_t *ts) {
  dwTime_t timestamp;
  memset(&timestamp,0,sizeof(timestamp));
  memcpy(timestamp.raw, ts, sizeof(timestamp.raw));

  return timestamp.full;
}

static double calcClockCorrection(const double frameTime, const double previuosFrameTime) {
    double clockCorrection = 1.0;

    if (frameTime != 0.0) {
      clockCorrection = previuosFrameTime / frameTime;
    }

    return clockCorrection;
}

static uint64_t truncateToTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFFFul;
}

static bool isValidRxTime(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static int numnotnull(bool vect[], int size){
    uint8_t count = 0;
    for(int i = 0; i < size; i++){
        if(vect[i]){
            count++;
        }
    }
    return count;
}

AP_TDoA::AP_TDoA() : _dev(nullptr), _sem(nullptr){
    hal.console->printf("\n AP_TDoA.cpp constructor \n");
    dwDev = new dwDevice_t;
    memset(dwDev,0,sizeof(*dwDev));
    prevPackTime = new uint64_t;
    *prevPackTime = 0;

    previousAnchor = 0; // I think initializing to 0 is ok
    memset(rxPacketBuffer, 0, sizeof(rxPacketBuffer));
    memset(arrivals, 0, sizeof(arrivals));
    memset(frameTime_in_cl_A, 0, sizeof(frameTime_in_cl_A));
    memset(clockCorrection_T_To_A, 0, sizeof(clockCorrection_T_To_A));
    memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));
    ongoing = false;
    memset(received, 0, sizeof(received));
    memset(distdiff, 0, sizeof(distdiff));
}

void AP_TDoA::init(){
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
    hal.console->printf("\n AP_TDoA.cpp DEV_ID CHECKED \n");
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    if(_dev->register_periodic_callback( 1000000, FUNCTOR_BIND_MEMBER(&AP_TDoA::dummy, void)) == nullptr){
        hal.console->printf("\n callback    nullptr \n");
    }
}

void AP_TDoA::dummy(void){
    hal.console->printf("\n hello world \n");
}

void AP_TDoA::loop(){
    //hal.console->printf("\n loop \n");
    //*
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA.cpp loop SEMAPHORE ERROR \n");
        return;
    }
    //*/

    dw_read(SYS_STATUS, NO_SUB, dwDev->sysstatus, LEN_SYS_STATUS);
    if(getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXDFR_BIT)){
        uint64_t* sysst = new uint64_t;
        memcpy(sysst,&(dwDev->sysstatus[0]), LEN_SYS_STATUS);

        dwTime_t* time = new dwTime_t;
        _dev->read_registers(RX_TIME, time->raw, 5);
        getTime(time);
        uint64_t rxtime = time->full;


        packet_t newpack;
        _dev->read_registers(RX_BUFFER, (uint8_t*)&newpack, sizeof(newpack));
        uint8_t anchor = newpack.sourceAddress & 0xff;
        //hal.console->printf("\n\n SYS_STATUS = %010llx", *sysst);
        //hal.console->printf("\n RX_TIME = %016llx", rxtime);
        //hal.console->printf("\n ANCHOR = %02u", anchor);
        //hal.console->printf("\n TX_TIME = %016llx \n", txtime);

        enqueue(anchor, newpack, rxtime);
        //


        delete time;
        delete sysst;
        newReceive();
    }
    else{
        if( getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXRFTO_BIT) || getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXRFTO_BIT) ){
        newReceive();
        }
        else{
            bool ldeErr, rxCRCErr, rxHeaderErr, rxDecodeErr;
            ldeErr = getBit(dwDev->sysstatus, LEN_SYS_STATUS, LDEERR_BIT);
            rxCRCErr = getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXFCE_BIT);
            rxHeaderErr = getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXPHE_BIT);
            rxDecodeErr = getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXRFSL_BIT);
            if(ldeErr || rxCRCErr || rxHeaderErr || rxDecodeErr) {
                newReceive();
            }
        }
    }
    _sem->give();
}

void AP_TDoA::enqueue(uint8_t anchor, packet_t newpack,uint64_t rxtime){
    if (anchor < LOCODECK_NR_OF_ANCHORS) {
        rangePacket_t* packet = (rangePacket_t*)newpack.payload;
        const int64_t previous_rxAn_by_T_in_cl_T  = arrivals[anchor].full;
        const int64_t rxAn_by_T_in_cl_T = rxtime;
        const int64_t previous_txAn_in_cl_An = timestampToUint64(rxPacketBuffer[anchor].timestamps[anchor]);
        const int64_t txAn_in_cl_An = timestampToUint64(packet->timestamps[anchor]);


        if (anchor != 0) {
            //  to me Ar is 0
            const int64_t previuos_rxAr_by_An_in_cl_An = timestampToUint64(rxPacketBuffer[anchor].timestamps[0]);
            const int64_t rxAr_by_An_in_cl_An = timestampToUint64(packet->timestamps[0]);
            const int64_t rxAn_by_Ar_in_cl_Ar = timestampToUint64(rxPacketBuffer[0].timestamps[anchor]);

            if (isValidRxTime(previuos_rxAr_by_An_in_cl_An) && isValidRxTime(rxAr_by_An_in_cl_An) && isValidRxTime(rxAn_by_Ar_in_cl_Ar)) {
                //statsAcceptedAnchorDataPackets++;

                // Caclculate clock correction from anchor to reference anchor
                const double frameTime_in_cl_An = truncateToTimeStamp(rxAr_by_An_in_cl_An - previuos_rxAr_by_An_in_cl_An);
                const double clockCorrection_An_To_Ar = calcClockCorrection(frameTime_in_cl_An, frameTime_in_cl_A[0]);

                const int64_t rxAr_by_T_in_cl_T  = arrivals[0].full;
                const int64_t txAr_in_cl_Ar = timestampToUint64(rxPacketBuffer[0].timestamps[0]);

                // Calculate distance diff
                const int64_t tof_Ar_to_An_in_cl_Ar = (((truncateToTimeStamp(rxAr_by_An_in_cl_An - previous_txAn_in_cl_An) * clockCorrection_An_To_Ar) - truncateToTimeStamp(txAr_in_cl_Ar - rxAn_by_Ar_in_cl_Ar))) / 2.0;
                const int64_t delta_txAr_to_txAn_in_cl_Ar = (tof_Ar_to_An_in_cl_Ar + truncateToTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An) * clockCorrection_An_To_Ar);
                const int64_t timeDiffOfArrival_in_cl_Ar =  truncateToTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection_T_To_A[0] - delta_txAr_to_txAn_in_cl_Ar;

                const float tdoaDistDiff = lightspeed * timeDiffOfArrival_in_cl_Ar / LOCODECK_TS_FREQ;
                uwbTdoaDistDiff[anchor] = tdoaDistDiff;
            }
        }

        switch(anchor){
        case 0:
            if(!ongoing){
                ongoing = true;
                received[anchor] = true;
                hal.console->printf("\n session started  \n");
            }
            else{
                if(numnotnull(received, LOCODECK_NR_OF_ANCHORS) >= 4){
                    tdoa();
                }
                else{
                    hal.console->printf("\n n anchor = %d \n", numnotnull(received, LOCODECK_NR_OF_ANCHORS));
                }
                memset(received, 0, sizeof(received));
                memset(distdiff, 0, sizeof(distdiff));
                received[anchor] = true;
            }
            break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
            if(ongoing){
                if(!received[anchor]){
                    if(uwbTdoaDistDiff[anchor] != 0.0f){
                            if(uwbTdoaDistDiff[anchor] > -MAX_DISTANCE_DIFF && uwbTdoaDistDiff[anchor] < MAX_DISTANCE_DIFF){
                                distdiff[anchor] = uwbTdoaDistDiff[anchor];
                                received[anchor] = true;
                                hal.console->printf("\n time1 = %llu \n", (txAn_in_cl_An - timestampToUint64(packet->timestamps[0])) );
                            }
                    }
                }
                else{
                    if(numnotnull(received, LOCODECK_NR_OF_ANCHORS) >= 4){
                        tdoa();
                    }
                    else{
                        hal.console->printf("\n n anchor = %d \n", numnotnull(received, LOCODECK_NR_OF_ANCHORS));
                    }
                    memset(received, 0, sizeof(received));
                    memset(distdiff, 0, sizeof(distdiff));
                    ongoing = false;
                }

            }
            else{
                hal.console->printf("\n else  \n");
            }
            break;
        default:
            break;
        }

        // Calculate clock correction for tag to anchor
        const double frameTime_in_T = truncateToTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);
        frameTime_in_cl_A[anchor] = truncateToTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
        clockCorrection_T_To_A[anchor] = calcClockCorrection(frameTime_in_T, frameTime_in_cl_A[anchor]);

        arrivals[anchor].full = rxtime;
        memcpy(&rxPacketBuffer[anchor], newpack.payload, sizeof(rangePacket_t));

        previousAnchor = anchor;
    }
}

void AP_TDoA::tdoa(){
    //*
    hal.console->printf("\n tdoa %d \n", numnotnull(received, LOCODECK_NR_OF_ANCHORS));
    hal.console->printf("\n distdiff1 %f \n", distdiff[1]);
    hal.console->printf("\n distdiff2 %f \n", distdiff[2]);
    hal.console->printf("\n distdiff3 %f \n", distdiff[3]);
    hal.console->printf("\n distdiff4 %f \n", distdiff[4]);
    hal.console->printf("\n distdiff5 %f \n", distdiff[5]);
    //*/

    //*
    int m = numnotnull(received, LOCODECK_NR_OF_ANCHORS);
    int n = 3;
    int dim = m*n;
    double *a = new double[dim];
    double *b = new double[m];
    double* svd;

    int j = 0;
    bool first = false;
    int8_t ref1 = -1, ref2 = -1;
    for(int i = 1; i<LOCODECK_NR_OF_ANCHORS; i++){
        if(distdiff[i] != 0){
            if(first){
                if(ref2 < 0){
                    ref2 = i;
                }
                a[j*n+0] = (anchorPosition[i][0] / uwbTdoaDistDiff[i]) - (anchorPosition[ref1][0] / uwbTdoaDistDiff[ref1]);
                a[j*n+1] = (anchorPosition[i][1] / uwbTdoaDistDiff[i]) - (anchorPosition[ref1][1] / uwbTdoaDistDiff[ref1]);
                a[j*n+2] = (anchorPosition[i][2] / uwbTdoaDistDiff[i]) - (anchorPosition[ref1][2] / uwbTdoaDistDiff[ref1]);
                double pm, pref;
                pm = (pow(anchorPosition[i][0],2) + pow(anchorPosition[i][1],2) + pow(anchorPosition[i][2],2)) / uwbTdoaDistDiff[i];
                pref = (pow(anchorPosition[ref1][0],2) + pow(anchorPosition[ref1][1],2) + pow(anchorPosition[ref1][2],2)) / uwbTdoaDistDiff[ref1];
                b[j] = - ( (uwbTdoaDistDiff[i] - uwbTdoaDistDiff[ref1]) - (pm + pref) );
                j++;
            }else{
                first = true;
                ref1 = i;
            }
        }
    }

    a[j*n+0] = (anchorPosition[ref1][0] / uwbTdoaDistDiff[ref1]) - (anchorPosition[ref2][0] / uwbTdoaDistDiff[ref2]);
    a[j*n+1] = (anchorPosition[ref1][1] / uwbTdoaDistDiff[ref1]) - (anchorPosition[ref2][1] / uwbTdoaDistDiff[ref2]);
    a[j*n+2] = (anchorPosition[ref1][2] / uwbTdoaDistDiff[ref1]) - (anchorPosition[ref2][2] / uwbTdoaDistDiff[ref2]);
    double pm, pref;
    pm = (pow(anchorPosition[ref1][0],2) + pow(anchorPosition[ref1][1],2) + pow(anchorPosition[ref1][2],2)) / uwbTdoaDistDiff[ref1];
    pref = (pow(anchorPosition[ref2][0],2) + pow(anchorPosition[ref2][1],2) + pow(anchorPosition[ref2][2],2)) / uwbTdoaDistDiff[ref2];
    b[j] = ( uwbTdoaDistDiff[ref1] - uwbTdoaDistDiff[ref2] - (pm + pref) );

    svd = svd_solve(m, n, a, b);

    hal.console->printf("\n pos x=%lf  y=%lf  z=%lf \n", svd[0], svd[1], svd[2]);

    delete[] a;
    delete[] b;

    //*/
}

void AP_TDoA::newReceive(){
    // IDLE
    memset(dwDev->sysctrl, 0, LEN_SYS_CTRL);
    dwDev->sysctrl[0] |= 1<<TRXOFF_BIT;
    dwDev->deviceMode = IDLE_MODE;
    dw_write(SYS_CTRL, NO_SUB, dwDev->sysctrl, LEN_SYS_CTRL);

    memset(dwDev->sysctrl, 0, LEN_SYS_CTRL);

    // Clear Receive Status
    uint8_t reg[LEN_SYS_STATUS] = {0};
    setBit(reg, LEN_SYS_STATUS, RXDFR_BIT, true);
    setBit(reg, LEN_SYS_STATUS, LDEDONE_BIT, true);
    setBit(reg, LEN_SYS_STATUS, LDEERR_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXPHE_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXFCE_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXFCG_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXRFSL_BIT, true);
    setBit(reg, LEN_SYS_STATUS, RXRFTO_BIT, true);
    dw_write(SYS_STATUS, NO_SUB, reg, LEN_SYS_STATUS);

    dwDev->deviceMode = RX_MODE;

    // Start Receive
    setBit(dwDev->sysctrl, LEN_SYS_CTRL, SFCST_BIT, !dwDev->frameCheck);
    setBit(dwDev->sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
    dw_write(SYS_CTRL, NO_SUB, dwDev->sysctrl, LEN_SYS_CTRL);
}

void AP_TDoA::getTime(dwTime_t* time){
    // base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
    float C, N, twoPower17 = 131072.0f;
    dw_read(RX_FQUAL,CIR_PWR_SUB,(uint8_t*)&C,LEN_CIR_PWR);
    //dwSpiRead16(dev, RX_FQUAL, CIR_PWR_SUB)
    uint8_t rxFrameInfo[LEN_RX_FINFO];
    _dev->read_registers(RX_FINFO, rxFrameInfo, LEN_RX_FINFO);
    N = (float)((((unsigned int)rxFrameInfo[2] >> 4) & 0xFF) | ((unsigned int)rxFrameInfo[3] << 4));
    float A, corrFac;

      if(TX_PULSE_FREQ_16MHZ == dwDev->pulseFrequency) {
          A = 115.72f;
          corrFac = 2.3334f;
      } else {
          A = 121.74f;
          corrFac = 1.1667f;
      }

      float estFpPwr = 10.0f * log10f((C * twoPower17) / (N * N)) - A;

      if(estFpPwr <= -88) {
          //return estFpPwr;
      } else {
          // approximation of Fig. 22 in user manual for dbm correction
          estFpPwr += (estFpPwr + 88) * corrFac;
      }

    float rxPowerBase = -(estFpPwr + 61.0f) * 0.5f;
    if (!isfinite(rxPowerBase)) {
      return;
    }
    int rxPowerBaseLow = (int)rxPowerBase;
    int rxPowerBaseHigh = rxPowerBaseLow + 1;
    if(rxPowerBaseLow < 0) {
        rxPowerBaseLow = 0;
        rxPowerBaseHigh = 0;
    } else if(rxPowerBaseHigh > 17) {
        rxPowerBaseLow = 17;
        rxPowerBaseHigh = 17;
    }
    // select range low/high values from corresponding table
    int rangeBiasHigh = 0;
    int rangeBiasLow = 0;
    if(dwDev->channel == CHANNEL_4 || dwDev->channel == CHANNEL_7) {
        // 900 MHz receiver bandwidth
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
            rangeBiasHigh <<= 1;
            rangeBiasLow = (rxPowerBaseLow < BIAS_900_16_ZERO ? -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
            rangeBiasLow <<= 1;
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
            rangeBiasHigh <<= 1;
            rangeBiasLow = (rxPowerBaseLow < BIAS_900_64_ZERO ? -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
            rangeBiasLow <<= 1;
        } else {
            // TODO proper error handling
        }
    } else {
        // 500 MHz receiver bandwidth
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
            rangeBiasLow = (rxPowerBaseLow < BIAS_500_16_ZERO ? -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            rangeBiasHigh = (rxPowerBaseHigh < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
            rangeBiasLow = (rxPowerBaseLow < BIAS_500_64_ZERO ? -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
        } else {
            // TODO proper error handling
        }
    }
    // linear interpolation of bias values
    float rangeBias = rangeBiasLow + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
    // range bias [mm] to timestamp modification value conversion
    dwTime_t adjustmentTime;
  adjustmentTime.full = (int)(rangeBias * DISTANCE_OF_RADIO_INV * 0.001f);
    // apply correction
    time->full += adjustmentTime.full;
}

void AP_TDoA::conf_dwm(){

    // CHECK DEVICE AND TAKE SEMAPHORE
    if (!_dev) {
        hal.console->printf("\n AP_TDoA.cpp conf_dwm DEV NOT FOUND \n");
        return;
    }
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA.cpp conf_dwm SEMAPHORE ERROR \n");
        return;
    }

    //  INIT dwDev
    initDwDev();

    dwEnableClock(dwClockAuto);
    hal.scheduler->delay(5);
    //  RESET DWM1000
    dwSoftReset();
    // Set default address
    memset(dwDev->networkAndAddress, 0xff, LEN_PANADR);
    dw_write(PANADR,NO_SUB,dwDev->networkAndAddress,LEN_PANADR);
    //dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
    memset(dwDev->syscfg, 0, LEN_SYS_CFG);
    setBit(dwDev->syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, true);
    setBit(dwDev->syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, true);
    memset(dwDev->sysmask, 0, LEN_SYS_MASK);
    dw_write(SYS_MASK,NO_SUB,dwDev->sysmask,LEN_SYS_MASK);
    //dwSpiWrite(dev, SYS_MASK, NO_SUB, dev->sysmask, LEN_SYS_MASK);
    dwEnableClock(dwClockXti);
    hal.scheduler->delay(5);

    // transfer any ldo tune values
    // uint8_t ldoTune[LEN_OTP_RDAT];
    // readBytesOTP(0x04, ldoTune); // TODO #define
    // if(ldoTune[0] != 0) {
    //  // TODO tuning available, copy over to RAM: use OTP_LDO bit
    // }
    // tell the chip to load the LDE microcode
    // TODO remove clock-related code (PMSC_CTRL) as handled separately
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    uint8_t otpctrl[LEN_OTP_CTRL];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    memset(otpctrl, 0, LEN_OTP_CTRL);
    dw_read(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiRead(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    dw_read(OTP_IF,OTP_CTRL_SUB,otpctrl,LEN_OTP_CTRL);
    //dwSpiRead(dev, OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
    pmscctrl0[0] = 0x01;
    pmscctrl0[1] = 0x03;
    otpctrl[0] = 0x00;
    otpctrl[1] = 0x80;
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    dw_write(OTP_IF,OTP_CTRL_SUB,otpctrl,LEN_OTP_CTRL);
    //dwSpiWrite(dev, OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
    hal.scheduler->delay(5);
    pmscctrl0[0] = 0x00;
    pmscctrl0[1] = 0x02;
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);

    hal.scheduler->delay(5);
    dwEnableClock(dwClockPll);
    hal.scheduler->delay(5);

    //  CONFIGURE SYS_CFG AND SYS_CTRL
    set_sys_reg();
    //  ENABLE ALL LEDS
    enableAllLeds();
    /*  SET DATA RATE
     *   SET PULSE FREQUENCY
     *   SET PREAMBLE LENGHT
     *   SET CHANNEL
     *   SET PREAMBLE CODE
    */
    newConfig();

    committConf();

    uint8_t send2[5] = {0x8D, 0x00, 0x01, 0x00, 0x00};
    _dev->transfer(send2,5,nullptr,0);

    //RELEASE SEMAPHORE
    _sem->give();

    //_dev->register_periodic_callback( 100000, FUNCTOR_BIND_MEMBER(&AP_TDoA::loop,void) );

    hal.console->printf("\n AP_TDoA.cpp CONF FINISHED \n");
}

void AP_TDoA::dwSoftReset(){
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    dw_read(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiRead(dwDev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[0] = 0x01;
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiWrite(dwDev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    pmscctrl0[3] = 0x00;
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiWrite(dwDev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    hal.scheduler->delay(10);
    pmscctrl0[0] = 0x00;
    pmscctrl0[3] = 0xF0;
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiWrite(dwDev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    // force into idle mode
    memset(dwDev->sysctrl, 0, LEN_SYS_CTRL);
    dwDev->sysctrl[0] |= 1<<TRXOFF_BIT;
    dwDev->deviceMode = IDLE_MODE;
    dw_write(SYS_CTRL,dwDev->sysctrl,LEN_SYS_CTRL);
    //dwSpiWrite(dwDev, SYS_CTRL, NO_SUB, dwDev->sysctrl, LEN_SYS_CTRL);
}

void AP_TDoA::set_sys_reg(){
    //value to set sys_cfg register
    uint8_t send1[5] = {0x84, 0x00, 0x12, 0x00, 0x20};
    //value to set sys_ctrl register.
    //Can't check to verify as it's automatically cleared to 0 by the DWM1000
    // 0x 0000 0100
    uint8_t send2[5] = {0x8D, 0x00, 0x01, 0x00, 0x00};
    if(!_dev->transfer(send1,5,nullptr,0)){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm WRITE ERROR \n");
            _sem->give();
            return;
    }
    if(!_dev->transfer(send2,5,nullptr,0)){
        hal.console->printf("\n AP_TDoA.cpp conf_dwm WRITE ERROR \n");
            _sem->give();
            return;
    }
    hal.console->printf("\n sys_reg finished \n");
}

void AP_TDoA::initDwDev(){
    //dev->ops = ops;
    dwDev->userdata = nullptr;

    /* Device default state */
    dwDev->extendedFrameLength = FRAME_LENGTH_NORMAL;
    dwDev->pacSize = PAC_SIZE_8;
    dwDev->pulseFrequency = TX_PULSE_FREQ_16MHZ;
    dwDev->dataRate = TRX_RATE_6800KBPS;
    dwDev->preambleLength = TX_PREAMBLE_LEN_128;
    dwDev->preambleCode = PREAMBLE_CODE_16MHZ_4;
    dwDev->channel = CHANNEL_5;
    dwDev->smartPower = false;
    dwDev->frameCheck = true;
    dwDev->permanentReceive = false;
    //dwDev->deviceMode = IDLE_MODE;

    writeValueToBytes(dwDev->antennaDelay.raw, 16384, LEN_STAMP);

    /*
    // Dummy callback handlers
    dwDev->handleSent = dummy;
    dwDev->handleReceived = dummy;
    dwDev->handleReceiveFailed = dummy;
    //*/
}

void AP_TDoA::enableAllLeds(){
    uint32_t reg;

    // Set all 4 GPIO in LED mode
    dw_read(GPIO_CTRL,GPIO_MODE_SUB,(uint8_t*)&reg,LEN_GPIO_MODE);
    //reg = dwSpiRead32(dev, GPIO_CTRL, GPIO_MODE_SUB);
    reg &= ~0x00003FC0ul;
    reg |= 0x00001540ul;
    dw_write(GPIO_CTRL,GPIO_MODE_SUB,(uint8_t*)&reg,LEN_GPIO_MODE);
    //dwSpiWrite32(dev, GPIO_CTRL, GPIO_MODE_SUB, reg);

    // Enable debounce clock (used to clock the LED blinking)
    dw_read(PMSC,PMSC_CTRL0_SUB,(uint8_t*)&reg,LEN_PMSC_CTRL0);
    //reg = dwSpiRead32(dev, PMSC, PMSC_CTRL0_SUB);
    reg |= 0x00840000ul;
    dw_write(PMSC,PMSC_CTRL0_SUB,(uint8_t*)&reg,LEN_PMSC_CTRL0);
    //dwSpiWrite32(dev, PMSC, PMSC_CTRL0_SUB, reg);

    // Enable LED blinking and set the rate
    reg = 0x00000110ul;
    dw_write(PMSC,PMSC_LEDC,(uint8_t*)&reg,LEN_PMSC_LEDC);
    //dwSpiWrite32(dev, PMSC, PMSC_LEDC, reg);

    // Trigger a manual blink of the LEDs for test
    reg |= 0x000f0000ul;
    dw_write(PMSC,PMSC_LEDC,(uint8_t*)&reg,LEN_PMSC_LEDC);
    //dwSpiWrite32(dev, PMSC, PMSC_LEDC, reg);
    reg &= ~0x000f0000ul;
    dw_write(PMSC,PMSC_LEDC,(uint8_t*)&reg,LEN_PMSC_LEDC);
    //dwSpiWrite32(dev, PMSC, PMSC_LEDC, reg);

    hal.console->printf("\n enableAllLeds finished \n");
}

void AP_TDoA::newConfig(){
    dwDev->antennaDelay.full = 0;
    _dev->read_registers(PANADR, dwDev->networkAndAddress, LEN_PANADR);
    _dev->read_registers(SYS_CFG, dwDev->syscfg, LEN_SYS_CFG);
    _dev->read_registers(CHAN_CTRL, dwDev->chanctrl, LEN_CHAN_CTRL);
    _dev->read_registers(SYS_MASK, dwDev->sysmask, LEN_SYS_MASK);

    //  SET DATA RATE
    uint8_t rate = MODE_SHORTDATA_FAST_ACCURACY[0];
    rate &= 0x03;
    dwDev->txfctrl[1] &= 0x83;
    dwDev->txfctrl[1] |= (uint8_t)((rate << 5) & 0xFF);
    if(rate == TRX_RATE_110KBPS) {
        setBit(dwDev->syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
    } else {
        setBit(dwDev->syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
    }
    // SFD mode and type (non-configurable, as in Table )
    if(rate == TRX_RATE_6800KBPS) {
        setBit(dwDev->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
        setBit(dwDev->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
        setBit(dwDev->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
    } else {
        setBit(dwDev->chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
        setBit(dwDev->chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
        setBit(dwDev->chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);

    }
    uint8_t sfdLength;
    if(rate == TRX_RATE_6800KBPS) {
        sfdLength = 0x08;
    } else if(rate == TRX_RATE_850KBPS) {
        sfdLength = 0x10;
    } else {
        sfdLength = 0x40;
    }
    dw_write(USR_SFD,SFD_LENGTH_SUB,(uint8_t*)&sfdLength,LEN_SFD_LENGTH);
    //dwSpiWrite(dev, USR_SFD, SFD_LENGTH_SUB, &sfdLength, LEN_SFD_LENGTH);
    dwDev->dataRate = rate;

    //  SET PULSE FREQUENCY
    uint8_t freq = MODE_SHORTDATA_FAST_ACCURACY[1];
    freq &= 0x03;
    dwDev->txfctrl[2] &= 0xFC;
    dwDev->txfctrl[2] |= (uint8_t)(freq & 0xFF);
    dwDev->chanctrl[2] &= 0xF3;
    dwDev->chanctrl[2] |= (uint8_t)((freq << 2) & 0xFF);
    dwDev->pulseFrequency = freq;

    //  SET PREAMBLE LENGTH
    uint8_t prealen = MODE_SHORTDATA_FAST_ACCURACY[2];
    prealen &= 0x0F;
    dwDev->txfctrl[2] &= 0xC3;
    dwDev->txfctrl[2] |= (uint8_t)((prealen << 2) & 0xFF);
    if(prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
        dwDev->pacSize = PAC_SIZE_8;
    } else if(prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
        dwDev->pacSize = PAC_SIZE_16;
    } else if(prealen == TX_PREAMBLE_LEN_1024) {
        dwDev->pacSize = PAC_SIZE_32;
    } else {
        dwDev->pacSize = PAC_SIZE_64;
    }
    dwDev->preambleLength = prealen;

    //  SET CHANNEL
    uint8_t channel = CHANNEL_2;
    channel &= 0xF;
    dwDev->chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
    dwDev->channel = channel;

    // SET PREAMBLE CODE
    uint8_t preacode = PREAMBLE_CODE_64MHZ_9;
    preacode &= 0x1F;
    dwDev->chanctrl[2] &= 0x3F;
    dwDev->chanctrl[2] |= ((preacode << 6) & 0xFF);
    dwDev->chanctrl[3] = 0x00;
    dwDev->chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
    dwDev->preambleCode = preacode;

    //  USE SMART POWER
    bool smartPower = true;
    dwDev->smartPower = smartPower;
    setBit(dwDev->syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);

    //  SET RX TIMEOUT
    uint16_t timeout = RX_TIMEOUT;
    dw_write(RX_FWTO,NO_SUB,(uint8_t*)&timeout,2);
    //dwSpiWrite(dev, RX_FWTO, NO_SUB, &timeout, 2);
    setBit(dwDev->syscfg, LEN_SYS_CFG, RXWTOE_BIT, timeout!=0);
    setBit(dwDev->syscfg, LEN_SYS_CFG, RXAUTR_BIT, true);

    hal.console->printf("\n newConfig finished \n");
}

void AP_TDoA::committConf(){
    dw_write(PANADR,NO_SUB,dwDev->networkAndAddress,LEN_PANADR);
    //dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
    dw_write(SYS_CFG,NO_SUB,dwDev->syscfg,LEN_SYS_CFG);
    //dwSpiWrite(dev, SYS_CFG, NO_SUB, dev->syscfg, LEN_SYS_CFG);
    dw_write(CHAN_CTRL,NO_SUB,dwDev->chanctrl,LEN_CHAN_CTRL);
    //dwSpiWrite(dev, CHAN_CTRL, NO_SUB, dev->chanctrl, LEN_CHAN_CTRL);
    dw_write(TX_FCTRL,NO_SUB,dwDev->txfctrl,LEN_TX_FCTRL);
    //dwSpiWrite(dev, TX_FCTRL, NO_SUB, dev->txfctrl, LEN_TX_FCTRL);
    dw_write(SYS_MASK,NO_SUB,dwDev->sysmask,LEN_SYS_MASK);
    //dwSpiWrite(dev, SYS_MASK, NO_SUB, dev->sysmask, LEN_SYS_MASK);
    tune();
    dw_write(TX_ANTD,NO_SUB,dwDev->antennaDelay.raw,LEN_TX_ANTD);
    //dwSpiWrite(dev, TX_ANTD, NO_SUB, dev->antennaDelay.raw, LEN_TX_ANTD);
    dw_write(LDE_IF,LDE_RXANTD_SUB,dwDev->antennaDelay.raw,LEN_LDE_RXANTD);
    //dwSpiWrite(dev, LDE_IF, LDE_RXANTD_SUB, dev->antennaDelay.raw, LEN_LDE_RXANTD);

    hal.console->printf("\n commitConfig finished \n");
}

void AP_TDoA::tune(){
    // these registers are going to be tuned/configured
    uint8_t agctune1[LEN_AGC_TUNE1];
    uint8_t agctune2[LEN_AGC_TUNE2];
    uint8_t agctune3[LEN_AGC_TUNE3];
    uint8_t drxtune0b[LEN_DRX_TUNE0b];
    uint8_t drxtune1a[LEN_DRX_TUNE1a];
    uint8_t drxtune1b[LEN_DRX_TUNE1b];
    uint8_t drxtune2[LEN_DRX_TUNE2];
    uint8_t drxtune4H[LEN_DRX_TUNE4H];
    uint8_t ldecfg1[LEN_LDE_CFG1];
    uint8_t ldecfg2[LEN_LDE_CFG2];
    uint8_t lderepc[LEN_LDE_REPC];
    uint8_t txpower[LEN_TX_POWER];
    uint8_t rfrxctrlh[LEN_RF_RXCTRLH];
    uint8_t rftxctrl[LEN_RF_TXCTRL];
    uint8_t tcpgdelay[LEN_TC_PGDELAY];
    uint8_t fspllcfg[LEN_FS_PLLCFG];
    uint8_t fsplltune[LEN_FS_PLLTUNE];
    // uint8_t fsxtalt[LEN_FS_XTALT];
    // AGC_TUNE1
    if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
        writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
    } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
        writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
    } else {
        // TODO proper error/warning handling
    }
    // AGC_TUNE2
    writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
    // AGC_TUNE3
    writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
    // DRX_TUNE0b (already optimized according to Table 20 of user manual)
    if(dwDev->dataRate == TRX_RATE_110KBPS) {
        writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
    } else if(dwDev->dataRate == TRX_RATE_850KBPS) {
        writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
    } else if(dwDev->dataRate == TRX_RATE_6800KBPS) {
        writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
    } else {
        // TODO proper error/warning handling
    }
    // DRX_TUNE1a
    if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
        writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
    } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
        writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
    } else {
        // TODO proper error/warning handling
    }
    // DRX_TUNE1b
    if(dwDev->preambleLength ==  TX_PREAMBLE_LEN_1536 || dwDev->preambleLength ==  TX_PREAMBLE_LEN_2048 ||
            dwDev->preambleLength ==  TX_PREAMBLE_LEN_4096) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->preambleLength != TX_PREAMBLE_LEN_64) {
        if(dwDev->dataRate == TRX_RATE_850KBPS || dwDev->dataRate == TRX_RATE_6800KBPS) {
            writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
        } else {
            // TODO proper error/warning handling
        }
    } else {
        if(dwDev->dataRate == TRX_RATE_6800KBPS) {
            writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
        } else {
            // TODO proper error/warning handling
        }
    }
    // DRX_TUNE2
    if(dwDev->pacSize == PAC_SIZE_8) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->pacSize == PAC_SIZE_16) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->pacSize == PAC_SIZE_32) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->pacSize == PAC_SIZE_64) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
        } else {
            // TODO proper error/warning handling
        }
    } else {
        // TODO proper error/warning handling
    }
    // DRX_TUNE4H
    if(dwDev->preambleLength == TX_PREAMBLE_LEN_64) {
        writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
    } else {
        writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
    }
    // RF_RXCTRLH
    if(dwDev->channel != CHANNEL_4 && dwDev->channel != CHANNEL_7) {
        writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
    } else {
        writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
    }
    // RX_TXCTRL
    if(dwDev->channel == CHANNEL_1) {
        writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
    } else if(dwDev->channel == CHANNEL_2) {
        writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
    } else if(dwDev->channel == CHANNEL_3) {
        writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
    } else if(dwDev->channel == CHANNEL_4) {
        writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
    } else if(dwDev->channel == CHANNEL_5) {
        writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
    } else if(dwDev->channel == CHANNEL_7) {
        writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
    } else {
        // TODO proper error/warning handling
    }
    // TC_PGDELAY
    if(dwDev->channel == CHANNEL_1) {
        writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
    } else if(dwDev->channel == CHANNEL_2) {
        writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
    } else if(dwDev->channel == CHANNEL_3) {
        writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
    } else if(dwDev->channel == CHANNEL_4) {
        writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
    } else if(dwDev->channel == CHANNEL_5) {
        writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
    } else if(dwDev->channel == CHANNEL_7) {
        writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
    } else {
        // TODO proper error/warning handling
    }
    // FS_PLLCFG and FS_PLLTUNE
    if(dwDev->channel == CHANNEL_1) {
        writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
    } else if(dwDev->channel == CHANNEL_2 || dwDev->channel == CHANNEL_4) {
        writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
    } else if(dwDev->channel == CHANNEL_3) {
        writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0x5E, LEN_FS_PLLTUNE);
    } else if(dwDev->channel == CHANNEL_5 || dwDev->channel == CHANNEL_7) {
        writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
        writeValueToBytes(fsplltune, 0xA6, LEN_FS_PLLTUNE);
    } else {
        // TODO proper error/warning handling
    }
    // LDE_CFG1
    writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
    // LDE_CFG2
    if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
        writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
    } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
        writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
    } else {
        // TODO proper error/warning handling
    }
    // LDE_REPC
    if(dwDev->preambleCode == PREAMBLE_CODE_16MHZ_1 || dwDev->preambleCode == PREAMBLE_CODE_16MHZ_2) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_16MHZ_3 || dwDev->preambleCode == PREAMBLE_CODE_16MHZ_8) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_16MHZ_4) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_16MHZ_5) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_16MHZ_6) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_16MHZ_7) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_64MHZ_9) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_64MHZ_10 || dwDev->preambleCode == PREAMBLE_CODE_64MHZ_17) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_64MHZ_11) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_64MHZ_12) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_64MHZ_18 || dwDev->preambleCode == PREAMBLE_CODE_64MHZ_19) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
        }
    } else if(dwDev->preambleCode == PREAMBLE_CODE_64MHZ_20) {
        if(dwDev->dataRate == TRX_RATE_110KBPS) {
            writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
        } else {
            writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
        }
    } else {
        // TODO proper error/warning handling
    }
    // TX_POWER (enabled smart transmit power control)
    if(dwDev->channel == CHANNEL_1 || dwDev->channel == CHANNEL_2) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
            }
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->channel == CHANNEL_3) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
            }
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->channel == CHANNEL_4) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
            }
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->channel == CHANNEL_5) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
            }
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else if(dwDev->channel == CHANNEL_7) {
        if(dwDev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
            }
        } else if(dwDev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
            if(dwDev->smartPower) {
                writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
            } else {
                writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
            }
        } else {
            // TODO proper error/warning handling
        }
    } else {
        // TODO proper error/warning handling
    }
    // mid range XTAL trim (TODO here we assume no calibration data available in OTP)
    //writeValueToBytes(fsxtalt, 0x60, LEN_FS_XTALT);
    // write configuration back to chip
    dw_write(AGC_TUNE,AGC_TUNE1_SUB,agctune1,LEN_AGC_TUNE1);
    //dwSpiWrite(dwDev, AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
    dw_write(AGC_TUNE,AGC_TUNE2_SUB,agctune2,LEN_AGC_TUNE2);
    //dwSpiWrite(dwDev, AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
    dw_write(AGC_TUNE,AGC_TUNE3_SUB,agctune3,LEN_AGC_TUNE3);
    //dwSpiWrite(dwDev, AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
    dw_write(DRX_TUNE,DRX_TUNE0b_SUB,drxtune0b,LEN_DRX_TUNE0b);
    //dwSpiWrite(dwDev, DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
    dw_write(DRX_TUNE,DRX_TUNE1a_SUB,drxtune1a,LEN_DRX_TUNE1a);
    //dwSpiWrite(dwDev, DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
    dw_write(DRX_TUNE,DRX_TUNE1b_SUB,drxtune1b,LEN_DRX_TUNE1b);
    //dwSpiWrite(dwDev, DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
    dw_write(DRX_TUNE,DRX_TUNE2_SUB,drxtune2,LEN_DRX_TUNE2);
    //dwSpiWrite(dwDev, DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
    dw_write(DRX_TUNE,DRX_TUNE4H_SUB,drxtune4H,LEN_DRX_TUNE4H);
    //dwSpiWrite(dwDev, DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
    dw_write(LDE_IF,LDE_CFG1_SUB,ldecfg1,LEN_LDE_CFG1);
    //dwSpiWrite(dwDev, LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
    dw_write(LDE_IF,LDE_CFG2_SUB,ldecfg2,LEN_LDE_CFG2);
    //dwSpiWrite(dwDev, LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
    dw_write(LDE_IF,LDE_REPC_SUB,lderepc,LEN_LDE_REPC);
    //dwSpiWrite(dwDev, LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
    dw_write(TX_POWER,NO_SUB,txpower,LEN_TX_POWER);
    //dwSpiWrite(dwDev, TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
    dw_write(RF_CONF,RF_RXCTRLH_SUB,rfrxctrlh,LEN_RF_RXCTRLH);
    //dwSpiWrite(dwDev, RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
    dw_write(RF_CONF,RF_TXCTRL_SUB,rftxctrl,LEN_RF_TXCTRL);
    //dwSpiWrite(dwDev, RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
    dw_write(TX_CAL,TC_PGDELAY_SUB,tcpgdelay,LEN_TC_PGDELAY);
    //dwSpiWrite(dwDev, TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
    dw_write(FS_CTRL,FS_PLLTUNE_SUB,fsplltune,LEN_FS_PLLTUNE);
    //dwSpiWrite(dwDev, FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
    dw_write(FS_CTRL,FS_PLLCFG_SUB,fspllcfg,LEN_FS_PLLCFG);
    //dwSpiWrite(dwDev, FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);

    //dwSpiWrite(dwDev, FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);

    hal.console->printf("\n tune finished \n");
}

void AP_TDoA::dwEnableClock(dwClock_t clock){
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    dw_read(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiRead(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    if(clock == dwClockAuto) {
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);
        pmscctrl0[0] = dwClockAuto;
        pmscctrl0[1] &= 0xFE;
    } else if(clock == dwClockXti) {
        _dev->set_speed(AP_HAL::Device::SPEED_LOW);
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= dwClockXti;
    } else if(clock == dwClockPll) {
        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
        pmscctrl0[0] &= 0xFC;
        pmscctrl0[0] |= dwClockPll;
    } else {
        // TODO deliver proper warning
    }
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,1);
    //dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, 1);
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
    //dwSpiWrite(dev, PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val) {
    unsigned int idx;
    unsigned int shift;

    idx = bit / 8;
    if(idx >= n) {
        return; // TODO proper error handling: out of bounds
    }
    uint8_t* targetByte = &data[idx];
    shift = bit % 8;
    if(val) {
        *targetByte |= (1<<shift);
    } else {
      *targetByte &= ~(1<<shift);
    }
}

static bool getBit(uint8_t data[], unsigned int n, unsigned int bit) {
    unsigned int idx;
    unsigned int shift;

    idx = bit / 8;
    if(idx >= n) {
        return false; // TODO proper error handling: out of bounds
    }
    uint8_t targetByte = data[idx];
    shift = bit % 8;

    return (targetByte>>shift)&0x01;
}

static void writeValueToBytes(uint8_t data[], long val, unsigned int n) {
    unsigned int i;
    for(i = 0; i < n; i++) {
        data[i] = ((val >> (i * 8)) & 0xFF);
    }
}

void AP_TDoA::dw_read(uint8_t reg, uint8_t* val, uint8_t len){
    uint8_t head[1];
    head[0] = reg & 0x3F;
    _dev->transfer(head, 1, val, len);
}

void AP_TDoA::dw_read(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len){
    if(sub == 0){
        dw_read(reg,val,len);
        return;
    }
    uint8_t head[3];
    head[0] = (reg & 0x3F) | 0x40;
    if(sub <= 0x7F){
        head[1] = sub;
        _dev->transfer(head, 2, val, len);
    }
    else{
        head[1] = (sub & 0x7F) | 0x80;
        sub >>= 7;
        head[2] = sub & 0xff;
        _dev->transfer(head, 3, val, len);
    }
}

void AP_TDoA::dw_write(uint8_t reg, uint8_t* val, uint8_t len){
    //std::vector<uint8_t> packets(len+1,0);
    uint8_t* packets;
    packets = new uint8_t[len+1];
    packets[0] = reg | 0x80;
    memcpy((uint8_t*)&packets[1],val, len);
    _dev->transfer(packets, len+1, nullptr, 0);
    delete[] packets;
}

void AP_TDoA::dw_write(uint8_t reg, uint16_t sub , uint8_t* val, uint8_t len){
    if(sub == 0){
        dw_write(reg,val,len);
        return;
    }
    uint8_t* packets;
    if(sub <= 0x7F){
        packets = new uint8_t[len+2];
        packets[0] = reg | 0xC0;
        packets[1] = sub;
        memcpy((uint8_t*)&packets[2],val, len);
        _dev->transfer(packets, len+2, nullptr, 0);
    }
    else{
        packets = new uint8_t[len+3];
        packets[0] = reg | 0xC0;
        packets[1] = (sub & 0x7F) | 0x80;
        sub >>= 7;
        packets[2] = sub & 0xff;
        memcpy((uint8_t*)&packets[3],val, len);
        _dev->transfer(packets, len+3, nullptr, 0);
    }
    delete[] packets;
}
