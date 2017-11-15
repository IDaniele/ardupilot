#include "AP_TDoA.h"
#include "matrixmath.h"
#include <DataFlash/DataFlash.h>

using namespace std;

//*
# include "qr_solve.h"
# include "r8lib.h"
//*/

extern const AP_HAL::HAL& hal;

static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
//static float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];
//static uint16_t anchorDistanceLog[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];
static double frameTime_in_cl_A[LOCODECK_NR_OF_ANCHORS];
static double clockCorrection_T_To_A[LOCODECK_NR_OF_ANCHORS];
static float clockCorrectionLog[LOCODECK_NR_OF_ANCHORS];
static uint8_t sequenceNrs[LOCODECK_NR_OF_ANCHORS];
static uint8_t previousAnchor;
static double position[3];
static double pos[3] = { 3.10, 2.44, 1.20 };  //fake position

static void error();

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val);
static bool getBit(uint8_t data[], unsigned int n, unsigned int bit);
static void writeValueToBytes(uint8_t data[], long val, unsigned int n);

static bool ongoing;
static bool received[LOCODECK_NR_OF_ANCHORS];
static double distdiff[LOCODECK_NR_OF_ANCHORS];
static int8_t prevQueue;
static int8_t countQueue[DIM_QUEUE];
static int count_received = 1;

double *A2, *A3;
double I3[9] = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};

bool printflag = false;

static uint64_t timestampToUint64(const uint8_t *ts) {
  dwTime_t timestamp;
  memset(&timestamp,0,sizeof(timestamp));
  memcpy(timestamp.raw, ts, sizeof(timestamp.raw));

  return timestamp.full;
}

static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static bool isSeqNrConsecutive(uint8_t prevSeqNr, uint8_t currentSeqNr) {
  return (currentSeqNr == ((prevSeqNr + 1) & 0xff));
}

static bool calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
  const int64_t previous_txAn_in_cl_An = rxPacketBuffer[anchor].timestamps[anchor];

  if (! isSeqNrConsecutive(rxPacketBuffer[anchor].sequenceNrs[anchor], packet->sequenceNrs[anchor])) {
    return false;
  }

  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t previous_rxAn_by_T_in_cl_T = arrivals[anchor].full;
  const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
  const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);

  *clockCorrection = frameTime_in_cl_An / frameTime_in_T;
  return true;
}

static uint64_t truncateToTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFFFul;
}

static bool isValidRxTime(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static bool calcDistanceDiff(float* tdoaDistDiff, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
  const int64_t rxAn_by_T_in_cl_T  = arrival->full;
  const int64_t rxAr_by_An_in_cl_An = packet->timestamps[0];
  const int64_t tof_Ar_to_An_in_cl_An = packet->distances[0];
  const double clockCorrection = clockCorrection_T_To_A[anchor];

  const bool isSeqNrInTagOk = isSeqNrConsecutive(rxPacketBuffer[anchor].sequenceNrs[previousAnchor], packet->sequenceNrs[previousAnchor]);
  const bool isSeqNrInAnchorOk = isSeqNrConsecutive(sequenceNrs[anchor], packet->sequenceNrs[anchor]);
  const bool isAnchorDistanceOk = isValidTimeStamp(tof_Ar_to_An_in_cl_An);
  const bool isRxTimeInTagOk = isValidTimeStamp(rxAr_by_An_in_cl_An);
  const bool isClockCorrectionOk = (clockCorrection != 0.0);

  if (! (isSeqNrInTagOk && isSeqNrInAnchorOk && isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
    return false;
  }

  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t rxAr_by_T_in_cl_T = arrivals[0].full;

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection - delta_txAr_to_txAn_in_cl_An;

  *tdoaDistDiff = lightspeed * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;
//*
  if(printflag){
      hal.console->printf("\n tof_Ar_to_An_in_cl_An : %lld \n", tof_Ar_to_An_in_cl_An);
      hal.console->printf("\n delta_txAr_to_txAn_in_cl_An : %lld \n", delta_txAr_to_txAn_in_cl_An);
      hal.console->printf("\n rxAn_by_T_in_cl_T : %lld \n", rxAn_by_T_in_cl_T);
      hal.console->printf("\n rxAr_by_T_in_cl_T : %lld \n", rxAr_by_T_in_cl_T);
  }//*/

  return true;
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
    memset(clockCorrectionLog, 0, sizeof(clockCorrectionLog));
    memset(sequenceNrs, 0, sizeof(sequenceNrs));
    //memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));
    //memset(anchorDistanceLog, 0, sizeof(anchorDistanceLog));
    ongoing = false;
    memset(received, 0, sizeof(received));
    memset(distdiff, 0, sizeof(distdiff));
    memset(position, 0, sizeof(position));
    A2 = nullptr;
    A3 = nullptr;
    memset(recvQueue, 0, LOCODECK_NR_OF_ANCHORS * DIM_QUEUE * sizeof(recvData));
    currentQueue = 0;
    prevQueue = -1;
    memset(countQueue, 0, DIM_QUEUE * sizeof(int8_t));
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
    //_dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

void AP_TDoA::dummy(void){

    //double dist[5];
    double uwbTdoaDistDiff2[5] = {1.219493, 0.637888, 1.313300, -0.365847, 1.843310};
    double a[5*4], b[5], teta0[3], badteta0[3];
    A2 = new double[5*3];
    A3 = new double[5];
/*
    double dist0 = sqrt(
        pow((anchorPositionRAW[0][0] - pos[0]), 2) +
        pow((anchorPositionRAW[0][1] - pos[1]), 2) +
        pow((anchorPositionRAW[0][2] - pos[2]), 2)
    );
    //*/
    for (int i = 0; i<5; i++) {
        int j = i + 1;
        /*
        dist[i] = sqrt(
            pow((anchorPositionRAW[j][0] - pos[0]), 2) +
            pow((anchorPositionRAW[j][1] - pos[1]), 2) +
            pow((anchorPositionRAW[j][2] - pos[2]), 2)
        );//*/
        A2[(i*3)+0] = a[(i*4)+0] = anchorPositionRAW[j][0] - anchorPositionRAW[0][0];
        A2[(i*3)+1] = a[(i*4)+1] = anchorPositionRAW[j][1] - anchorPositionRAW[0][1];
        A2[(i*3)+2] = a[(i*4)+2] = anchorPositionRAW[j][2] - anchorPositionRAW[0][2];
        A3[i] = a[(i*4)+3] = uwbTdoaDistDiff2[i];
        b[i] = (pow(a[(i*4)+0],2) + pow(a[(i*4)+1], 2) + pow(a[(i*4)+2], 2) - pow(a[(i*4)+3], 2)) / 2.0;
    }
/*
    double *svd;
    svd = svd_solve(5, 4, a, b);
    badteta0[0] = svd[0];
    badteta0[1] = svd[1];
    badteta0[2] = svd[2];
    svd[0] += anchorPositionRAW[0][0];
    svd[1] += anchorPositionRAW[0][1];
    svd[2] += anchorPositionRAW[0][2];
    hal.console->printf("\n pos x=%lf  y=%lf  z=%lf r0=%lf \n", svd[0]+anchorPositionRAW[0][0], svd[1]+anchorPositionRAW[0][1], svd[2]+anchorPositionRAW[0][2], svd[3]);
    //*/
    teta0[0] = pos[0] - anchorPositionRAW[0][0];
    teta0[1] = pos[1] - anchorPositionRAW[0][1];
    teta0[2] = pos[2] - anchorPositionRAW[0][2];
    double dist0 = 2.47797;
    minimize(b, teta0, 5, dist0);

    hal.console->printf("\n NEW pos x=%lf  y=%lf  z=%lf \n", teta0[0]+anchorPositionRAW[0][0], teta0[1]+anchorPositionRAW[0][1], teta0[2]+anchorPositionRAW[0][2]);

    delete[] A2;
    delete[] A3;
}

void AP_TDoA::loopblock(){
    if (!_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("\n AP_TDoA.cpp loop SEMAPHORE ERROR \n");
        return;
    }
    loop();
    _sem->give();
}

void AP_TDoA::loop(){
    //hal.console->printf("\n loop \n");
    dw_read(SYS_STATUS, NO_SUB, dwDev->sysstatus, LEN_SYS_STATUS);
    if(getBit(dwDev->sysstatus, LEN_SYS_STATUS, RXDFR_BIT)){
        //hal.console->printf("\n received \n");
        uint64_t* sysst = new uint64_t;
        memcpy(sysst,&(dwDev->sysstatus[0]), LEN_SYS_STATUS);

        dwTime_t* time = new dwTime_t;
        _dev->read_registers(RX_TIME, time->raw, 5);
        getTime(time);
        uint64_t rxtime = time->full;
        //*
        unsigned int len = 0;
        int dataLength;
        // 10 bits of RX frame control register
        uint8_t rxFrameInfo[LEN_RX_FINFO];
        dw_read(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
        len = ((((unsigned int)rxFrameInfo[1] << 8) | (unsigned int)rxFrameInfo[0]) & 0x03FF);
        if(dwDev->frameCheck && len > 2) {
            dataLength = len-2;
        }
        dataLength = len;
        packet_t newpack;

        if(dataLength <= 0) {
            return;
        }
        _dev->read_registers(RX_BUFFER, (uint8_t*)&newpack, dataLength);
        //*/
/*
        packet_t newpack;
        _dev->read_registers(RX_BUFFER, (uint8_t*)&newpack, sizeof(newpack));
//*/
        uint8_t anchor = newpack.sourceAddress & 0xff;
        //hal.console->printf("\n\n SYS_STATUS = %010llx", *sysst);
        //hal.console->printf("\n RX_TIME = %016llx", rxtime);
        //hal.console->printf("\n ANCHOR = %02u", anchor);
        //hal.console->printf("\n TX_TIME = %016llx \n", txtime);

        //enqueue(anchor, newpack, rxtime);
        //count_received++;
        recvData recv;
        recv.anchor = anchor;
        recv.newpack = newpack;
        recv.rxtime = rxtime;
        if(anchor <= prevQueue){
            currentQueue = (currentQueue + 1) % DIM_QUEUE;
            memset(recvQueue[currentQueue], 0, LOCODECK_NR_OF_ANCHORS * sizeof(recvData));
            countQueue[currentQueue] = 0;
        }
        recvQueue[currentQueue][anchor] = recv;
        countQueue[currentQueue]++;
        prevQueue = anchor;

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
}

void AP_TDoA::enqueue() {
    //hal.console->printf("\n received %d packets \n", count_received);
    //count_received++;
    if((count_received%20) == 0){
        //printflag=true;
    }
    uint8_t readyQueue = (currentQueue + DIM_QUEUE - 1) % DIM_QUEUE;
    if( (countQueue[readyQueue] >= LOCODECK_NR_OF_ANCHORS) && (recvQueue[readyQueue][0].rxtime != 0) ){
    //if( (countQueue[readyQueue] >= 0) && (recvQueue[readyQueue][0].rxtime != 0) ){
        //STORE DATA RIGHT AWAY BECAUSE OF RUN CONDITION
        int8_t num = countQueue[readyQueue];
        recvData queue[LOCODECK_NR_OF_ANCHORS];
        memcpy(queue, recvQueue[readyQueue], LOCODECK_NR_OF_ANCHORS*sizeof(recvData));
        for(int i=0; i< LOCODECK_NR_OF_ANCHORS; i++) {
            if ((queue[i].anchor < LOCODECK_NR_OF_ANCHORS) && (queue[i].rxtime != 0)) {
/*
                DataFlash_Class::instance()->Log_Write("TDOA", "TimeUS,tdoa10", "Qf",
                                                       AP_HAL::micros64(),
                                                       ((rangePacket_t)queue[i].newpack.payload).distances[0]
                                                       );
                dwTime_t arrival;
                arrival.full = 0;
                  dw_read(RX_TIME, RX_STAMP_SUB, arrival.raw, LEN_RX_STAMP);
                  //,0x15,0x00, ,5
                  // correct timestamp (i.e. consider range bias)
                  getTime(&arrival);
                  //*/
                dwTime_t arrival;
                arrival.full = queue[i].rxtime;
                getTime(&arrival);
                //hal.console->printf("\n Timestamp: %lld \n", arrival.full);

                const uint8_t anchor = queue[i].anchor;

                if (anchor < LOCODECK_NR_OF_ANCHORS) {
                  const rangePacket_t* packet = (rangePacket_t*)queue[i].newpack.payload;

                  calcClockCorrection(&clockCorrection_T_To_A[anchor], anchor, packet, &arrival);
                  clockCorrectionLog[anchor] = clockCorrection_T_To_A[anchor];

                  if ((anchor != previousAnchor)  && (anchor != 0)) {
                    float tdoaDistDiff = 0.0f;
                    if (calcDistanceDiff(&tdoaDistDiff, anchor, packet, &arrival)) {
                      //enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);
                        distdiff[anchor] = tdoaDistDiff;
                    }
                    else{
                        num--;
                    }
                  }

                  arrivals[anchor].full = arrival.full;
                  memcpy(&rxPacketBuffer[anchor], queue[i].newpack.payload, sizeof(rangePacket_t));
                  sequenceNrs[anchor] = packet->sequenceNrs[anchor];

                  if(printflag){
                      hal.console->printf("\n queue[%d].rxtime: %lld \n", i, queue[i].rxtime);
                      hal.console->printf("\n arrival.full: %lld \n", arrival.full);
                      hal.console->printf("\n arrivals[%d].full: %lld \n", anchor, arrivals[anchor].full);
                  }

                  previousAnchor = anchor;
                }
            }
        }
        if(num >= NR_OF_TDOA){
        //if(countQueue[readyQueue] >= 0){
            tdoa(readyQueue);
            //error();
        }
        else{
            //hal.console->printf("\n not enough: %d \n", countQueue[readyQueue]);
        }
        memset(distdiff, 0, LOCODECK_NR_OF_ANCHORS * sizeof(double));
    }
    printflag = false;
}



void AP_TDoA::tdoa(int8_t readyQueue){
/*
    if(count_received%400 == 0){
        DataFlash_Class::instance()->Log_Write("TDOA", "TimeUS,count_received", "Qd",
                                                   AP_HAL::micros64(),
                                                   count_received
                                                   );
    }

   for(int i=1; i<LOCODECK_NR_OF_ANCHORS; i++){
       hal.console->printf("\n tdoa %d: %f \n", i, distdiff[i]);
   }
   //*/
   //*
    DataFlash_Class::instance()->Log_Write("TDOA", "TimeUS,tdoa10,tdoa20,tdoa30,tdoa40,tdoa50,tdoa60,tdoa70", "Qfffffff",
                                           AP_HAL::micros64(),
                                           distdiff[1],
                                           distdiff[2],
                                           distdiff[3],
                                           distdiff[4],
                                           distdiff[5],
                                           distdiff[6],
                                           distdiff[7]
                                           );
    //*/
    /*
    int m = countQueue[readyQueue];

    hal.console->printf("\n tdoa %d \n", countQueue[readyQueue]);
    hal.console->printf("\n distdiff1 %f \n", distdiff[1]);
    hal.console->printf("\n distdiff2 %f \n", distdiff[2]);
    hal.console->printf("\n distdiff3 %f \n", distdiff[3]);
    hal.console->printf("\n distdiff4 %f \n", distdiff[4]);
    hal.console->printf("\n distdiff5 %f \n", distdiff[5]);

    int n = 4;
    double *a = new double[m*n];
    double *b = new double[m];
    double* svd;

    A2 = new double[m*3];
    A3 = new double[m];
    for(int i=1; i<LOCODECK_NR_OF_ANCHORS; i++){
        if(distdiff[i] != 0.0f){
            int j = i - 1;
            A2[(j*3)+0] = a[(j*n)+0] = (anchorPositionRAW[i][0] - anchorPositionRAW[0][0]);
            A2[(j*3)+1] = a[(j*n)+1] = (anchorPositionRAW[i][1] - anchorPositionRAW[0][1]);
            A2[(j*3)+2] = a[(j*n)+2] = (anchorPositionRAW[i][2] - anchorPositionRAW[0][2]);
            A3[j] = a[(j*n)+3] = distdiff[i];
            b[j] = ( pow(a[(j*n)+0],2) + pow(a[(j*n)+1],2) + pow(a[(j*n)+2],2) - pow(a[(j*n)+3],2) ) / 2.0l;
        }
    }

    svd = svd_solve(m, n, a, b);
    //svd = new double[4];
    //memset(svd, 0, 4*sizeof(double));

    double teta0[3];
    teta0[0] = svd[0];
    teta0[1] = svd[1];
    teta0[2] = svd[2];
    //minimize(b, teta0, m, svd[3]);

    position[0] = teta0[0]+anchorPositionRAW[0][0];
    position[1] = teta0[1]+anchorPositionRAW[0][1];
    position[2] = teta0[2]+anchorPositionRAW[0][2];

    //hal.console->printf("\n pos x=%lf  y=%lf  z=%lf r0 =%lf\n", svd[0]+anchorPositionRAW[0][0], svd[1]+anchorPositionRAW[0][1], svd[2]+anchorPositionRAW[0][2], svd[3]);
    //hal.console->printf("\n NEW pos x=%lf  y=%lf  z=%lf \n", teta0[0]+anchorPositionRAW[0][0], teta0[1]+anchorPositionRAW[0][1], teta0[2]+anchorPositionRAW[0][2]);

    delete[] A2;
    delete[] A3;
    delete[] a;
    delete[] b;
    //*/
}

// QUESTA È LA FUNZIONE CHE ESEGUE L'OTTIMIZZAZIONE DEL RISULTATO COME NEL PAPER.
void AP_TDoA::minimize(double b[], double teta0[], int m, double r1){
    double *A2t = transpose(A2,m,3);
    double *A3t = transpose(A3,m,1);

    //bool stop = false;
    //double prevnorm = 0;

    int count_min = 1;
    do{
        double *teta0t = transpose(teta0, 3, 1);

        double *G = new double[m*m];
        double *G1 = new double[m*m];
        memset(G,0,sizeof(double)*m*m);
        memset(G1,0,sizeof(double)*m*m);
        for(int i=0; i<m; i++){
            G1[(i*m)+i] = 1;
            G[(i*m)+i] = r1+A3[i];
        }
        // tetaT*teta
        double *ttt = multiply(teta0t, teta0, 1, 3, 3, 1);
        double ttt1, ttt2;
        ttt1 = pow(*ttt, 0.5);
        ttt2 = pow(*ttt, -0.5);

        double *A2teta0 = multiply(A2, teta0, m, 3, 3, 1);
        double *A3ttt1 = multiplyscalar(A3, ttt1, m, 1);
        double *somma01 = addmatrix(A2teta0,A3ttt1, m, 1);
        double *beta = submatrix(somma01, b, m, 1);

        double *Gt = transpose(G, m, m);
        double *GGt = multiply(G, Gt, m, m, m, m);
        double *fi = invert(GGt, m);
        double *mu = multiply(fi, beta, m, m, m, 1);

        double *teta0A3t = multiply(teta0, A3t, 3, 1, 1, m);
        double *teta0A3tscalttt2 = multiplyscalar(teta0A3t, ttt2, 3, m);
        double *TETA = addmatrix(A2t, teta0A3tscalttt2 , 3, m);
        double *TETAmu = multiply(TETA, mu, 3, m, m, 1);
        double *TETAmuscal2 = multiplyscalar(TETAmu, 2, 3, 1);
        double *mut = transpose(mu, m, 1);
        double *teta0mut = multiply(teta0, mut, 3, 1, 1, m);
        double *teta0mutG1 = multiply(teta0mut, G1, 3, m, m, m);
        double *teta0mutG1Gt = multiply(teta0mutG1, Gt, 3, m, m, m);
        double *teta0mutG1Gtmu = multiply(teta0mutG1Gt, mu, 3, m, m, 1);
        double *alpha = submatrix(TETAmuscal2, teta0mutG1Gtmu, 3, 1);

        double *G1t = transpose(G1, m, m);
        double *G1Gt = multiply(G1, Gt, m, m, m, m);
        double *GG1t = multiply(G, G1t, m, m, m, m);
        double *omega = addmatrix(G1Gt, GG1t, m, m);

        double *teta0teta0t = multiply(teta0, teta0t, 3, 1, 1, 3);
        double *teta0teta0tscalttt = multiplyscalar(teta0teta0t, pow(*ttt, -1.5), 3, 3);
        double *I3scalttt2 = multiplyscalar(I3, ttt2, 3, 3);
        double *sottr01 = submatrix( I3scalttt2, teta0teta0tscalttt,3,3);
        double *sott01scal2 = multiplyscalar(sottr01, 2, 3, 3);
        double *mutG1 = multiply(mut, G1, 1, m, m, m);
        double *mutG1Gt = multiply(mutG1,Gt, 1, m, m, m);
        double *mutG1Gtmu = multiply(mutG1Gt, mu, 1,m,m,1);
        double *A3tmu = multiply(A3t, mu, 1, m, m, 1);
        double *psi1 = multiplyscalar(sott01scal2, (  (*A3tmu)-( *mutG1Gtmu ) ), 3, 3);
        double *TETAfi = multiply(TETA, fi, 3, m, m, m);
        double *TETAfiomega = multiply(TETAfi, omega, 3, m, m, m);
        double *TETAfiomegamu = multiply(TETAfiomega, mu, 3, m, m, 1);
        double *TETAfiomegamuteta0t = multiply(TETAfiomegamu, teta0t, 3, 1, 1, 3);
        double *psi2 = multiplyscalar(TETAfiomegamuteta0t, 2.0l*ttt2, 3, 3);
        double *G1G1t = multiply(G1, G1t, m, m, m, m);
        double *omegafi = multiply(omega, fi, m, m, m, m);
        double *omegafiomega = multiply(omegafi, omega, m, m, m, m);
        double *sottr02 = submatrix(omegafiomega,G1G1t,m,m);
        double *mutsottr02 = multiply(mut, sottr02, 1, m, m, m);
        double *mutsottr02mu = multiply(mutsottr02, mu, 1, m, m, 1);
        double x = *mutsottr02mu;//mutsottr02mu è uno scalare
        x = 2*x;
        x = pow(*ttt, -1)*x;
        double *psi3 = multiplyscalar( teta0teta0t, x, 3, 3 );
        double *TETAt = transpose(TETA, 3, m);
        double *teta0mutomega = multiply(teta0mut, omega, 3, m, m, m);
        double *teta0mutomegafi = multiply(teta0mutomega, fi, 3, m, m, m);
        double *teta0mutomegafiTETAt = multiply(teta0mutomegafi, TETAt, 3, m, m, 3);
        double *psi4 = multiplyscalar(teta0mutomegafiTETAt, 2*ttt2, 3, 3);
        double *TETAfiTETAt = multiply(TETAfi, TETAt, 3, m, m, 3);
        double *psi5 = multiplyscalar(TETAfiTETAt, 2, 3, 3);
        double *sottr03 = submatrix(psi1, psi2, 3, 3);
        double *somma02 = addmatrix(sottr03, psi3, 3, 3);
        double *sottr04 = submatrix(somma02, psi4, 3, 3);
        double *psi = addmatrix(sottr04, psi5, 3, 3);

        double *psiinv = invert(psi, 3);
        double *psiinvalpha = multiply(psiinv, alpha, 3, 3, 3, 1);
        double *newteta0 = submatrix(teta0, psiinvalpha, 3, 1);

        memcpy(teta0, newteta0, 3*sizeof(double));

        /*
        double norm = frobeniusnorm(psi, 3, 1);;
        if(prevnorm != 0){
            if( (norm / prevnorm) < 0.8){
                stop = true;
            }
        }
        prevnorm = norm;
        //*/

        delete[] teta0t;
        delete[] G;
        delete[] G1;
        delete[] ttt;
        delete[] A2teta0;
        delete[] A3ttt1;
        delete[] somma01;
        delete[] beta;
        delete[] Gt;
        delete[] GGt;
        delete[] fi;
        delete[] mu;
        delete[] teta0A3t;
        delete[] teta0A3tscalttt2;
        delete[] TETA;
        delete[] TETAmu;
        delete[] TETAmuscal2;
        delete[] mut;
        delete[] teta0mut;
        delete[] teta0mutG1;
        delete[] teta0mutG1Gt;
        delete[] teta0mutG1Gtmu;
        delete[] alpha;
        delete[] G1t;
        delete[] G1Gt;
        delete[] GG1t;
        delete[] omega;
        delete[] teta0teta0t;
        delete[] teta0teta0tscalttt;
        delete[] I3scalttt2;
        delete[] sottr01;
        delete[] sott01scal2;
        delete[] mutG1;
        delete[] mutG1Gt;
        delete[] mutG1Gtmu;
        delete[] A3tmu;
        delete[] psi1;
        delete[] TETAfi;
        delete[] TETAfiomega;
        delete[] TETAfiomegamu;
        delete[] TETAfiomegamuteta0t;
        delete[] psi2;
        delete[] G1G1t;
        delete[] omegafi;
        delete[] omegafiomega;
        delete[] sottr02;
        delete[] mutsottr02;
        delete[] mutsottr02mu;
        delete[] psi3;
        delete[] TETAt;
        delete[] teta0mutomega;
        delete[] teta0mutomegafi;
        delete[] teta0mutomegafiTETAt;
        delete[] psi4;
        delete[] TETAfiTETAt;
        delete[] psi5;
        delete[] sottr03;
        delete[] somma02;
        delete[] sottr04;
        delete[] psi;
        delete[] psiinv;
        delete[] psiinvalpha;
        delete[] newteta0;

        count_min--;
    }
    while(count_min > 0);
    //while(!stop);

    delete[] A2t;
    delete[] A3t;
}

static void error(){
    double dist0 = sqrt(
        pow((anchorPositionRAW[0][0] - pos[0]), 2) +
        pow((anchorPositionRAW[0][1] - pos[1]), 2) +
        pow((anchorPositionRAW[0][2] - pos[2]), 2)
    );
    for(uint8_t i = 1; i < LOCODECK_NR_OF_ANCHORS; i++){
        double distanchor = sqrt(
            pow((anchorPositionRAW[i][0] - pos[0]), 2) +
            pow((anchorPositionRAW[i][1] - pos[1]), 2) +
            pow((anchorPositionRAW[i][2] - pos[2]), 2)
        );
        double diffreal = distanchor - dist0;
        hal.console->printf("\n error%d= %lf \n", i, distdiff[i]-diffreal);
    }
}

void AP_TDoA::printpos(){
    hal.console->printf("\n x=%lf y=%lf z=%lf \n", position[0], position[1], position[2]);
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

    //*
    if(_dev->register_periodic_callback( 990, FUNCTOR_BIND_MEMBER(&AP_TDoA::loop, void)) == nullptr){
        hal.console->printf("\n callback    nullptr \n");
    }
    //*/

    hal.console->printf("\n AP_TDoA.cpp CONF FINISHED \n");
}

void AP_TDoA::prova(){

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

    writeValueToBytes(dwDev->antennaDelay.raw, 16384, LEN_STAMP);
}

void AP_TDoA::enableAllLeds(){
    uint32_t reg;

    // Set all 4 GPIO in LED mode
    dw_read(GPIO_CTRL,GPIO_MODE_SUB,(uint8_t*)&reg,LEN_GPIO_MODE);
    reg &= ~0x00003FC0ul;
    reg |= 0x00001540ul;
    dw_write(GPIO_CTRL,GPIO_MODE_SUB,(uint8_t*)&reg,LEN_GPIO_MODE);

    // Enable debounce clock (used to clock the LED blinking)
    dw_read(PMSC,PMSC_CTRL0_SUB,(uint8_t*)&reg,LEN_PMSC_CTRL0);
    reg |= 0x00840000ul;
    dw_write(PMSC,PMSC_CTRL0_SUB,(uint8_t*)&reg,LEN_PMSC_CTRL0);

    // Enable LED blinking and set the rate
    reg = 0x00000110ul;
    dw_write(PMSC,PMSC_LEDC,(uint8_t*)&reg,LEN_PMSC_LEDC);

    // Trigger a manual blink of the LEDs for test
    reg |= 0x000f0000ul;
    dw_write(PMSC,PMSC_LEDC,(uint8_t*)&reg,LEN_PMSC_LEDC);
    reg &= ~0x000f0000ul;
    dw_write(PMSC,PMSC_LEDC,(uint8_t*)&reg,LEN_PMSC_LEDC);

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
    setBit(dwDev->syscfg, LEN_SYS_CFG, RXWTOE_BIT, timeout!=0);
    setBit(dwDev->syscfg, LEN_SYS_CFG, RXAUTR_BIT, true);

    hal.console->printf("\n newConfig finished \n");
}

void AP_TDoA::committConf(){
    dw_write(PANADR,NO_SUB,dwDev->networkAndAddress,LEN_PANADR);
    dw_write(SYS_CFG,NO_SUB,dwDev->syscfg,LEN_SYS_CFG);
    dw_write(CHAN_CTRL,NO_SUB,dwDev->chanctrl,LEN_CHAN_CTRL);
    dw_write(TX_FCTRL,NO_SUB,dwDev->txfctrl,LEN_TX_FCTRL);
    dw_write(SYS_MASK,NO_SUB,dwDev->sysmask,LEN_SYS_MASK);
    tune();
    dw_write(TX_ANTD,NO_SUB,dwDev->antennaDelay.raw,LEN_TX_ANTD);
    dw_write(LDE_IF,LDE_RXANTD_SUB,dwDev->antennaDelay.raw,LEN_LDE_RXANTD);

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
    dw_write(AGC_TUNE,AGC_TUNE2_SUB,agctune2,LEN_AGC_TUNE2);
    dw_write(AGC_TUNE,AGC_TUNE3_SUB,agctune3,LEN_AGC_TUNE3);
    dw_write(DRX_TUNE,DRX_TUNE0b_SUB,drxtune0b,LEN_DRX_TUNE0b);
    dw_write(DRX_TUNE,DRX_TUNE1a_SUB,drxtune1a,LEN_DRX_TUNE1a);
    dw_write(DRX_TUNE,DRX_TUNE1b_SUB,drxtune1b,LEN_DRX_TUNE1b);
    dw_write(DRX_TUNE,DRX_TUNE2_SUB,drxtune2,LEN_DRX_TUNE2);
    dw_write(DRX_TUNE,DRX_TUNE4H_SUB,drxtune4H,LEN_DRX_TUNE4H);
    dw_write(LDE_IF,LDE_CFG1_SUB,ldecfg1,LEN_LDE_CFG1);
    dw_write(LDE_IF,LDE_CFG2_SUB,ldecfg2,LEN_LDE_CFG2);
    dw_write(LDE_IF,LDE_REPC_SUB,lderepc,LEN_LDE_REPC);
    dw_write(TX_POWER,NO_SUB,txpower,LEN_TX_POWER);
    dw_write(RF_CONF,RF_RXCTRLH_SUB,rfrxctrlh,LEN_RF_RXCTRLH);
    dw_write(RF_CONF,RF_TXCTRL_SUB,rftxctrl,LEN_RF_TXCTRL);
    dw_write(TX_CAL,TC_PGDELAY_SUB,tcpgdelay,LEN_TC_PGDELAY);
    dw_write(FS_CTRL,FS_PLLTUNE_SUB,fsplltune,LEN_FS_PLLTUNE);
    dw_write(FS_CTRL,FS_PLLCFG_SUB,fspllcfg,LEN_FS_PLLCFG);

    //dwSpiWrite(dwDev, FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);

    hal.console->printf("\n tune finished \n");
}

void AP_TDoA::dwEnableClock(dwClock_t clock){
    uint8_t pmscctrl0[LEN_PMSC_CTRL0];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    dw_read(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
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
    dw_write(PMSC,PMSC_CTRL0_SUB,pmscctrl0,LEN_PMSC_CTRL0);
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
