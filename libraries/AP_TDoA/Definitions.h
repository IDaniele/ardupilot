#pragma once

#include "dw1000.h"

#define DWM1000_ID              0xDECA0130
#define LOCODECK_NR_OF_ANCHORS  8
#define NR_OF_TDOA (LOCODECK_NR_OF_ANCHORS-1)
#define ANTENNA_OFFSET 154.6    // In meter
#define RX_TIMEOUT 10000
#define lightspeed (299702547.23582)           // speed of light
#define LOCODECK_TS_FREQ (499.2e6 * 128)    // Timestamp counter frequency

const uint8_t MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
const uint8_t MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
const uint8_t MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
const uint8_t MODE_LONGDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
const uint8_t MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};

static const uint8_t BIAS_500_16_ZERO = 10;
static const uint8_t BIAS_500_64_ZERO = 8;
static const uint8_t BIAS_900_16_ZERO = 7;
static const uint8_t BIAS_900_64_ZERO = 7;

// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
static const uint8_t BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31,   0,  36,  65,  84,  97, 106, 110, 112};
static const uint8_t BIAS_500_64[] = {110, 105, 100,  93,  82,  69,  51, 27,  0, 21,  35,  42,  49,  62,  71,  76,  81,  86};
static const uint8_t BIAS_900_16[] = {137, 122, 105, 88, 69,  47,  25,  0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
static const uint8_t BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29,  0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};

#define MAX_DISTANCE_DIFF (7.0f)

//                                                                              x          y           z
static const double anchorPositionRAW[LOCODECK_NR_OF_ANCHORS][3] = {    {   4.890,      0.546,      0.177},     //    0
                                                                        {   4.676,      5.741,      0.178},     //    1
                                                                        {   0.02,       3.079,      0.178},     //    2
                                                                        {   0.083,      0.875,      3.138},     //    3
                                                                        {   4.363,      3.113,      3.121},     //    4
                                                                        {   0.342,      5.460,      3.158}      //    5
                                                                 };
//                                                                            x          y           z
static const double anchorPosition[LOCODECK_NR_OF_ANCHORS][3] = {   {        0.0,       0.0,        0.000},     //      0
                                                                    {       -0.214,     5.195,      0.001},     //      1
                                                                    {       -4.87,      2.533,      0.001},     //      2
                                                                    {       -4.807,     0.329,      2.961},     //      3
                                                                    {       -0.527,     2.567,      2.944},     //      4
                                                                    {       -4.548,     4.914,      2.981}      //      5
                                                                };
//                                                                              x          y           z
static const double anchorPositionNEW[LOCODECK_NR_OF_ANCHORS][3] =  {
                                                                        { -2.713,       2.635,      0.178 },     //    0
                                                                        {  2.471,       2.419,      0.178 },     //    1
                                                                        { -0.182,      -2.215,      0.183 },     //    2
                                                                        { -2.413,      -2.185,      3.145 },     //    3
                                                                        {  0.038,       2.641,      3.209 },     //    4
                                                                        {  2.173,      -1.745,      3.167 }      //    5
#if LOCODECK_NR_OF_ANCHORS>6
                                                                       ,{  0.038,       2.641,      0.163 }      //    6
#endif
                                                                    };

//Time stamp data type
typedef union dwTime_u {
  uint8_t raw[5];
  uint64_t full;    //I need This
  struct {
    uint32_t low32;
    uint8_t high8;
  } __attribute__((packed));
  struct {
    uint8_t low8;
    uint32_t high32;
  } __attribute__((packed));
} dwTime_t;

// Packets data type
typedef uint64_t locoAddress_t;

typedef struct packet_s {
    union {
      uint16_t fcf;
      struct {
        uint16_t type:3;
        uint16_t security:1;
        uint16_t framePending:1;
        uint16_t ack:1;
        uint16_t ipan:1;
        uint16_t reserved:3;
        uint16_t destAddrMode:2;
        uint16_t version:2;
        uint16_t srcAddrMode:2;
      } fcf_s;
    };
    uint8_t seq;
    uint16_t pan;
    locoAddress_t destAddress;
    locoAddress_t sourceAddress;
    uint8_t payload[64];
} __attribute__((packed)) packet_t;

typedef struct rangePacket_s {
  uint8_t type;
  uint8_t sequenceNrs[LOCODECK_NR_OF_ANCHORS];
  uint32_t timestamps[LOCODECK_NR_OF_ANCHORS];
  uint16_t distances[LOCODECK_NR_OF_ANCHORS];
} __attribute__((packed)) rangePacket_t;

//dwDevice
typedef struct dwDevice_s {
  struct dwOps_s *ops;
  void *userdata;

  /* State */
  uint8_t sysctrl[LEN_SYS_CTRL];
  uint8_t deviceMode;
  uint8_t networkAndAddress[LEN_PANADR];
  uint8_t syscfg[LEN_SYS_CFG];
  uint8_t sysmask[LEN_SYS_MASK];
  uint8_t chanctrl[LEN_CHAN_CTRL];
  uint8_t sysstatus[LEN_SYS_STATUS];
  uint8_t txfctrl[LEN_TX_FCTRL];

  uint8_t extendedFrameLength;
  uint8_t pacSize;
  uint8_t pulseFrequency;
  uint8_t dataRate;
  uint8_t preambleLength;
  uint8_t preambleCode;
  uint8_t channel;
  bool smartPower;
  bool frameCheck;
  bool permanentReceive;
  bool wait4resp;

  dwTime_t antennaDelay;

  // Callback handles
  /*
  dwHandler_t handleSent;
  dwHandler_t handleReceived;
  dwHandler_t handleReceiveTimeout;
  dwHandler_t handleReceiveFailed;
  //*/
} dwDevice_t;

struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s point_t;

typedef uint64_t locoAddress_t;

typedef struct {
  const uint64_t antennaDelay;
  const int rangingFailedThreshold;

  const locoAddress_t tagAddress;
  const locoAddress_t anchorAddress[LOCODECK_NR_OF_ANCHORS];

  point_t anchorPosition[LOCODECK_NR_OF_ANCHORS];
  bool combinedAnchorPositionOk;

  float distance[LOCODECK_NR_OF_ANCHORS];
  float pressures[LOCODECK_NR_OF_ANCHORS];
  int failedRanging[LOCODECK_NR_OF_ANCHORS];
  volatile uint16_t rangingState;
} lpsAlgoOptions_t;

typedef enum {dwClockAuto = 0x00, dwClockXti = 0x01, dwClockPll = 0x02} dwClock_t;

typedef struct{
    uint8_t anchor;
    packet_t newpack;
    uint64_t rxtime;
} recvData;

/*
static lpsAlgoOptions_t algoOptions = {
  .tagAddress = 0xbccf000000000008,
  .anchorAddress = {
    0xbccf000000000000,
    0xbccf000000000001,
    0xbccf000000000002,
    0xbccf000000000003,
    0xbccf000000000004,
    0xbccf000000000005,
#if LOCODECK_NR_OF_ANCHORS > 6
    0xbccf000000000006,
#endif
#if LOCODECK_NR_OF_ANCHORS > 7
    0xbccf000000000007,
#endif
  },
  .antennaDelay = (ANTENNA_OFFSET*499.2e6*128)/299792458.0, // In radio tick
  .rangingFailedThreshold = 6,

  .combinedAnchorPositionOk = false,

  // To set a static anchor position from startup, uncomment and modify the
  // following code:
//   .anchorPosition = {
//     {timestamp: 1, x: 0.99, y: 1.49, z: 1.80},
//     {timestamp: 1, x: 0.99, y: 3.29, z: 1.80},
//     {timestamp: 1, x: 4.67, y: 2.54, z: 1.80},
//     {timestamp: 1, x: 0.59, y: 2.27, z: 0.20},
//     {timestamp: 1, x: 4.70, y: 3.38, z: 0.20},
//     {timestamp: 1, x: 4.70, y: 1.14, z: 0.20},
//   },
//
//   .combinedAnchorPositionOk = true,
};
//*/
