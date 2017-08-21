#pragma once


//DWM1000 registers...some
#define DEV_ID      0x00    //LEN_DEV_ID        DWM1000_ID
#define SYS_CFG     0x04    //LEN_SYS_CFG       System Configuration bitmap
#define SYS_CTRL    0x0D    //LEN_SYS_CTRL
#define RX_FINFO    0x10    //LEN_RX_FINFO      info on received data
#define RX_BUFFER   0x11    //1024 bytes(max)   data that I need
#define RX_TIME     0x15    //LEN_RX_TIME       time stamp that I need


//registers' length
#define LEN_DEV_ID      4
#define LEN_SYS_CFG     4
#define LEN_SYS_CTRL    4
#define LEN_RX_FINFO    4
#define LEN_RX_BUFFER   1024
#define LEN_RX_TIME     14

//Initialization values
#define SET_SYS_CFG     0x20001200
                            //HIRQ_POL  bit 09
                            //DIS_DRXB  bit 12
                            //RXAUTR    bit 29
#define SET_SYS_CTRL    0x00000100
                            //RXENAB    bit 08

//stuff
#define DWM1000_ID              0xDECA0130
#define LOCODECK_NR_OF_ANCHORS  6


//Time stamp data type
typedef union dwTime_u {
  uint8_t raw[5];
  uint64_t full;    //I need This :)
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
  uint8_t txMaster[5];
  uint8_t timestamps[LOCODECK_NR_OF_ANCHORS][5];
} __attribute__((packed)) rangePacket_t;
