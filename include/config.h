#ifndef __CONFIG_H__
#define __CONFIG_H__


typedef struct
{
    uint8_t  version; //eeprom version stored
    uint8_t  flsip;
    uint8_t  throtramp; //which output is used to control the AC relay
    uint8_t  fweak;
    uint8_t  canControl;
    uint8_t  type;
    uint8_t  phaseconfig;
    uint16_t voltSet;
    uint16_t tVolt;
    uint16_t currReq;
    uint32_t can0Speed;
    uint32_t can1Speed;
    uint16_t dcdcsetpoint;
}   LDUParams;

#endif