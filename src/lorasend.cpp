#include "ATOM_DTU_LoRaWAN.h"
#include "M5Atom.h"

ATOM_DTU_LoRaWAN LoRaWAN;
String response;

typedef enum {
    kError = 0,
    kConnecting,
    kConnected, 
    kSending
} DTUState_t;

DTUState_t State = kConnecting;

void sendData(String data) {
    M5.begin(true, true, true);
    //InIt
    LoRaWAN.Init();

    //Reset Module
    LoRaWAN.writeCMD("AT+ILOGLVL=0\r\n");
    LoRaWAN.writeCMD("AT+IREBOOT=0\r\n");
    delay(2000);

    LoRaWAN.configOTTA(
        "70b3d57ed005b3b8",//Device EUI
        "0000000000000000",//APP EUI
        "6D391AB343CB01BADFC8ACC0C3083B04",//APP KEY
        "2"//Upload Download Mode
    );

    response = LoRaWAN.waitMsg(1000);
    
    //Set Class Mode 
    LoRaWAN.setClass("2");

    LoRaWAN.writeCMD("AT+CWORKMODE=2\r\n");

    //LoRaWAN868
    LoRaWAN.setRxWindow("869525000");

    LoRaWAN.setFreqMask("0001");

    delay(100);
    
    LoRaWAN.startJoin();
    Serial.print("Start Join.....");

    while(1){
        response = LoRaWAN.waitMsg(1000);
        
        if (response.indexOf("+CJOIN:") != -1) {
            State = kConnected;
            Serial.println("Join OK.");
            break;
        } else if (response.indexOf("ERROR") != -1) {
            State = kError;
            Serial.print("Join ERROR.");
            ESP.restart();
        }
    }

    delay(6000);
    LoRaWAN.sendMsg(1,15,8,data);

    while(1) {
        State = kSending;
        response = LoRaWAN.waitMsg(3000);
        Serial.println(response);
        if(response.indexOf("OK") != -1) {
            break;
        }else if(response.indexOf("ERR") != -1){
            State = kError;
            break;
        }
    }
}
