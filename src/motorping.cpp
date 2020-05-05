#include <Arduino.h>
#include <OBD2.h>
#include <DueTimer.h>

cAcquireCAN CANport0(CAN_PORT_0);   //do i need to show that this applies to the correct port on the due or is that handled automaticallY?

cCANFrame RAW_CAN_Frame1;
cCANFrame RAW_CAN_Frame2;

void CAN_RxTx()
{
    //run CAN acquisition schedulers on both ports including OBD and RAW CAN mesages (RX/TX)
    CANport0.run(TIMER_2mS);
}

void setup() {
    Serial.begin(115200);
    CANport0.initialize(_1000K);

    RAW_CAN_Frame1.ID = 0x1FF; // using 0011 for pdo and 127 in binary for motor node id
    RAW_CAN_Frame1.rate = QUERY_MSG; //this is sending the message over and over again, is there a way to send it just once?
    //do we want to send it just once?

    RAW_CAN_Frame2.ID = 0x200; //this might have to be changed depending on receiving filter
    RAW_CAN_Frame2.rate  = QUERY_MSG; //change to send only once
    CANport0.addMessage(&RAW_CAN_Frame1, TRANSMIT);
    CANport0.addMessage(&RAW_CAN_Frame2, RECEIVE);

    Timer3.attachInterrupt(CAN_RxTx).setFrequency(500).start();
}

void loop() {
    RAW_CAN_Frame1.U.b[0] = 0x40; //turn to hex
    RAW_CAN_Frame1.U.b[1] = 0x41;
    RAW_CAN_Frame1.U.b[2] = 0x60; //can be replaced to read motor specs (default)
    RAW_CAN_Frame1.U.b[3] = 0x00;
    RAW_CAN_Frame1.U.b[4] = 0x00;
    RAW_CAN_Frame1.U.b[5] = 0x00;
    RAW_CAN_Frame1.U.b[6] = 0x00;
    RAW_CAN_Frame1.U.b[7] = 0x00;

    for (int i=0; i<=7; i++) {
        Serial.print(RAW_CAN_Frame2.U.b[i], HEX);
    }
}

