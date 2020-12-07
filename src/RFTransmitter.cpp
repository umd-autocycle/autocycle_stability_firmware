#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // pins for CE, CSN
const byte address[6] = "00001";

char str[80];

void setup() {
    radio.begin();
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);// we can increase this if they dont connect but then a bypass diode is needed
    radio.stopListening();
}
void loop() {
    sprintf(str,"%d %d %d %d %d %d \n",state,phi,dphi,delta,ddelta,velocity);
    radio.write(&str, sizeof(str));
    delay(1000);
}

//
// Created by Crosby on 12/6/2020.
//

