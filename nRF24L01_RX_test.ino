#include <SPI.h>
#include "RF24.h"

const uint64_t pipe = 0xE8E8F0F0E1LL; // адрес канала передачи
RF24 radio(9,10);
char msg[210] = "";

void setup(){
  Serial.begin(9600);
  //============================================================Модуль NRF24
  radio.begin();                      // Включение модуля
  radio.setAutoAck(1);                // Установка режима подтверждения приема;
  radio.setDataRate(RF24_250KBPS);    // Устанавливаем скорость
  radio.setChannel(10);               // Устанавливаем канал
//  radio.setCRCLength(RF24_CRC_8); 
  radio.setPayloadSize(16);
  radio.openReadingPipe(1,pipe);      // Открываем 1 канал приема
  radio.startListening();             // Начинаем слушать эфир
  
}

void loop(){
  if (radio.available()){
    while (radio.available()){
      radio.read(&msg, sizeof(msg));
      Serial.println(msg);
    }
  }
}

