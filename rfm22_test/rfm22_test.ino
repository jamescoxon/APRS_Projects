#include <SPI.h>
#include <RFM22.h>

#define GPS_ENABLE 5

rfm22 radio1(10);

void setupRadio(){ 
  digitalWrite(A5, LOW);
  delay(1000);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  radio1.setFrequency(434.00);
  radio1.write(0x6D, 0x06);
  radio1.write(0x07, 0x08); 
  delay(1000);

}

void setup() {
  //pinMode(GPS_ENABLE, OUTPUT);
  //digitalWrite(GPS_ENABLE, HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  pinMode(A5, OUTPUT);  
  digitalWrite(A5, HIGH);
  delay(500);
  setupRadio();
    radio1.write(0x07, 0x08);
  digitalWrite(7, HIGH);
}

void loop() {
  delay(5000);
}
