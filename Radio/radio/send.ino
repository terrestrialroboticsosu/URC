#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RF95_FREQ  915.0

#define START_BYTE 0xBE

RH_RF95 rf95(RFM95_CS, RFM95_INT);
uint8_t seq = 0;

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  Serial.begin(921600);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) while(1);
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

  // FAST settings
  rf95.setSpreadingFactor(7);
  rf95.setSignalBandwidth(250000);
  rf95.setCodingRate4(5);
  rf95.setPreambleLength(4);  // lower preamble = faster
}

void loop() {
  // Read Serial and send over LoRa
  if (Serial.available()) {
    uint8_t buf[200];
    int len = Serial.readBytes(buf, sizeof(buf));
    if (len > 0) {
      uint8_t packet[len + 2];
      packet[0] = START_BYTE;
      packet[1] = seq++;
      memcpy(&packet[2], buf, len);

      rf95.send(packet, len + 2);
      // DO NOT use waitPacketSent() → removes 1.5s delay
    }
  }
}
