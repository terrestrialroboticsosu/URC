#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RF95_FREQ  915.0

#define START_BYTE 0xBE

RH_RF95 rf95(RFM95_CS, RFM95_INT);

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
  rf95.setPreambleLength(4);
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      if (len >= 2 && buf[0] == START_BYTE) {
        Serial.write(&buf[2], len - 2);  // forward payload
      }
    }
  }
}
