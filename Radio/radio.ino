#include <SPI.h>
#include <RH_RF95.h>

#define RADIO_FREQ_MHZ  915.0
#define RADIO_RECV_TIMEOUT 200  // 0.2 seconds (converted to ms)
#define ACK_PACKETS false

#define RFM95_CS   10
#define RFM95_RST  9
#define RFM95_INT  2

RH_RF95 rfm95(RFM95_CS, RFM95_INT);

void setup() {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    
    Serial.begin(115200);
    while (!Serial);

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (!rfm95.init()) while (1);
    if (!rfm95.setFrequency(RADIO_FREQ_MHZ)) while (1);
}

void loop() {
    
    if (Serial.available() > 0) {
        String serial_rx = Serial.readString();
        Serial.print("Sending: ");
        Serial.println(serial_rx);
        if (ACK_PACKETS) {
            rfm95.send((uint8_t*)serial_rx.c_str(), serial_rx.length());
            rfm95.waitPacketSent();
        } else {
            rfm95.send((uint8_t*)serial_rx.c_str(), serial_rx.length());
            rfm95.waitPacketSent();
        }
    }

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rfm95.waitAvailableTimeout(RADIO_RECV_TIMEOUT)) {
        if (rfm95.recv(buf, &len)) {
            Serial.write(buf, len);
        }
    }
}
