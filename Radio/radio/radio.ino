#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17

#define RF95_FREQ  915.0
#define START_BYTE1 0xBE
#define START_BYTE2 0xEF
#define CRC16 0xA001 // CRC-16

RH_RF95 rf95(RFM95_CS, RFM95_INT);

uint16_t gen_crc16(const uint8_t *data, uint16_t size) {
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    if (data == NULL)
        return 0;

    while (size > 0) {
        bit_flag = out >> 15;
        out <<= 1;
        out |= (*data >> bits_read) & 1;

        bits_read++;
        if (bits_read > 7) {
            bits_read = 0;
            data++;
            size--;
        }
        if (bit_flag)
            out ^= CRC16;
    }
    for (int i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if (bit_flag)
            out ^= CRC16;
    }
    uint16_t crc = 0;
    int i = 0x8000, j = 0x0001;
    for (; i != 0; i >>= 1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}

void setup() {
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    Serial.begin(115200);
    Serial.setTimeout(100); 
    while (!Serial);

    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if (!rf95.init()) {
        Serial.println("LoRa init failed");
        while (1);
    }

    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("Failed to set frequency");
        while (1);
    }
    // Changed 5-10km range to 1-3km
    rf95.setModemConfig(RH_RF95::Bw125Cr45Sf9);

    rf95.setTxPower(23, false);
    Serial.println("LoRa setup complete");
}

void sendPacket(const uint8_t *data, uint16_t length) {
    uint8_t packet[length + 6];

    packet[0] = START_BYTE1;
    packet[1] = START_BYTE2;

    packet[2] = length & 0xFF;
    packet[3] = (length >> 8) & 0xFF;

    memcpy(&packet[4], data, length);

    uint16_t crc = gen_crc16(data, length);
    packet[length + 4] = crc & 0xFF;
    packet[length + 5] = (crc >> 8) & 0xFF;

    rf95.send(packet, sizeof(packet));
    rf95.waitPacketSent();
    Serial.println("Sent!");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            sendPacket((uint8_t *)input.c_str(), input.length());
        }
    }

    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {

        if (len > 5 && buf[0] == START_BYTE1 && buf[1] == START_BYTE2) {

            uint16_t msg_len = buf[2] | (buf[3] << 8);

            if (msg_len + 6 != len) {
                Serial.println("Length Mismatch");
                return;
            }

            
            uint16_t received_crc = (buf[len - 1] << 8) | buf[len - 2];
            uint16_t computed_crc = gen_crc16(&buf[4], msg_len);

            if (received_crc == computed_crc) {
                Serial.print("Received: ");
                Serial.write(&buf[4], msg_len);
                Serial.println();
                Serial.print(" (Size: ");
                Serial.print(msg_len);
                Serial.println(" bytes)");
            } else {
                Serial.println("CRC Mismatch");
            }
        } else {
            Serial.println("Corrupted or Unrecognized Packet");
        }
    }
}
