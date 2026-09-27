#include <CAN.h>
#include <SPI.h>
#include "j1939_address.h"

typedef struct {
  int distance;
  int strength;
  int temp;
  boolean receiveComplete;
} TF;

TF Lidar = {0, 0, 0, false};
int previous_distance = 0;
unsigned long previous_time = 0;
const unsigned long sampling_interval = 100; // Sampling interval in milliseconds

// Provisional local addresses for this three-node test network.
const uint8_t J1939_SA_MCU = 0xA2;
const uint32_t J1939_PGN_LMSD_MCU = 0x00FF00;
J1939AddressClaim j1939Node;

void getLidarData(TF* lidar) 
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (Serial.available()) 
  {
    rx[i] = Serial.read();
    if (rx[0] != 0x59) 
    {
      i = 0;
    } 
    else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } 
    else if (i == 8) 
    {
      for (j = 0; j < 8; j++) 
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) 
      {
          const uint16_t distance = rx[2] + (uint16_t)rx[3] * 256;
          const uint16_t strength = rx[4] + (uint16_t)rx[5] * 256;
          if (distance > 0 && strength > 0) {
            lidar->distance = distance;
            lidar->strength = strength;
            lidar->temp = (rx[6] + rx[7] * 256) / 8 - 256;
            lidar->receiveComplete = true;
          }
      }
      i = 0;
    } 
    else 
    {
      i++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("CAN Sender");

  // Start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // Identity 3 is unique within this project. The manufacturer field in this
  // test NAME is provisional; replace it with an SAE-assigned code later.
  j1939Node.begin(J1939_SA_MCU, j1939MakeTestName(3));
}

void pollJ1939Network() {
  const int packetSize = CAN.parsePacket();
  if (packetSize <= 0) return;

  const uint32_t id = (uint32_t)CAN.packetId();
  const bool extended = CAN.packetExtended();
  uint8_t payload[8];
  int bytesRead = 0;
  while (CAN.available()) {
    const int value = CAN.read();
    if (bytesRead < (int)sizeof(payload)) payload[bytesRead] = (uint8_t)value;
    bytesRead++;
  }
  j1939Node.handleFrame(id, extended, payload, bytesRead);
}

void loop() {
  pollJ1939Network();
  j1939Node.update();
  getLidarData(&Lidar);

  const unsigned long current_time = millis();
  if (Lidar.receiveComplete && current_time - previous_time >= sampling_interval) {
    const uint16_t distance_cm = (uint16_t)Lidar.distance;
    Lidar.receiveComplete = false;

    const unsigned long time_elapsed = current_time - previous_time;
    const int32_t distance_change = (int32_t)previous_distance - distance_cm;
    int32_t speed_cm_s = 0;
    if (previous_time != 0 && time_elapsed > 0) {
      speed_cm_s = distance_change * 1000L / (int32_t)time_elapsed;
    }
    if (speed_cm_s > 32767) speed_cm_s = 32767;
    if (speed_cm_s < -32768) speed_cm_s = -32768;

    previous_distance = distance_cm;
    previous_time = current_time;

    // PGN 0xFF00, 8-byte J1939 frame. Multi-byte signals are little-endian;
    // unused bytes are set to 0xFF (not available).
    const uint16_t speed_raw = (uint16_t)(int16_t)speed_cm_s;
    if (j1939Node.mayTransmitApplication()) {
      const uint32_t id = j1939MakeId(6, J1939_PGN_LMSD_MCU,
                                      J1939_GLOBAL_ADDRESS, j1939Node.address());
      CAN.beginExtendedPacket(id);
      CAN.write((uint8_t)(distance_cm & 0xFF));
      CAN.write((uint8_t)(distance_cm >> 8));
      CAN.write((uint8_t)(speed_raw & 0xFF));
      CAN.write((uint8_t)(speed_raw >> 8));
      CAN.write(0xFF);
      CAN.write(0xFF);
      CAN.write(0xFF);
      CAN.write(0xFF);
      CAN.endPacket();
    }

    Serial.print("Sent distance [cm]: ");
    Serial.print(distance_cm);
    Serial.print(" speed [cm/s]: ");
    Serial.println((int)speed_cm_s);
  }
}
