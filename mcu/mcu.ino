#include <CAN.h>
#include <SPI.h>

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
          lidar->distance = rx[2] + rx[3] * 256;
          lidar->strength = rx[4] + rx[5] * 256;
          lidar->temp = (rx[6] + rx[7] * 256) / 8 - 256;
          lidar->receiveComplete = true;
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
}

void loop() {
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

    // CAN ID 0x12, 4-byte payload: distance (uint16 cm), speed (int16 cm/s).
    // Both signals use network byte order (most significant byte first).
    const uint16_t speed_raw = (uint16_t)(int16_t)speed_cm_s;
    CAN.beginPacket(0x12);
    CAN.write((uint8_t)(distance_cm >> 8));
    CAN.write((uint8_t)(distance_cm & 0xFF));
    CAN.write((uint8_t)(speed_raw >> 8));
    CAN.write((uint8_t)(speed_raw & 0xFF));
    CAN.endPacket();

    Serial.print("Sent distance [cm]: ");
    Serial.print(distance_cm);
    Serial.print(" speed [cm/s]: ");
    Serial.println((int)speed_cm_s);
  }
}
