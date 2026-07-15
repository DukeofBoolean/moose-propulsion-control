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
  
  int current_distance = 20000;
  getLidarData(&Lidar);       // Acquisition of LiDAR data
  
  if (Lidar.receiveComplete) {  // Process only if new data is received
    current_distance = Lidar.distance;
    Lidar.receiveComplete = false;  // Reset the flag after processing
  }

  //8 bits are 1 byte
  //1 char is 8 bits = 1 byte
  //Can send max 8 bytes on the CAN network, so maximum of 8 chars
  unsigned long current_time = millis();
  if((current_time - previous_time >= sampling_interval)){

     // Calculate speed
    int time_elapsed = current_time - previous_time; // Time difference in milliseconds
    int current_distance = Lidar.distance;
    int speed = 0; // Speed in cm/s
    int distance_change = previous_distance - current_distance;
    if (time_elapsed > 0 && distance_change > 0) {
      speed = distance_change * 1000 / time_elapsed; // Speed in cm/s
    }

    else{
      speed = 0;
    }

    previous_distance = current_distance;
    previous_time = current_time;
    
    Serial.print("Sending packet ... ");
    CAN.beginPacket(0x12); // Begin sending a message from the node with address 0x12

    bool leading_zero = true;

    int thousands = current_distance / 1000;
    int hundreds = (current_distance / 100) % 10;
    int tens = (current_distance / 10) % 10;
    int units = current_distance % 10;

    if (thousands > 0) {
        CAN.write(thousands + '0');
        leading_zero = false;
    }
    
    if (hundreds > 0 || !leading_zero) {
        CAN.write(hundreds + '0');
        leading_zero = false;
    }
    
    if (tens > 0 || !leading_zero) {
        CAN.write(tens + '0');
        leading_zero = false;
    }
    
    CAN.write(units + '0');  //Always write the units digit

    CAN.write(' ');

    leading_zero = true;

    int speed_thousands = speed / 1000;
    int speed_hundreds = (speed / 100) % 10;
    int speed_tens = (speed / 10) % 10;
    int speed_units = speed % 10;

    if (speed_thousands > 0) {
        CAN.write(speed_thousands + '0');
        leading_zero = false;
    }

    if (speed_hundreds > 0 || !leading_zero) {
        CAN.write(speed_hundreds + '0');
        leading_zero = false;
    }

    if (speed_tens > 0 || !leading_zero) {
        CAN.write(speed_tens + '0');
        leading_zero = false;
    }

    CAN.write(speed_units + '0'); // Always send units digit
    
    CAN.endPacket();
    Serial.println("done");
  }

  //delay(100); //Use delay if necessary
}
